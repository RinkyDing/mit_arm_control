"""Local, bounded IPC and nonblocking telemetry isolation."""
import json
import os
import queue
import selectors
import socket
import threading
import time
from .safety import validate_command

VERSION, MAX_MESSAGE = 1, 32768


class Mailbox:
    def __init__(self):
        self.lock = threading.Lock()
        self.latest = None
        self.actions = queue.Queue(maxsize=8)
        self.stop = threading.Event()
        self.stop_reason = 'operator stop'
        self.fault = False
        self.snapshot = {'version': 1, 'state': 'IDLE', 'last_seq': -1}
        self.last_error = None
        self.serial = 0
        self.completed_ticket = 0
        self.stop_ticket = 0

    def request_stop(self, reason, fault=False):
        self.stop_reason, self.fault = reason, fault
        self.stop.set()

    def put_command(self, command):
        with self.lock:
            self.latest = command

    def take_command(self):
        with self.lock:
            command, self.latest = self.latest, None
        return command


class IPCServer:
    def __init__(self, path, mailbox, config):
        self.path, self.mailbox, self.config = path, mailbox, config
        self.done = threading.Event()
        self.owner = None
        self.previous = None
        self.selector = selectors.DefaultSelector()
        self.clients = {}
        if os.path.lexists(path):
            raise RuntimeError(f'socket path already exists: {path}; check service before removing stale socket')
        self.server = socket.socket(socket.AF_UNIX, socket.SOCK_SEQPACKET)
        self.server.bind(path)
        os.chmod(path, 0o600)
        self.server.listen(8)
        self.server.setblocking(False)
        self.selector.register(self.server, selectors.EVENT_READ)
        self.thread = threading.Thread(target=self.run, daemon=True, name='mit-ipc')
        self.thread.start()

    def disconnect(self, client):
        if self.owner is client:
            self.owner = None
            self.previous = None
            self.mailbox.request_stop('algorithm disconnected', fault=True)
        self.clients.pop(client, None)
        try:
            self.selector.unregister(client)
        except (KeyError, ValueError):
            pass
        client.close()

    def dispatch(self, client, msg):
        if not isinstance(msg, dict) or msg.get('version') != VERSION or type(msg.get('id')) is not int:
            raise ValueError('invalid IPC envelope/version')
        op = msg.get('op')
        if op == 'connect':
            if self.clients[client] is not None:
                raise ValueError('already connected')
            if msg.get('role', 'control') == 'control':
                if self.owner is not None:
                    raise ValueError('another algorithm owns control')
                self.owner = client
                self.previous = None
                self.clients[client] = 'control'
            elif msg.get('role') == 'observer':
                self.clients[client] = 'observer'
            else:
                raise ValueError('unknown role')
            return {'last_seq': self.mailbox.snapshot.get('last_seq', -1)}
        if self.clients[client] is None:
            raise ValueError('connect first')
        if op == 'get_state':
            # Immutable-by-convention snapshot, replaced by the control owner.
            snap = self.mailbox.snapshot
            return dict(snap, last_request_error=self.mailbox.last_error,
                        completed_ticket=self.mailbox.completed_ticket,
                        feedback={n: dict(f, age_ms=(time.monotonic()-f['timestamp'])*1000)
                                  for n, f in snap.get('feedback', {}).items()})
        if self.owner is not client:
            raise ValueError('observer has no motor-control permission')
        if op == 'stop':
            self.mailbox.serial += 1
            self.mailbox.stop_ticket = self.mailbox.serial
            self.mailbox.request_stop('operator stop')
            return {'queued': True, 'ticket': self.mailbox.serial}
        if op == 'submit':
            if self.mailbox.snapshot['state'] not in ('READY', 'RUNNING', 'DEGRADED'):
                raise ValueError('submit requires READY/RUNNING/DEGRADED')
            cmd = msg.get('command')
            try:
                validate_command(self.config, cmd, time.monotonic(), self.previous)
            except (ValueError, TypeError, KeyError) as exc:
                self.mailbox.request_stop(f'invalid algorithm command: {exc}', fault=True)
                raise
            self.previous = cmd
            self.mailbox.put_command(cmd)
            return {'queued': True, 'seq': cmd['seq']}
        if op in ('arm', 'reset_fault'):
            self.mailbox.last_error = None
            self.mailbox.serial += 1
            self.mailbox.actions.put_nowait((op, msg, self.mailbox.serial))
            if op == 'arm':
                self.previous = None
            return {'queued': True, 'ticket': self.mailbox.serial}
        raise ValueError('unknown operation')

    def run(self):
        try:
            while not self.done.is_set():
                for key, _ in self.selector.select(.05):
                    client = key.fileobj
                    if client is self.server:
                        client, _ = self.server.accept()
                        if len(self.clients) >= 8:
                            client.close()
                            continue
                        client.setblocking(False)
                        self.clients[client] = None
                        self.selector.register(client, selectors.EVENT_READ)
                        continue
                    try:
                        raw, _, flags, _ = client.recvmsg(MAX_MESSAGE)
                        if not raw or flags & socket.MSG_TRUNC:
                            self.disconnect(client)
                            continue
                        msg = json.loads(raw)
                        try:
                            result = self.dispatch(client, msg)
                            reply = dict(version=VERSION, id=msg['id'], ok=True, result=result)
                        except (ValueError, TypeError, KeyError, queue.Full) as exc:
                            reply = dict(version=VERSION, id=msg.get('id') if isinstance(msg, dict) else None,
                                         ok=False, error=str(exc))
                            if client is self.owner and self.mailbox.snapshot['state'] in ('RUNNING', 'DEGRADED'):
                                self.mailbox.request_stop(f'IPC request rejected: {exc}', fault=True)
                        encoded = json.dumps(reply, allow_nan=False).encode()
                        if len(encoded) > MAX_MESSAGE or client.send(encoded) != len(encoded):
                            self.disconnect(client)
                    except (OSError, ValueError, TypeError):
                        self.disconnect(client)
        except Exception as exc:
            self.mailbox.request_stop(f'IPC worker failed: {exc}', fault=True)
        finally:
            for client in list(self.clients):
                self.disconnect(client)

    def close(self):
        self.done.set()
        self.thread.join(.2)
        self.server.close()
        self.selector.close()
        if os.path.exists(self.path):
            os.unlink(self.path)


class AsyncLogger:
    def __init__(self, sink=None):
        # A separate nonblocking file description avoids blocking buffered
        # stdout's interpreter lock during shutdown when a pipe stops draining.
        self.output_fd = None
        if sink is None:
            try:
                self.output_fd = os.open('/proc/self/fd/1', os.O_WRONLY | os.O_NONBLOCK | os.O_APPEND | os.O_CLOEXEC)
            except OSError:
                pass
            sink = self.write_stdout
        self.queue = queue.Queue(maxsize=2)
        self.sink, self.dropped, self.failures = sink, 0, 0
        self.done = threading.Event()
        self.thread = threading.Thread(target=self.run, daemon=True, name='mit-telemetry')
        self.thread.start()

    def write_stdout(self, line):
        if self.output_fd is None:
            raise BrokenPipeError('stdout unavailable')
        data = (line+'\n').encode()
        while data and not self.done.is_set():
            written = os.write(self.output_fd, data)
            data = data[written:]

    def offer(self, snapshot):
        try:
            self.queue.put_nowait(snapshot)
        except queue.Full:
            self.dropped += 1

    def run(self):
        while not self.done.is_set():
            try:
                snapshot = self.queue.get(timeout=.05)
            except queue.Empty:
                continue
            try:
                self.sink(json.dumps(snapshot, ensure_ascii=False, allow_nan=False))
            except Exception:
                self.failures += 1

    def close(self):
        self.done.set()
        self.thread.join(.1)  # blocked terminal must never block shutdown
        if self.output_fd is not None and not self.thread.is_alive():
            os.close(self.output_fd)
            self.output_fd = None
