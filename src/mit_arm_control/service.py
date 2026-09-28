"""Single CAN owner, independent from algorithm and telemetry workers."""
import fcntl
import json
import os
from pathlib import Path
import queue
import re
import signal
import subprocess
import time
from .backend import SimBackend, SocketCAN
from .config import readiness
from .controller import Controller
from .ipc import Mailbox, IPCServer, AsyncLogger


def hardware_locks(config, commissioned=True):
    errors = readiness(config, hardware=True) if commissioned else []
    if errors:
        raise ValueError('hardware configuration incomplete: '+ '; '.join(errors))
    locks = []
    try:
        for bus in sorted({j['bus'] for j in config['joints']}):
            if not re.fullmatch(r'[a-zA-Z0-9_.-]{1,15}', bus):
                raise ValueError('invalid CAN interface name')
            fd = os.open(f'/tmp/mit-arm-{os.getuid()}-{bus}.lock',
                         os.O_CREAT | os.O_RDWR | os.O_NOFOLLOW, 0o600)
            locks.append(fd)
            fcntl.flock(fd, fcntl.LOCK_EX | fcntl.LOCK_NB)
            data = json.loads(subprocess.check_output(['ip', '-j', '-details', 'link', 'show', bus], timeout=2))[0]
            can = data.get('linkinfo', {}).get('info_data', {})
            if (data.get('mtu') != 72 or 'UP' not in data.get('flags', [])
                    or can.get('bittiming', {}).get('bitrate') != 1000000
                    or can.get('data_bittiming', {}).get('bitrate') != 5000000):
                raise ValueError(f'{bus}: require UP CAN FD, arbitration 1M, data 5M; configure externally')
        return locks
    except BaseException:
        for fd in locks:
            os.close(fd)
        raise


def run(config, socket_path, hardware=False, result_path=None):
    # Simulation is the default. Real bus access needs explicit --hardware and
    # complete commissioning config, but still does not enable at construction.
    errors = readiness(config, hardware)
    if errors:
        raise ValueError('; '.join(errors))
    locks, backend, ipc = [], None, None
    mailbox, logger = Mailbox(), AsyncLogger()
    exiting = False
    controller = None
    old_signals = {}

    def request_exit(signum, frame):
        nonlocal exiting
        exiting = True
        mailbox.request_stop(f'service signal {signum}')

    def publish():
        if controller is not None:
            mailbox.snapshot = dict(controller.snapshot(),
                                    log_dropped=logger.dropped, log_failures=logger.failures)

    try:
        if hardware:
            locks = hardware_locks(config)
        backend = SocketCAN(config['joints']) if hardware else SimBackend(config['joints'])
        controller = Controller(config, backend, hardware, cancelled=mailbox.stop.is_set, publish=publish)
        publish()
        ipc = IPCServer(socket_path, mailbox, config)
        for sig in (signal.SIGINT, signal.SIGTERM):
            old_signals[sig] = signal.signal(sig, request_exit)
        next_snapshot = next_log = time.monotonic()
        while not exiting:
            if mailbox.stop.is_set():
                reason, fault = mailbox.stop_reason, mailbox.fault
                mailbox.take_command()
                while True:
                    try:
                        mailbox.actions.get_nowait()
                    except queue.Empty:
                        break
                # Never auto-reset a latched fault on a second stop/disconnect.
                controller.stop(reason, fault=fault or controller.state == 'FAULT')
                mailbox.completed_ticket = max(mailbox.completed_ticket, mailbox.stop_ticket)
                mailbox.stop_ticket = 0
                mailbox.stop.clear()
                mailbox.fault = False
                publish()
            else:
                try:
                    op, args, ticket = mailbox.actions.get_nowait()
                except queue.Empty:
                    op = None
                if op:
                    try:
                        mailbox.take_command()
                        if op == 'arm':
                            controller.arm(args.get('supported'), args.get('zero_pose'))
                        else:
                            controller.reset_fault()
                    except Exception as exc:
                        mailbox.last_error = str(exc)
                    mailbox.completed_ticket = ticket
                    publish()
                command = mailbox.take_command()
                if command is not None and not mailbox.stop.is_set():
                    try:
                        controller.submit(command)
                    except Exception as exc:
                        mailbox.last_error = str(exc)
                    publish()
                controller.step()
                # Publish only lightweight feedback each loop; histogram/statistics
                # snapshots are less frequent and logging never runs here.
                mailbox.snapshot = dict(mailbox.snapshot,
                    feedback={n: dict(f) for n, f in backend.feedback.items()},
                    feedback_snapshot_timestamp=time.monotonic())
            now = time.monotonic()
            if now >= next_snapshot:
                publish()
                next_snapshot = now+.02
            if now >= next_log:
                logger.offer(mailbox.snapshot)
                next_log = now+1.
            remaining = controller.next_tick-now if controller.state in controller.ACTIVE else .001
            time.sleep(max(0., min(.0002, remaining)))
    finally:
        # Stop before waiting on IPC, logger or disk. Even an unexpected service
        # exception attempts confirmation; SIGKILL/power loss cannot run finally.
        if controller is not None:
            if controller.state not in ('IDLE',) or controller.stop_confirmed is False:
                controller.stop('service exit', fault=controller.state == 'FAULT')
            publish()
        if ipc:
            ipc.close()
        if backend:
            backend.close()
        logger.close()
        for fd in locks:
            os.close(fd)
        for sig, handler in old_signals.items():
            signal.signal(sig, handler)
        if result_path and controller is not None:
            # Final result is separate from the lossy periodic telemetry channel.
            path = Path(result_path)
            temporary = path.with_name(path.name+'.tmp')
            temporary.write_text(json.dumps(mailbox.snapshot, ensure_ascii=False, indent=2, allow_nan=False))
            temporary.replace(path)
    return mailbox.snapshot
