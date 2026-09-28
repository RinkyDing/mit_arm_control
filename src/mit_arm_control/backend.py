"""Single-owner backends. Neither constructor enables, zeros or writes motors."""
from collections import deque
import select
import socket
import struct
import time
from . import protocol as p
from .config import MODELS


class SocketCAN:
    def __init__(self, joints, clock=time.monotonic):
        self.joints, self.clock = joints, clock
        self.feedback, self.parameters, self.sockets = {}, {}, {}
        self._round_robin = 0
        self.rx_seq = {j['name']: 0 for j in joints}
        try:
            for bus in {j['bus'] for j in joints if j.get('bus')}:
                s = socket.socket(socket.AF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
                self.sockets[bus] = s
                s.setsockopt(socket.SOL_CAN_RAW, socket.CAN_RAW_FD_FRAMES, 1)
                # Linux timestamp ancillary data prevents queued old frames from
                # looking fresh simply because Python processed them just now.
                s.setsockopt(socket.SOL_SOCKET, getattr(socket, 'SO_TIMESTAMPNS', 35), 1)
                s.setblocking(False)
                s.bind((bus,))
        except BaseException:
            self.close()
            raise

    def send(self, frame):
        raw = struct.pack('=IBBBB64s', frame.can_id, len(frame.data), 1, 0, 0,
                          frame.data.ljust(64, b'\x00'))
        if self.sockets[frame.bus].send(raw) != 72:
            raise OSError('short CAN FD write')

    def poll(self, timeout=0):
        ready, _, _ = select.select(list(self.sockets.values()), [], [], timeout)
        if not ready:
            return False
        # One frame per poll; the owner budgets receive work against TX deadline.
        s = ready[self._round_robin % len(ready)]
        self._round_robin += 1
        raw, ancillary, flags, _ = s.recvmsg(72, 128)
        if flags & (socket.MSG_TRUNC | socket.MSG_CTRUNC):
            return True
        received_at = None
        for level, kind, data in ancillary:
            if level == socket.SOL_SOCKET and kind == getattr(socket, 'SO_TIMESTAMPNS', 35):
                sec, ns = struct.unpack('@ll', data[:struct.calcsize('@ll')])
                age = time.time()-(sec+ns/1e9)
                if age >= -.001:
                    received_at = self.clock()-max(0., age)
        if received_at is None:
            return True  # never label an un-timestamped hardware sample as fresh
        if len(raw) not in (16, 72):
            return True
        can_id = struct.unpack_from('=I', raw)[0]
        if can_id & 0xE0000000:  # reject EFF, RTR and CAN error frames as feedback
            return True
        size = raw[4]
        if size > (64 if len(raw) == 72 else 8):
            return True
        bus = next(b for b, sock in self.sockets.items() if sock is s)
        frame = p.Frame(bus, can_id, raw[8:8+size], len(raw) == 72, raw[5])
        for j in self.joints:
            # Diagnostics may read registers before calibration is configured.
            parser_joint = dict(j, direction=j.get('direction') or 1,
                                zero_joint=j.get('zero_joint') or 0)
            decoded = p.decode(parser_joint, frame)
            if decoded is None:
                continue
            kind, data = decoded
            if kind == 'parameter':
                op, rid, value = data
                self.parameters[(j['name'], op, rid)] = (self.clock(), value)
            else:
                data['timestamp'] = received_at
                self.feedback[j['name']] = data
                self.rx_seq[j['name']] += 1
            break
        return True

    def special(self, j, code):
        self.send(p.special(j, code))

    def refresh(self, j):
        self.send(p.register_request(j, p.STATUS))

    def mit(self, j, target):
        self.send(p.pack_mit(j, target))

    def register(self, j, op, rid, value=0, interrupt=lambda: None):
        # Sequential transactions only; a cached/late value cannot satisfy a new read.
        for _ in range(128):
            if not self.poll(0):
                break
        key = (j['name'], op, rid)
        self.parameters.pop(key, None)
        self.send(p.register_request(j, op, rid, value))
        deadline = self.clock()+.2
        while self.clock() < deadline:
            interrupt()
            if key in self.parameters:
                return self.parameters.pop(key)[1]
            self.poll(min(.002, max(0, deadline-self.clock())))
        raise TimeoutError(f"{j['name']}: register {rid} opcode {op} timeout")

    def close(self):
        for s in self.sockets.values():
            s.close()
        self.sockets.clear()


class SimBackend:
    """Functional simulation, NOT a dynamics/safety certification model.

    Fault injection is explicit and only implemented here.
    """
    def __init__(self, joints, clock=time.monotonic):
        self.joints, self.clock = joints, clock
        self.feedback, self.parameters = {}, {}
        self.rx_seq = {j['name']: 0 for j in joints}
        self.state = {j['name']: dict(q=j['zero_joint'], dq=0., tau=0., status=0,
                      mos_temperature=25, rotor_temperature=25) for j in joints}
        self.registers = {}
        for j in joints:
            pr, vr, tr = MODELS[j['model']]
            self.registers[j['name']] = {10: 1, 21: pr, 22: vr, 23: tr}
        self.queue = deque(maxlen=256)
        self.drop = set()
        self.drop_next = {}
        self.send_error = False
        self.ignore_disable = set()
        self.sent = 0
        self.special_history = []

    def _send(self):
        if self.send_error:
            raise OSError(105, 'injected ENOBUFS')
        self.sent += 1

    def _reply(self, j):
        name = j['name']
        if name in self.drop:
            return
        if self.drop_next.get(name, 0):
            self.drop_next[name] -= 1
            return
        self.queue.append((name, dict(self.state[name])))

    def poll(self, timeout=0):
        if not self.queue:
            if timeout:
                time.sleep(timeout)
            return False
        name, data = self.queue.popleft()
        data['timestamp'] = self.clock()
        self.feedback[name] = data
        self.rx_seq[name] += 1
        return True

    def special(self, j, code):
        self._send()
        self.special_history.append((j['name'], code))
        state = self.state[j['name']]
        if code == p.DISABLE and j['name'] not in self.ignore_disable:
            state.update(status=0, dq=0, tau=0)
        elif code == p.ENABLE:
            state['status'] = 1
        elif code == p.ZERO:
            if state['status'] != 0:
                raise RuntimeError('cannot zero enabled motor')
            state.update(q=j['zero_joint'], dq=0, tau=0)
        self._reply(j)

    def refresh(self, j):
        self._send()
        self._reply(j)

    def mit(self, j, target):
        p.pack_mit(j, target)  # exercise the real codec/ranges, without any socket
        self._send()
        s = self.state[j['name']]
        if s['status'] == 1:
            tau = target['kp']*(target['q_des']-s['q'])+target['kd']*(target['dq_des']-s['dq'])+target['tau_ff']
            s['tau'] = tau
            s['dq'] += .001*(tau-.1*s['dq'])/.2
            s['q'] += .001*s['dq']
        self._reply(j)

    def register(self, j, op, rid, value=0, interrupt=lambda: None):
        interrupt()
        self._send()
        if j['name'] in self.drop:
            raise TimeoutError('injected register timeout')
        if op == p.WRITE:
            self.registers[j['name']][rid] = value
        return self.registers[j['name']][rid]

    def close(self):
        self.queue.clear()
