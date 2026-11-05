"""Single-owner backends. Neither constructor enables, zeros or writes motors."""
from collections import deque
import select
import socket
import struct
import time
from . import protocol as p
from .config import MODELS


# 仅订阅状态类错误；不把逐位错误/仲裁丢失流量加入实时收包路径。
CAN_ERROR_MASK = 0x001 | 0x004 | 0x020 | 0x040 | 0x100
CAN_RAW_ERR_FILTER = getattr(socket, 'CAN_RAW_ERR_FILTER', 2)
SO_RXQ_OVFL = getattr(socket, 'SO_RXQ_OVFL', 40)


def transport_counters():
    return dict(error_frames=0, bus_off=0, ack_errors=0, controller_errors=0,
                tx_timeouts=0, restarted=0, rx_queue_dropped=0,
                last_error_classes=None, last_error_timestamp=None)


# 实机适配层：只负责协议收发，生命周期由 Controller 编排。
class SocketCAN:
    def __init__(self, joints, clock=time.monotonic):
        self.joints, self.clock = joints, clock
        self.feedback, self.parameters, self.sockets = {}, {}, {}
        self._round_robin = 0
        self.last_feedback_name = None
        self.feedback_revision = 0
        self._mit_cache = {}
        self.transport = {}
        # arm 成功读取全组量程后统一替换；启动前默认量程仅用于诊断解析。
        self.protocol_ranges = {}
        self.rx_seq = {j['name']: 0 for j in joints}
        try:
            for bus in {j['bus'] for j in joints if j.get('bus')}:
                s = socket.socket(socket.AF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
                self.sockets[bus] = s
                s.setsockopt(socket.SOL_CAN_RAW, socket.CAN_RAW_FD_FRAMES, 1)
                # 只接收本总线配置的标准回复 ID；EFF/RTR 位也参与匹配。
                filters = b''.join(struct.pack('=II', j['master_id'], 0xC00007FF)
                                   for j in joints if j['bus'] == bus)
                s.setsockopt(socket.SOL_CAN_RAW, socket.CAN_RAW_FILTER, filters)
                s.setsockopt(socket.SOL_CAN_RAW, CAN_RAW_ERR_FILTER,
                             struct.pack('=I', CAN_ERROR_MASK))
                s.setsockopt(socket.SOL_SOCKET, SO_RXQ_OVFL, 1)
                self.transport[bus] = transport_counters()
                # 使用内核接收时间戳，避免把队列中积压的旧帧误认为新反馈。
                s.setsockopt(socket.SOL_SOCKET, getattr(socket, 'SO_TIMESTAMPNS', 35), 1)
                s.setblocking(False)
                s.bind((bus,))
            self._socket_list = tuple(self.sockets.values())
            self._bus_by_socket = {s: bus for bus, s in self.sockets.items()}
            self._joint_by_id = {(j['bus'], j['master_id']): j for j in joints}
            self._queries = {j['name']: self.encode(p.register_request(j, p.STATUS)) for j in joints}
        except BaseException:
            self.close()
            raise

    @staticmethod
    def encode(frame):
        # Linux canfd_frame 共 72 字节；flags=1 开启 BRS，MIT 有效载荷仍为 8 字节。
        return struct.pack(
            '=IBBBB64s', frame.can_id, len(frame.data), 1, 0, 0,
            frame.data,
        )

    def send(self, frame):
        raw = self.encode(frame)
        if self.sockets[frame.bus].send(raw) != 72:
            raise OSError('short CAN FD write')

    def wait(self, timeout):
        # 等待接收或下一次调度，不消耗报文；运行时保留 200 μs 等待上限。
        select.select(self._socket_list, [], [], timeout)

    def poll(self, timeout=0):
        # 周期收包直接读非阻塞 socket，省去每帧一次 select；事务等待才 select。
        self.last_feedback_name = None
        sockets = self._socket_list
        if timeout:
            sockets, _, _ = select.select(sockets, [], [], timeout)
        for _ in range(len(sockets)):
            s = sockets[self._round_robin % len(sockets)]
            self._round_robin += 1
            try:
                raw, ancillary, flags, _ = s.recvmsg(72, 128)
                break
            except BlockingIOError:
                continue
        else:
            return False
        if flags & (socket.MSG_TRUNC | socket.MSG_CTRUNC):
            return True
        bus = self._bus_by_socket[s]
        for level, kind, data in ancillary:
            if level == socket.SOL_SOCKET and kind == SO_RXQ_OVFL and len(data) >= 4:
                # Linux 给出此 socket 自创建以来的累计丢包数（uint32）。
                self.transport[bus]['rx_queue_dropped'] = struct.unpack_from('=I', data)[0]
        if len(raw) not in (16, 72):
            return True
        can_id = struct.unpack_from('=I', raw)[0]
        size = raw[4]
        if size > (64 if len(raw) == 72 else 8):
            return True
        if can_id & 0x20000000:
            counters = self.transport[bus]
            counters['error_frames'] += 1
            counters['last_error_classes'] = can_id & 0x1FFFFFFF
            counters['last_error_timestamp'] = self.clock()
            for mask, key in ((1, 'tx_timeouts'), (4, 'controller_errors'),
                              (32, 'ack_errors'), (64, 'bus_off'), (256, 'restarted')):
                counters[key] += bool(can_id & mask)
            return True  # 总线错误永远不能刷新电机反馈有效期。
        if can_id & 0xC0000000:
            return True
        j = self._joint_by_id.get((bus, can_id))
        if j is None:
            return True
        received_at = None
        for level, kind, data in ancillary:
            if level == socket.SOL_SOCKET and kind == getattr(socket, 'SO_TIMESTAMPNS', 35):
                sec, ns = struct.unpack('@ll', data[:struct.calcsize('@ll')])
                # 内核时间为墙上时钟；用样本年龄换算到控制器的单调时钟。
                age = time.time()-(sec+ns/1e9)
                if age >= -.001:
                    received_at = self.clock()-max(0., age)
        if received_at is None:
            return True  # 缺失时间戳的实机帧不能刷新反馈有效期。
        frame = p.Frame(bus, can_id, raw[8:8+size], len(raw) == 72, raw[5])
        decoded = p.decode(j, frame, self.protocol_ranges.get(j['name']))
        if decoded is None:
            return True
        kind, data = decoded
        # 参数事务与运动反馈分流：参数回复不能刷新运动反馈的时间或计数。
        if kind == 'parameter':
            op, rid, value = data
            self.parameters[(j['name'], op, rid)] = (self.clock(), value)
        else:
            data['timestamp'] = received_at
            self.feedback[j['name']] = data
            self.rx_seq[j['name']] += 1
            self.last_feedback_name = j['name']
            self.feedback_revision += 1
        return True

    def special(self, j, code):
        self.send(p.special(j, code))

    def refresh(self, j):
        # 查询帧不随目标/量程变化，避免每周期重新构造。
        if self.sockets[j['bus']].send(self._queries[j['name']]) != 72:
            raise OSError('short CAN FD write')

    def mit(self, j, target):
        # 每轴仅缓存一份已验证帧；值、路由或运行时量程变化都会重新编码。
        ranges = tuple(self.protocol_ranges.get(j['name'], MODELS[j['model']]))
        key = (j['bus'], j['can_id'], ranges, *(target[k] for k in p.FIELDS))
        cached = self._mit_cache.get(j['name'])
        if cached is None or cached[0] != key:
            cached = (key, self.encode(p.pack_mit(j, target, ranges)))
            self._mit_cache[j['name']] = cached
        if self.sockets[j['bus']].send(cached[1]) != 72:
            raise OSError('short CAN FD write')

    def register(self, j, op, rid, value=0, interrupt=lambda: None):
        # 逐台串行参数事务：先排空并清除旧缓存，再等待本轮响应。
        # 协议无事务序号，不据此宣称能严格区分任意迟到的同类型回复。
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


# 仅用于流程和故障注入验证，不是实机动力学模型。
class SimBackend:
    """Functional simulation, NOT a dynamics/safety certification model.

    Fault injection is explicit and only implemented here.
    """
    def __init__(self, joints, clock=time.monotonic):
        self.joints, self.clock = joints, clock
        self.feedback, self.parameters = {}, {}
        self.last_feedback_name = None
        self.feedback_revision = 0
        self.transport = {}
        # arm 成功读取全组量程后统一替换；启动前默认量程仅用于诊断解析。
        self.protocol_ranges = {}
        self.rx_seq = {j['name']: 0 for j in joints}
        self.state = {j['name']: dict(q=0., dq=0., tau=0., status=0,
                      mos_temperature=25, rotor_temperature=25) for j in joints}
        self.registers = {}
        for j in joints:
            pr, vr, tr = MODELS[j['model']]
            self.registers[j['name']] = {10: 1, 21: pr, 22: vr, 23: tr}
        self.queue = deque(maxlen=256)
        # 测试可注入持续/单次漏收、发送失败和忽略失能，不影响实机后端。
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
        self.last_feedback_name = None
        if not self.queue:
            if timeout:
                time.sleep(timeout)
            return False
        name, data = self.queue.popleft()
        data['timestamp'] = self.clock()
        self.feedback[name] = data
        self.rx_seq[name] += 1
        self.last_feedback_name = name
        self.feedback_revision += 1
        return True

    def wait(self, timeout):
        if not self.queue:
            time.sleep(timeout)

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
            state.update(q=0., dq=0, tau=0)
        self._reply(j)

    def refresh(self, j):
        self._send()
        self._reply(j)

    def mit(self, j, target):
        p.pack_mit(j, target, self.protocol_ranges.get(j['name']))  # 复用真实编码和量程校验，但不访问 CAN。
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
