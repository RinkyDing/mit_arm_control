"""Algorithm SDK: no direct motor access; local service owns the CAN interface."""
import json
import socket
import time
from .ipc import VERSION, MAX_MESSAGE


# 算法侧接口；一个实例由一个线程顺序调用，内部不直接访问 CAN。
class ArmClient:
    def __init__(self, path='/tmp/mit-arm-control.sock', role='control'):
        self.path, self.role = path, role
        self.socket = None
        # request_id 配对 IPC 请求/响应；sequence 排序运动目标，均不是电机帧序号。
        self.request_id, self.sequence = 0, 0

    def connect(self):
        if self.socket is not None:
            raise RuntimeError('client already connected')
        self.socket = socket.socket(socket.AF_UNIX, socket.SOCK_SEQPACKET)
        self.socket.settimeout(1.)
        try:
            self.socket.connect(self.path)
            result = self._rpc('connect', role=self.role)
            self.sequence = result['last_seq']+1
            return self
        except BaseException:
            self.close()
            raise

    def _rpc(self, op, **kwargs):
        # 同步等待服务回执；服务回执不等于电机已执行目标。
        if self.socket is None:
            raise RuntimeError('connect first')
        self.request_id += 1
        request = dict(version=VERSION, id=self.request_id, op=op, **kwargs)
        data = json.dumps(request, allow_nan=False).encode()
        if len(data) > MAX_MESSAGE:
            raise ValueError('message too large')
        self.socket.sendall(data)
        raw, _, flags, _ = self.socket.recvmsg(MAX_MESSAGE)
        if not raw or flags & socket.MSG_TRUNC:
            raise RuntimeError('IPC disconnected/truncated response')
        reply = json.loads(raw)
        if reply.get('id') != self.request_id or reply.get('version') != VERSION:
            raise RuntimeError('IPC reply mismatch')
        if not reply['ok']:
            raise RuntimeError(reply['error'])
        return reply['result']

    def get_state(self):
        return self._rpc('get_state')

    def _wait(self, target, timeout, ticket=0, reject_fault=True):
        # 既检查目标状态，也检查本次 ticket 已完成，避免读到上一次的结果。
        deadline = time.monotonic()+timeout
        while time.monotonic() < deadline:
            state = self.get_state()
            completed = state.get('completed_ticket', 0) >= ticket
            if completed and state.get('last_request_error'):
                raise RuntimeError(state['last_request_error'])
            if completed and state['state'] == target:
                return state
            if reject_fault and state['state'] == 'FAULT':
                raise RuntimeError(state.get('reason'))
            time.sleep(.01)
        raise TimeoutError(f'waiting for {target}; inspect service state')

    def arm(self, *, timeout=15.):
        # 是否归零由服务 --set-zero 决定；返回 READY 后，首条有效目标才触发使能。
        ack = self._rpc('arm')
        return self._wait('READY', timeout, ack['ticket'])

    def observe(self, *, timeout=15.):
        # 观察先失能并验证静止；服务 --set-zero 决定是否校准零点；观察期间不使能。
        ack = self._rpc('observe')
        return self._wait('OBSERVING', timeout, ack['ticket'])

    def submit(self, joints, *, timestamp=None, sequence=None):
        seq = self.sequence if sequence is None else sequence
        result = self._rpc('submit', command=dict(seq=seq,
                           timestamp=time.monotonic() if timestamp is None else timestamp, joints=joints))
        self.sequence = max(self.sequence, seq+1)
        return result  # 仅确认进入最新目标槽，不等待逐台电机执行。

    def stop(self, timeout=4.):
        # stop 额外等待全组失能确认；超时或未确认会抛异常。
        ack = self._rpc('stop')
        deadline = time.monotonic()+timeout
        while time.monotonic() < deadline:
            state = self.get_state()
            if state.get('completed_ticket', 0) >= ack['ticket'] and state['state'] in ('IDLE', 'FAULT'):
                if not state.get('stop_required', True):
                    return state
                if not state['stop_confirmed']:
                    raise RuntimeError(state['reason'])
                return state
            time.sleep(.01)
        raise TimeoutError('stop confirmation timeout')

    def reset_fault(self, timeout=4.):
        ack = self._rpc('reset_fault')
        return self._wait('IDLE', timeout, ack['ticket'], reject_fault=False)

    def close(self):
        if self.socket is not None:
            self.socket.close()
            self.socket = None

    def __enter__(self):
        return self.connect()

    def __exit__(self, *_):
        self.close()  # 断联也请求停止；需要得到失能结果时应先显式调用 stop。
