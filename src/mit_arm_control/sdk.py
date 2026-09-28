"""Algorithm SDK: no direct motor access; local service owns the CAN interface."""
import json
import socket
import time
from .ipc import VERSION, MAX_MESSAGE


class ArmClient:
    def __init__(self, path='/tmp/mit-arm-control.sock', role='control'):
        self.path, self.role = path, role
        self.socket = None
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

    def arm(self, *, workspace_ready=None, zero_pose, timeout=15., supported=None):
        # Legacy supported=True remains a valid prepared-workspace confirmation.
        if workspace_ready is None:
            workspace_ready = supported
        ack = self._rpc('arm', workspace_ready=workspace_ready,
                        supported=workspace_ready, zero_pose=zero_pose)
        # READY is deliberately disabled. The first valid command enables motors.
        return self._wait('READY', timeout, ack['ticket'])

    def submit(self, joints, *, timestamp=None, sequence=None):
        seq = self.sequence if sequence is None else sequence
        result = self._rpc('submit', command=dict(seq=seq,
                           timestamp=time.monotonic() if timestamp is None else timestamp, joints=joints))
        self.sequence = max(self.sequence, seq+1)
        return result  # queued, not a per-motor execution acknowledgement

    def stop(self, timeout=4.):
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
        self.close()  # owner disconnect also requests stop; explicit stop gives confirmation
