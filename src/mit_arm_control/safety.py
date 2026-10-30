"""Validation is separate from transport; rejected commands are never clipped."""
from .config import finite, MODELS
from .protocol import FIELDS, pack_mit


class SafetyError(ValueError):
    pass


def validate_command(c, cmd, now, previous=None, *, ranges=None):
    # c=配置、cmd=整组目标；先核对信封/时效/顺序，再逐轴检查协议可编码范围。
    if not isinstance(cmd, dict) or set(cmd) != {'seq', 'timestamp', 'joints'}:
        raise SafetyError('command requires seq, timestamp, joints')
    seq, stamp = cmd['seq'], cmd['timestamp']
    if type(seq) is not int or seq < 0:
        raise SafetyError('invalid sequence')
    if not finite(stamp) or stamp > now+.001 or now-stamp >= c['command_timeout']:
        raise SafetyError('command expired or timestamp in future')
    if previous and (seq <= previous['seq'] or stamp <= previous['timestamp']):
        raise SafetyError('out-of-order command')
    if not isinstance(cmd['joints'], dict) or set(cmd['joints']) != {j['name'] for j in c['joints']}:
        raise SafetyError('command must include exactly the active joints')
    for j in c['joints']:
        name = j['name']
        t = cmd['joints'][name]
        if not isinstance(t, dict) or set(t) != set(FIELDS) or not all(finite(v) for v in t.values()):
            raise SafetyError(f'{name}: invalid MIT fields')
        actual = MODELS[j['model']] if ranges is None else ranges.get(name)
        if actual is None:
            raise SafetyError(f'{name}: missing runtime protocol range')
        # 复用实际编码规则，对全组预检，避免后面的轴越界时前面的轴已经使能。
        try:
            pack_mit(j, t, actual)
        except ValueError as exc:
            raise SafetyError(f'{name}: {exc}') from exc


def check_feedback(c, feedback, now, enabled=True):
    # 只使用接收时间判断新鲜度，发送成功不能延长反馈有效期。
    for j in c['joints']:
        name = j['name']
        f = feedback.get(name)
        if not f or now-f['timestamp'] >= c['feedback_timeout']:
            raise SafetyError(f'{name}: feedback timeout')
        if not all(finite(f[k]) for k in ('q', 'dq', 'tau', 'mos_temperature', 'rotor_temperature')):
            raise SafetyError(f'{name}: non-finite feedback')
        if f['status'] != (1 if enabled else 0):
            raise SafetyError(f"{name}: status {f['status']}")
