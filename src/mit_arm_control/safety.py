"""Validation is separate from transport; rejected commands are never clipped."""
import math
from .config import NAMES, finite
from .protocol import FIELDS


class SafetyError(ValueError):
    pass


def validate_command(c, cmd, now, previous=None):
    # c=配置、cmd=整组目标；先核对信封/时效/顺序，再逐轴检查数值与变化率。
    if not isinstance(cmd, dict) or set(cmd) != {'seq', 'timestamp', 'joints'}:
        raise SafetyError('command requires seq, timestamp, joints')
    seq, stamp = cmd['seq'], cmd['timestamp']
    if type(seq) is not int or seq < 0:
        raise SafetyError('invalid sequence')
    if not finite(stamp) or stamp > now+.001 or now-stamp >= c['command_timeout']:
        raise SafetyError('command expired or timestamp in future')
    if previous and (seq <= previous['seq'] or stamp <= previous['timestamp']):
        raise SafetyError('out-of-order command')
    if not isinstance(cmd['joints'], dict) or set(cmd['joints']) != set(NAMES):
        raise SafetyError('command must include all seven axes')
    for j in c['joints']:
        name, lim = j['name'], j['limits']
        t = cmd['joints'][name]
        if not isinstance(t, dict) or set(t) != set(FIELDS) or not all(finite(v) for v in t.values()):
            raise SafetyError(f'{name}: invalid MIT fields')
        if not (lim['q_min'] <= t['q_des'] <= lim['q_max'] and abs(t['dq_des']) <= lim['dq_max']
                and abs(t['tau_ff']) <= lim['tau_max'] and 0 <= t['kp'] <= lim['kp_max']
                and 0 <= t['kd'] <= lim['kd_max']):
            raise SafetyError(f'{name}: command limit violation')
        if previous:
            # 时间间隔封顶为命令有效期，防止停更后获得无限大的变化额度。
            dt = min(stamp-previous['timestamp'], c['command_timeout'])
            for field, rate in zip(FIELDS, ('q_rate', 'dq_rate', 'kp_rate', 'kd_rate', 'tau_rate')):
                if abs(t[field]-previous['joints'][name][field]) > lim[rate]*dt+1e-9:
                    raise SafetyError(f'{name}: {field} slew limit')


def check_feedback(c, feedback, now, enabled=True):
    # 只使用接收时间判断新鲜度，发送成功不能延长反馈有效期。
    for j in c['joints']:
        name, lim = j['name'], j['limits']
        f = feedback.get(name)
        if not f or now-f['timestamp'] >= c['feedback_timeout']:
            raise SafetyError(f'{name}: feedback timeout')
        if not all(finite(f[k]) for k in ('q', 'dq', 'tau', 'mos_temperature', 'rotor_temperature')):
            raise SafetyError(f'{name}: non-finite feedback')
        if f['status'] != (1 if enabled else 0):
            raise SafetyError(f"{name}: status {f['status']}")
        if (not lim['q_min'] <= f['q'] <= lim['q_max'] or abs(f['dq']) > lim['dq_max']
                or abs(f['tau']) > lim['tau_max']
                or max(f['mos_temperature'], f['rotor_temperature']) >= lim['temperature_max']):
            raise SafetyError(f'{name}: feedback safety limit')


def check_effort(c, cmd, feedback):
    # MIT 力矩估算只作为主机侧检查，不是驱动器瞬时力矩硬限制。
    for j in c['joints']:
        name = j['name']
        t, f = cmd['joints'][name], feedback[name]
        effort = (
            t['kp'] * (t['q_des'] - f['q'])
            + t['kd'] * (t['dq_des'] - f['dq'])
            + t['tau_ff']
        )
        if not math.isfinite(effort) or abs(effort) > j['limits']['tau_max']:
            raise SafetyError(f'{name}: estimated MIT torque limit')
