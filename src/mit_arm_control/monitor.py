"""只读监视：读取实际量程并查询反馈，不改变模式、零位或使能状态。"""
import math
import os
import signal
import time

from .backend import SocketCAN
from .config import finite
from .protocol import READ
from .service import hardware_locks


def select_joints(config, names):
    if not names or len(names) != len(set(names)):
        raise ValueError('monitor requires unique joint names')
    by_name = {joint['name']: joint for joint in config['joints']}
    joints, ids = [], set()
    for name in names:
        if name not in by_name:
            raise ValueError(f'unknown joint: {name}')
        joint = by_name[name]
        if not isinstance(joint.get('bus'), str) or not joint['bus']:
            raise ValueError(f'{name}: bus required')
        for key, maximum in (('can_id', 15), ('master_id', 0x7FE)):
            value = joint.get(key)
            if type(value) is not int or not 1 <= value <= maximum:
                raise ValueError(f'{name}: invalid {key}')
            identity = (joint['bus'], value)
            if identity in ids:
                raise ValueError(f'{name}: duplicate bus ID')
            ids.add(identity)
        # 尚未标定方向/零位时，明确显示电机原始坐标，不冒充机械臂关节坐标。
        joints.append(dict(joint, direction=1, zero_joint=0.0))
    return joints


def query_loop(bus, joints, rate=10., duration=0., *, clock=time.monotonic,
               cancelled=lambda: False, emit=print):
    def interrupt():
        if cancelled():
            raise InterruptedError('monitor stopped')

    ranges = {}
    for joint in joints:
        values = [bus.register(joint, READ, rid, interrupt=interrupt)
                  for rid in (10, 21, 22, 23)]
        if any(not finite(value) or value <= 0 for value in values[1:]):
            raise ValueError(f"{joint['name']}: invalid PMAX/VMAX/TMAX")
        ranges[joint['name']] = tuple(values[1:])
        emit(f"{joint['name']} CAN=0x{joint['can_id']:02X} "
             f"Master=0x{joint['master_id']:02X} mode={values[0]} "
             f"PMAX={values[1]:g} VMAX={values[2]:g} TMAX={values[3]:g}")
    bus.protocol_ranges = ranges
    bus.feedback.clear()
    baseline = dict(bus.rx_seq)
    sent = {joint['name']: 0 for joint in joints}
    errors = {joint['name']: 0 for joint in joints}
    period = 1. / rate
    started = clock()
    next_query = started
    next_report = started + 1.

    def report():
        now = clock()
        for joint in joints:
            name = joint['name']
            feedback = bus.feedback.get(name)
            prefix = f"{name} queries={sent[name]} RX={bus.rx_seq[name]-baseline[name]} send_errors={errors[name]}"
            if feedback is None:
                emit(f'{prefix} NO_FEEDBACK')
                continue
            age = now - feedback['timestamp']
            freshness = 'STALE' if age > max(.2, 3 * period) else 'RECENT'
            emit(f"{prefix} {freshness} age={age*1000:.1f}ms "
                 f"q={feedback['q']:+.5f}rad dq={feedback['dq']:+.4f}rad/s "
                 f"tau={feedback['tau']:+.4f}Nm status={feedback['status']} "
                 f"MOS={feedback['mos_temperature']}C rotor={feedback['rotor_temperature']}C")

    # 退出只关闭查询；本程序从不接管电机，也不发送失能来改变已有状态。
    while not cancelled() and (not duration or clock() - started < duration):
        now = clock()
        if now >= next_query:
            next_query += (math.floor((now - next_query) / period) + 1) * period
            for joint in joints:
                if cancelled():
                    break
                try:
                    bus.refresh(joint)  # 0x7FF 状态读取，无 MIT 运动帧。
                    sent[joint['name']] += 1
                except OSError as exc:
                    errors[joint['name']] += 1
                    emit(f"{joint['name']} query failed: {exc}")
        bus.poll(min(.005, max(0., next_query - clock())))
        if clock() >= next_report:
            report()
            next_report = clock() + 1.
    report()


def run_monitor(config, names, rate=10., duration=0.):
    if not finite(rate) or not 0 < rate <= 100:
        raise ValueError('query-rate must be in (0, 100] Hz per axis')
    if not finite(duration) or duration < 0:
        raise ValueError('duration must be nonnegative; 0 means until Ctrl+C')
    joints = select_joints(config, names)
    locks, bus, handlers = [], None, {}
    stopping = False

    def request_exit(signum, frame):
        nonlocal stopping
        stopping = True

    try:
        locks = hardware_locks(dict(config, joints=joints), commissioned=False)
        for sig in (signal.SIGINT, signal.SIGTERM):
            handlers[sig] = signal.signal(sig, request_exit)
        bus = SocketCAN(joints)
        print('READ ONLY: motor coordinates; no zero/mode writes/enable/disable/MIT commands.')
        query_loop(bus, joints, rate, duration, cancelled=lambda: stopping)
    except InterruptedError:
        if not stopping:
            raise
    finally:
        if bus is not None:
            bus.close()
        for fd in locks:
            os.close(fd)
        for sig, handler in handlers.items():
            signal.signal(sig, handler)
