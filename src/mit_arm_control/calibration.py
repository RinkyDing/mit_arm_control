"""监视前的失能归零；不调用控制器 arm，也不发送使能或 MIT 目标。"""
import time
from .protocol import DISABLE, ZERO
from .config import finite


def zero_disabled(bus, joints, *, clock=time.monotonic,
                  cancelled=lambda: False, emit=print, timeout=2.):
    def interrupt():
        if cancelled():
            raise InterruptedError('zero calibration interrupted; do not assume all axes were zeroed')

    def pause(seconds):
        deadline = clock() + seconds
        while clock() < deadline:
            interrupt()
            bus.poll(min(.002, max(0., deadline - clock())))

    def fresh(*, disable=False, zero=False):
        # 先清旧反馈，再要求接收计数和内核接收时间均晚于本轮请求。
        for _ in range(256):
            if not bus.poll(0):
                break
        else:
            raise RuntimeError('zeroing: receive queue did not drain')
        before = dict(bus.rx_seq)
        pending = {joint['name']: joint for joint in joints}
        requested = {}
        deadline, next_query = clock() + timeout, 0.
        last_error = None
        while pending and clock() < deadline:
            interrupt()
            if clock() >= next_query:
                for name, joint in pending.items():
                    try:
                        stamp = clock()
                        if disable:
                            bus.special(joint, DISABLE)
                        bus.refresh(joint)
                        requested.setdefault(name, stamp)
                    except OSError as exc:
                        last_error = str(exc)
                next_query = clock() + .02
            bus.poll(min(.002, max(0., deadline - clock())))
            for name in list(pending):
                feedback = bus.feedback.get(name)
                if (name not in requested or not feedback
                        or bus.rx_seq[name] <= before[name]
                        or feedback['timestamp'] < requested[name]):
                    continue
                if feedback['status'] != 0:
                    # 失能事务允许暂态并继续重试，后续校准阶段则直接中止。
                    if disable:
                        continue
                    raise RuntimeError(f'{name}: zeroing requires disabled status=0')
                if not finite(feedback['q']) or not finite(feedback['dq']):
                    raise RuntimeError(f'{name}: non-finite calibration feedback')
                if abs(feedback['dq']) > .1:
                    raise RuntimeError(f'{name}: moving during calibration; hold the fixed pose still')
                if zero and abs(feedback['q']) > .02:
                    raise RuntimeError(f'{name}: zero verification failed')
                del pending[name]
        if pending:
            raise RuntimeError(f'zeroing feedback timeout: {list(pending)}; last I/O error={last_error}')

    emit('ZEROING: disabling selected motors; keep the fixed zero pose still.')
    fresh(disable=True)
    # 所有轴都确认失能静止之后，才向任意轴发送设零命令。
    for _ in range(3):
        fresh()
        pause(.05)
    for joint in joints:
        interrupt()
        bus.special(joint, ZERO)
        pause(.02)
    pause(.2)
    for _ in range(3):
        fresh(zero=True)
        pause(.05)
    emit('ZERO VERIFIED: all selected motors remain disabled. You may now move the arm by hand.')
