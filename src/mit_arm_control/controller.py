"""Transport-independent lifecycle and single-owner scheduled control loop."""
from dataclasses import dataclass, field
import math
import time
from . import protocol as p
from .config import readiness
from .safety import SafetyError, validate_command, check_feedback, check_effort


# 每轴统计；计数差只用于观察，不用于判断是否可以继续发送。
@dataclass
class Metrics:
    tx: int = 0
    rx: int = 0
    # 连续发送但尚未观测到新反馈的次数；收到反馈即清零。
    unobserved: int = 0
    backpressure: int = 0
    recovery_tx: int = 0
    max_tx_gap: float = 0.
    max_rx_gap: float = 0.
    last_tx: float = None
    last_rx: float = None
    max_lateness: float = 0.
    # 固定内存直方图：每桶 10 μs，覆盖至 20 ms，另设溢出桶。
    histogram: list = field(default_factory=lambda: [0]*2002)

    def sent(self, now, deadline, recovery):
        if self.last_tx is not None:
            self.max_tx_gap = max(self.max_tx_gap, now-self.last_tx)
        self.last_tx = now
        self.tx += 1
        self.unobserved += 1
        self.recovery_tx += bool(recovery)
        delay = max(0., now-deadline)
        self.max_lateness = max(self.max_lateness, delay)
        self.histogram[min(int(delay/.00001), 2001)] += 1

    def received(self, now):
        if self.last_rx is not None:
            self.max_rx_gap = max(self.max_rx_gap, now-self.last_rx)
        self.last_rx = now
        self.rx += 1
        self.unobserved = 0

    def percentile(self, fraction):
        threshold, count = math.ceil(self.tx*fraction), 0
        if not threshold:
            return 0.
        for i, n in enumerate(self.histogram):
            count += n
            if count >= threshold:
                return (i+1)*.00001 if i < 2001 else self.max_lateness
        return self.max_lateness

    def snapshot(self, elapsed):
        return dict(
            tx=self.tx,
            rx=self.rx,
            tx_hz=self.tx / max(elapsed, 1e-9),
            rx_hz=self.rx / max(elapsed, 1e-9),
            unobserved_tx=self.unobserved,
            count_gap=self.tx - self.rx,
            backpressure=self.backpressure,
            recovery_tx=self.recovery_tx,
            max_tx_gap_ms=self.max_tx_gap * 1000,
            max_rx_gap_ms=self.max_rx_gap * 1000,
            lateness_p95_ms=self.percentile(.95) * 1000,
            lateness_p99_ms=self.percentile(.99) * 1000,
            lateness_max_ms=self.max_lateness * 1000,
        )


# 只有控制服务主循环调用此对象；IPC 线程不直接读写电机。
class Controller:
    ACTIVE = ('RUNNING', 'DEGRADED')

    def __init__(self, config, backend, hardware=False, clock=time.monotonic,
                 cancelled=lambda: False, publish=lambda: None):
        self.config, self.backend, self.hardware, self.clock = config, backend, hardware, clock
        self.cancelled, self.publish = cancelled, publish
        self.state, self.reason, self.stop_confirmed = 'IDLE', None, None
        self.command, self.previous_command = None, None
        self.last_seq = -1
        self.metrics = {j['name']: Metrics() for j in config['joints']}
        self.seen = dict(backend.rx_seq)
        self.started = self.finished = None
        self.next_tick = 0.
        self.skipped = 0
        self.events = []
        self.mode_verified = {}
        # 表示已进入过需停止确认的启动流程；不是机械支撑或安全状态证明。
        self.workspace_confirmed = False
        self.next_diagnostic = 0.
        self.diagnostic_error = None

    def transition(self, state, reason=None):
        self.state = state
        if reason is not None:
            reason = str(reason)[:512]
            self.reason = reason
        self.events.append(dict(timestamp=self.clock(), state=state, reason=reason))
        self.events = self.events[-32:]
        try:
            self.publish()
        except Exception:
            pass  # 遥测发布失败不能阻止电机停止。

    def interrupt(self):
        if self.cancelled():
            raise SafetyError('operator stop or algorithm disconnected during startup')

    def poll(self, timeout=0):
        result = self.backend.poll(timeout)
        for name, count in self.backend.rx_seq.items():
            if count != self.seen[name]:
                self.seen[name] = count
                if self.state in self.ACTIVE:
                    self.metrics[name].received(self.backend.feedback[name]['timestamp'])
        return result

    def drain(self):
        for _ in range(256):
            if not self.poll(0):
                return
        raise SafetyError('receive queue did not drain')

    def fresh(self, status, zero=False, timeout=2.0):
        # 排空旧数据，只接受本轮请求后的反馈，并核对状态、静止及可选零位。
        self.drain()
        before = dict(self.backend.rx_seq)
        requested_after = self.clock()
        deadline, next_request = self.clock()+timeout, 0.
        while self.clock() < deadline:
            self.interrupt()
            pending = [j for j in self.config['joints']
                       if self.backend.rx_seq[j['name']] <= before[j['name']]
                       or self.backend.feedback.get(j['name'], {}).get('timestamp', 0) < requested_after]
            if not pending:
                for j in self.config['joints']:
                    f = self.backend.feedback[j['name']]
                    if f['status'] != status or abs(f['dq']) > .1:
                        raise SafetyError(f"{j['name']}: expected stationary status={status}")
                    if zero and abs(f['q']-j['zero_joint']) > .02:
                        raise SafetyError(f"{j['name']}: zero mismatch")
                return
            if self.clock() >= next_request:
                for j in pending:
                    self.backend.refresh(j)
                next_request = self.clock()+.02
            self.poll(.001)
        raise SafetyError('fresh startup feedback timeout')

    def pause(self, seconds):
        end = self.clock()+seconds
        while self.clock() < end:
            self.interrupt()
            self.poll(min(.002, max(0, end-self.clock())))

    def disable(self):
        # 本次失能必须有新反馈确认，缓存中的 status=0 不算成功。
        try:
            self.drain()
        except (OSError, SafetyError):
            pass
        before = dict(self.backend.rx_seq)
        sent_after = {}
        pending = {j['name']: j for j in self.config['joints']}
        sent = set()
        deadline, next_try = self.clock()+self.config['stop_timeout'], 0.
        last_error = None
        while pending and self.clock() < deadline:
            if self.clock() >= next_try:
                for name, j in list(pending.items()):
                    try:
                        stamp = self.clock()
                        self.backend.special(j, p.DISABLE)
                        sent_after.setdefault(name, stamp)
                        sent.add(name)
                        self.backend.refresh(j)
                    except OSError as exc:
                        last_error = str(exc)
                next_try = self.clock()+.02
            try:
                self.poll(min(.002, max(0, deadline-self.clock())))
            except OSError as exc:
                last_error = str(exc)
                time.sleep(.001)
            # 同时要求：已发送失能、接收计数增加、时间戳足够新、状态为失能。
            for name in list(pending):
                f = self.backend.feedback.get(name)
                if (name in sent and self.backend.rx_seq[name] > before[name] and f
                        and f['timestamp'] >= sent_after[name] and f['status'] == 0):
                    del pending[name]
        if pending:
            raise SafetyError(f"STOP UNCONFIRMED: {list(pending)}; last I/O error={last_error}")

    def arm(self, workspace_ready=False, zero_pose=False):
        if self.state != 'IDLE':
            raise SafetyError('arm requires IDLE; faults require explicit reset')
        errors = readiness(self.config, self.hardware)
        if errors or workspace_ready is not True or zero_pose is not True:
            raise SafetyError('; '.join(errors) or 'workspace and fixed zero-pose confirmations required')
        self.reason, self.stop_confirmed = None, None
        self.command = self.previous_command = None
        self.metrics = {j['name']: Metrics() for j in self.config['joints']}
        self.started = self.finished = None
        self.skipped = 0
        self.workspace_confirmed = True
        self.transition('ARMING')
        try:
            # 1. 逐轴核对 PMAX / VMAX / TMAX，避免量程不一致造成错误指令。
            for j in self.config['joints']:
                for rid, expected in zip((21, 22, 23), p.MODELS[j['model']]):
                    actual = self.backend.register(j, p.READ, rid, interrupt=self.interrupt)
                    if not math.isclose(actual, expected, rel_tol=1e-5, abs_tol=1e-5):
                        raise SafetyError(f"{j['name']}: register {rid} range mismatch")
            # 2. 全组失能，并连续验证静止后才能设置零点。
            self.disable()
            for _ in range(3):
                self.fresh(0)
                self.pause(.05)
            # 3. 每轴只发送一次设零，再等待并多次验证反馈零位。
            for j in self.config['joints']:
                self.interrupt()
                self.backend.special(j, p.ZERO)
                self.pause(.02)
            self.pause(.2)
            for _ in range(3):
                self.fresh(0, zero=True)
                self.pause(.05)
            # 4. 写入并读回 MIT 模式；READY 阶段仍保持失能。
            for j in self.config['joints']:
                self.backend.register(j, p.WRITE, 10, 1, interrupt=self.interrupt)
                actual = self.backend.register(j, p.READ, 10, interrupt=self.interrupt)
                if actual != 1:
                    raise SafetyError(f"{j['name']}: MIT mode mismatch")
                self.mode_verified[j['name']] = 1
            self.transition('READY')  # 首条有效目标到来后才使能。
        except Exception as exc:
            self.stop(str(exc), fault=True)
            raise

    def submit(self, command):
        if self.state not in ('READY', *self.ACTIVE):
            raise SafetyError('submit requires READY/RUNNING/DEGRADED')
        try:
            now = self.clock()
            if command.get('seq', -1) <= self.last_seq:
                raise SafetyError('sequence not newer than previous command')
            validate_command(self.config, command, now, self.previous_command)
            if self.state == 'READY':
                self.fresh(0, zero=True, timeout=self.config['feedback_timeout'])
                check_feedback(self.config, self.backend.feedback, self.clock(), enabled=False)
                # 首条目标须贴合当前静止零位；进入 RUNNING 后再逐步改变目标。
                for j in self.config['joints']:
                    t, f = command['joints'][j['name']], self.backend.feedback[j['name']]
                    if abs(t['q_des']-f['q']) > .02 or abs(t['dq_des']) > .1 or abs(t['tau_ff']) > .1:
                        raise SafetyError(f"{j['name']}: initial target must match stationary zero pose")
                check_effort(self.config, command, self.backend.feedback)
                for j in self.config['joints']:
                    self.interrupt()
                    validate_command(self.config, command, self.clock())
                    # 失能时覆盖旧目标；使能后立即重发同一个已验证目标。
                    self.backend.mit(j, command['joints'][j['name']])
                    self.backend.special(j, p.ENABLE)
                    self.backend.mit(j, command['joints'][j['name']])
                    self.poll(0)
                self.fresh(1, zero=True, timeout=self.config['feedback_timeout'])
                validate_command(self.config, command, self.clock())
                self.started = self.clock()
                self.next_tick = self.started
                self.seen = dict(self.backend.rx_seq)
                self.transition('RUNNING')
            self.last_seq = command['seq']
            self.command = self.previous_command = command
        except Exception as exc:
            self.stop(str(exc), fault=True)
            raise

    def step(self):
        if self.state not in self.ACTIVE:
            # 故障保持锁存；低频查询只更新诊断信息，不自动恢复使能。
            try:
                if self.state == 'FAULT' and self.workspace_confirmed and self.clock() >= self.next_diagnostic:
                    self.next_diagnostic = self.clock()+1.
                    for joint in self.config['joints']:
                        self.backend.refresh(joint)
                for _ in range(16):
                    if not self.poll(0):
                        break
                self.diagnostic_error = None
            except OSError as exc:
                self.diagnostic_error = str(exc)
            return
        try:
            self.interrupt()
            now = self.clock()
            if not self.command or now-self.command['timestamp'] >= self.config['command_timeout']:
                raise SafetyError('algorithm command expired')
            check_feedback(self.config, self.backend.feedback, now)
            check_effort(self.config, self.command, self.backend.feedback)
            # 按绝对时刻调度：跳过错过的周期，不集中补发历史目标。
            if now >= self.next_tick:
                period = 1/self.config['rate_hz']
                missed = int((now-self.next_tick+1e-12)/period)
                self.skipped += missed
                deadline = self.next_tick+missed*period
                self.next_tick = deadline+period
                # 连续无反馈达到阈值后，全组降频探测；不是按累计 TX−RX 限流。
                blocked = [
                    m for m in self.metrics.values()
                    if m.unobserved >= self.config['max_unobserved']
                ]
                retry = min(.02, self.config['feedback_timeout']/4)
                if blocked and any(now-m.last_tx < retry for m in blocked):
                    for m in self.metrics.values():
                        m.backpressure += 1
                    if self.state != 'DEGRADED':
                        self.transition('DEGRADED', 'feedback flow control')
                else:
                    for j in self.config['joints']:
                        self.interrupt()
                        if self.clock()-self.command['timestamp'] >= self.config['command_timeout']:
                            raise SafetyError('command expired during send group')
                        self.backend.mit(j, self.command['joints'][j['name']])
                        self.metrics[j['name']].sent(self.clock(), deadline, bool(blocked))
            # 接收最多 32 帧，并受时间预算约束；即使迟到也至少尝试接收一次。
            budget = min(self.next_tick, self.clock()+.0003)
            for index in range(32):
                if (index and self.clock() >= budget) or not self.poll(0):
                    break
            if self.state == 'DEGRADED' and all(
                m.unobserved < self.config['max_unobserved']
                for m in self.metrics.values()
            ):
                self.transition('RUNNING')
            check_feedback(self.config, self.backend.feedback, self.clock())
        except Exception as exc:
            self.stop(str(exc), fault=True)

    def stop(self, reason='operator stop', fault=False):
        if self.state == 'FAULT' and self.reason:
            reason = self.reason if reason in self.reason else self.reason+'; '+reason
            fault = True
        # 先丢弃运动目标；停止失败也不能继续重发旧运动命令。
        self.command = None
        if not self.workspace_confirmed:
            self.stop_confirmed = None
            self.transition('FAULT' if fault else 'IDLE', reason)
            return
        if self.started is not None and self.finished is None:
            self.finished = self.clock()
        self.transition('STOPPING', reason)
        try:
            self.disable()
            self.stop_confirmed = True
            self.transition('FAULT' if fault else 'IDLE', reason)
        except Exception as exc:
            self.stop_confirmed = False
            self.transition('FAULT', f'{reason}; {exc}')

    def reset_fault(self):
        if self.state != 'FAULT':
            raise SafetyError('reset requires FAULT')
        # 显式复位也重新确认失能；返回 IDLE 不代表自动恢复运动。
        if self.workspace_confirmed:
            self.disable()
            self.stop_confirmed = True
        self.reason, self.command, self.previous_command = None, None, None
        self.transition('IDLE')

    def snapshot(self):
        now = self.clock()
        elapsed = max(0., (self.finished if self.finished is not None else now)-(self.started or now))
        feedback = {name: dict(f, age_ms=(now-f['timestamp'])*1000) for name, f in self.backend.feedback.items()}
        return dict(
            version=1,
            state=self.state,
            reason=self.reason,
            stop_confirmed=self.stop_confirmed,
            supported_only=False,
            stop_required=self.workspace_confirmed,
            backend='socketcan' if self.hardware else 'simulation',
            timestamp=now,
            elapsed=elapsed,
            skipped=self.skipped,
            last_seq=self.last_seq,
            diagnostic_error=self.diagnostic_error,
            mode_verified=dict(self.mode_verified),
            feedback=feedback,
            metrics={n: m.snapshot(elapsed) for n, m in self.metrics.items()},
            events=list(self.events),
            configuration_errors=readiness(self.config, self.hardware),
        )
