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
from .config import readiness, select_joints, finite
from .controller import Controller
from .ipc import Mailbox, IPCServer, AsyncLogger



# 同一项目内按总线互斥；这里只核对接口参数，不修改系统 CAN 配置。
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


def run(config, socket_path, hardware=False, result_path=None, *,
        observation_only=False, joints=None, query_rate=None, set_zero=False):
    # 默认模拟后端；实机需显式开启并完成配置，构造后端本身不会使能。
    if observation_only:
        rate = config['rate_hz'] if query_rate is None else query_rate
        if not finite(rate) or not 0 < rate <= 1000:
            raise ValueError('observation query rate must be in (0, 1000] Hz per axis')
        config = dict(config, joints=select_joints(config, joints or ['J1','J2','J3','J4','J5','J6']),
                      rate_hz=rate)
    errors = [] if observation_only else readiness(config, hardware)
    if errors:
        raise ValueError('; '.join(errors))
    locks, backend, ipc = [], None, None
    mailbox, logger = Mailbox(), AsyncLogger()
    feedback_revision = -1
    exiting = False
    controller = None
    old_signals = {}

    def request_exit(signum, frame):
        nonlocal exiting
        # 信号处理只请求退出；实际 CAN 停止事务由控制循环执行。
        exiting = True
        mailbox.request_stop(f'service signal {signum}')

    def publish(statistics=False):
        nonlocal feedback_revision
        if controller is not None:
            # 状态/目标更新立即可见；完整直方图仅定时或生命周期完成后计算。
            snapshot = controller.snapshot(statistics=statistics)
            mailbox.snapshot = dict(mailbox.snapshot, **snapshot,
                log_dropped=logger.dropped, log_failures=logger.failures,
                feedback_snapshot_timestamp=snapshot['timestamp'])
            feedback_revision = backend.feedback_revision

    try:
        if hardware:
            locks = hardware_locks(config, commissioned=not observation_only)
        backend = SocketCAN(config['joints']) if hardware else SimBackend(config['joints'])
        controller = Controller(config, backend, hardware, cancelled=mailbox.stop.is_set,
                                publish=publish, observation_only=observation_only, set_zero=set_zero)
        publish(statistics=True)
        ipc = IPCServer(socket_path, mailbox, config)
        for sig in (signal.SIGINT, signal.SIGTERM):
            old_signals[sig] = signal.signal(sig, request_exit)
        next_snapshot = next_log = time.monotonic()
        while not exiting:
            # 优先级：停止请求 → 生命周期动作/最新目标 → 定时通信 → 统计。
            if mailbox.stop.is_set():
                reason, fault = mailbox.stop_reason, mailbox.fault
                mailbox.take_command()
                while True:
                    try:
                        mailbox.actions.get_nowait()
                    except queue.Empty:
                        break
                # 再次停止或断联不能清除已锁存的故障。
                controller.stop(reason, fault=fault or controller.state == 'FAULT')
                mailbox.completed_ticket = max(mailbox.completed_ticket, mailbox.stop_ticket)
                mailbox.stop_ticket = 0
                mailbox.stop.clear()
                mailbox.fault = False
                publish(statistics=True)
            else:
                try:
                    op, args, ticket = mailbox.actions.get_nowait()
                except queue.Empty:
                    op = None
                if op:
                    try:
                        mailbox.take_command()
                        if op == 'arm':
                            controller.arm()
                        elif op == 'observe':
                            controller.observe()
                        else:
                            controller.reset_fault()
                    except Exception as exc:
                        mailbox.last_error = str(exc)
                    mailbox.completed_ticket = ticket
                    publish(statistics=True)
                command = mailbox.take_command()
                if command is not None and not mailbox.stop.is_set():
                    try:
                        controller.submit(command)
                    except Exception as exc:
                        mailbox.last_error = str(exc)
                    publish()
                controller.step()
                # 无新反馈时不复制整组字典；帧时间戳和 get_state 的年龄仍独立更新。
                if backend.feedback_revision != feedback_revision:
                    mailbox.snapshot = dict(mailbox.snapshot,
                        feedback={n: dict(f) for n, f in backend.feedback.items()},
                        feedback_snapshot_timestamp=time.monotonic())
                    feedback_revision = backend.feedback_revision
            now = time.monotonic()
            if now >= next_snapshot:
                publish(statistics=True)
                next_snapshot = time.monotonic()+.02
            if now >= next_log:
                logger.offer(mailbox.snapshot)
                next_log = now+1.
            # 发布开销也占用调度时间；用当前时间重新算剩余量，避免多睡一段。
            remaining = controller.next_tick-time.monotonic() if controller.state in controller.ACTIVE else .001
            if remaining > 0 and not mailbox.stop.is_set():
                backend.wait(min(.0002, remaining))
    finally:
        # 先停止电机，再关闭 IPC/日志并写磁盘。
        # 可捕获异常也会到这里；SIGKILL 或掉电无法执行 finally。
        if controller is not None:
            if controller.state not in ('IDLE',) or controller.stop_confirmed is False:
                controller.stop('service exit', fault=controller.state == 'FAULT')
            publish(statistics=True)
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
            # 最终停止结果单独保存，不依赖可能丢弃的周期日志。
            path = Path(result_path)
            temporary = path.with_name(path.name+'.tmp')
            temporary.write_text(json.dumps(mailbox.snapshot, ensure_ascii=False, indent=2, allow_nan=False))
            temporary.replace(path)
    return mailbox.snapshot
