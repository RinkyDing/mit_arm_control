# 算法接口与状态机

## 控制边界

算法只使用 `ArmClient`，不能持有或操作 CAN socket。控制服务独立存活，单一控制客户端拥有命令权限，可同时连接只读 observer。Unix socket 权限 0600，供同一 Linux 用户的可信本机程序使用，不是网络 API，也不是对同用户恶意进程的安全隔离。

SDK 实例不是线程安全的；由算法一个线程依次调用。协议版本为 1，最大消息 32768 字节，使用 SOCK_SEQPACKET 保持消息边界。算法发送整组七轴命令，新目标覆盖旧目标，不补发历史轨迹。`submit` 返回排入最新目标槽的确认，不是电机逐帧执行确认。驱动没有序号，不能证明每条命令与每条反馈严格配对。

## 公共接口

```python
from mit_arm_control import ArmClient

client = ArmClient('/tmp/mit-arm-control.sock').connect()
state = client.get_state()
# 以下确认必须来自现场操作者，不能由算法在实机上无条件写 True。
client.arm(workspace_ready=True, zero_pose=True)
# arm 返回 READY，此时电机仍失能，等待初始有效目标。
client.submit(all_seven_joint_targets)
state = client.get_state()
# 算法持续更新目标；如要回桌，由算法完成轨迹后再调用 stop。
result = client.stop()
client.close()
```

| 方法 | 语义 |
| --- | --- |
| connect() | 连接并获得控制权，不触发电机动作；第二个控制客户端被拒绝 |
| get_state() | 最新反馈快照、反馈年龄、状态、故障、配置错误和统计；未观测轴不会伪造为零 |
| arm(workspace_ready, zero_pose) | 在完整配置及双重确认后读取并采用实际协议量程、校验机械限制兼容性、失能静止、设零验证、逐轴模式确认；返回 READY |
| submit(joints, timestamp=None, sequence=None) | 完整七轴 MIT 目标；SDK 默认生成主机 monotonic 时间和递增序号 |
| stop() | 请求全体失能并等待本次请求结果；未确认抛异常；未曾 arm 的会话不写电机、不声称失能已确认 |
| reset_fault() | 要求 FAULT；曾启动时重新取得失能反馈，返回 IDLE，不自动恢复运动 |
| close() | 关闭连接；控制客户端掉线时服务请求故障停止；应先显式 stop 获取结果 |

只读观察者：`ArmClient(path, role='observer')`，仅支持读取状态和关闭连接。

## 命令与单位

`joints` 必须恰好包含 `J1`..`J6` 和 `gripper`：

```python
{
    'J1': {'q_des': 0.0, 'dq_des': 0.0, 'kp': 2.0, 'kd': 0.1, 'tau_ff': 0.0},
    # J2 ... J6, gripper 同样的五个字段，不能省略。
}
```

上述数值只是模拟示例，不是实机推荐增益。

- q_des：关节输出侧 rad；dq_des：rad/s；tau_ff：N·m。
- kp：N·m/rad；kd：N·m/(rad/s)。kp、kd 必须非负。
- 夹爪首版按驱动轴角度表示，不自动转换为毫米或夹持力。
- 输出侧必须由实际安装与驱动固件量纲核对；本版没有额外减速比换算。存在外部传动时先扩展并验证适配层。
- `q_joint = direction * q_motor + zero_joint`，速度和力矩使用同一个 ±1 方向变换。
- 序号在服务当前命令历史内严格递增；时间戳来自同机 `time.monotonic()`，容许不超过 1 ms 的未来偏差，不支持远程时钟。
- 默认命令 TTL 50 ms，算法建议先按 200 Hz 模拟联调；首版不插值，控制服务保持最新未过期目标。
- 首条目标必须与归零后实测位置相差不超过 0.02 rad，速度与前馈力矩绝对值不超过 0.1，并满足所有配置限制。先提交静止初始目标，再逐步变化。

参数变化率按连续命令时间戳检查，长时间中断不能获得无限大的变化额度。缺轴、额外字段、乱序、过期、NaN/Inf、限幅或变化率违规均拒绝；运行中违规会锁存故障并全体停止，不静默裁剪。

`tau_est = kp*(q_des-q) + kd*(dq_des-dq) + tau_ff` 在每轮发送前受配置约束；这是基于最新反馈的主机估计，不是电机瞬时输出的硬件限幅。

## 状态机

```text
IDLE → ARMING → READY → RUNNING ⇄ DEGRADED
                    初始有效目标       ↓
任一启动/运行失败 ───────────────→ STOPPING → FAULT
正常 stop ─────────────────────→ STOPPING → IDLE
FAULT → 显式 reset_fault + 失能确认 → IDLE
```

没有自动恢复使能。READY 可以等待算法，电机保持失能；50 ms 命令超时从有效运动命令控制阶段开始。未启用过的会话掉线不操作电机。进入 FAULT 后恢复连接仍需 reset_fault、重新确认缓冲工作区域与固定零姿态，再 arm。

服务 SIGINT/SIGTERM 会先停止电机再关闭日志。SIGKILL、内核崩溃、断电无法执行 Python 清理，此时本版不能保证失能命令送达；不配置设备侧超时保护是当前实验边界。

## 反馈与统计

反馈包括 q、dq、估计 tau、MOS/转子温度、状态码、接收 monotonic 时间和年龄。SocketCAN 使用 Linux 接收时间戳换算样本年龄，避免旧队列数据被误认为刚收到；这不是电机内部采样时间。参数回复不更新反馈年龄。系统实时时钟跳变可能触发保守的超时，联调期间不要手动修改系统时钟。

控制循环逐轮更新轻量反馈快照；完整统计约每 20 ms 更新，日志每秒输出。读取反馈不是等待新样本，算法应检查时间戳与 age_ms。七轴 CAN 帧依次到达，不是同步采样。

| 字段 | 含义 |
| --- | --- |
| tx/rx、tx_hz/rx_hz | 运动阶段成功提交发送/处理有效反馈的计数及平均频率，不含启动和失能事务 |
| skipped | 全组错过的调度时隙，不是总线丢包 |
| unobserved_tx | 自最近反馈以来成功发出的命令数；收到反馈清零 |
| backpressure | 因任一轴达到阈值，全组跳过的周期数 |
| count_gap | 累计 tx−rx，仅诊断，不参与限速 |
| recovery_tx | 降级期间低频发送最新、仍未过期目标的次数 |
| max_tx_gap_ms/max_rx_gap_ms | 运动阶段相邻发送/接收样本的最大间隔，不是往返延迟 |
| lateness_p95_ms/p99_ms/max_ms | 该轴发送提交结束相对当前计划周期的延迟；P95/P99 使用 10 µs 分桶的上界估计，20 ms 以上溢出用最大值报告 |
| stop_confirmed | true：取得新失能反馈；false：停止未确认；null：未执行需确认的停止 |
| log_dropped/log_failures | 日志队列满而丢弃的周期统计数/日志消费者输出失败数 |

量化位宽：位置16位、速度/Kp/Kd/前馈力矩各12位；CAN FD+BRS 仍携带8字节 MIT 数据。1 kHz 是目标，并非硬实时保证。绝对时间调度不集中补发，迟到量也不包括 socket 写入之后的 USB/总线排队延迟。

## 运行时协议量程

arm 自动读取每轴寄存器 21/22/23，不写回这些寄存器，不改 JSON 或机械限制。全部有效且兼容限制后用于发送编码和反馈解码。`get_state()["protocol_ranges"]["J1"]` 返回例如 `{"pmax": 12.5, "vmax": 50.0, "tmax": 10.0}`；尚未完成本轮量程检查时该映射为空。无需算法提交量程。每次 arm 重读，读取失败不自动使用默认值继续启动。
