# 算法组接入指南

## 1. 仓库结构

Python 3.10+、Linux，无需 ROS。

| 目录 | 内容 |
| --- | --- |
| `src/mit_arm_control/` | 控制服务和 `ArmClient` SDK |
| `configs/` | 模拟与实机配置 |
| `examples/` | 可运行的算法接入示例 |
| `tests/` | 离线与模拟测试 |
| `docs/API.md` | 本接入指南 |

算法代码位置不限。确保算法的 Python 环境能导入 `mit_arm_control`，再通过 `ArmClient().connect()` 连接已启动的服务即可。

## 2. 接口速查

| 接口 | 用法及结果 |
| --- | --- |
| `ArmClient().connect()` | 连接服务；也可使用 `with ArmClient() as client:` 自动连接/关闭 |
| `client.get_state()` | 返回最新状态字典，不等待下一份反馈，不触发归零或运动 |
| `client.arm(timeout=15.)` | 检查通信配置、实际协议量程、失能及模式；仅服务加 `--set-zero` 时置零；返回 `READY`，此时仍失能，首条有效目标才触发使能 |
| `client.observe(timeout=15.)` | 仅用于观察服务；按服务 `--set-zero` 决定是否置零，返回 `OBSERVING`，电机保持失能 |
| `client.submit(joints)` | 提交完整一组目标；返回服务接收确认，执行状态通过 `get_state()` 检查 |
| `client.stop(timeout=4.)` | 停止并等待失能确认，返回状态；未确认会抛异常 |
| `client.reset_fault(timeout=4.)` | 故障处理后显式复位，返回 `IDLE`；不会自动归零或恢复运动 |
| `client.close()` | 关闭连接；不能代替需要取得停止结果的 `stop()` |

一个客户端实例由一个线程顺序调用。只允许一个控制客户端。其他程序可用 `ArmClient(role='observer')` 连接，只读取状态；此角色不能调用 `observe()` 或发起控制操作。

如需不同服务地址，服务使用 `--socket /tmp/my-arm.sock`，客户端使用 `ArmClient('/tmp/my-arm.sock')`。

## 3. 提交目标

使用 `--no-gripper` 启动时，目标必须恰好包含 `J1`～`J6`，不填写 `gripper`。安装夹爪后，完成其配置并去掉 `--no-gripper`，目标须额外包含 `gripper`。

算法通过 `client.get_state()["joint_names"]` 获取本次服务的轴列表；未启用轴不会被初始化、归零或等待反馈。`--no-gripper` 同样适用于 `check-config`、`diagnose`。每轴有以下五个字段：

| 字段 | 含义 | 单位 |
| --- | --- | --- |
| `q_des` | 期望关节位置 | rad |
| `dq_des` | 期望关节速度 | rad/s |
| `kp` | 位置误差增益，非负 | N·m/rad |
| `kd` | 速度误差增益，非负 | N·m/(rad/s) |
| `tau_ff` | 前馈力矩，如算法计算的重力补偿 | N·m |

运动目标和反馈均使用电机协议坐标：服务只按实际量程解码，不做方向取反、零点偏移或减速比变换。算法自行处理模型坐标与电机坐标的转换。夹爪仍用驱动轴角度，不提供毫米开口或夹持力换算。位置、姿态、速度、力矩、增益选择、轨迹变化率和机械限位均由算法负责。服务不检查主机侧运动限制，不估算 MIT 合成力矩，不裁剪或平滑目标。

服务只接受协议可编码的目标：`q_des` / `dq_des` / `tau_ff` 分别在本次读回的 ±PMAX / ±VMAX / ±TMAX 内，`kp` 在 0～500，`kd` 在 0～5；与特殊控制命令编码冲突的目标也会拒绝。这些是协议表示范围，不是额定能力或可用运动范围。整组目标会在发送前完成检查；IPC 回执仍只表示接收。

服务继续检查有限数值、启用轴集合、时间戳、递增序号、反馈状态和通信超时。温度保留为遥测，不设置主机侧温度阈值；电机报告故障状态时仍触发停止。


算法建议先按约 200 Hz 更新目标。默认目标有效期 50 ms；算法计算或休眠不能长期阻断更新。服务保持最新有效目标，不自动插值，旧目标不会排队依次执行。

高级调用 `submit(joints, timestamp=..., sequence=...)` 可显式指定同机 `time.monotonic()` 时间戳和严格递增序号；一般省略，由 SDK 自动生成。不要使用 `time.time()`。

## 4. 算法程序结构

下面是接入结构，`your_algorithm` 由算法组实现，运动约束和增益选择由算法实现。先在模拟环境验证。

```python
import time
from mit_arm_control import ArmClient
from your_algorithm import initial_targets, compute_targets

with ArmClient() as client:
    try:
        state = client.arm()  # 检查并准备；默认保留现有零点
        # 返回全部启用轴的初始控制目标。
        client.submit(initial_targets(state))
        while True:
            state = client.get_state()
            if state['state'] == 'FAULT':
                raise RuntimeError(state['reason'])
            targets = compute_targets(state)  # 全部启用轴、五字段字典
            client.submit(targets)
            time.sleep(0.005)  # 示例节拍；计算耗时也计入更新周期
    except KeyboardInterrupt:
        pass
    finally:
        result = client.stop()
        print('失能确认：', result['stop_confirmed'])
```

`submit()` 成功不表示目标已经执行；后续仍须检查状态。算法异常、连接断开或目标过期都会触发服务的停止处理。`stop()` 是失能，不是回零、回桌或制动轨迹；需要回桌时由算法先完成轨迹再调用。

## 5. 启动方式：两个进程

以下服务和示例命令在项目根目录执行。先启动控制服务，再启动算法程序。两者使用同一台机器、同一 Linux 用户。服务启动和 SDK 连接不会使能或归零。

### 模拟联调

终端一，在项目根目录执行：

```bash
PYTHONPATH=src python3 -m mit_arm_control serve --config configs/simulation.json --no-gripper
```

终端二：

```bash
PYTHONPATH=src python3 examples/sim_algorithm.py --duration 30
```

运行自己的算法时，先在其 Python 环境安装 SDK：在本项目根目录执行 `python3 -m pip install -e .`，之后可从任意目录按算法自身入口启动，无需把代码放进本仓库。

模拟后端用于验证接口与流程，不代表真实机械臂动力学。

### 实机运动

由设备负责人提供已确认型号、总线和 ID 的配置，例如 `configs/arm.hardware.json`；该文件名是约定示例，仓库模板不能直接作为已验收配置。

```bash
PYTHONPATH=src python3 -m mit_arm_control serve \
  --config configs/arm.hardware.json --hardware --no-gripper
```

另一终端运行算法程序。`sim_algorithm.py` 的运动示例只允许模拟后端，不用于实机运动。

默认不置零，保留电机现有位置坐标。需要置零时，在上述服务命令末尾加 `--set-zero`；该选项对本服务会话中每次 `arm()` / `observe()` 生效，服务启动本身不操作电机。算法接口保持不变。

仅启用 `--set-zero` 时，由算法／操作者确定设置零点的姿态。服务不检查姿态或静止速度；设零后仍校验反馈零位是否与请求一致。归零不会自动移动机械臂。

### 六轴手动拖动观察

```bash
# 终端一：先停掉其他控制服务
PYTHONPATH=src python3 -m mit_arm_control serve \
  --config configs/arm.hardware.template.json --hardware --no-gripper --observe-only \
  --joints J1 J2 J3 J4 J5 J6 --query-rate 400

# 终端二：启动观察
PYTHONPATH=src python3 examples/sim_algorithm.py --observe --duration 30 --print-rate 1
```

`observe()` 返回后才开始手动拖动。观察模式始终不使能，不接收 `submit()`。观察模式的数据为电机角度坐标，不能直接当作算法模型坐标。

## 6. 读取状态与处理故障

```python
state = client.get_state()
mode = state['state']
f = state['feedback'].get('J1')
if f is not None:
    print(f['q'], f['dq'], f['tau'], f['age_ms'])
```

| 字段 | 含义 |
| --- | --- |
| `state` | `IDLE` 待启动；`ARMING` 初始化；`READY` 等初始目标；`RUNNING` 运行；`OBSERVING` 观察；`DEGRADED` 降级；`STOPPING` 停止中；`FAULT` 故障锁存 |
| `reason` | 当前状态原因；恢复运行后清除，历史状态变化保留在 `events` 中 |
| `feedback[name].q / dq / tau` | 位置、速度、反馈估计力矩；力矩不是外力传感器读数 |
| `feedback[name].timestamp / age_ms` | 样本时间与年龄；连续读取可能得到同一份样本，各轴不是严格同步采样 |
| `feedback[name].mos_temperature / rotor_temperature` | 驱动及电机温度，℃ |
| `feedback[name].status` | 驱动状态，0 为失能、1 为使能；其他状态交由服务处理 |
| `stop_confirmed` | `True` 已确认失能；`False` 未确认；`None` 尚未执行需确认的停止 |
| `protocol_ranges[name].pmax / vmax / tmax` | 本次启动读回的编解码量程；启动成功前可能为空 |
| `configuration_errors` | 尚未满足的通信配置／协议量程条件 |
| `metrics_timestamp` | 完整统计的单调时钟时间；与各轴反馈时间分别更新 |
| `transport[bus]` | 实机 socket 自创建以来的总线错误、接收队列丢包计数；与完整统计一起更新，模拟后端为空 |

缺少某轴反馈时不能自行补零。故障后停止算法更新，记录 `reason`；处理原因后调用 `reset_fault()`，再 `arm()`；若启用置零，先摆好约定姿态。不要编写自动复位、自动恢复运动的无限重试循环。

有新反馈时更新反馈快照，没有新帧时复用已有数据；目标序号和状态变化立即发布；完整统计在运行时约每 20 ms 更新，生命周期操作结束及退出时额外刷新。`get_state()` 不触发统计计算或 CAN 查询。`metrics` 的频率是本次运行以来的累计平均值，并非最近一秒的频率；它可能比反馈和当前状态稍旧。

`skipped` 是调度跳过的周期数，`backpressure` 是流控阻止发送的次数，均不等于 CAN 丢帧数。`tx_queue_full` 统计该轴发送遇到 `ENOBUFS/EAGAIN/EWOULDBLOCK` 的次数。队列满时结束本轮发送，设置一个配置周期（不超过反馈探测间隔）的退避门限，之后在下个调度点重试最新目标；不补发历史目标。连续缺反馈仍按 `min(20 ms, feedback_timeout/4)` 间隔探测。持续发送拥塞和反馈超时仍会触发停止，不因降低实际发送频率而延长超时。

实机后端在内核中过滤配置的标准 Master ID，并额外接收状态类 CAN 错误帧。`transport[bus]` 包括 `error_frames`、`bus_off`、`ack_errors`、`controller_errors`、`tx_timeouts`、`restarted`、`rx_queue_dropped`、`last_error_classes`、`last_error_timestamp`。这些总线计数不能解释成某一轴的丢帧数；错误帧和参数回复均不刷新运动反馈有效期。`last_error_timestamp` 是错误被软件处理的时间，不是线缆事件时间。`rx_queue_dropped` 是 Linux 提供的该 socket 累计 uint32 接收溢出数，可回绕；无后续报文到达时也可能尚未收到最新计数。计数为零不能证明物理线路无错误，错误报告依赖驱动和适配器，且服务未订阅每次位错误。

服务保持运行期间 200 μs 的单次等待上限，CAN 到帧可以提前唤醒；该值不是实时保证。收发仍由同一个 CAN 所有者处理，尚未拆成独立收发线程。同一目标的 MIT 帧可复用已验证编码，目标数值、路由或运行时量程变化都会重新编码；目标过期检查仍在每次发送前执行。

七轴 500 Hz 可通过观察服务的 `--joints J1 J2 J3 J4 J5 J6 gripper --query-rate 500` 测试。该值是调度目标，不是已保证的硬件吞吐量；需逐轴检查 TX/RX、跳周期、反馈间隔及停止确认。观察模式测试不能代替使能后的 MIT 控制验收。

## 7. 启动与读取的区别

`arm()` 是有副作用的启动操作：读参数并校验、确认失能、按选项设置并验证零点、确认控制模式；成功后返回 `READY`，首条有效目标才触发使能。每次工作调用一次，不放在算法循环中。

`get_state()` 只读取服务维护的最新快照，可以在算法循环中重复调用，不重新归零、不使能、不重新执行启动检查。

默认服务支持运动接口，无额外实机验收开关；连接服务不会自动使能。`--observe-only` 仅用于测试，实机仍须显式指定 `--hardware`。默认不修改电机零点；仅服务加 `--set-zero` 时在 `arm()` / `observe()` 期间设零，反馈以电机当前保存的零点为基准。

## 8. 电机配置与协议量程

每个轴配置 `name`、`model`、`bus`、`can_id`、`master_id`，不配置 `limits`；含旧 `limits` 字段的配置不能通过运动启动检查。型号由配置选择，不依赖关节名。目前支持 `4310_48V`、`4340_48V`、`8009_48V`、`3507_24V`。实机模板与模拟配置使用下表选型；J7 在 SDK 中仍命名为 `gripper`。

| 关节 | SDK 名称 | 型号 | CAN ID | Master ID |
| --- | --- | --- | --- | --- |
| J1 | `J1` | `4310_48V` | `0x01` | `0x11` |
| J2 | `J2` | `8009_48V` | `0x02` | `0x12` |
| J3 | `J3` | `4310_48V` | `0x03` | `0x13` |
| J4 | `J4` | `3507_24V` | `0x04` | `0x14` |
| J5 | `J5` | `3507_24V` | `0x05` | `0x15` |
| J6 | `J6` | `3507_24V` | `0x06` | `0x16` |
| J7 | `gripper` | `3507_24V` | `0x07` | `0x17` |

未安装夹爪时使用 `--no-gripper`。JSON 中 ID 使用十进制整数。周期发送仍按总线和 Master ID 排序；此配置下顺序即 J1～J7。

每次 `arm()` / `observe()` 读取各轴 21/22/23 号寄存器；全组读回值均为正有限数后，更新本次会话的编解码量程并重新获取反馈。读取失败则中止，不回退到默认值。不会写入量程寄存器或修改 JSON；`arm()` 仍会写入并读回 MIT 模式。

型号默认量程只供模拟及启动前诊断使用：`4310_48V` 为 `(12.5, 50, 10)`，`4340_48V` 为 `(12.5, 20, 28)`，`8009_48V` 为 `(12.5, 45, 54)`，`3507_24V` 为 `(12.566, 100, 5)`，单位依次为 rad、rad/s、N·m。3507 的协议依据为达妙官方 [DM-J3507-2EC V1.1 手册](https://github.com/dmBots/DM-J3507-2EC/tree/master/manual)，其 MIT 位布局与现有编解码一致。实际量程始终以本次读回值为准。

8009 默认量程采用达妙官方 [motor-sdk 的 DM8009 定义](https://github.com/dmBots/motor-sdk/blob/main/Python%E4%BE%8B%E7%A8%8B/u2can/DM_CAN.py)，只作为模拟及启动前诊断默认值；实机仍逐轴读回量程。
