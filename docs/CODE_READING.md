# 代码阅读路线

本文同时用于代码阅读、算法组 API 对接以及模拟/实机操作。以下路径均相对于项目根目录。

建议阅读顺序：

1. 阅读入口、后端和各命令的区别，了解程序如何启动。
2. 与算法组一起核对 API 接入约定、单位、初始目标和更新频率。
3. 按测试/启动步骤先模拟，再填写实机配置、诊断并联调。
4. 需要追踪实现时，按下表阅读源码。

## 建议顺序

| 顺序 | 文件与入口 | 关注点 |
| --- | --- | --- |
| 1 | `examples/sim_algorithm.py` | 算法如何连接、确认零姿态、提交初始目标、循环更新、退出停止 |
| 2 | `src/mit_arm_control/sdk.py`：`ArmClient` | 公共接口；IPC 请求回执与电机失能确认的区别 |
| 3 | `src/mit_arm_control/service.py`：`run` | 主循环与所有权；停止、动作、目标、通信和日志的执行顺序 |
| 4 | `src/mit_arm_control/controller.py`：`arm → submit → step → stop` | 启动、使能、周期发送、反压降级与故障停止 |
| 5 | `src/mit_arm_control/safety.py` | 命令有效期、完整七轴、限位、变化率、反馈及力矩估算 |
| 6 | `src/mit_arm_control/protocol.py` | MIT 位布局、型号量程、关节与电机坐标变换、回复分类 |
| 7 | `src/mit_arm_control/backend.py` | CAN FD 布局、内核接收时间戳、参数事务、模拟故障注入 |
| 8 | `src/mit_arm_control/ipc.py` | 单控制权、最新目标槽、请求 ticket、异步日志 |
| 9 | `src/mit_arm_control/config.py`、`__main__.py` | 配置完整性与命令行入口 |

## 一条目标的路径

```text
算法 ArmClient.submit()
    → IPCServer.dispatch() 校验、Mailbox.put_command() 覆盖最新目标
    → 服务 run() 取出目标、Controller.submit() 校验并接收
    → Controller.step() 到发送时刻后调用 backend.mit()
    → protocol.pack_mit() 编码、SocketCAN.send() 提交内核
```

首次 submit 在 READY 状态下会额外执行初始目标和使能验证。IPC 的 queued 回执在该流程完成之前即可返回。运行阶段保持最新且未过期的目标，不插值、不积压历史运动目标。

## 反馈与停止的路径

- `SocketCAN.poll()` 接收、分类反馈，记录内核接收时间对应的单调时间戳。
- `Controller.poll()` 观察每轴接收计数，更新间隔统计并清零连续无反馈发送次数。
- `check_feedback()` 检查年龄、状态、位置/速度/力矩/温度。
- `stop()` 清除目标并进入 STOPPING；`disable()` 逐轴重试并等待本轮发送之后的新失能反馈。
- 故障锁存为 FAULT；`reset_fault()` 只返回待启动状态，不自动使能。

## 容易混淆的名称

| 名称 | 含义 |
| --- | --- |
| request_id | SDK 与服务之间的请求/响应编号 |
| sequence / seq | 算法目标序号，用于拒绝乱序；不会写入 MIT 帧 |
| ticket | arm、stop、reset_fault 等服务动作的完成凭据 |
| rx_seq | 主机每轴有效反馈接收计数，不是电机回复的命令序号 |
| unobserved | 自上一次反馈以来发送了多少条命令，收到反馈清零 |
| count_gap | 累计 TX−RX，仅供统计，不限制发送 |
| deadline / next_tick | 绝对发送时间表，不是相对 sleep 的累积 |
| workspace_confirmed | 已进入需执行停止确认的启动流程，不代表已证明机械安全 |
| j / t / f | 紧凑局部变量通常分别为单轴配置、该轴目标、该轴反馈 |

本次阅读性整理仅调整排版与注释。核心源码的 Python 抽象语法树与整理前一致；测试统一保留在 `tests/`。

## 常见问题：入口、后端和进程

### 没有 serve.py，为什么可以运行 serve？

```bash
PYTHONPATH=src python3 -m mit_arm_control serve --config configs/simulation.json
```

在项目根目录执行上述命令：

- `PYTHONPATH=src`：让 Python 能找到 `src/mit_arm_control` 包；它不是服务参数。
- `python3 -m mit_arm_control`：执行包入口 `src/mit_arm_control/__main__.py`。
- `serve`：传给入口的 operation 参数，不是文件名。
- `--config`：指定服务使用的配置文件。
- `__main__.py` 中的 `main()` 解析参数，最终调用 `service.py` 的 `run()`。

如果在算法项目自己的目录运行，可使用绝对路径：

```bash
PYTHONPATH=/home/rinky/damiao_ws/mit_arm_control/src python3 your_algorithm.py
```

也可以在算法使用的 Python 环境中安装本项目，使其可以直接 `from mit_arm_control import ArmClient`；不要求算法文件放在本仓库。

### 模拟后端与模拟算法有什么区别？

| 名称 | 所在位置 | 作用 |
| --- | --- | --- |
| `SimBackend` | `backend.py` | 在内存模拟状态和反馈，不打开 CAN；用于验证流程和故障处理，不是精确机械臂动力学仿真 |
| `SocketCAN` | `backend.py` | 使用 Linux CAN socket 与真实电机收发 |
| `sim_algorithm.py` | `examples/` | 调用 SDK 的演示客户端；主动拒绝实机后端，不作为实机测试轨迹 |

服务默认选择 SimBackend；只有显式传入 `--hardware` 才选择 SocketCAN。实机配置文件本身不会切换后端，配置里的 `bus: can0` 在模拟模式也不会打开真实总线。

“构造后端不会使能”指 `SocketCAN(...)` 只建立通信通道，不主动发使能命令；它也不保证已经被其他程序使能的电机会自动失能。完整初始化由 `arm()` 触发。

### 实际运行要几个进程？

通常是两个进程、两个终端：

```text
算法程序进程：ArmClient → 本机 Unix socket
                                  ↓
控制服务进程：IPC 线程 → 最新目标槽 → 主控制循环 → SimBackend 或 SocketCAN
                                    └→ 有界日志队列 → 日志线程
```

一个控制服务管理全部七轴，不需要每轴启动一个服务。`serve` 不会自动拉起算法程序；仅启动服务不会自动运动。以后可以用启动脚本同时拉起两个进程。

独立进程让算法计算、崩溃或退出与控制服务分开；服务仍可在命令超时或断联后执行停止。但二者仍共享 CPU 和操作系统，不能据此承诺硬实时。

## serve、check-config、diagnose、monitor 的区别

| 操作 | 是否打开真实 CAN | 是否读写真实电机 | 完成后是否退出 | 用途 |
| --- | --- | --- | --- | --- |
| `check-config` | 否，即使带 `--hardware` 也不打开 | 不读、不写 | 是 | 检查 JSON、轴映射、必要字段、范围与准入标记 |
| `diagnose --hardware` | 是 | 仅发送参数读取请求；不写参数、不归零、不使能、不失能 | 是 | 确认填写的 ID 能回应，查看当前模式和量程 |
| `monitor --hardware` | 是 | 读取实际量程并循环查询所选轴反馈，不改变使能/模式/零位 | 按 duration 或 Ctrl+C 退出 | 未完成运动配置时先做六轴只读检查 |
| `serve` | 否 | 仅内存模拟 | 否 | 启动模拟控制服务，等待算法连接 |
| `serve --hardware` | 是 | 启动时不使能；算法请求 arm/submit/stop 后执行相应事务 | 否 | 启动真实控制服务 |

`check-config --hardware` 的“hardware”仅表示按实机准入条件检查，例如要求 `hardware_commissioned=true`。检查通过不证明电机接线、反馈、机械限位测量或总线带宽正确。实机离线检查不使用型号默认值判定量程是否足够；该项留到 arm 读取实际量程后检查。

`diagnose` 允许机械限制等配置尚未补齐，选取具有整数 can_id/master_id 和 bus 的关节执行读取；仍会检查总线接口、占用锁及 CAN FD 参数。它不运行完整的实机 readiness 检查，因此仍应确认填写的 ID 正确。与 `serve --hardware` 不能同时占用同一总线，先完成诊断再启动服务。

诊断打印的寄存器编号如下（JSON 对象键显示为字符串）：

| 编号 | 含义 | 型号表参考值（实际运行以读取结果为准） |
| --- | --- | --- |
| `10` | 控制模式 | MIT 模式为 1；diagnose 只读，不修正 |
| `21` | PMAX，位置编码量程 | 4310/4340 均为 12.5 rad |
| `22` | VMAX，速度编码量程 | 4310 为 50 rad/s，4340 为 20 rad/s |
| `23` | TMAX，力矩编码量程 | 4310 为 10 N·m，4340 为 28 N·m |

diagnose 只输出读数，不自动判定它们与模型 MATCH。正式 arm 每次都会逐轴读取 21/22/23，校验为有限正数，并检查配置的机械位置、速度和力矩限制是否落在实际协议量程内；全部通过后，统一作为本次运行的发送编码和反馈解码量程。无需手动把这三个参数抄到 JSON，也不要求与型号表参考值完全相同。

这只更新运行时量程，不改写电机的 21/22/23，不覆盖 JSON，更不会把机械限制自动扩大到协议最大值。实际量程可通过 `get_state()["protocol_ranges"]` 查看，每轴包含 `pmax/vmax/tmax`，也会进入统计快照与最终报告。读取失败、非有限数、零/负数或机械限制超出量程，都会阻止启动并执行停止。运动中不要用其他工具改量程；需停止后重新 arm 读取。

寄存器 10 则仍会写入 MIT 模式并读回确认。量程不是机械安全限位，也不是建议运动参数。

## 核心配置参数怎么解释？

### 服务级参数

| 参数 | 当前默认/用途 | 调整时关注 |
| --- | --- | --- |
| `version` | 配置格式版本 1 | 不是电机固件版本 |
| `rate_hz` | 每轴目标发送频率 1000 Hz | 七轴意味着目标约 7000 条运动命令帧/秒，反馈另计；实际频率须测量 |
| `command_timeout` | 最新算法目标有效期，默认 0.05 秒 | 算法需在有效期内持续更新，包含计算和调度余量；过期停止，不无限保持旧目标 |
| `feedback_timeout` | 每轴反馈超时，默认 0.05 秒 | 任一轴无新反馈超过阈值触发全组停止；发送不能刷新它 |
| `stop_timeout` | 全组失能确认重试期限，默认 2 秒 | 是服务端停止事务时间，不是急停响应时间保证；最大配置值 5 秒 |
| `max_unobserved` | 连续无反馈发送阈值，默认 4 | 触发全组反压降级；不是累计 TX−RX，也不是精确在途帧数 |
| `hardware_commissioned` | 实机配置已由操作者核实 | 必须在补齐并核实配置后设为 true，不是通过检查的快捷开关 |
| `hardware_watchdog_verified` | 可选验收记录 | 当前不作为启动条件，不会自动写电机 TIMEOUT 或启用 MCU 保护 |

反压时的探测间隔为 `min(20 ms, feedback_timeout / 4)`，默认 12.5 ms；仅重发仍有效的最新目标。命令或反馈达到超时阈值仍停止，故障锁存后不自动使能。

### 每轴参数

| 参数 | 含义 |
| --- | --- |
| `name` | SDK 使用 J1～J6、gripper；gripper 就是 J7 |
| `model` | J2/J3 是 4340_48V，其余沿用 4310_48V；用于识别型号及提供模拟默认量程；实机 arm 自动读取实际编码量程 |
| `bus` | Linux 总线名，例如 can0；七轴共线就七轴都填写 can0，无需 CLI 再指定 --can |
| `can_id/master_id` | 命令/反馈 ID；J1～J6 为 5/3/2/4/1/6 与 21/19/18/20/17/22；夹爪仍为 7/23；JSON 用十进制数字，17 对应 0x11 |
| `direction` | 电机正向到算法正向的映射，必须实测为 +1 或 -1 |
| `zero_joint` | 电机设零后，该姿态对应的算法关节角；当前六轴约定为 0 rad |
| `calibration_pose` | 固定零姿态的现场说明；每次复现，不是自动寻找零姿态的指令 |

坐标变换：`q_joint = direction * q_motor + zero_joint`；速度和力矩也应用 direction。算法始终使用关节侧 rad、rad/s、N·m。夹爪暂用电机角度，不直接接收毫米或夹持力。

### limits：限制值与算法目标不是同一回事

| 限制字段 | 单位/含义 |
| --- | --- |
| `q_min/q_max` | rad，允许的位置范围，必须包含零姿态并避开协议回绕边界 |
| `dq_max` | rad/s，允许的速度绝对值 |
| `tau_max` | N·m，约束前馈目标、反馈力矩与主机 MIT 力矩估算；不是硬件力矩钳位 |
| `kp_max` | N·m/rad，允许的 Kp 上限，实际值由 submit 提供 |
| `kd_max` | N·m/(rad/s)，允许的 Kd 上限，实际值由 submit 提供 |
| `q_rate` | rad/s，相邻目标位置的最大变化率，不直接等于实测速度 |
| `dq_rate` | rad/s²，相邻目标速度的最大变化率 |
| `kp_rate/kd_rate` | 对应增益单位/秒，相邻增益目标的变化率 |
| `tau_rate` | N·m/s，相邻前馈力矩目标的变化率 |
| `temperature_max` | °C，MOS 或转子温度达到此阈值即停止 |

变化率按命令时间戳间隔核对；程序拒绝违规目标，不静默裁剪，也不会自动生成渐变轨迹。例如 q_rate=2 rad/s 且两次目标相隔 5 ms，则目标位置差最多约 0.01 rad。这仅解释规则，不是实机配置建议。

MIT 力矩估算为 `kp*(q_des-q) + kd*(dq_des-dq) + tau_ff`。只给正的速度目标且 Kp=0 不会提供固定位置保持；具体控制律和增益由算法组设计，并在配置边界内提交。

## 给算法组的 API 接入约定

算法组编写自己的程序替代示例客户端的角色，不需覆盖 `examples/sim_algorithm.py`。算法通过 ArmClient 连接服务，不自行打开 CAN，也不负责拼接 MIT 帧。

| 接口 | 返回/作用 | 接入注意 |
| --- | --- | --- |
| `connect()` | 获得本机控制连接，返回客户端 | 不使能；单控制客户端；观察者用 role='observer' |
| `get_state()` | 最近状态、反馈、年龄、故障与统计快照 | 不等待新采样，七轴不是同步采样；检查 state、reason、age_ms |
| `arm(workspace_ready=True, zero_pose=True)` | 等待归零等流程完成并进入 READY | 双确认来自现场；仍失能；默认 SDK 等待期限 15 秒 |
| `submit(joints)` | 返回 queued/seq，确认接收进最新目标槽 | 接收后控制循环还会校验；不是逐帧执行确认 |
| `stop()` | 请求并等待本次全组失能结果 | 默认 SDK 等待 4 秒；若服务 stop_timeout 调大，应相应留出等待余量；未确认会抛异常 |
| `reset_fault()` | 显式复位到 IDLE | 重新确认失能，不自动恢复运动；之后重新确认并 arm |
| `close()` | 关闭连接 | 控制客户端断联会请求故障停止，但 close 不等待停止结果，应先 stop |

每次 submit 必须同时包含 J1～J6、gripper 七轴，每轴五个字段：

```python
# 结构示意，省略号需由算法填写；不是可直接运行的运动目标。
all_targets = {
    name: {
        "q_des": ...,   # rad，目标位置
        "dq_des": ...,  # rad/s，目标速度
        "kp": ...,      # N·m/rad
        "kd": ...,      # N·m/(rad/s)
        "tau_ff": ...,  # N·m，前馈力矩
    }
    for name in ("J1", "J2", "J3", "J4", "J5", "J6", "gripper")
}
```

接入顺序：

1. 导入 `from mit_arm_control import ArmClient`，连接与服务相同的 socket 路径。
2. 获取状态并核对 backend：`simulation` 或 `socketcan`，避免连错服务。
3. 现场确认固定零姿态与缓冲区域，调用 arm，等待 READY。
4. 提交完整七轴初始静止目标。当前位置误差须不超过 0.02 rad，目标速度/前馈力矩绝对值须不超过 0.1，并满足配置限制；不要直接从运动轨迹中段开始。
5. 进入目标更新循环，持续 get_state/submit；首条提交仅返回接收确认，通过 get_state 观察 RUNNING 或 FAULT，不将 queued 当作已经使能。不要等待很久才更新下一条命令，以免超过 TTL。
6. 退出或异常时在 finally 请求 stop，并在另一个 finally 中 close；不要吞掉失能未确认异常。

SDK 默认生成同机单调时间戳和递增序号，一般无需自己传。若算法目标在生成后还会排队/耗时处理，可传生成时的 `time.monotonic()`，防止旧目标到提交时才被标为新目标。不要传 time.time() 或远程机器时间。

ArmClient 实例不是线程安全的，一个实例由一个线程顺序调用；算法有多个工作线程时，应由一个发送线程汇总完整目标。当前是本机 Unix socket，不支持直接跨电脑连接。

推荐先在模拟环境以约 200 Hz 更新算法目标，服务独立按 1 kHz 目标发送；这不是对所有实机算法的频率建议。一个目标可被重复发送多次，新目标也可能在控制循环读取前覆盖旧目标，不保证每次 submit 都各对应一个 CAN 帧。

### 0600、回执与延迟

Unix socket 是本机进程通信入口，文件权限 0600 表示仅创建它的用户可读写，组和其他用户无权限；特权用户不受同样限制，也不隔离同用户的其他程序。服务额外限制单一控制客户端。

MIT 帧没有携带 SDK 的目标序号，电机回复也没有回显该序号，因此 TX/RX 数量相同不能证明逐命令执行成功；失能确认使用新反馈及状态判断，也不是机械静止证明。

IPC 会增加序列化、请求应答和进程调度开销，独立服务则减少算法计算或打印直接阻塞发送的机会。总延迟包括算法计算、IPC、等待发送周期、内核/USB/CAN 排队及电机处理。发送迟到量只相对于计划发送时间，并非端到端延迟；需另测 submit 往返时间、目标生成至首次发送的时间及其 P95/P99/最大值。

## 自己如何测试和启动？

以下命令均在 `/home/rinky/damiao_ws/mit_arm_control` 下执行。配置名 `arm.hardware.json` 是你复制模板后填写的文件，不是已完成配置的内置文件。

### 第一步：离线检查和模拟联调

```bash
cd /home/rinky/damiao_ws/mit_arm_control
PYTHONPATH=src python3 -m mit_arm_control check-config --config configs/simulation.json
PYTHONPATH=src python3 -m unittest discover -s tests -v
```

完整测试需要本机 Unix socket 权限；不会访问实机。日常仅改文档无需重跑全部测试。

终端一启动服务：

```bash
PYTHONPATH=src python3 -m mit_arm_control serve \
  --config configs/simulation.json \
  --socket /tmp/mit-arm-control.sock \
  --final-state /tmp/mit-arm-final-state.json
```

终端二运行已有模拟示例，或算法组自己的客户端：

```bash
PYTHONPATH=src python3 examples/sim_algorithm.py --duration 5
```

观察 READY/RUNNING、目标和反馈、超时/反压及最终失能结果。模拟的速度、力矩和频率不能替代实机性能验收。

### 第二步：填写实机配置并检查

首次创建配置时复制模板；若自己的配置已存在，不要用此命令覆盖它：

```bash
cp -n configs/arm.hardware.template.json configs/arm.hardware.json
```

补齐方向、实际限制、夹爪零位等，核实后填写 hardware_commissioned。六轴暂定零姿态见 [ROBOT_MODEL.md](ROBOT_MODEL.md)。不要照搬模拟限制。

```bash
PYTHONPATH=src python3 -m mit_arm_control check-config \
  --config configs/arm.hardware.json --hardware
```

正常配置检查输出 `ready: true`、`errors: []`；缺项时输出错误且退出码非零，格式无法解析等错误则直接报错。此时尚未打开 CAN。

### 第三步：准备系统 CAN 并只读诊断

系统侧需事先把 can0 配为 CAN FD、UP、MTU 72、仲裁 1 Mbps、数据 5 Mbps；服务不会自动配置接口。配置中每轴的 bus 决定连接哪个接口，没有 --can 参数。

```bash
ip -details link show can0
PYTHONPATH=src python3 -m mit_arm_control diagnose \
  --config configs/arm.hardware.json --hardware
```

退出其他电机控制/参数写入脚本，diagnose 完成后再启动实机服务。它不对当前已使能的电机执行停止，只读不等于自动失能。核对各轴寄存器读数；没有回应时先处理供电、ID、总线和速率，再尝试运动。

### 第四步：两个进程启动实机

终端一：

```bash
PYTHONPATH=src python3 -m mit_arm_control serve \
  --config configs/arm.hardware.json \
  --hardware \
  --socket /tmp/mit-arm-control.sock \
  --final-state /tmp/mit-arm-final-state.json
```

终端二（替换为实际算法入口）：

```bash
PYTHONPATH=/home/rinky/damiao_ws/mit_arm_control/src python3 /你的算法目录/your_algorithm.py
```

运行前按固定姿态摆放机械臂，算法程序按上述双确认和初始目标流程启动。不要直接用会拒绝实机的 sim_algorithm.py 充当实机算法。启动服务、连接 SDK 不会自动使能；arm 后仍为 READY，首条有效目标触发使能。

### 第五步：停止与查看结果

算法先 stop 并检查结果，再 close。需要退出服务时在服务终端 Ctrl+C；正常退出、SIGINT/SIGTERM 和可捕获异常会执行停止流程。服务退出后查看 `--final-state` 指定文件中的 state、reason、stop_confirmed 及各轴统计。

- `stop_confirmed=true`：收到本次停止后的全组失能确认，不代表主动制动或已静止。
- `stop_confirmed=false`：停止未确认，应查看故障原因；不能宣称已成功失能。
- `stop_confirmed=null`：没有执行需要确认的停止流程，例如服务尚未 arm；不能据此认为电机一定失能。

SIGKILL、电脑掉电或 CAN 断开时无法保证失能命令送达。当前实验允许失能后落到下方缓冲垫，不提供防坠、重力保持或故障回桌。退出服务后最终报告才写入，需注意不要将上一轮文件当作本轮结果。

若 socket 路径已存在，先确认是否另一个服务仍在运行，不要直接删除正在使用的 socket。算法和服务必须使用同一路径、具备相应用户访问权限。


## 首次六轴只读查询（monitor）

尚未补齐方向和机械限制时，用下面的独立命令观察六轴。只需一个进程，不启动 serve 或算法，不调用 arm/submit，也不接触夹爪。

```bash
PYTHONPATH=src python3 -m mit_arm_control monitor \
  --config configs/arm.hardware.template.json --hardware \
  --joints J1 J2 J3 J4 J5 J6 --query-rate 10 --duration 0
```

monitor 读取所选轴模式与实际量程，然后周期发送状态查询；每秒显示一组反馈。queries 是已成功提交的查询数，RX 是收到的有效反馈数，不保证逐帧配对；age 是反馈年龄。NO_FEEDBACK 表示尚未收到反馈；STALE 表示年龄超过 max(0.2秒, 3个查询周期)。RECENT 仅表示近期收到，不是整臂安全判定。status=0 为失能、1 为使能，其他值需核查驱动故障；mode=1 仅说明 MIT 模式，不表示使能。

q/dq/tau 是电机当前坐标下的量，不应用尚未确认的方向或零位。monitor 不会归零、写模式或发送任何 MIT/使能/失能命令；Ctrl+C 仅结束查询，不会改变电机已有使能状态。本模式不要求 hardware_commissioned，但仍检查 ID、总线锁和接口 CAN FD 参数。只读成功不表示已通过运动配置验收。

现有正式控制服务要求整组七轴参与启动；不要通过向其他轴提交零增益来冒充“只测试 J6”。单轴 J6 运动测试需独立限定参与轴与经确认的运动参数，完成只读检查后再安排。
