# 代码阅读路线

建议先理解数据流，再阅读状态机和底层编码。以下路径均相对于项目根目录。

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
