# 配置与固定姿态联调

## 首版明确限制

当前实验采用固定零姿态启动，运动时无机械支撑，下方有缓冲垫，允许故障失能后下落。不提供防坠、重力保持或故障回桌功能。缓冲垫不是制动器；本版也不保证下落不会损坏机构。

正常退出、SIGINT/SIGTERM、可捕获异常及算法掉线时，控制服务请求全体失能并等待反馈确认。SIGKILL、电脑掉电和通信中断不能保证失能送达。设备侧通信看门狗可选，不作为启动准入条件，也不会自动设置；通常电机内部已有相关固件能力，不一定需要外加 MCU。

## 配置文件

- `simulation.json`：完整虚拟参数，仅用于模拟，`hardware_commissioned=false` 阻止直接用于实机。
- `arm.hardware.template.json`：七轴型号与 ID 已填写，六轴暂定固定零姿态已记录；方向、限制和夹爪零位仍待配置。

先复制模板到自己的配置文件，再逐项核实。硬件模式仍要求 `hardware_commissioned=true`，仅在实际检查完整配置后填写。`hardware_watchdog_verified` 仅为可选的验收记录，不作为准入条件。

| 配置 | 要求 |
| --- | --- |
| bus/can_id/master_id | 显式指定；同总线上七轴控制和反馈 ID 不冲突；控制 ID 1..15，反馈 ID 1..0x7FE |
| model | J2/J3=4340_48V；其余=4310_48V |
| direction | 实测 +1/-1，影响位置、速度及力矩符号 |
| zero_joint | 每次手动摆到标定姿态并将电机置零后，该姿态在算法坐标中的角度 |
| calibration_pose | 操作者可识别的标定姿态说明，不能留空 |
| q_min/q_max | 实际机械软限位，包含 zero_joint，距电机协议回绕边界保留至少0.1 rad；不支持多圈展开 |
| dq_max/tau_max | 关节允许速度及力矩；同时约束指令和反馈，不能直接用额定值代替机械安全值 |
| kp_max/kd_max | 每轴独立，不能从另一硬件版本直接复制 |
| q_rate/dq_rate/kp_rate/kd_rate/tau_rate | 相应目标量每秒允许变化的最大值 |
| temperature_max | MOS 与转子共同使用的温度阈值，达到阈值停止 |
| command_timeout/feedback_timeout | 默认各50 ms，可配置，需结合算法周期与实测通信验证 |
| rate_hz | 默认每轴1000 Hz；多条总线可配置，但不保证实机吞吐量 |
| max_unobserved | 默认4；连续无反馈发送阈值，不是精确在途队列长度 |

校验配置（不打开 CAN）：

```bash
PYTHONPATH=src python3 -m mit_arm_control check-config \
  --config configs/arm.hardware.template.json --hardware
```

不完整模板应返回非零状态并列出缺失字段，这是预期行为。

## 只读诊断

在填写已知关节的 bus、can_id、master_id 后，可读取这些轴的模式与量程；未填 ID 的轴跳过。不发送模式写入、归零、使能或失能指令。

```bash
PYTHONPATH=src python3 -m mit_arm_control diagnose \
  --config configs/arm.hardware.template.json --hardware
```

实机总线须事先配置为 CAN FD，仲裁1M、数据5M、接口 UP、MTU72；本程序只核对，不替用户改变接口。电机端速率也须一致。

控制服务和诊断工具按接口取得进程锁，防止本项目多个进程同时操作同一接口。锁不能阻止不遵守该锁的旧脚本或其他工具；联调时必须退出其他控制和参数读写程序。被动 candump 可用于观察。

## 实机启动流程

配置完整后，使用 `serve --hardware --config <已核实配置>` 启动。服务仍不自动使能。现场操作者确认缓冲工作区域已准备（workspace_ready）、已摆好固定零姿态（zero_pose），算法通过 arm 请求执行：

1. 只读核对全部量程。
2. 全体失能并获取新失能反馈，确认多次静止。
3. 每轴只执行一次当前位置设零，等待并多次核对。
4. 逐轴确认 MIT 模式，保持失能，进入 READY。
5. 验证算法初始目标与当前零姿态一致，将目标写入并使能，再验证全部状态，进入 RUNNING。

归零容差沿用现有验证经验：位置0.02 rad、速度0.1 rad/s；等待0.2秒不是厂家 Flash 完成时间保证。重复设零可能写入电机非易失存储，按实际固件要求限制频繁重启归零。

## 降级和退出

达到连续无反馈阈值时，全组降级。默认每12.5 ms最多尝试一组最新有效目标：实际间隔为 `min(20 ms, feedback_timeout/4)`，默认50 ms反馈超时对应12.5 ms。收到新反馈恢复速率，不抹掉历史计数差；命令过期、反馈超时或发送失败仍全体停止。

FAULT 状态最多每秒发送一轮只读状态查询，不自动使能。总线恢复后仍需显式 reset_fault 和重新启动；USB 拔插导致接口重建时可能需要停止服务并重新连接。

停止在默认2秒内有界重试，必须取得每轴新失能反馈。未确认则锁存 FAULT，报告具体轴，禁止把 socket 写成功或缓存状态当确认。失能不是主动制动，也不证明机械已经静止。

结果由 SDK 获取，服务退出后另写 `--final-state` JSON 文件。日志消费者阻塞时仍优先处理电机；周期日志可能被丢弃。写最终文件失败应作为报告保存失败处理，不能据此倒推电机状态。
