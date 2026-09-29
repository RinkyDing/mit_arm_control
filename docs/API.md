# 算法组接入指南

## 1. 代码放在哪里

以下命令均在克隆后的项目根目录 `mit_arm_control/` 执行。Python 3.10+、Linux，无需 ROS。

- 算法程序建议放在 `algorithms/`，例如 `algorithms/main.py`，也可使用独立仓库。
- 算法只导入 `from mit_arm_control import ArmClient`，无需修改 `src/mit_arm_control/`。
- `examples/sim_algorithm.py` 是可运行的模拟运动/实机观察示例。
- 在项目根目录使用 `PYTHONPATH=src python3 algorithms/main.py` 运行。独立仓库可先在算法的虚拟环境中执行 `python3 -m pip install -e ../mit_arm_control`。

## 2. 启动方式：两个进程

先启动控制服务，再启动算法程序。两者使用同一台机器、同一 Linux 用户。服务启动和 SDK 连接不会使能或归零。

### 模拟联调

终端一，在项目根目录执行：

```bash
PYTHONPATH=src python3 -m mit_arm_control serve --config configs/simulation.json --no-gripper
```

终端二：

```bash
PYTHONPATH=src python3 examples/sim_algorithm.py --duration 30
# 接入自己的算法后改为：
PYTHONPATH=src python3 algorithms/main.py
```

模拟后端用于验证接口与流程，不代表真实机械臂动力学。

### 实机运动

由设备负责人提供已确认 ID 和限制设置的配置，例如 `configs/arm.hardware.json`；该文件名是约定示例，仓库模板不能直接作为已验收配置。

```bash
PYTHONPATH=src python3 -m mit_arm_control serve \
  --config configs/arm.hardware.json --hardware --no-gripper
```

另一终端运行算法程序。`sim_algorithm.py` 的运动示例只允许模拟后端，不用于实机运动。

**每次调用 `arm()` 或 `observe()` 前，把机械臂摆回约定固定启动姿态并保持静止；调用后会自动执行失能、设零及验证。** 不再传入确认参数，也没有跳过归零的开关。归零不会自动把机械臂运动到固定姿态。

### 六轴手动拖动观察

```bash
# 终端一：先停掉其他控制服务
PYTHONPATH=src python3 -m mit_arm_control serve \
  --config configs/arm.hardware.template.json --hardware --no-gripper --observe-only \
  --joints J1 J2 J3 J4 J5 J6 --query-rate 400

# 终端二：先摆好固定启动姿态
PYTHONPATH=src python3 examples/sim_algorithm.py --observe --duration 30 --print-rate 1
```

`observe()` 返回后才开始手动拖动。观察模式始终不使能，不接收 `submit()`。观察模式的数据为电机角度坐标，不能直接当作尚未完成方向标定的机械臂模型坐标。

## 3. 接口速查

| 接口 | 用法及结果 |
| --- | --- |
| `ArmClient().connect()` | 连接服务；也可使用 `with ArmClient() as client:` 自动连接/关闭 |
| `client.get_state()` | 返回最新状态字典，不等待下一份反馈，不触发归零或运动 |
| `client.arm(timeout=15.)` | 每次归零、检查配置及模式；返回 `READY`，此时仍失能，首条有效静止目标才触发使能 |
| `client.observe(timeout=15.)` | 仅用于观察服务；每次归零，返回 `OBSERVING`，电机保持失能 |
| `client.submit(joints)` | 提交完整一组目标；返回服务接收确认，执行状态通过 `get_state()` 检查 |
| `client.stop(timeout=4.)` | 停止并等待失能确认，返回状态；未确认会抛异常 |
| `client.reset_fault(timeout=4.)` | 故障处理后显式复位，返回 `IDLE`；不会自动归零或恢复运动 |
| `client.close()` | 关闭连接；不能代替需要取得停止结果的 `stop()` |

一个客户端实例由一个线程顺序调用。只允许一个控制客户端。其他程序可用 `ArmClient(role='observer')` 连接，只读取状态；此角色不能调用 `observe()` 或发起控制操作。

如需不同服务地址，服务使用 `--socket /tmp/my-arm.sock`，客户端使用 `ArmClient('/tmp/my-arm.sock')`。

## 4. 提交目标

使用 `--no-gripper` 启动时，目标必须恰好包含 `J1`～`J6`，不填写 `gripper`。安装夹爪后，完成其配置并去掉 `--no-gripper`，目标须额外包含 `gripper`。

算法通过 `client.get_state()["joint_names"]` 获取本次服务的轴列表；未启用轴不会被初始化、归零或等待反馈。`--no-gripper` 同样适用于 `check-config`、`diagnose`。每轴有以下五个字段：

| 字段 | 含义 | 单位 |
| --- | --- | --- |
| `q_des` | 期望关节位置 | rad |
| `dq_des` | 期望关节速度 | rad/s |
| `kp` | 位置误差增益，非负 | N·m/rad |
| `kd` | 速度误差增益，非负 | N·m/(rad/s) |
| `tau_ff` | 前馈力矩，如算法计算的重力补偿 | N·m |

运动目标和反馈均使用电机协议坐标：服务只按实际量程解码，不做方向取反、零点偏移或减速比变换。算法自行处理模型坐标与电机坐标的转换。夹爪仍用驱动轴角度，不提供毫米开口或夹持力换算。配置范围和变化率限制由设备负责人提供，越界会拒绝并可能触发停止，不会静默截断。

首条目标：位置接近归零后反馈（误差不超过 0.02 rad），速度和前馈力矩绝对值不超过 0.1，并满足配置限制。之后逐渐过渡到算法目标。

算法建议先按约 200 Hz 更新目标。默认目标有效期 50 ms；算法计算或休眠不能长期阻断更新。服务保持最新有效目标，不自动插值，旧目标不会排队依次执行。

高级调用 `submit(joints, timestamp=..., sequence=...)` 可显式指定同机 `time.monotonic()` 时间戳和严格递增序号；一般省略，由 SDK 自动生成。不要使用 `time.time()`。

## 5. 算法程序结构

下面是接入结构，`your_algorithm` 由算法组实现，增益需使用双方确认的配置。先在模拟环境验证。

```python
import time
from mit_arm_control import ArmClient
from your_algorithm import initial_targets, compute_targets

with ArmClient() as client:
    try:
        state = client.arm()  # 自动归零；调用前已摆好约定姿态
        # 返回全部启用轴静止目标，位置使用 state['feedback'][name]['q']。
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
| `reason` | 最近状态原因；当前是否故障以 `state` 判断 |
| `feedback[name].q / dq / tau` | 位置、速度、反馈估计力矩；力矩不是外力传感器读数 |
| `feedback[name].timestamp / age_ms` | 样本时间与年龄；连续读取可能得到同一份样本，各轴不是严格同步采样 |
| `feedback[name].mos_temperature / rotor_temperature` | 驱动及电机温度，℃ |
| `feedback[name].status` | 驱动状态，0 为失能、1 为使能；其他状态交由服务处理 |
| `stop_confirmed` | `True` 已确认失能；`False` 未确认；`None` 尚未执行需确认的停止 |
| `configuration_errors` | 尚未满足的运动配置条件 |

缺少某轴反馈时不能自行补零。故障后停止算法更新，记录 `reason`；处理原因后调用 `reset_fault()`，重新摆好启动姿态，再 `arm()`。不要编写自动复位、自动恢复运动的无限重试循环。

## 7. 启动与读取的区别

`arm()` 是有副作用的启动操作：读参数并校验、确认失能和静止、设置并验证零点、确认控制模式；成功后返回 `READY`，首条有效静止目标才触发使能。每次工作调用一次，不放在算法循环中。

`get_state()` 只读取服务维护的最新快照，可以在算法循环中重复调用，不重新归零、不使能、不重新执行启动检查。

默认服务支持运动接口，无额外实机验收开关；连接服务不会自动使能。`--observe-only` 仅用于测试，实机仍须显式指定 `--hardware`。每次 `arm()` / `observe()` 的电机设零行为保留，反馈角度以本次电机零点为基准。
