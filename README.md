# MIT 机械臂控制框架

Python 控制服务与算法 SDK，六关节加夹爪，不使用 ROS。首版仅用于**有可靠机械支撑的联调**；软件失能确认不是制动、静止确认或机械安全认证。

本项目独立于旧测试仓库。默认使用模拟后端，导入包、连接 SDK 和启动服务均不会使能电机。未知的实机 ID、机械限位、方向和零姿态保留为空；实机模板不能直接运行运动。

## 快速开始：完全模拟

Python 3.10+，Linux。运行时仅使用标准库，不必先安装依赖。在项目目录启动服务：

```bash
cd /home/rinky/damiao_ws/mit_arm_control
PYTHONPATH=src python3 -m mit_arm_control serve \
  --config configs/simulation.json \
  --socket /tmp/mit-arm-control.sock \
  --final-state /tmp/mit-arm-final-state.json
```

另一个终端运行算法示例：

```bash
cd /home/rinky/damiao_ws/mit_arm_control
PYTHONPATH=src python3 examples/sim_algorithm.py --duration 5
```

示例算法约 200 Hz 更新目标，控制服务独立按每轴 1 kHz 目标调度；示例会拒绝连接实机后端。结束时显示失能确认；服务按 Ctrl+C 退出，并保存最终报告。不要同时运行旧电机控制程序。

也可在自己的虚拟环境安装 `python3 -m pip install -e .`，安装后使用 `mit-arm` 命令。构建工具需要 setuptools；直接 `PYTHONPATH=src` 方式无需联网。

## 项目结构

```text
src/mit_arm_control/
  protocol.py     MIT 编解码、参数/状态分流
  backend.py      SocketCAN 和可注入故障的模拟后端
  config.py       配置与实机就绪检查
  safety.py       命令、变化率、反馈及估算力矩约束
  controller.py   状态机、归零、调度、背压、失能确认
  ipc.py          有界本机 IPC、控制权、非阻塞日志
  service.py      单一 CAN 拥有者与进程生命周期
  sdk.py          算法组 Python 接口
configs/          模拟配置、不可直接使能的实机模板
examples/         模拟算法示例
tests/           离线与模拟进程集成测试
docs/            接口、配置、安全边界、来源和改动报告
```

## 文档

- [算法接口与状态机](docs/API.md)
- [配置、受支撑联调与故障边界](docs/COMMISSIONING.md)
- [架构、改动与验证报告](docs/REPORT.md)
- [协议来源](docs/PROVENANCE.md)

## 测试

```bash
PYTHONPATH=src python3 -m unittest discover -s tests -v
```

测试从不打开实机 CAN。进程集成测试需要系统允许本机 Unix socket 和启动子进程。模拟通过不能证明七轴实机 1 kHz、抱持能力或机械限位正确。
