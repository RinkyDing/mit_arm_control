# 协议与已有设计来源

MIT 位布局、DM4310_48V/DM4340_48V 的量程、0x7FF 参数协议和 0xFC/FD/FE 控制指令参考用户已有仓库：

- 上游：https://github.com/dmBots/motor-control-routine
- 本机：`/home/rinky/damiao_ws/src/motor-control-routine/SocketCan控制例程/Python例程/damiao_socketcan.py`
- 既有启动验证、退出确认与背压经验来自同目录的 `motor_zeroing.py`、`mit_test_common.py` 和 `mit_three_motor_test.py`。

本项目重新分离纯编码、被动构造的通信后端、显式生命周期和算法 IPC，不导入或更改旧项目。保留了8字节 MIT格式、逐台参数回复验证、静止/归零验证、有界失能重试及“历史收发差不占用发送额度”的原则。

原目录未发现覆盖 Python 驱动的明确许可证文本。本项目不擅自为参考实现声明新的第三方授权；若计划公开分发，先核实上游适用许可证与厂商协议使用条件。第三方资料不构成对本项目机械安全或固件兼容性的认证。
