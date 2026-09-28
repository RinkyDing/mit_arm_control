# 离线与进程集成测试

从项目根目录执行：

```bash
PYTHONPATH=src python3 -m unittest discover -s tests -v
```

- test_core.py：虚拟时间、配置/协议/状态机/限幅/背压/失败停止/日志阻塞。
- test_transport.py：mock SocketCAN，检查 FD+BRS 打包、接收时间戳和参数反馈分流，不创建真实 CAN socket。
- test_service.py：启动真实 Python 服务和 Unix socket，但后端始终 SimBackend；验证 SDK、控制权、进程崩溃、信号退出和最终报告。

测试需要 Linux 本机 Unix socket 权限。测试进程清理只针对自己启动的子进程。模拟不是经过辨识的机械臂动力学模型；通过测试不能作为无支撑实机运行许可。
