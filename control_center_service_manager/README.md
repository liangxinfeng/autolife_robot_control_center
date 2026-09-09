# control_center_service_manager

综合控制台内的轻量级 systemd 用户服务管理界面，不依赖 ROS 节点。

主要功能：

- 管理视觉、SLAM、导航控制和机械臂四个核心服务；
- 启动、停止、重启、查看状态、启用或禁用开机自启；
- 查看全部运行中或已启用的用户服务；
- 查看最近日志并按需实时跟踪日志；
- 启动导航组合时按 `gv-slam-service.service`、`gv-control-service.service` 顺序启动。
- 提供带二次确认的“全身关节复位”按钮，调用厂家 `/control_reset_<后缀>` 官方默认位置；
  不读取姿态库，也不自行发送关节角度。

集成控制中心会在导航、建图或全身关节模块运行时拒绝复位。执行前还必须停止 VR、
抓取及其它动作源，并确认机器人站立稳定、没有负载且周围无人。

所有外部命令均通过程序名和参数列表执行，不经过 Shell，也不接受任意命令文本。

独立运行：

```bash
ros2 run control_center_service_manager service_manager
```
