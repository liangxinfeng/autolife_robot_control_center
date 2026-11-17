# Autolife Robot Control Center Suite

345 的导航和当前建图包已独立到 `ros2_ws/src`；控制中心通过 ROS 包索引和声明依赖加载它们。

```text
ros2_ws/src/
├── control_center_navigation/       导航、航点、导航地图目录
├── control_center_slam/             当前使用的 Cartographer 建图
└── autolife_robot_control_center/
    ├── control_center_shell/
    ├── control_center_chassis_guard/
    ├── control_center_joint_control/
    ├── control_center_service_manager/
    └── text_voice_control/
```

构建与运行：

```bash
cd "/home/ubuntu/ros2_ws"
source /opt/ros/jazzy/setup.bash
colcon build --packages-select \
  text_voice_control \
  control_center_navigation \
  control_center_slam \
  control_center_joint_control \
  control_center_service_manager \
  control_center_shell
source install/setup.bash
ros2 launch control_center_shell control_center.launch.py
```

迁移时需一起复制 `autolife_robot_control_center/`、独立的 `control_center_navigation/`
和 `control_center_slam/`。VR 导航另外复制 `openarmx_teleop_vr_navigation_306/`。
`text_voice_control` 保持在控制中心套件内。旧 `multi_lidar_slam` 已删除，
其地图统一位于 `control_center_navigation/maps/`，无需再构建旧包。

导航是默认主页和默认运行模块；建图、关节控制和系统服务管理按需启动。
建图开始前会停止导航，避免 AMCL 与 Cartographer 同时发布 `map -> odom`。

迁移到其他机器人编号时，先阅读根目录的
[AI_MIGRATION_GUIDE.md](AI_MIGRATION_GUIDE.md)。
