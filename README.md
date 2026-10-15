# Autolife Robot Control Center Suite

该目录是一个可独立迁移的 ROS 2 多功能包套件，顶层故意不放 `package.xml`。七个子功能包
由 `colcon` 分别发现和构建，互不合并源码：

```text
autolife_robot_control_center/
├── control_center_shell/
├── control_center_navigation/
├── control_center_slam/
├── control_center_joint_control/
├── control_center_service_manager/
├── multi_lidar_slam/
└── text_voice_control/
```

构建与运行：

```bash
cd "/home/ubuntu/ros2_ws"
source /opt/ros/jazzy/setup.bash
colcon build --packages-select \
  text_voice_control \
  multi_lidar_slam \
  control_center_navigation \
  control_center_slam \
  control_center_joint_control \
  control_center_service_manager \
  control_center_shell
source install/setup.bash
ros2 launch control_center_shell control_center.launch.py
```

迁移时只需完整复制 `autolife_robot_control_center/` 目录；不要再从工作空间顶层单独寻找
`multi_lidar_slam` 或 `text_voice_control`。

导航是默认主页和默认运行模块；建图、关节控制和系统服务管理按需启动。
建图开始前会停止导航，避免 AMCL 与 Cartographer 同时发布 `map -> odom`。

迁移到其他机器人编号时，先阅读根目录的
[AI_MIGRATION_GUIDE.md](AI_MIGRATION_GUIDE.md)。
