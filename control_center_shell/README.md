# Autolife Robot Control Center

综合控制台由四个独立 ROS 2 功能包组成：

- `control_center_shell`：浏览器式主界面和模块调度。
- `control_center_navigation`：导航、航点、地图编辑和全局重定位。
- `control_center_slam`：优化 Cartographer 建图。
- `control_center_joint_control`：全身关节监控、姿态和动作控制。

## 启动

```bash
cd "/home/ubuntu/ros2_ws"
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch control_center_shell control_center.launch.py
```

导航默认启动。建图和全身关节控制仅在界面中点击“启动模块”后运行。
导航和建图互斥；切换建图时控制台会先停止导航，避免 AMCL 与
Cartographer 重复发布 `map -> odom`。退出控制台时会恢复启动前正在运行的
`gv-slam-service.service`。

启动前还会检查手动遗留的 Cartographer、AMCL 和旧版关节控制进程。检测到
冲突时只提示并拒绝启动对应模块，不会自动结束用户手动启动的程序。

## 独立启动

```bash
ros2 launch control_center_navigation bringup_all.launch.py
ros2 launch control_center_slam multi_lidar_slam_optimized.launch.py
ros2 launch control_center_joint_control whole_body_joint_control.launch.py topic_suffix:=0_306
```
