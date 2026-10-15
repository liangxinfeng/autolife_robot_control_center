# multi_lidar_slam

## 功能说明

`multi_lidar_slam` 用于双雷达 Cartographer 建图，当前默认使用：

- 前雷达话题：`/topic_gv_front_lidar_0_300`
- 后雷达话题：`/topic_gv_rear_lidar_0_300`
- 里程计话题：`/topic_gv_wheel_odom_0_300`

启动后会拉起：

- 前后雷达到 `base_link` 的静态 TF
- `cartographer_node`
- `cartographer_occupancy_grid_node`
- `rviz2`（默认开启）

## 编译

```bash
source /opt/ros/jazzy/setup.bash
cd /home/ubuntu/ros2_ws
colcon build --symlink-install --base-paths src/autolife_robot_control_center/multi_lidar_slam
source install/setup.bash
```

## 启动建图

```bash
source /opt/ros/jazzy/setup.bash
cd /home/ubuntu/ros2_ws
source install/setup.bash
ros2 launch multi_lidar_slam multi_lidar_slam.launch.py
```

如果不想自动打开 `rviz2`：

```bash
ros2 launch multi_lidar_slam multi_lidar_slam.launch.py open_rviz:=false
```

## 建图前检查

建议先确认下面几个话题有数据：

```bash
ros2 topic list
ros2 topic hz /topic_gv_front_lidar_0_300
ros2 topic hz /topic_gv_rear_lidar_0_300
ros2 topic hz /topic_gv_wheel_odom_0_300
```

如果后雷达没有数据，Cartographer 通常会一直等待 `scan_2`，导致地图不更新。

## RViz 使用

启动后在 `rviz2` 中建议检查：

1. `Fixed Frame` 是否为 `map`
2. 激光是否正常显示
3. 机器人移动时地图是否持续扩展
4. 轨迹是否平滑，没有明显跳变

## 保存地图

建图完成后，建议同时保存两份：

- `pbstream`：Cartographer 原始状态文件，后续还能继续处理
- `pgm + yaml`：导航直接可用地图

### 1. 结束当前轨迹

```bash
ros2 service call /finish_trajectory cartographer_ros_msgs/srv/FinishTrajectory "{trajectory_id: 0}"
```

### 2. 保存 `pbstream`

```bash
ros2 service call /write_state cartographer_ros_msgs/srv/WriteState "{filename: '/home/ubuntu/ros2_ws/src/autolife_robot_control_center/multi_lidar_slam/maps/my_first_map.pbstream', include_unfinished_submaps: true}"
```

### 3. 保存导航地图 `pgm + yaml`

```bash
ros2 run nav2_map_server map_saver_cli -f /home/ubuntu/ros2_ws/src/autolife_robot_control_center/multi_lidar_slam/maps/map
```

保存后会得到：

- `my_first_map.pbstream`
- `my_first_map.pgm`
- `my_first_map.yaml`

当前默认地图目录：

```bash
/home/ubuntu/ros2_ws/src/autolife_robot_control_center/multi_lidar_slam/maps
```

## 后续导航使用

如果要给导航包使用保存好的地图，通常使用：

- `my_first_map.yaml`
- `my_first_map.pgm`

其中：

- `yaml` 提供地图元信息
- `pgm` 是实际占用栅格图

## 常见问题

### 1. 启动后地图不更新

常见原因：

- 后雷达 `scan_2` 没有数据
- 里程计没有数据
- TF 不完整

### 2. 提示一直等待 `scan_2`

说明后雷达话题虽然被配置了，但当前没有实际消息流。

重点检查：

```bash
ros2 topic echo /topic_gv_rear_lidar_0_300 --once
```

### 3. 保存地图失败

先确认以下节点是否在线：

```bash
ros2 node list
```

应至少看到：

- `/cartographer_node`
- `/cartographer_occupancy_grid_node`

### 4. 目标机器人编号不是 283

如果移植到新机器人后话题变成 `284`，需要同步修改 launch 文件中的：

- `/topic_gv_front_lidar_0_300`
- `/topic_gv_rear_lidar_0_300`
- `/topic_gv_wheel_odom_0_300`

## 最常用命令

启动建图：

```bash
ros2 launch multi_lidar_slam multi_lidar_slam.launch.py
```

结束轨迹：

```bash
ros2 service call /finish_trajectory cartographer_ros_msgs/srv/FinishTrajectory "{trajectory_id: 0}"
```

保存 `pbstream`：

```bash
ros2 service call /write_state cartographer_ros_msgs/srv/WriteState "{filename: '/home/ubuntu/ros2_ws/src/autolife_robot_control_center/multi_lidar_slam/maps/my_first_map.pbstream', include_unfinished_submaps: true}"
```

保存导航地图：

```bash
ros2 run nav2_map_server map_saver_cli -f /home/ubuntu/ros2_ws/src/autolife_robot_control_center/multi_lidar_slam/maps/my_first_map
```
