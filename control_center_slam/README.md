# control_center_slam

该包从 `multi_lidar_slam` 独立复制而来，旧包保持不变。新包面向 306 机器人，
使用前后两路原始 0.5° LaserScan、轮式里程计和 Cartographer 2D SLAM。

## 旧包问题诊断

现场检查确认旧包存在以下问题：

1. launch 和融合节点仍写死机器人 `283` 的雷达与里程计话题，306 机器人启动后
   Cartographer 无法接收正确输入。
2. 系统已经发布标定后的 `base_link -> front/rear_laser_frame`，旧包又发布一套
   同名 TF；旧值高度为 0.200 m，而系统标定值为 0.1515 m，并带约 0.248°修正。
   重复且不一致的 TF 会造成墙体加厚、重影和闭环不一致。
3. 两只雷达时间戳最近相差约 35 ms，必须保留逐点时间戳并由 Cartographer 合并；
   不能改用只有约 1°分辨率的系统合并扫描。
4. 自适应体素最大尺寸 0.20 m，点云过度稀疏，不利于 0.05 m 栅格的精细建图。
5. 每 90 个节点才执行一次位姿图优化，回环最低分设置为 0.65，长期建图时回环发现和
   漂移纠正过慢。
6. Cartographer 位姿以 5 ms 周期发布，达到 200 Hz，超过实际需要并增加 CPU/DDS 压力。
7. 位姿图里程计权重只有 200，远低于 Cartographer 默认量级，长轨迹缺少稳定约束。

## 主要优化

- 默认使用 306 话题，可通过 launch 参数迁移到其他编号。
- 默认发布 306 已标定的雷达静态 TF，保证 SLAM 单独启动时也能建图。
- 两路原始雷达直接输入 Cartographer，不使用低分辨率合并扫描。
- 保持 `num_accumulated_range_data = 2`，将前后两路数据组成一次匹配观测；
  实测设为 1 会导致两只雷达交替单独插图并快速产生大量子图。
- 子图/占用地图分辨率从 0.05 m 提升到 0.03 m，体素尺寸为 0.025 m，
  自适应体素最大长度为 0.06 m。
- 室内有效量程限制为 12 m，减少远距离玻璃反射和稀疏噪声进入匹配。
- 在线相关匹配限制为 0.15 m / 12°并提高位姿变化惩罚，避免长走廊中逐帧游走。
- 每 20 个节点优化一次位姿图，以 0.7 采样率搜索 8 m 内回环；普通/全局最低分为
  0.60 / 0.65，兼顾闭环发现和重复房间中的误匹配抑制。
- 厂商轮速里程计协方差为 `-1`（未知），因此只保留为平滑先验；降低全局图中的
  里程计权重，让高质量雷达回环可以纠正长期轮滑累计误差。
- 后台线程增加到 8，位姿发布降低到 50 Hz。

参数文件：

```text
config/multi_lidar_optimized.lua
```

## 编译

当前机器人工作空间目录名末尾有一个空格：

```bash
cd '/home/ubuntu/ros2_ws'
source /opt/ros/jazzy/setup.bash

colcon build \
  --base-paths src/control_center_slam \
  --packages-select control_center_slam \
  --symlink-install

source install/setup.bash
```

## 启动

完整启动并打开 RViz：

```bash
ros2 launch control_center_slam \
  control_center_slam.launch.py
```

无界面测试：

```bash
ros2 launch control_center_slam \
  control_center_slam.launch.py \
  open_rviz:=false
```

## 建图键盘控制

键盘控制默认随 SLAM 一起启动，直接控制 306 底盘目标速度：

```text
W/S：前进/后退
A/D：左转/右转
Q/E：左平移/右平移
Z/C：降低/提高速度
X 或空格：立即停车
H：显示帮助
```

默认前后速度为 0.18 m/s、旋转速度为 0.45 rad/s，按住移动，松键停车。该速度范围
专门用于低速建图，减少快速运动造成的激光拖影。桌面终端启动时使用按下/松开监听；
通过 SSH 单独运行时使用终端兼容模式：

```bash
ros2 run control_center_slam mapping_keyboard_teleop.py
```

如果不需要键盘控制，或已有其他程序在控制底盘：

```bash
ros2 launch control_center_slam \
  control_center_slam.launch.py \
  enable_keyboard_control:=false
```

迁移到其他机器人时可覆盖输出话题：

```bash
ros2 launch control_center_slam \
  control_center_slam.launch.py \
  keyboard_cmd_vel_topic:=/目标底盘速度话题
```

长按平滑性使用 30 Hz 持续发布，并对 X11 自动重复产生的瞬时松键事件做 80 ms
消抖。真实松键通常在 80～115 ms 内停车，`X` 或空格不经过消抖、仍然立即停车。

## RViz 一键优化与保存

RViz 左侧包含“地图优化与保存”面板，提供两种保存方式：

1. 在“地图名称”中填写名称，可使用中文、字母、数字、下划线和连字符。
2. 建图中途点击“保存当前地图（继续建图）”，保存当前快照且不会结束轨迹。
3. 全部建图完成后点击“最终优化并保存（结束建图）”。
4. 面板自动保存 pbstream 并生成 YAML/PGM，完成状态会显示完整路径。

标准 Cartographer 没有“对活动轨迹执行最终优化但保持同一轨迹继续”的服务。
普通保存使用 `include_unfinished_submaps=true`，配合每 20 个节点执行的在线位姿图优化，
可以不中断建图；最终优化仍然只能在结束轨迹时执行。

文件自动保存到本功能包 `maps` 目录：

```text
/home/ubuntu/ros2_ws/src/control_center_slam/maps/

<名称>.pbstream
<名称>.yaml
<名称>.pgm
```

RViz 面板优先识别工作空间中的源码包目录；只有部署环境不包含 `src` 源码包时，才会
回退到安装空间的 `share/control_center_slam/maps`。

RViz 默认隐藏红色/绿色的两路原始 LaserScan 调试层，改为显示白色
`/scan_matched_points2`。原始点在 `map` 固定坐标系中会受到实时位姿修正、两雷达约
35 ms 相位差以及历史点残留影响，看起来像“乱飘”，它们不是已经写入地图的栅格点。
需要排查传感器时可手动启用 `Front/Rear Raw Laser (debug)`；其残留时间已设为 0，
不会叠加旧帧拖影。

## 控制中心建图界面

通过 `control_center_shell` 打开的建图模块具有以下显示与编辑能力：

- 红色/绿色雷达点按照每帧 LaserScan 的采集时间查询 TF 后投影到 `map`，避免机器人
  运动时使用最新 TF 造成点云与栅格错位。激光帧会短暂等待对应时刻的 TF，由定时器
  异步投影，不会阻塞 ROS 回调或拖慢界面。
- 蓝线记录机器人本次建图的行驶轨迹；发生较大的回环位姿修正时自动断开线段，不绘制
  一条横跨地图的错误连接线。
- 参数与保存区域使用深色主题。
- “最终优化并保存”成功后，可以切换“描绘障碍”或“擦除障碍”，设置笔刷尺寸，
  并支持撤销、重做和还原。
- “覆盖当前已保存地图”会覆盖本次最终保存生成的同名 PGM/YAML；覆盖前会在同一目录
  自动生成带时间戳的 `.bak` 备份。PBStream 不会修改。

地图仍在活动建图时不会开放编辑，防止后续 `/map` 更新覆盖人工修改；中途快照保存也
不会开放编辑。

单独启动该 SLAM 包时保持默认值：

```text
publish_lidar_static_tf:=true
```

如果确认另一个正在运行的底盘/传感器进程已经发布同名雷达 TF，必须关闭包内发布，
避免同一个子坐标系存在两个发布者：

```bash
ros2 launch control_center_slam \
  control_center_slam.launch.py \
  publish_lidar_static_tf:=false
```

## 建图操作建议

- 启动后先静止 3～5 秒。
- 速度保持在 0.15～0.30 m/s，转弯角速度尽量低于 0.4 rad/s。
- 优先走闭合路线，不要只沿同一方向不断延伸。
- 每进入一个新区域后，回到之前经过且几何特征明显的位置形成回环。
- 经过长走廊时不要快速原地旋转，转角处停留约 1 秒。
- 完成整圈后再走一小圈复核，等待位姿图优化后再保存。

## 保存地图

先结束轨迹：

```bash
ros2 service call /finish_trajectory \
  cartographer_ros_msgs/srv/FinishTrajectory \
  "{trajectory_id: 0}"
```

保存 Cartographer 状态：

```bash
ros2 service call /write_state \
  cartographer_ros_msgs/srv/WriteState \
  "{filename: '/home/ubuntu/ros2_ws/src/control_center_slam/maps/optimized_map.pbstream', include_unfinished_submaps: false}"
```

生成导航地图：

```bash
ros2 run nav2_map_server map_saver_cli \
  -f '/home/ubuntu/ros2_ws/src/control_center_slam/maps/optimized_map' \
  --ros-args -p map_subscribe_transient_local:=true
```

## 检查

```bash
ros2 topic hz /topic_gv_front_lidar_0_306
ros2 topic hz /topic_gv_rear_lidar_0_306
ros2 topic hz /topic_gv_wheel_odom_0_306
ros2 topic hz /map

ros2 run tf2_ros tf2_echo base_link front_laser_frame
ros2 run tf2_ros tf2_echo base_link rear_laser_frame
```

正常情况下前后雷达约 10 Hz、轮式里程计约 62.5 Hz，地图持续更新。

## 306 静止测试结果

2026-07-29 在 306 机器人上进行约 42 秒无界面测试：

- Cartographer 和占用栅格节点均正常启动、正常退出，无 `ERROR`/TF 外推错误；
- 修正占用栅格节点参数传递后，`/map` 分辨率为 0.04 m、约 2 Hz；
- 子图 0 的版本从 9 增到 21，约 0.4 个节点/秒，没有再出现前后雷达交替
  单独插图导致的节点暴增；
- 测试前后 `map -> odom` 均为 0 m / 0°，静止阶段未观察到漂移；
- Cartographer 进程约占用 0.4 个 CPU 核（`ps` 中约 40%，不是整机 40%）。

后续现场发现系统静态 TF 发布进程并非始终存在；当其未运行时，Cartographer 会收到
雷达话题但 `/submap_list` 始终为空，RViz 显示 `No map received`。因此本包最终改为
默认发布经过核对的 306 雷达 TF。

日志中的 `Dropped ... earlier points` 来自两只雷达约 35 ms 相位差、不同单帧扫描时长
以及逐点时间窗口重叠。两路话题均确认只有一个发布者。这类警告不等同于 TF 或 SLAM
失败；不要通过伪造/重置传感器时间戳来消除日志，否则机器人运动时会失去正确的逐点
运动补偿。若警告在实车运动时持续每帧出现，应优先检查雷达驱动硬件时间同步。

## 重要限制

参数优化可以减少局部匹配误差和累计漂移，但不能替代真实外参标定和闭环路线。
最终精度必须通过机器人实际行驶完整闭环后比较墙体重影、起终点误差和回环日志确认。
