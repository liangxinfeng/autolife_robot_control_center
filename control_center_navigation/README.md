# control_center_navigation

从 `multi_lidar_nav_amcl_waypoint_teleop_global_relocalization` 复制并独立演进的无 RViz2 导航功能包。

保留原包的 AMCL、Nav2、自动全局重定位、航点、充电桩、语音、底盘仲裁和键盘控制逻辑，并新增与 `control_center_joint_control` 风格一致的原生 Qt 导航界面。

## 原生界面

- 直接显示 `/map` 二维栅格地图。
- 以 30Hz `map → base_link` TF 实时显示机器人本体位置与朝向，保留 AMCL 协方差并在 TF 不可用时自动回退。
- 显示前后激光、全局/局部路径、蓝色航点名称和航点朝向箭头。
- 地图拖动与滚轮缩放。
- 地图障碍描绘与擦除，支持可调圆形笔刷、撤销、重做和丢弃修改。
- 编辑结果只允许另存为新地图，旧地图禁止覆盖；地图库可选择原地图或编辑地图并交给 `map_server` 动态加载。
- 在地图上按下并沿朝向拖动，可设置 Nav2 导航目标或 AMCL 初始位姿。
- 显示全局重定位阶段、匹配次数和得分，可一键重新定位。
- 航点保存、刷新、导航、重命名和删除；多航点导航支持重复加入、拖动排序、逐点停留时间和循环执行。
- 界面按钮及 W/S/A/D/Q/E 键控制全向底盘，支持前进/后退、转向和横移组合按键；人工速度由独立 50Hz 心跳线程平滑发送。
- 激光订阅只保留最新扫描并以高频批量处理 ROS 回调，避免显示旧点云造成延迟。
- 关闭界面、失去焦点或点击急停时主动发布零速度。

编辑生成的 `.yaml` 与 `.pgm` 默认保存在本功能包源码的 `maps/` 目录。加载另一张地图前必须停止当前导航，加载成功后界面会自动重新执行全局定位。

该包的编译、启动和运行依赖中均不包含 RViz2。

## 编译

当前工作空间路径末尾包含空格，必须完整引用：

```bash
cd "/home/ubuntu/ros2_ws"
source /opt/ros/jazzy/setup.bash

colcon build \
  --packages-select control_center_navigation

source install/setup.bash
```

## 启动完整系统

```bash
ros2 launch control_center_navigation \
  bringup_all.launch.py
```

默认行为：

- 自动打开原生导航界面。
- 自动执行全局重定位。
- 定位稳定后激活 Nav2。
- 启动航点管理和语音反馈。
- 启动前方近距离障碍/行人避让语音提醒。
- 旧的全局键盘监听及 Marker 状态浮层默认关闭，避免与原生界面重复。

## 前方避让语音提醒

仅当 `/navigate_to_pose` 存在活动导航目标时，`proximity_voice_alert` 才读取前雷达
并允许播报；待机、到达、取消或失败状态下不会触发。306 前雷达的机器人正前方对应
LaserScan 的 ±180°；节点默认在该方向 ±40°、0.8 m 内
连续 3 帧检测到至少 3 个有效激光点时，向 `/text_voice_command` 发布：

```text
前方有人或障碍物，请注意避让
```

同一目标持续存在时最多每 15 秒重复一次；距离恢复到 0.95 m 之外并连续确认后重新
待命。参数位于 `config/proximity_voice_alert.yaml`。需要临时关闭时：

```bash
ros2 launch control_center_navigation bringup_all.launch.py \
  enable_proximity_voice_alert:=false
```

## 常用启动参数

不打开界面，只启动后台导航：

```bash
ros2 launch control_center_navigation \
  bringup_all.launch.py open_navigation_ui:=false
```

禁用自动全局重定位：

```bash
ros2 launch control_center_navigation \
  bringup_all.launch.py enable_global_relocalization:=false
```

计算全局位置但不写入 AMCL：

```bash
ros2 launch control_center_navigation \
  bringup_all.launch.py \
  global_relocalization_publish_initial_pose:=false
```

单独启动原生界面：

```bash
ros2 run control_center_navigation \
  navigation_ui_node.py
```

界面操作说明见 [USAGE.md](USAGE.md)，跨机器人迁移的基础事项继续参考继承的 [AI_MIGRATION_GUIDE.md](AI_MIGRATION_GUIDE.md)。
