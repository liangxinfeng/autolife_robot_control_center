> 345 于 2026-10-19 调整布局：`control_center_navigation` 和当前使用的 `control_center_slam`
> 位于 `ros2_ws/src/` 顶层，需与控制中心一起复制、构建；旧 `multi_lidar_slam` 已删除，历史地图位于 `control_center_navigation/maps/`。
> ROS 包名、控制中心与 VR 的启动入口保持不变。当前目录结构以 README.md 为准。

# Autolife Robot Control Center 跨机器人迁移指南（AI 阅读版）

适用于同型号 AutoLife S2（`robot_v2_2`）之间迁移，例如从 306 迁移到 283/300。
只记录目标机器人相关改动、依赖和验收；不要根据本文件重写已有功能。

## 1. 先确定三个变量

```text
<ID>      目标机器人编号，例如 300
<DOMAIN>  ROS_DOMAIN_ID，通常为 0
<SUFFIX>  <DOMAIN>_<ID>，例如 0_300
```

必须先从目标机器人实际话题和用户服务确认，不能只按编号猜测：

```bash
ros2 topic list | grep -E 'lidar|wheel_odom|battery|whole_body|control_reset'
systemctl --user cat arm-control-service.service | \
  grep -E 'ROBOT_ID|ROS_DOMAIN_ID|RMW_IMPLEMENTATION|CYCLONEDDS_URI'
systemctl --user status vision-service.service arm-control-service.service
```

当前套件默认 DDS 为 CycloneDDS、Domain 0、回环接口 `lo`。如果厂家节点不在同一台主机，
不能沿用 `lo`，必须改成实际网卡并验证跨机发现。

## 2. 复制前保留运行数据

迁移时不要被空模板覆盖：

```text
control_center_navigation/config/waypoints.yaml       航点
control_center_navigation/maps/                       导航编辑地图
control_center_slam/maps/                             建图结果和备份
control_center_joint_control/motion_library/          姿态、动作 YAML 和 PKL
```

航点坐标依赖具体地图。目标机器人使用不同地图时，只能重新采集航点，不能直接沿用旧坐标。

## 3. 编号修改清单

### 控制中心入口

修改 `control_center_shell/launch/control_center.launch.py` 默认值：

```text
robot_id: 306 -> <ID>
topic_suffix: 0_306 -> <SUFFIX>
```

`control_center_shell/control_center_shell/app.py` 中的 306 只是直接运行时的后备默认值，
也建议同步修改。若 `<DOMAIN>` 不是 0，还要修改其中传给导航的
`text_voice_ros_domain_id`。

### 导航模块

在以下运行文件中将 `_0_306` 改为 `_<SUFFIX>`，并把机器人 ID 306 改为 `<ID>`：

```text
control_center_navigation/config/nav2_params.yaml
control_center_navigation/config/global_relocalization.yaml
control_center_navigation/config/docking_params.yaml
control_center_navigation/config/voice_feedback.yaml
control_center_navigation/config/proximity_voice_alert.yaml
control_center_navigation/launch/bringup_all.launch.py
control_center_navigation/launch/bringup_localization.launch.py
control_center_navigation/launch/bringup_navigation.launch.py
```

同时核对：

- `map_yaml` 指向目标机器人实际要用的地图。
- 电池、雷达、里程计、底盘输出话题真实存在。
- `base_link -> front_laser_frame/rear_laser_frame` 外参来自目标机器人标定。
- 不允许厂家节点和本包同时发布同名雷达静态 TF。
- `proximity_voice_alert.yaml` 的 `forward_angle_deg` 必须实测。306 为 180°，其他机器人
  不一定相同；错误时会导致站在正前方也不触发提醒。

脚本和 C++ 中仍有 306 的后备默认话题；正常 launch 会用上述参数覆盖。若需要单独
`ros2 run` 某节点，再同步修改对应源码默认值。

### 建图模块

综合控制中心会按 `<SUFFIX>` 传入前/后雷达和里程计话题。仍需核对：

```text
control_center_slam/launch/multi_lidar_slam_optimized.launch.py
```

其中前后雷达的 x/y/z/roll/pitch/yaw 是 306 标定值，不能直接当成所有机器人通用值。
单独启动 SLAM 时，还要覆盖 `front_scan_topic`、`rear_scan_topic`、`odom_topic` 和
`keyboard_cmd_vel_topic`。

### 全身关节模块

厂家 `arm-control-service.service` 的 `ROBOT_ID`、`ROS_DOMAIN_ID` 必须与 `<ID>`、
`<DOMAIN>` 一致；控制中心启动参数使用 `<SUFFIX>`。如果目标机不是 `robot_v2_2`，
必须更换 URDF、mesh、关节名称/顺序和限位，禁止直接控制真机。

首次验收只读反馈，不要立即解锁：

```bash
ros2 topic echo --once \
  /topic_arm_whole_body_and_gripper_current_joints_status_<SUFFIX>
ros2 topic info /control_reset_<SUFFIX>
```

### 系统服务模块

确认以下用户服务在目标机存在：

```text
vision-service.service
gv-slam-service.service
gv-control-service.service
arm-control-service.service
```

并核对厂家设置路径：

```text
/home/ubuntu/miniconda3/envs/robot_env/lib/python3.12/site-packages/
autolife_robot_arm/settings.toml
```

Python 环境版本或安装位置不同，需要修改
`control_center_service_manager/control_center_service_manager/panel.py` 中的路径。
“全身关节复位”按钮会发布到 `/control_reset_<SUFFIX>`；目标机器人必须存在厂家订阅者，
且控制中心入口传入的 `topic_suffix` 必须正确。

## 4. 依赖与编译

控制中心目录现在包含七个 ROS 子包。导航模块会在启动文件解析阶段直接查找其中两个依赖包：

```text
text_voice_control   语音播报与语音命令
control_center_navigation  导航配置及统一地图目录（含 yundie_2f）
```

迁移时必须完整复制整个 `autolife_robot_control_center/`，不要只复制五个界面子包。
缺少任意一个时，`bringup_all.launch.py` 会在 `map_server` 创建前退出，界面表现通常只是
一直“等待 /map 地图”。必须查看 launch 日志，不能误判为地图话题或 QoS 故障。

```bash
sudo apt update
sudo apt install -y \
  ros-jazzy-rmw-cyclonedds-cpp \
  ros-jazzy-cartographer-ros \
  ros-jazzy-navigation2 ros-jazzy-nav2-bringup \
  ros-jazzy-opennav-docking ros-jazzy-robot-state-publisher \
  python3-pyqt5 python3-pyqt5.qtopengl python3-opengl \
  python3-numpy python3-yaml python3-pynput

cd '<WS>'
source /opt/ros/jazzy/setup.bash
rosdep install --from-paths \
  src/autolife_robot_control_center \
  --ignore-src -r -y

# 用 tar/zip 中转后必须恢复 launch 所调用 Python 节点的可执行权限。
chmod 755 src/control_center_navigation/scripts/*.py

colcon build --packages-select \
  text_voice_control \
  control_center_navigation control_center_slam \
  control_center_joint_control control_center_service_manager \
  control_center_shell
source install/setup.bash
```

若目标机提示 `rosdep installation has not been initialized yet`，说明该机器人从未初始化
`rosdep`，不是功能包错误。可先执行一次：

```bash
sudo rosdep init
rosdep update
```

若现场网络不适合初始化 `rosdep`，先根据各子包 `package.xml` 逐项核对，并至少安装本套件
实际缺少的 `ros-jazzy-cartographer-ros`；不要因此顺带安装与本控制中心无关的 MoveIt、
pick_ik 等大型依赖。

注意：306 当前工作空间名 `/home/ubuntu/ros2_ws` 末尾有空格；目标机路径以实际为准，
带空格时所有 shell 路径都必须加引号。

## 5. 安全启动与验收

首次先不自动启动导航：

```bash
ros2 launch control_center_shell control_center.launch.py \
  robot_id:=<ID> topic_suffix:=<SUFFIX> start_navigation:=false
```

按顺序验收：

1. 系统服务页能读取四个服务状态，语音测试可正常播报。
2. 导航页地图正确；前后雷达与墙体重合；全局重定位后朝向没有相差 180°。
3. `/topic_gv_target_cmd_vel_<SUFFIX>` 只有预期控制源，低速测试底盘并随时准备急停。
4. 导航过程中前方近障碍会播报，待机时不播报。
5. 建图时导航/AMCL 已停止，系统中只有一个 `map -> odom` 发布源。
6. 关节反馈稳定后再解锁；关闭 VR、抓取和其它动作播放程序，避免多控制源冲突。

快速检查：

```bash
python3 -c 'from OpenGL import GL; from PyQt5 import QtOpenGL; print("OpenGL OK")'
ros2 pkg prefix text_voice_control
ros2 pkg prefix control_center_navigation
ros2 topic hz /topic_gv_front_lidar_<SUFFIX>
ros2 topic hz /topic_gv_rear_lidar_<SUFFIX>
ros2 topic hz /topic_gv_wheel_odom_<SUFFIX>
ros2 run tf2_ros tf2_echo base_link front_laser_frame
ros2 run tf2_ros tf2_echo base_link rear_laser_frame
ros2 topic info /text_voice_command -v
pgrep -af 'nav2_amcl|cartographer_node|adaptive_object_grasping'
```

## 6. 283 实际迁移记录（2026-08-04）

本次从 306（`192.168.50.138`）迁移到 283（`192.168.50.67`）确认：

- 306 工作空间实际为 `/home/ubuntu/ros2_ws`（末尾有空格）；283 为正常路径
  `/home/ubuntu/ros2_ws`，并且迁移前连 `src` 目录都不存在，需要先创建。
- 两台机器均为 ROS 2 Jazzy；283 厂家服务实际使用 `ROS_DOMAIN_ID=0`、
  `ROBOT_ID=300`、`rmw_cyclonedds_cpp` 和回环接口 `lo`。
- 使用 Fast DDS 在本机测试未发现厂家话题。诊断时必须切回与厂家服务相同的
  CycloneDDS 配置；若出现 participant index 耗尽，可在临时诊断配置中提高
  `MaxAutoParticipantIndex`，不要修改厂家服务配置来绕过问题。
- 已实测存在并验证类型：

  ```text
  /topic_gv_front_lidar_0_300      sensor_msgs/msg/LaserScan
  /topic_gv_rear_lidar_0_300       sensor_msgs/msg/LaserScan
  /topic_gv_wheel_odom_0_300       nav_msgs/msg/Odometry
  /topic_gv_target_cmd_vel_0_300   geometry_msgs/msg/Twist
  /control_reset_0_300              std_msgs/msg/String
  ```

- 前后雷达 `frame_id` 分别为 `front_laser_frame`、`rear_laser_frame`，轮速里程计为
  `odom -> base_link`。厂家当时没有提供雷达静态 TF，因此先沿用同型号 306 外参只能作为
  启动基线，导航前仍要做静止墙面重合测试和 180° 朝向检查。
- `/control_reset_0_300` 已确认有厂家节点
  `node_arm_vr_control_service_0_300` 订阅，系统服务页复位按钮可使用 283 后缀；迁移验收
  仍只检查订阅关系，不应远程直接触发真机复位。
- 283 迁移前缺少 `cartographer_ros`，且 `rosdep` 未初始化；这是本次编译前必须处理的
  实际依赖问题。`moveit_py`、`pick_ik` 不属于本控制中心七个子包的依赖。
- 283 首次打开全身关节模块时“三维视图初始化失败”，根因是缺少
  `python3-opengl` 和 `python3-pyqt5.qtopengl`，不是 URDF/mesh 丢失。补装后已用项目自带
  `render_robot_view_smoke.py` 在 Xvfb/GLX 下完成渲染、关节拾取与截图测试。
- 283 导航一直“等待 /map”实际是 launch 先后因缺少 `text_voice_control`、
  `multi_lidar_slam` 而在解析阶段退出；随后又发现源码中 Python 节点没有可执行位。
  已迁入并编译两个外部包、恢复 `control_center_navigation/scripts/*.py` 的 755 权限，并在
  `control_center_navigation/package.xml` 增加两项运行依赖。隔离域实测 `map_server` 成功读取
  `yundie_2f.png`（546×920、0.05 m/cell）、激活并向 AMCL 发布地图。
- 从 Windows 中转 `text_voice_control` 时，要排除文件名本身含反斜杠的历史重复文件，保留
  Linux 标准的 `config/`、`launch/`、`scripts/` 目录；推荐直接在 Linux 端按规范目录打 tar。
- 2026-08-04 已在 283、306 同步把 `multi_lidar_slam` 和 `text_voice_control` 移入
  `autolife_robot_control_center/`。以后打包迁移只复制该根目录；工作空间顶层不再保留这两个
  包的第二份副本，避免 colcon 报重复包名。
- `colcon test` 会把 `control_center_service_manager` 和 `control_center_shell` 的“未注册
  pytest（0 tests）”报告为代码 5；这不表示功能失败。应另外执行两个包 `test/` 目录中的
  smoke/integration 脚本，并放在独立 ROS Domain 中，避免测试速度或复位消息触达真机。
- 本次离屏退出测试发现导航 Qt 定时器可能在 ROS 节点销毁后执行一次，报
  `InvalidHandle: destruction was requested`，甚至导致 `exit code -11`。已在 timer 回调
  本身增加窗口、timer、executor、node 生命周期守卫，并在关闭时先清空成员引用、停止并
  断开 timer，再 shutdown executor 和 destroy node；迁移旧版本时要检查是否包含该修复。
- 航点管理器运行时读取 `~/.ros/control_center_navigation/waypoints.yaml`，不是直接读取包内
  `config/waypoints.yaml`。本次 306 和迁移前 283 的运行文件都不存在，因此在 283 目标文件
  不存在的前提下，用包内模板完成一次初始化；以后迁移必须优先备份运行文件，禁止用模板
  覆盖已保存的新航点。
- 迁移包含中文动作文件名。不要在会破坏 UTF-8 文件名的 Windows 解压/重打包流程中
  中转源码；优先在 Linux 上用 `tar` 解压，并核对“打招呼、比心、爱心、鞠躬”等文件。
- 迁移时只对运行文件定向替换 `0_306 -> 0_300` 和默认 `robot_id 306 -> 283`；
  URDF、地图、动作 PKL、历史文档和 306 标定记录均未做全仓库替换。

## 7. AI 执行约束

- 修改前先备份地图、航点、姿态和动作；不要删除用户数据。
- 不要对整个仓库盲目替换 `306`：历史文档、测试基线和 306 标定记录应保留。
- 先用实际话题、TF 和厂家 service 配置验证编号，再改参数。
- 导航与 Cartographer 不得同时运行；关节控制不得与 VR/抓取/其它动作源并行。
- 真机运动、关节使能和覆盖地图前必须明确检查当前状态，并从低速开始。
