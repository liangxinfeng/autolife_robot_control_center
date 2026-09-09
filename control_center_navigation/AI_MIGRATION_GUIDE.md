# control_center_navigation 迁移指南（供 AI 使用）

## 1. 目标

把 `control_center_navigation` 迁移到另一台 ROS 2 Jazzy
机器人，并完成编译、完整启动、RViz、航点和语音播报验证。

本手册完整继承自 `multi_lidar_nav_amcl_waypoint_teleop_fine` 的迁移手册。
第 1～15 节继续覆盖原功能包已有的导航、精确入桩、航点、键盘控制、DDS、
RViz 和语音迁移经验；第 16 节只补充本包新增的自动全局重定位迁移要求。

不要只复制源码后直接编译。本功能依赖机器人编号、ROS Domain、DDS、地图、
运行时航点、`text_voice_control`、机器人视觉语音服务以及系统声卡配置。

## 2. 迁移前必须收集

- 源机器人和目标机器人的 `ROBOT_ID`。
- 目标机器人的 `ROS_DOMAIN_ID`，当前通常为 `0`。
- 目标工作区的真实绝对路径。
- 源机器人实际使用的地图及地图坐标系。
- 源机器人运行时航点文件。
- 目标机器人的扬声器声卡和 TTS 服务情况。

SSH 地址、用户名和密码由操作者单独提供，不要写死在功能包中。

## 3. 先确认工作区真实路径

不要默认工作区一定是 `/home/ubuntu/ros2_ws`。

本次 306 机器人目录名实际带有尾随空格：

```text
/home/ubuntu/ros2_ws
```

可用以下命令显示不可见字符：

```bash
find /home/ubuntu -maxdepth 1 -type d -printf '<%p>\n'
```

进入带空格的目录时必须完整引用：

```bash
cd '/home/ubuntu/ros2_ws'
```

所有日志、RViz 路径及 `install` 路径判断都必须以实际路径为准。

## 4. 复制并检查功能包

目标位置：

```text
<workspace>/src/control_center_navigation
```

复制后检查以下内容：

```bash
grep -RIn '283\|ROBOT_ID\|robot_id\|ros_domain_id' \
  <workspace>/src/control_center_navigation
```

只修改确实代表机器人编号的配置，不要盲目替换坐标、时间、备份文件或无关数字。

当前 306 配置应满足：

```text
text_voice_robot_id = 306
text_voice_ros_domain_id = 0
```

同时确认 `bringup_all.launch.py` 中所有使用的
`LaunchConfiguration` 都有对应的 `DeclareLaunchArgument`，并且声明动作在依赖它的
`SetEnvironmentVariable` 和节点之前加入 `LaunchDescription`。

## 5. 清理源码

迁移包中不要保留历史备份和缓存：

```text
*.bak*
*backup*
*.orig
*.rej
*~
__pycache__/
*.pyc
```

删除前必须确认解析后的绝对路径仍位于目标功能包目录内。不要删除正式源码、配置、
RViz 文件、接口文件、README 或本指南。

## 6. 安装依赖

基础环境：

```bash
source /opt/ros/jazzy/setup.bash
```

安装 Cartographer：

```bash
sudo apt update
sudo apt install -y ros-jazzy-cartographer-ros python3-pynput
```

`python3-pynput` 是随 `bringup_all.launch.py` 自动启动键盘控制时的必需依赖，
并会同时安装 `python3-xlib` 和 `python3-evdev`。虽然功能包的 `package.xml`
已经声明了该运行依赖，迁移后仍需确认目标系统实际完成安装：

```bash
dpkg-query -W python3-pynput
DISPLAY=:0 XAUTHORITY="$HOME/.Xauthority" \
  python3 -c "from pynput import keyboard; print(keyboard.Listener.__module__)"
```

正常 X11 环境应输出类似：

```text
pynput.keyboard._xorg
```

如环境允许，再检查其他依赖：

```bash
cd '<workspace>'
rosdep install --from-paths src --ignore-src -r -y
```

还必须确认下列包存在：

```bash
ros2 pkg prefix text_voice_control
ros2 pkg prefix cartographer_ros
```

## 7. 编译

先编译 SLAM，再编译导航功能包：

```bash
cd '<workspace>'
source /opt/ros/jazzy/setup.bash

colcon build --packages-select multi_lidar_slam
colcon build --packages-select control_center_navigation

source install/setup.bash
```

若旧 `build/install` 中包含错误绝对路径，应只清理这两个目标包对应的构建和安装目录，
再重新编译。不要清空整个工作区。

验证：

```bash
ros2 pkg prefix control_center_navigation
ros2 launch control_center_navigation \
  bringup_all.launch.py --show-args
```

## 8. DDS 与 RViz

本机 ROS 参与者较多时，CycloneDDS 默认参与者扫描范围可能不足，表现为：

- RViz 退出，`exit code -6`。
- 节点或话题明明存在，但 `ros2 topic info` 显示 `Unknown topic`。
- TTS 发布端找不到机器人语音订阅端。

`bringup_all.launch.py` 中的 CycloneDDS 配置应包含：

```xml
<Discovery>
  <ParticipantIndex>auto</ParticipantIndex>
  <MaxAutoParticipantIndex>200</MaxAutoParticipantIndex>
</Discovery>
```

机器人 `vision-service` 使用：

```text
RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
ROS_DOMAIN_ID=0
NetworkInterfaceAddress=127.0.0.1
```

调试语音话题时，测试进程必须使用相同的 RMW、Domain 和回环接口，并把
`MaxAutoParticipantIndex` 设为 `200`。

RViz 配置路径必须由 ROS 包 share 目录生成，不要手工拼接包含多余空格的路径。

## 9. 航点迁移

当前功能包把 RViz 新增航点直接保存在源码中的：

```text
config/waypoints.yaml
```

该文件既是初始模板，也是用户实际保存的航点文件。启动文件会根据当前
colcon 安装前缀定位同一工作区内的源码目录，不使用 `~/.ros` 回退路径。
迁移时应连同整个功能包源码一起复制，特别注意不要用空模板覆盖已有航点。

格式：

```yaml
waypoints:
  航点名称:
    frame_id: map
    x: 0.0
    y: 0.0
    yaw: 0.0
```

注意：

- `yaw` 使用弧度，不是角度。
- 必须确认航点属于当前加载的同一张地图。
- 不要把平台地图管理器 `maps_index.json` 中的预设点直接当作用户在本功能包保存的航点。
- 不要跨地图混用坐标。
- 找不到源运行时文件时，应明确报告缺失，不要根据相似名称猜坐标。
- RViz 航点显示话题为 `/waypoints/markers`，使用 `MarkerArray` 和
  `TRANSIENT_LOCAL` QoS。
- 写入或替换航点文件后需要重启 `waypoint_manager` 或完整 launch。

## 10. 语音播报

功能包内配置：

```text
config/voice_feedback.yaml
```

应包含：

```yaml
voice_feedback_enabled: true
voice_command_topic: "/text_voice_command"
voice_departure_template: "我现在前往{waypoint}"
voice_arrival_template: "我已到达{waypoint}"
```

完整链路：

```text
waypoint_manager
  -> /text_voice_command
  -> text_voice_control
  -> /topic_tts_<DOMAIN>_<ROBOT_ID>
  -> vision-service
  -> 本地 TTS
  -> 扬声器
```

机器人系统配置不在导航功能包内。需要检查：

```text
/home/ubuntu/miniconda3/envs/robot_env/lib/python3.12/site-packages/
autolife_robot_vision/settings.toml
```

关键配置：

```toml
audio_enabled = true
ai_chatbot_enabled = false
tts_enabled = true

[app_settings.tts]
TTS_PROVIDER = "piper"
```

`ai_chatbot_enabled` 不需要为航点播报开启。

不要在无网络环境使用缺少本地音色的 Kokoro。此次 Kokoro 会尝试从
Hugging Face 下载 `zf_001.pt`，网络不可用时不会发声。安装包已经带有 Piper
中文离线模型：

```text
autolife_robot_vision/assets/tts/piper/zh_CN-huayan-medium.onnx
```

修改系统语音设置后：

```bash
systemctl --user restart vision-service.service
systemctl --user status vision-service.service
```

确认日志中出现：

```text
Local Piper TTS initialized
```

## 11. 音量

应用音量位于：

```text
autolife_robot_vision/configs/robot_v2_2.json
```

设置：

```json
"speaker_volume": 90
```

当前模拟扬声器是 ALSA card 3（Intel PCH）：

```bash
amixer -c 3 sset Master 90% unmute
amixer -c 3 sset PCM 90% unmute
sudo alsactl store 3
```

硬件 Master 档位不能精确表示 90%，可能显示为最接近的 91%；PCM 可以显示 90%。

## 12. 语音独立测试

先检查实际 TTS 订阅端：

```bash
ros2 topic info --no-daemon /topic_tts_0_<ROBOT_ID> -v
```

应看到：

```text
Subscription count: 1
Node name: vision_service_0_<ROBOT_ID>
```

真实航点链路测试：

```bash
ros2 launch text_voice_control bringup.launch.py \
  robot_id:=<ROBOT_ID> ros_domain_id:=<DOMAIN_ID>
```

另开终端：

```bash
ros2 topic pub --once /text_voice_command std_msgs/msg/String \
  "{data: '我现在前往测试航点'}"

ros2 topic pub --once /text_voice_command std_msgs/msg/String \
  "{data: '我已到达测试航点'}"
```

同时检查：

```bash
journalctl --user -u vision-service.service --since '3 minutes ago' \
  --no-pager | grep 'tts:'
```

日志收到文本仍不代表一定发声。还要排查 TTS 模型加载错误、网络下载失败、
声卡占用、静音和音量。

## 13. 完整启动与验收

```bash
cd '<workspace>'
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch control_center_navigation bringup_all.launch.py
```

验收清单：

- Launch 无缺失的 `LaunchConfiguration`。
- Cartographer 和 `multi_lidar_slam` 可启动。
- RViz 不再以 `exit code -6` 退出。
- 地图、TF、定位和 Nav2 节点正常。
- 正确的运行时航点显示在 RViz。
- 导航出发和到达时分别播报对应航点名称。
- `/topic_tts_<DOMAIN>_<ROBOT_ID>` 有 `vision-service` 订阅。
- `vision-service` 使用 Piper 离线 TTS。
- 应用和系统音量均符合要求。
- 按住 `W/S/A/D/Q/E` 时 `/manual_cmd_vel` 能收到非零 `Twist`，松键后收到零速度。
- 源码和安装目录没有 `.bak*` 等历史备份。

## 14. 键盘控制无反应的迁移排查

键盘节点默认随完整 launch 自动启动：

```text
keyboard_waypoint_control_node.py
```

它发布：

```text
/manual_cmd_vel
  -> cmd_vel_arbiter
  -> /topic_gv_target_cmd_vel_0_<ROBOT_ID>
```

本次“节点存在但按键完全无反应”的根因是目标系统没有安装
`python3-pynput`。节点因此退回终端兼容模式，但 `ros2 launch` 启动的子进程
标准输入实际是管道而非交互终端，例如：

```text
fd0 -> pipe:[...]
```

此时即使终端里按键，子节点也无法收到输入。处理方法：

```bash
sudo apt install -y python3-pynput
```

安装后必须重启键盘节点或完整退出并重新启动 `bringup_all.launch.py`，因为已经
运行的 Python 进程不会重新导入新安装的模块。启动日志应出现：

```text
键盘控制已启用按下/松开监听模式。
```

检查链路：

```bash
ros2 node info --no-daemon /keyboard_waypoint_control
ros2 topic info --no-daemon -v /manual_cmd_vel
ros2 topic info --no-daemon -v /topic_gv_target_cmd_vel_0_<ROBOT_ID>
```

应确认：

- `/keyboard_waypoint_control` 是 `/manual_cmd_vel` 的发布者。
- `/cmd_vel_arbiter` 订阅 `/manual_cmd_vel`。
- `/cmd_vel_arbiter` 向机器人底盘目标速度话题发布。
- 非导航状态下 `/nav_cmd_vel` 没有持续发布并占用仲裁优先级。
- `DISPLAY` 和 `~/.Xauthority` 属于当前机器人桌面会话。

不要仅通过“节点进程仍在”判断键盘正常；必须同时验证监听模式和
`/manual_cmd_vel` 实际消息。

## 15. 本次最重要的经验

1. 先识别真实路径、机器人编号、Domain 和地图，再修改。
2. 源码、运行时数据和机器人系统配置是三类不同资产，必须分别迁移。
3. 航点文件在功能包的 `config/waypoints.yaml`，迁移时必须保留，不能用空模板或平台预设点猜测恢复。
4. ROS 话题存在但发现不到时，要检查 RMW、Domain、网卡和 CycloneDDS
   participant 上限。
5. TTS 订阅成功不等于喇叭有声，要继续验证模型是否真正生成音频。
6. 无网络机器人优先使用随安装包提供的 Piper 离线模型。
7. 清理和重建只针对目标包，避免破坏同工作区的其他功能包。
8. 键盘节点由 `ros2 launch` 启动时不能依赖子进程标准输入，必须安装并验证
   `python3-pynput` 的 X11 全局按下/松开监听。

## 16. 自动全局重定位迁移补充

### 16.1 新增文件和启动流程

相对于 `multi_lidar_nav_amcl_waypoint_teleop_fine`，本包新增或修改：

```text
src/cartographer_global_relocalization.cpp
config/global_relocalization.yaml
scripts/navigation_startup_on_initialpose_node.py
launch/bringup_all.launch.py
```

完整启动顺序为：

```text
map_server 与 AMCL
  -> 前后雷达全地图、全方向相关匹配
  -> 3 个有效候选一致性确认
  -> /initialpose
  -> AMCL 协方差稳定
  -> 自动激活 Nav2 生命周期
```

实现参考 `src/cartographer-master` 中 Fast Correlative Scan Matching 的思想，
但根据 ROS 2 `OccupancyGrid` 的坐标和行列约定重新实现，不直接链接
Cartographer C++ 库。

### 16.2 必须确认的机器人参数

迁移前确认并修改 `config/global_relocalization.yaml`：

```yaml
map_topic: /map
scan_topic: /目标机器人前雷达话题
rear_scan_topic: /目标机器人后雷达话题
base_frame: base_link
map_frame: map
```

当前 306 机器人使用：

```text
/topic_gv_front_lidar_0_306
/topic_gv_rear_lidar_0_306
```

同时核对 `launch/bringup_localization.launch.py` 中：

- `base_link -> front_laser_frame`
- `base_link -> rear_laser_frame`

必须使用机器人真实外参。尤其不能只提取 yaw 处理激光点：本机器人前雷达 TF
包含绕 Y 轴 180°翻转，后雷达 TF 包含绕 X 轴 180°翻转。全局匹配器必须使用
完整四元数三维 TF，否则定位方向会相差约 180°。

可用以下命令检查：

```bash
ros2 run tf2_ros tf2_echo base_link front_laser_frame
ros2 run tf2_ros tf2_echo base_link rear_laser_frame
```

### 16.3 可靠性保护

默认配置要求：

- 前后雷达均可用，接收时间差不超过 0.30 秒。
- 匹配分不低于 `0.52`。
- 第一、第二候选得分差不低于 `0.03`。
- 3 个通过校验的候选彼此一致。
- 一致候选位置差不超过 0.25 m，方向差不超过 5°。
- 最多尝试 20 次；歧义帧不会覆盖已确认候选。

不要为了让节点快速返回而直接取消得分差和多次一致性校验。重复走廊中单雷达、
单帧匹配容易获得错误位置。

### 16.4 编译

```bash
cd '<workspace>'
source /opt/ros/jazzy/setup.bash

colcon build \
  --base-paths src/control_center_navigation \
  --packages-select control_center_navigation \
  --symlink-install

source install/setup.bash
```

若只需要运行本包的全局匹配器，不强制安装 Cartographer。只有需要运行参考包时才安装：

```bash
sudo apt install ros-jazzy-cartographer-ros
```

### 16.5 先做不写入 AMCL 的安全测试

```bash
ros2 launch control_center_navigation \
  bringup_all.launch.py \
  global_relocalization_publish_initial_pose:=false
```

检查：

```bash
ros2 topic echo /global_relocalization/status
ros2 topic echo /global_relocalization/result_pose
```

应看到：

```text
waiting
map_ready
waiting_scans
matching
confirming
succeeded
```

不能只看 `succeeded`。还必须在 RViz 中确认雷达点与地图墙体重合，机器人朝向正确。

### 16.6 正式启动和验收

```bash
ros2 launch control_center_navigation \
  bringup_all.launch.py
```

验收日志应依次包含：

```text
Global relocalization succeeded
Published global pose to /initialpose
AMCL convergence sample 3/3
AMCL localization is stable
Navigation lifecycle started successfully
```

继续检查：

```bash
ros2 topic echo /amcl_pose
ros2 lifecycle get /amcl
ros2 lifecycle get /planner_server
ros2 lifecycle get /controller_server
```

手动重新触发：

```bash
ros2 service call \
  /cartographer_global_relocalization/relocalize \
  std_srvs/srv/Trigger '{}'
```

当前 306 机器人修正完整雷达 TF 后的现场测试结果约为：

```text
x=14.960
y=-12.741
yaw=1.571
score=0.84～0.86
```

该数值只用于确认本次迁移结果，不应写死为其他机器人或其他地图的初始位姿。
# 本包原生界面迁移补充（优先于下方继承内容）

本包继承原全局重定位导航包的迁移经验，但已经用
`navigation_ui_node.py` 完全替代 RViz2 和原 RViz 控制面板。下方继承章节中涉及
RViz 安装、配置、插件或 RViz 画面验证的内容不再适用于本包，应改为验证原生界面。

迁移到 283、300 等其他编号机器人时，额外执行：

1. 安装原生界面依赖：

   ```bash
   sudo apt install python3-pyqt5 python3-numpy
   ```

2. 将以下文件中的 `_0_306` 雷达、里程计话题改为目标机器人编号：

   - `config/global_relocalization.yaml`
   - `config/nav2_params.yaml`
   - `config/proximity_voice_alert.yaml`
   - `launch/bringup_all.launch.py`

3. 修改 `bringup_all.launch.py` 中的 `ROBOT_ID`、`text_voice_robot_id`，
   并核对 `/topic_gv_wheel_odom_0_<编号>`。
4. 默认地图不再写死在 launch 中，统一读取
   `config/active_map.yaml`。地图切换成功后原生界面会原子更新此文件；迁移时应确认
   `active_map` 指向存在的 YAML 地图。
5. 编译后确认 `ros2 pkg executables` 中存在
   `navigation_ui_node.py`，启动时不得出现任何 `rviz2` 进程。
6. 在原生界面中验证地图、AMCL 位姿、前后激光、路径、航点名称、全局重定位状态，
   再进行实际导航。
7. 在前雷达 0.8 m 内连续放置物体，确认 `/text_voice_command` 收到
   “前方有人或障碍物，请注意避让”；阈值、扇区和重播周期统一在
   `config/proximity_voice_alert.yaml` 调整。306 的 `forward_angle_deg` 为 180；
   迁移后必须实测目标机器人前方对应的 LaserScan 角度，不能默认沿用。

## 地图与航点存储规则

- 建图原文件保存在 `control_center_slam/maps/`；保存成功后 YAML、PGM 和
  PBStream 会同步一份到 `control_center_navigation/maps/`。同名文件存在时拒绝
  覆盖，必须换地图名。
- 导航选择“加载选中地图”成功后，该地图会写入
  `control_center_navigation/config/active_map.yaml`，下次启动默认加载它。
- 每张地图使用独立目录：
  `control_center_navigation/maps/waypoints/<地图名>/`。目录内同时保存原生
  `<地图名>.yaml` 航点镜像和厂商兼容格式 `maps_index.json`。
- `maps_index.json` 是唯一实际读写的数据源；读取、增加、删除和重命名航点都以
  `prepared_positions` 为准。`yaw` 在 JSON 中使用角度，程序内部转换为弧度。
- `<地图名>.yaml` 只在每次写入时同步生成，供兼容或人工查看，运行时绝不读取，
  因而不能通过只修改 YAML 来改变航点。新地图首次加载时自动创建空 JSON 主库和 YAML 镜像。
- 旧版 `config/waypoints.yaml` 只作为升级前数据保留。首次升级必须先转换成当前地图目录内的
  `maps_index.json`，并核对航点数量和坐标后再启动；不能只复制旧 YAML。
- 雷达为倒装安装时，点云显示必须使用完整四元数旋转，不能只取 yaw；迁移后检查
  前后雷达 frame、静态 TF 和时间戳。界面只接收递增时间戳，TF 等待最多约
  0.18 秒，并用三帧/4 cm 世界坐标栅格抑制孤立抖点；数据停止后约 0.30 秒清除。
- 导航语音需要同时满足 `config/voice_feedback.yaml` 的
  `voice_feedback_enabled: true` 和厂商视觉包 `settings.toml` 的
  `tts_enabled = true`。系统服务模块的“导航语音播报”会同时维护两项并重启
  `vision-service.service`，迁移后默认均应开启。导航短提示默认使用随包安装的离线
  `TTS_PROVIDER = "piper"`，避免 Kokoro 权重缺失时反复等待外网下载。
- 中继 IP 保存后由系统服务模块立即重启 `autolife-relay.service`；无需重启整机。
  中文拼音输入依赖 `ibus`、`ibus-libpinyin`，并将 `libpinyin` 加入 GNOME 输入源。

---
