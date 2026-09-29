# control_center_joint_control 跨机器人迁移简要手册

适用于同型号 AutoLife S2（robot_v2_2）机器人，例如从 306 迁移到 283 或 300。功能包自身功能不再重复说明。

## 1. 安装依赖

```bash
source /opt/ros/jazzy/setup.bash
sudo apt update
sudo apt install -y \
  ros-jazzy-rmw-cyclonedds-cpp \
  ros-jazzy-robot-state-publisher \
  ros-jazzy-tf2-ros \
  python3-pyqt5 python3-opengl python3-yaml python3-numpy \
  python3-pytest

cd ~/ros2_ws
rosdep install --from-paths src/control_center_joint_control --ignore-src -r -y
```

目标机器人还必须已有厂家 `robot_env` 环境、`autolife_robot_arm` 和可正常启动的：

```bash
systemctl --user status arm-control-service.service
```

如果机器人不是 `robot_v2_2`，必须先核对并替换 URDF、mesh、关节名称/顺序及限位，不能直接使用本包模型。

## 2. 设置机器人编号和通信域

假设目标机器人编号为 `283`、ROS Domain 为 `0`，则话题后缀为 `0_307`。迁移到 300 时相应改成 `0_300`。

检查厂家服务：

```bash
systemctl --user cat arm-control-service.service | \
  grep -E 'ROBOT_ID|ROS_DOMAIN_ID|RMW_IMPLEMENTATION|CYCLONEDDS_URI|TimeoutStopSec'
```

厂家服务至少应保持：

```ini
TimeoutStopSec=30s
Environment="ROS_DOMAIN_ID=0"
Environment="ROBOT_ID=307"
Environment="RMW_IMPLEMENTATION=rmw_cyclonedds_cpp"
Environment="CYCLONEDDS_URI=<CycloneDDS><Domain><General><Interfaces><NetworkInterface name=\"lo\"/></Interfaces></General><Discovery><ParticipantIndex>auto</ParticipantIndex><MaxAutoParticipantIndex>200</MaxAutoParticipantIndex></Discovery></Domain></CycloneDDS>"
```

如果修改了用户级 service 文件，执行：

```bash
systemctl --user daemon-reload
systemctl --user restart arm-control-service.service
```

不要把厂家关节心跳超时从默认值 `0.25s` 调大。“解锁并复位”必须通过厂家 `/control_reset_<后缀>` 状态机完成复位和使能，不要另写节点直接发布清错或使能消息。“解锁（不复位）”不会替代厂家使能流程，只能在厂家服务已完成使能且反馈正常时使用。

## 3. 编译

```bash
cd ~/ros2_ws
source /opt/ros/jazzy/setup.bash
rosdep install --from-paths src/control_center_joint_control --ignore-src -r -y
colcon build --packages-select control_center_joint_control
source install/setup.bash
```

## 4. 启动

283号机器人：

```bash
export ROS_DOMAIN_ID=0
ros2 launch control_center_joint_control \
  control_center_joint_control.launch.py \
  robot_id:=307 topic_suffix:=0_307
```

300号机器人只需把两处 `283` 改为 `300`。`topic_suffix` 是最终使用的话题编号，必须与厂家服务实际话题完全一致。

## 5. 首次运行检查

```bash
ros2 topic info /topic_arm_whole_body_and_gripper_current_joints_status_0_307
ros2 topic info /control_reset_0_307
ros2 topic echo --once \
  /topic_arm_whole_body_and_gripper_current_joints_status_0_307
```

应能收到关节反馈，且 `/control_reset_0_307` 有厂家订阅者。点击“解锁并复位”后机器人可能先移动到厂家复位姿态；必须等到提示“厂家复位与使能已完成，可以控制真机关节”后再操作。“解锁（不复位）”仅用于厂家硬件已经正常使能、需要保持当前姿态的场景。

测试时关闭 VR、抓取、动作播放等可能同时发布全身关节目标的程序，避免多控制源冲突。

## 6. 快速排错

- “关节反馈已断开”：检查 `ROS_DOMAIN_ID`、`ROBOT_ID`、`topic_suffix` 和厂家服务状态。
- 有反馈但不运动：确认已完成厂家复位/使能，而非只做软件解锁。
- 解锁后心跳丢失：检查是否仍有旧代码直接发布关节使能；不要靠增大心跳超时规避。
- CycloneDDS participant index 错误：确认 DDS 配置含 `MaxAutoParticipantIndex=200`，并关闭重复启动的控制进程。
