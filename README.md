# 云蝶机器人综合控制中心 · 306版本

ROS 2 桌面控制套件，集中管理导航、建图、全身关节、系统服务、语音及LOGO胸屏。

## 版本区分

- **本分支 `robot306-20260920`**：306机器人2026年9月20日当前源码快照，默认编号 `306`、话题后缀 `0_306`。
- **原分支 `main`**：保留2026年9月9日上传的307版本，不覆盖、不替换。
- 306导航已回退到9月20日首次导航修改前，不包含当天尝试的轨迹导航重构和前进约束导航。建图及其他模块保留导出时的现状。

## 主要功能

| 模块 | 功能 |
| --- | --- |
| 导航 | 地图与航点管理、地图编辑保存/删除、全局重定位、MPPI导航、目标占用等待重试、语音提示、自动回充 |
| 建图 | 双雷达建图、旧地图增量建图、地图保存及编辑 |
| 关节 | 三维模型、关节调节、身体高度联动、姿态与动作库 |
| 底盘控制 | 手动速度渐变、控制权互斥和停止保护 |
| 系统服务 | 服务管理、音量及业务配置、胸屏入口 |
| 语音与胸屏 | 厂商ASR/TTS桥接、LOGO显示 |

自动导航使用定制 `ExactGoalSmacPlanner`、`SettledMPPIController` 和停稳到达判断，普通航点不再接独立末端精调。5厘米是配置中的到达判据，不代表所有场景实测误差保证。

## 获取、构建与启动

环境：Ubuntu 24.04、ROS 2 Jazzy、Python 3.12。真机还需厂商底盘、机械臂和视觉/语音服务。本仓库不是完整机器人系统，也不包含独立VR遥操程序。

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
git clone --branch robot306-20260920 https://github.com/liangxinfeng/autolife_robot_control_center.git
cd ~/ros2_ws
source /opt/ros/jazzy/setup.bash
rosdep install --from-paths src/autolife_robot_control_center --ignore-src -r -y
sudo apt install -y ros-jazzy-rmw-cyclonedds-cpp python3-pyqt5 \
  python3-pyqt5.qtopengl python3-opengl python3-numpy python3-yaml python3-pynput
colcon build --base-paths src/autolife_robot_control_center --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
ros2 launch control_center_shell control_center.launch.py \
  robot_id:=306 topic_suffix:=0_306 start_navigation:=false
```

这是8个ROS 2子包组成的套件，顶层没有 `package.xml`。已有同名源码时先备份，不要同时保留两份同名ROS包。`launcher/` 桌面模板依赖原机路径及外部启动器，迁移时需核对。

## 目录与配置

| 目录 | 内容 |
| --- | --- |
| `control_center_shell/` | 主界面与模块管理 |
| `control_center_navigation/` | 导航、航点、重定位与回充 |
| `control_center_slam/` | 建图、增量建图与引擎 |
| `control_center_chassis_guard/` | 底盘控制权与速度渐变 |
| `control_center_joint_control/` | 关节、模型、姿态与动作库 |
| `control_center_service_manager/` | 系统服务管理 |
| `multi_lidar_slam/` | 雷达配置与共用地图 |
| `text_voice_control/` | 语音桥接 |
| `logo/`、`launcher/` | 胸屏资源与桌面入口 |

导航配置在 `control_center_navigation/config/`；地图与航点路径以 `active_map.yaml` 为准。航点库的 `maps_index.json` 是读写主文件。地图、航点及动作来自306现场，不是通用安全标定。

`control_center_slam/runtime/` 是运行需要的x86_64/Jazzy引擎，`vendor/` 保留对应源码与许可，不应作为缓存删除。发布仅排除日志、缓存与历史备份；未更改真机程序，也未为上传执行机器人运动。

## 使用注意

- 更换机器人需核对编号、DDS、设备接口、雷达外参和关节限位，不应仅修改编号就直接运行。
- 导航、建图、厂商导航及VR不可争用控制入口；首次使用由现场人员低速验证。
- 不加载不可信PKL动作文件；保留碰撞、反馈超时及控制权保护。
- Cartographer、Nav2等组件保留原作者声明和许可。公开托管不改变厂商模型及其他第三方资源的权利归属。
- 本次为源码归档及静态核验，不代表重新完成全套真机验收。

详细资料：[迁移指南](AI_MIGRATION_GUIDE.md) · [导航](control_center_navigation/README.md) · [建图](control_center_slam/README.md) · [关节](control_center_joint_control/README.md)。历史说明中的旧版结论以实际源码和配置为准。
