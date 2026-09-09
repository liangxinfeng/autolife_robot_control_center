# 云蝶机器人综合控制中心

面向 AutoLife S2 机器人的 ROS 2 桌面控制套件，将**导航、建图、全身关节、系统服务和胸屏展示**集中在一个界面中。各模块保持独立，便于维护和跨机器人迁移。

本仓库来自 **307机器人，2026-09-09版本**。默认编号为 `307`、话题后缀为 `0_307`；不是通用机器人驱动，也不包含完整VR遥操系统。

## 主要功能

| 模块 | 功能 |
| --- | --- |
| 导航与航点 | 地图库、地图编辑、自动/手动重定位、多航点导航、到点精调、语音提示、自动回充 |
| 地图构建 | 双雷达建图、旧图增量建图、地图优化保存、障碍物绘制/擦除 |
| 全身关节 | 内嵌三维模型、关节反馈与调节、身体高度联动、姿态库和关键帧动作库 |
| 系统服务 | 查看及管理厂商服务、控制模式切换、音量、业务IP配置和胸屏入口 |
| LOGO胸屏 | 云蝶LOGO动画、全屏展示及显示方向切换 |

主界面不需要单独打开RViz。**导航默认关闭**，建图和关节控制按需启动；导航与建图互斥，避免定位坐标冲突。构建依赖仍包含用于可选插件的RViz库。

## 目录结构

```text
autolife_robot_control_center/
├── control_center_shell/            综合界面与模块管理
├── control_center_navigation/       导航、航点、重定位和回充
├── control_center_slam/             建图与增量建图
├── control_center_joint_control/    三维关节界面、姿态和动作库
├── control_center_service_manager/  系统服务管理
├── multi_lidar_slam/                共用雷达配置及地图资源
├── text_voice_control/              厂商ASR/TTS接口桥接
├── logo/                            胸屏网页与启动脚本
└── launcher/                        桌面入口模板
```

这是**7个ROS 2子包组成的套件**，顶层没有 `package.xml`，应整体放到工作空间的 `src` 内。不要在工作空间中再保留同名子包的第二份副本。

## 安装与启动

环境：Ubuntu 24.04、ROS 2 Jazzy、Python 3.12。真机还需要已安装并正确配置的厂商底盘、机械臂和视觉/语音服务；这些服务不在本仓库中。

在已安装ROS 2 Jazzy、colcon和rosdep的机器上：

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
git clone https://github.com/liangxinfeng/autolife_robot_control_center.git

cd ~/ros2_ws
source /opt/ros/jazzy/setup.bash
rosdep install --from-paths src/autolife_robot_control_center --ignore-src -r -y
sudo apt install -y ros-jazzy-rmw-cyclonedds-cpp \
  python3-pyqt5 python3-pyqt5.qtopengl python3-opengl \
  python3-numpy python3-yaml python3-pynput

colcon build --base-paths src/autolife_robot_control_center \
  --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash

ros2 launch control_center_shell control_center.launch.py \
  robot_id:=307 topic_suffix:=0_307 start_navigation:=false
```

rosdep若提示未初始化，先执行 `sudo rosdep init` 和 `rosdep update`。

307原环境也可运行 `bash launcher/start_autolife_control_center_307.sh`。桌面入口模板依赖 `/home/ubuntu/ros2_ws` 和 `.local/bin` 中的启动器，换机器后需修改路径并安装入口，不能直接照搬。

**建图引擎：** `control_center_slam/runtime/` 包含运行必需的x86_64/Jazzy引擎，不是普通构建缓存；`vendor/` 保留对应源码和许可。其他架构或依赖环境请按[建图引擎说明](control_center_slam/vendor/README.md)重新构建。

**回充修复：** 自研导航使用包内 `docking_server_compat`，修复旧版倒车目标投影方向，不覆盖 `/opt/ros` 的厂商程序。

## 配置与数据

| 内容 | 位置 |
| --- | --- |
| 编号与默认启动项 | `control_center_shell/launch/control_center.launch.py` |
| 导航、重定位、回充参数 | `control_center_navigation/config/` |
| 各地图航点 | `control_center_navigation/maps/waypoints/<地图目录>/maps_index.json` |
| 姿态及动作 | `control_center_joint_control/motion_library/` |
| 建图配置 / 建图结果 | `control_center_slam/config/` / `control_center_slam/maps/` |
| 胸屏样式 | `logo/index.html`、`logo/styles.css`、`logo/app.js` |

航点以各地图目录内的 **`maps_index.json` 为读写主文件**，YAML只作为同步写入的镜像。仓库内地图、航点和动作是源机器人数据，不代表适用于其他现场；请先备份再更换。

## 迁移与安全

- 更换机器人时核对编号、话题后缀、DDS、设备接口、雷达外参、关节零位和限位；仅改启动参数不等于完成迁移。
- 当前路径为无尾部空格的 `~/ros2_ws`。历史迁移记录中的旧路径、旧航点存储方式和测试结果不代表当前状态。
- 官方导航、自研导航、建图及VR不能争用同一控制入口。首次使用先只读检查，再由现场人员低速验证。
- 地图和姿态不可直接当作真机安全标定；不要加载不可信来源的PKL动作文件。
- 回充的“充电桩”航点必须是**已接触充电时保存的本体位姿**。应保留碰撞、反馈超时和偏离保护；没有相对桩体传感器时，仍需考虑打滑和定位误差。

详细资料：[迁移指南（含历史记录）](AI_MIGRATION_GUIDE.md) · [导航](control_center_navigation/README.md) · [建图](control_center_slam/README.md) · [关节控制](control_center_joint_control/README.md) · [回充修复原理](control_center_navigation/src/docking_compat/README.md)

## 第三方组件与验证范围

Cartographer、Nav2源码保留原作者声明和Apache-2.0许可，见各自 `vendor/`、`src/docking_compat/`。本仓库不改变其他文件、厂商模型和资源的原有权利归属；公开托管不代表对全部第三方内容重新授予统一许可证。

回充更新经过57项离线回归、控制律模拟和隔离节点验证，不等于所有现场均已通过真机验收。不要在真实控制域直接运行会发布运动或复位消息的测试脚本。
