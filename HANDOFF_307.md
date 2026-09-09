# 306 → 307 机器人综合控制中心迁移 HANDOFF

日期：2026-08-28（Asia/Shanghai）  
源机：`autolife-robot-306`（192.168.8.187）  
目标机：`autolife-robot-307`（192.168.8.68）

## 1. 迁移结果

306 Desktop 上“机器人综合控制中心”的完整前端、后端、启动器、配置、地图基线、模型资源和测试代码已迁移到 307，并完成 307 标识适配、依赖安装、七包编译和隔离验收。

目标路径：

- 源码：`/home/ubuntu/ros2_ws/src/autolife_robot_control_center`
- 编译结果：`/home/ubuntu/ros2_ws/install`
- 桌面入口：`/home/ubuntu/Desktop/机器人综合控制中心-307.desktop`
- 启动器：`/home/ubuntu/.local/bin/autolife-control-center-307`
- 证据与原始归档：`/home/ubuntu/migration-archives/control-center-306-20260828`

ROS 2 包共 7 个：

1. `text_voice_control`
2. `multi_lidar_slam`
3. `control_center_navigation`
4. `control_center_slam`
5. `control_center_joint_control`
6. `control_center_service_manager`
7. `control_center_shell`

## 2. 307 适配

- 默认机器人 ID：`307`
- 默认话题后缀：`0_307`
- 307 地址：`192.168.8.68`
- 桌面启动默认参数：`start_navigation:=false`
- 307 操作员 README 中的 306 示例已经改为 307；迁移指南和原始 306 启动器保留用于溯源。
- 306 的 VR reset/body/neck 姿态没有直接启用，仅隔离保存在：
  `/home/ubuntu/.config/autolife_robot_control_center/migrated-from-306-unverified/vr_reset_pose-306-unverified.json`
- 运行代码扫描未发现残留的 306 话题、机器人 ID 或 306 IP；唯一保留项是 `launcher/start_autolife_control_center_306.sh` 原始溯源启动器。

## 3. 原始归档与哈希

- 原始源码包：`control-center-306-source-20260828.tar.zst`
- 原始源码 SHA-256：`4407593688e608055cc9ebe949f483fc0859392363adfcec0a55acd4e71a23a2`
- 适配报告 SHA-256：`ac612f35c5fb6a68a5f6318d056aa80d89c741965a6367156f2d7bf1d5e037cc`
- 307 启动器 SHA-256：`b33f32a36a4e87e29d9c972d601424250a8e7d24976ec37481bf81ebdba656f5`
- 307 Desktop 文件 SHA-256：`51d6abf5cff5438a898a779cfd40f84a770f99bdbcef635ebd2ce640ea8ac250`
- 运行配置清单哈希：`98f0656a2e36ee08acf9c97bdcc9ba1e432d56be029e0f109b1e1ece1704bfa6`
- 安装树清单哈希：`2c954579a3890245bf5ac9e8843561d41af29db4189e178d2511a3a7e49f430e`

逐文件清单位于：

- `adapted-source-files.sha256`
- `runtime-config-files.sha256`
- `installed-files.sha256`
- `critical-artifacts.sha256`

## 4. 依赖与编译

- ROS 2 Jazzy
- Cartographer / Cartographer ROS
- Nav2 map server
- PCL ROS
- RViz2
- PyQt5 + QtOpenGL
- PyOpenGL
- pynput
- CycloneDDS RMW

完整版本见 `dependency-versions.txt`。七个包均完成 Release 编译；编译日志中唯一 stderr 是 PCL `FindFLANN.cmake` 的 CMake CMP0144 开发警告，不影响产物。

没有安装 `nav2_bringup` 元包：源码不引用它，306 也未安装；307 已具备本项目实际使用的 `nav2_map_server` 等组件，避免额外引入 Gazebo 仿真栈。307 的 `rosdep` 尚未初始化，因此未以 `rosdep check` 作为门禁；依赖闭合由 apt 版本清单、完整编译、Python 导入、ELF 链接和运行测试共同验证。

## 5. 验收结果

- 87 个活动 Python 文件语法编译通过。
- `colcon test`：40 个测试通过，0 失败、0 错误、0 跳过。
- 导航 pytest：19 个测试通过。
- 14 组直接 smoke / 集成 / GUI 渲染场景通过。
- 8 张 GUI/地图/三维视图验收截图已保存。
- 14 个 ELF 文件执行 `ldd`，0 个缺失动态库。
- 启动器 `--check` 通过；解析到 `robot_id=307`、`topic_suffix=0_307`、`start_navigation=false`。
- 307 的前/后雷达、里程计、目标速度、复位、关节状态和电池话题均已只读核对类型。
- 隔离测试使用 `ROS_DOMAIN_ID=97`、loopback DDS；没有向真实机器人域 0 发布测试控制指令。
- 系统服务状态在验收前后保持：视觉 active、厂家 SLAM inactive、底盘控制 active、机械臂控制 active。

主要证据：

- `build.log`
- `colcon-test-result.txt`
- `direct-smoke-tests.log`
- `acceptance-result.txt`
- `final-audit.log`
- `robot-307-topic-types.txt`
- `running-control-center-status.txt`
- `acceptance-screenshots.sha256`

## 6. 当前真机运行状态

在迁移验收结束后，307 桌面会话于 10:37:38 启动了控制中心，并于 10:38:14 打开关节模块。只读状态快照显示：

- `connected=true`
- `hardware_enabled=true`
- `hardware_ready=true`
- `hardware_unlock_mode=without_reset`
- `action_running=false`
- `active_goal_error_deg={}`
- `leg_waist_enabled=false`

这说明当前没有活动动作或跟踪目标、身体/腿腰控制未启用，但机械臂控制桥处于人工解锁状态。该状态来自桌面端运行会话，不是域 97 验收测试触发；迁移操作没有替用户停止进程或下发真机控制命令。继续操作前，应由现场人员确认机械臂周围安全。

## 7. 启动与边界

桌面双击“机器人综合控制中心（307）”，或运行：

```bash
/home/ubuntu/.local/bin/autolife-control-center-307
```

仅做环境检查：

```bash
/home/ubuntu/.local/bin/autolife-control-center-307 --check
```

边界：

- 306 的地图、雷达外参和导航参数已作为完整基线迁移，但尚未完成 307 实机标定；在标定和现场避障验证前，不应据此接受真实导航/SLAM。
- 隔离保存的 306 reset pose 未经 307 关节零位核对，不应直接启用。
- 当前验收覆盖复制完整性、构建、依赖、离线逻辑、隔离 ROS 集成、GUI 渲染及只读真机接口匹配；不等同于底盘移动、建图或机械臂动作安全验收。
- 本次未修改或编译 `icra2027_paper/root.tex`、`icra2027_paper/sections/*`、`icra2027_paper/output/*`。
