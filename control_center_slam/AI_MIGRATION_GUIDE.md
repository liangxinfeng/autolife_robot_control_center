# control_center_slam 迁移指南（供 AI 使用）

## 328实测修正（2026-09-07，优先于下文历史基线）

### 增量建图

- 初始光标支持局部自动精细对齐：X/Y各±1.5m、朝向±45°，后台粗细两级搜索。
  保留最终吻合门槛，拒绝多解/边缘解；搜索期间和确认期间移动底盘会作废结果。
  页面上方提示设置光标，拖动实时显示箭头。详细说明在包内增量使用说明顶部。

- 控制台地图构建启动时可选新建/导入PBStream；选YAML必须同名PBStream存在。
  导入后先在地图上拖动初始位置箭头，再核对激光吻合并人工确认，之后才创建新轨迹。
- 配置`multi_lidar_incremental.lua`继承质量版，启用重叠子图替换；至少两份新完整
  子图覆盖，旧图独有区域保留。搬走障碍物需充分复扫，不是导入即自动删除。
- 私有引擎还修复16位覆盖计数溢出，增量加载后将无接收器的历史轨迹标为FINISHED。
  UI最终保存使用真实新轨迹ID。必须一起迁移runtime/vendor/config/scripts和shell入口，
  新增python3-scipy依赖。普通新建地图不启用子图替换。
- 使用及能力边界见包内`328增量建图使用说明.md`。始终另存新名称，不覆盖导入原件。

### 第三轮：远距离细障碍物与外参复核

- `multi_lidar_quality.lua`启用距离相关扫空证据：≤1.5m保留miss=0.47，
  1.5～2.75m为0.476667，2.75～4m为0.483333，>4m为0.49；hit=0.60不变。
  按射线真实原点到每格的距离计算，不按终点距离。命中优先、近处证据优先，
  每次每格只更新一次。远处清除更慢，但并未禁止清除；需要近远实物对照验收。
- 删除这一Lua附加字典即可在新版私有引擎下回退到原固定扫空权重。
  runtime、config、vendor源码和校验文件必须一起迁移。
- 328实际`active_robot_version=robot_v2_2`：当前前/后雷达TF与厂商配置一致，
  x=±0.215m、y=0、z=0.1515m，roll=3.145926，前yaw=3.145926、后yaw=0。
  运行时只见一套前后静态TF。系统URDF与控制台URDF的雷达安装关节一致，
  合成后相对Link_Zero_Point的x=±0.215m、z≈0.1678m。
  URDF网格链接不等于LaserScan光学坐标系，且基准未证实相同，不能照搬旋转
  或直接把高度改成0.1678m；3.145926与π的差也需实物标定，不能凭猜测改。
- 厂商随附slam_toolbox配置为/merged单扫描、5cm栅格，合并器约1°；本包保持
  前后独立约0.5°扫描与2cm栅格及真实射线原点。厂商参数不是更精细的标定依据。
  原服务未启动，本次未改其文件或任何机器人运动控制。

### 第二轮：输入质量与子图内重影

- 默认配置改为 `multi_lidar_quality.lua`：继承原配置，栅格/输出分辨率2cm，
  匹配体素2cm；原 `multi_lidar_optimized.lua` 保持3cm作为回退基线。
- 建图适配器仅在轮速里程计证实静止0.3秒且反馈新鲜时，对每个角度做3帧中值。
  运动立即清空历史，距离跳变超过8cm不滤，NaN/Inf不填补，不做跨角度平均，
  不删除单点椅腿。微小变化可能有约1帧中值滞后，但只发生在底盘静止时。
- 新增**写图质量门控**：超过0.22m/s或0.10rad/s时继续匹配/定位，但不写入子图；
  运动降低后需持续0.2秒低运动观测再恢复。按扫描采集时间查历史里程计，
  而不是错误地拿最新/未来里程计与旧扫描比较。反馈过期也暂停写图。
  这不是驱动限速或急停功能；转动期间地图暂不扩展属于预期。
- 保留多雷达真实射线原点修复，不关闭障碍物清除，不修改外参或现有地图。
- 系统原版回退需同时指定三个参数：
  `cartographer_package:=cartographer_ros cartographer_executable:=cartographer_node cartographer_configuration_basename:=multi_lidar_optimized.lua`
  并可加 `map_resolution:=0.03`。系统原版不识别私有配置的质量门控字段。
- 根本限制：厂商当前LaserScan无可信逐束时间；门控不是运动去畸变的替代。
  要在连续快速转弯中精细建图，需要厂商提供真实采样时间/扫描顺序及外参标定。

- **多雷达真实射线起点修复**：建图默认启动本包的
  `cartographer_multi_origin_node`，不再启动系统原版建图节点。每条射线保留
  真实雷达原点用于空闲区更新；匹配点云仍降采样，栅格插入保留原始有效回波。
  同批次先写全部命中再写空闲，每格只更新一次，避免前后雷达互相覆盖命中。
  系统 `/opt/ros/jazzy` 原件不变，不改变导航、遥操或电机控制。
- 迁移需一起复制 `runtime/`、`vendor/` 与构建脚本；当前二进制用于Ubuntu24.04
  x86_64/Jazzy。换架构或依赖不匹配时运行
  `bash scripts/build_multi_origin_cartographer.sh` 后重新编译本包。
  固定上游版本、源代码和测试在 `vendor/`；细节见 `vendor/README.md`。
- 应急回退只需在建图启动参数中指定
  `cartographer_package:=cartographer_ros cartographer_executable:=cartographer_node cartographer_configuration_basename:=multi_lidar_optimized.lua`。
  不要同时启动新旧两套建图。已有损坏地图不会自动修复，保留旧图后另建新图验证。

- 当前包位于 `ros2_ws/src/autolife_robot_control_center/control_center_slam`；
  启动文件为 `multi_lidar_slam_optimized.launch.py`，328默认话题后缀为 `0_328`。
- 厂商已发布 `odom -> base_link`。Lua使用 `published_frame="odom"`、
  `provide_odom_frame=false`，Cartographer只发布 `map -> odom`。迁移到其他
  机器人时必须先确认厂商TF存在且连续，不能恢复两套同名里程计TF。
- 328实测双雷达周期约0.1秒，但 `scan_time` 出现约1.5微秒/0.05秒等跳变，
  Cartographer持续丢弃 earlier points。新增建图专用 `mapping_scan_adapter.py`，
  保留原始header时间、角度和距离，只将不可信的 `time_increment` 置0，按快照
  输入 `/mapping_scan/front`、`/mapping_scan/rear`，拒绝过期/重复/乱序帧。
  不改厂商原始话题，不伪造逐点时间，不声称已经完成真实运动去畸变。
  换成提供真实逐束采样时间的驱动后，应重新评估这一适配，不能照搬。
- UI与编辑保存统一以0～25为自由格、26～64为不确定格、65～100为障碍物；
  -1仍保留未知。不使用膨胀、填洞等方式将未观测区域强行变成自由空间。
- 原回退配置保持3cm，默认质量配置使用2cm；既有回环参数本轮不盲调。
- 旧地图未覆盖；修改需重启建图模块后生效。已损坏子图不能靠显示修复消除，
  建议保留旧地图后另建新图，低速闭环比较5/15/30分钟的重墙与回到起点偏差。
- 参数依据：[Cartographer配置](https://google-cartographer-ros.readthedocs.io/en/stable/configuration.html)、
  [调参方法](https://google-cartographer-ros.readthedocs.io/en/latest/tuning.html)。

## 1. 来源与原则

本包从 `multi_lidar_slam` 复制优化，旧包不修改。迁移时先确认真实传感器、TF 和地图
需求，再修改机器人编号；不要只做字符串替换后直接运行。

## 2. 迁移前收集

- 目标工作空间真实路径。
- `ROS_DOMAIN_ID`、RMW 和 CycloneDDS 网卡。
- 前后 `sensor_msgs/msg/LaserScan` 话题、频率、`frame_id` 和时间戳。
- `nav_msgs/msg/Odometry` 话题、`frame_id`、`child_frame_id` 和频率。
- `base_link` 到前后雷达的完整平移与四元数/欧拉角外参。
- 目标建图区域大小、走廊重复程度和期望地图分辨率。

## 3. 先检查系统是否已经发布雷达 TF

```bash
ros2 run tf2_ros tf2_echo base_link front_laser_frame
ros2 run tf2_ros tf2_echo base_link rear_laser_frame
ros2 topic info -v /tf_static
```

若系统已有真实标定 TF，启动时必须设置：

```text
publish_lidar_static_tf:=false
```

不要再发布同名子坐标系。若系统没有 TF，才启用包内备用值，并按目标机器人重新标定
launch 中的平移和旋转。

本包为了保证在 306 上单独启动即可建图，默认
`publish_lidar_static_tf:=true`。现场曾出现系统静态 TF 节点未运行，造成两路雷达
话题正常但 `/submap_list` 为空、RViz 显示 `No map received`。迁移时不能根据历史
状态假设 TF 存在，必须在本次启动环境中实际执行 `tf2_echo`。

306 机器人系统标定参考：

```text
front: xyz=(0.215, 0, 0.1515), roll=3.145926, pitch=0, yaw=3.145926
rear:  xyz=(-0.215, 0, 0.1515), roll=3.145926, pitch=0, yaw=0
```

## 4. 修改话题

默认 306：

```text
/topic_gv_front_lidar_0_306
/topic_gv_rear_lidar_0_306
/topic_gv_wheel_odom_0_306
```

可以不改源码，启动时覆盖：

```bash
ros2 launch control_center_slam \
  control_center_slam.launch.py \
  front_scan_topic:=/目标前雷达 \
  rear_scan_topic:=/目标后雷达 \
  odom_topic:=/目标里程计
```

## 5. 编译

```bash
cd '<workspace>'
source /opt/ros/jazzy/setup.bash

rosdep install \
  --from-paths src/control_center_slam \
  --ignore-src -r -y

colcon build \
  --base-paths src/control_center_slam \
  --packages-select control_center_slam \
  --symlink-install

source install/setup.bash
```

依赖至少包括：

```bash
sudo apt install \
  ros-jazzy-cartographer-ros \
  ros-jazzy-nav2-map-server \
  python3-pynput
```

## 6. 静止安全测试

```bash
ros2 launch control_center_slam \
  control_center_slam.launch.py \
  open_rviz:=false
```

检查 Cartographer 日志中不能出现：

```text
points2 waited too long
Lookup would require extrapolation
frame does not exist
Wrote checkpoint with unfinished submaps
```

检查：

```bash
ros2 node list | grep cartographer
ros2 topic hz /map
ros2 topic echo /submap_list --once
ros2 topic info /scan_matched_points2
```

## 7. 实车闭环验收

1. 静止 3～5 秒后开始。
2. 低速走完整闭环，至少回到起点一次。
3. 记录运行 5、15、30 分钟后的墙体厚度和轨迹偏差。
4. 检查日志是否产生 local/global constraint 和 optimization。
5. 起点回环后观察旧墙体是否收敛为一条，而不是保留双墙。
6. 完成后调用 `finish_trajectory`，等待最终优化，再保存 pbstream 和地图。

只做静止启动不能证明“长期不漂移”；必须完成闭环路线测试。

## 7.1 建图键盘控制

本包默认启动 `mapping_keyboard_teleop.py`，向
`/topic_gv_target_cmd_vel_0_306` 发布 `geometry_msgs/msg/Twist`。迁移时必须把
`keyboard_cmd_vel_topic` 改为目标机器人真实接收的速度话题。

```text
W/S 前后，A/D 旋转，Q/E 横移
Z/C 调速，X/空格急停
默认 0.18 m/s、0.45 rad/s
```

桌面模式依赖 `python3-pynput` 和有效图形会话；SSH 下可在交互式终端单独运行：

```bash
ros2 run control_center_slam mapping_keyboard_teleop.py
```

验收时不要直接以非零速度测试代码。先把 `output_topic` 改到测试话题，确认按键按下
产生非零 Twist、松键立即产生零 Twist，再恢复真实底盘话题。若其他控制器同时持续
发布速度，应关闭其中一方或增加速度仲裁，避免指令竞争。

桌面按键监听的 `on_release` 只能更新活动按键状态，不要在回调中直接发布零速度。
X11 长按自动重复可能产生瞬时 release/press 事件；若每次 release 都立即发零速度，
目标速度流会不断插入停车脉冲，表现为底盘一顿一顿。应像参考导航包一样，由固定频率
定时器统一发布当前按键状态，真实松键最多延迟一个发布周期停车。

当前版本进一步使用 80 ms 释放消抖和 30 Hz 发布：release 先进入待确认状态，期间
若同一键再次 press 则取消释放；只有超过消抖时间才真正清除运动键。`X/空格` 急停
不经过消抖。迁移后应在伪速度话题模拟自动重复，确认持续运动阶段没有零速度脉冲。

RViz 插件 `MapSavePanel` 提供地图名称输入和两种保存方式。依赖
`rviz_common`、`pluginlib`、`qtbase5-dev`、`ament_index_cpp`、`nav2_map_server`。
不中断建图的快照流程：

```text
/write_state (include_unfinished_submaps=true)
  -> map_saver_cli (<maps>/<name>.yaml + .pgm)
```

最终流程：

```text
/finish_trajectory (trajectory_id=0)
  -> /write_state (include_unfinished_submaps=false)
  -> map_saver_cli (<maps>/<name>.yaml + .pgm)
```

`finish_trajectory` 会结束本轮建图，因此面板必须明确提示用户停稳后再点击。迁移后
需要确认 RViz 配置加载 `control_center_slam/MapSavePanel`，且功能包的
`maps` 目录可写。

面板从安装前缀反推出 `<workspace>/src/control_center_slam/maps`，只要该
源码包及 `package.xml` 存在，所有 pbstream/YAML/PGM 都保存到源码包目录；目标机器
只有安装空间而没有源码时，才回退到 `<prefix>/share/<package>/maps`。

标准 Cartographer ROS 没有“运行最终优化但保持活动轨迹继续”的服务。若需要不中断，
只能保存含未完成子图的快照，并依靠 `optimize_every_n_nodes` 在线优化；本包设为每
25 个节点一次。不要用 finish 后自动开启新 trajectory 冒充无缝继续，否则容易引入
轨迹初始位姿误差和跨轨迹重影。

RViz 默认用 `/scan_matched_points2` 观察已匹配点云，并关闭两路原始 LaserScan。
原始扫描在 `map` 中受实时 TF 修正、双雷达时间相位差和 Decay 历史帧影响，视觉上会
漂动；它不代表栅格地图本身漂移。调试原始雷达时把 Decay Time 保持为 0。

## 8. 参数调整方向

参数文件：

```text
config/multi_lidar_optimized.lua
```

- 墙体粗糙：先检查 TF；再减小 `voxel_filter_size` 或提高地图分辨率。
- 运动时墙体拖影：降低速度并确认逐点时间戳；双 LaserScan 输入保持
  `num_accumulated_range_data = 2`，不要让前后雷达交替单独插图。
- 长期渐进漂移：增加回环路线，降低 `optimize_every_n_nodes`，检查约束构建是否积压。
  若目标机器人里程计协方差为 `-1`，不要直接提高全局里程计权重；先用闭环实测校准。
- 漏回环：小幅降低 `constraint_builder.min_score`，每次最多调整 0.02。
- 错误回环：提高 `min_score/global_localization_min_score`，不要盲目增加回环权重。
- 轮子打滑后拉不回来：降低局部平移先验权重，扩大相关匹配窗口。
- 轨迹抖动：适当提高里程计和旋转先验权重，先确认里程计没有跳变。

每次只改一组参数，并保存对应 rosbag、pbstream 和日志。

## 9. 本次 306 机器人诊断基线

2026-09 精度版通用参数基线：0.03 m栅格、12 m有效量程、0.15 m / 12°在线搜索、
每20节点优化、0.7回环采样率、8 m约束距离。306/307的轮式里程计均为约62.5 Hz，
但消息协方差为 `-1`，所以全局里程计平移/旋转权重采用 `2e3 / 5e3`，不能按高可信
编码器数据处理。迁移后必须通过完整闭环复核，不能只依据静止测试。

```text
前雷达：约 10 Hz，720 点，0.5°角分辨率
后雷达：约 10 Hz，720 点，0.5°角分辨率
最近雷达时间戳差：约 35 ms
轮式里程计：约 62.5 Hz
静止 10 秒里程计漂移：0 m / 0°
里程计 covariance：-1（未提供有效协方差）
```

旧包最重要的问题是 283 话题、重复且错误的雷达 TF、过度点云稀疏、
回环阈值过高及位姿图优化过慢。迁移时必须逐项重新核对，不能假设目标机器人相同。

本次优化包在 306 机器人上的静止验证结果：

```text
测试时长：约 42 秒
/map：0.04 m 分辨率，约 2 Hz
submap 0 version：9 -> 21（约 0.4 节点/秒）
map -> odom：前后均为 0 m / 0°
Cartographer：约 0.4 CPU 核
核心进程：正常退出，0 ERROR
```

Cartographer 偶尔报告 `Dropped ... earlier points`。现场已确认两路雷达各只有一个
发布者；该警告主要来自前后雷达约 35 ms 相位差、不同扫描时长以及逐点时间窗口重叠。
不要为了消除日志而直接改写雷达时间戳。若目标机器人上该警告持续每帧出现，应检查
雷达驱动硬件时钟/时间同步，而不是先放宽 SLAM 参数。
