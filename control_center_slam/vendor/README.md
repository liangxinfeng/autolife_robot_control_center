# 328多雷达建图射线修复（2026-09-07）

## 增量建图附加更新

新增增量Lua，使用上游重叠子图修剪逻辑；私有core修复覆盖格数uint16溢出和
子图数无符号减法。增量配置下，LoadState将导入的无传感器轨迹标为FINISHED，
新观测使用新轨迹。新增两项修剪测试，全部38项C++回归通过。
普通新建与未启用重叠修剪配置时保留原行为。使用说明见包根目录
`328增量建图使用说明.md`，不是对现有地图文件的就地修改。

## 第三轮：距离相关清空证据

`multi_lidar_quality.lua`新增`distance_weighted_free_space`。射线每个经过的格子
按距真实雷达原点的距离分档：≤1.5m仍为原miss=0.47，1.5～2.75m为0.476667，
2.75～4m为0.483333，>4m为0.49。越接近0.5，单次扫空证据越弱；命中0.60不变。
这是针对远处细物体漏扫的保守启发式，不是增加真实激光分辨率。

同批次全部命中优先，再依近到远写入空闲证据，每格只更新一次；多雷达排序
不会让远处弱证据抢占近处强证据。查表预计算，不对每格计算对数或平方根。
真正搬走的物体仍可清除，但远处可能清除更慢，近处维持原速。没有栅格冻结、
图像加粗或关闭避障。缺省未配置、旧无rays数据继续原逻辑。

新增7项测试，总计29项C++回归通过。外参核查及实测说明见
`328雷达外参与细障碍物保留优化.md`。新配置依赖新版私有二进制，迁移时必须
同时带runtime、config和vendor源包；不能只替换Lua给系统Cartographer使用。

## 第二轮质量改进

默认入口现在使用multi_lidar_quality.lua，2cm栅格，叠加按采集时间查询历史里程计的
写图质量门控。高于0.22m/s或0.10rad/s时继续定位、不写栅格；低运动持续0.2秒后恢复。
无新鲜里程计时暂停写图。门控6项测试覆盖快转、快移、过期、乱序、恢复以及
多雷达整理后扫描时间早于最新里程计的情况。最新源包内包含此逻辑。

Python适配器在确认静止时使用3帧时域中值，运动不滤，保留新物体距离跳变、
无效距离以及单点细腿；8项Python测试位于功能包test目录。
这不是逐束运动去畸变；真实雷达时间和外参仍需厂商/实物标定支持。

## 范围

仅替换 `control_center_slam` 的建图执行文件。系统Cartographer、导航、底盘和
机械臂服务不修改。运行中的旧建图不会被自动停止，原地图不覆盖。

原版2D汇总多雷达数据后，用跟踪坐标系的单个原点清空所有射线。
328两雷达距底盘原点分别约±21.5cm；某些清空线因此会穿过实际没被扫空的柱子边缘。
修复保留每条原始有效射线的原点、终点和命中类型，经过位姿变换、Z裁剪后
用于栅格插入。匹配仍使用原先的体素降采样。命中优先，每次插入每格只更新一次。
没有关闭空闲更新、无限保留障碍、加粗障碍、虚填灰区或放宽避障。

## 上游版本及交付

- [ros2/cartographer](https://github.com/ros2/cartographer/tree/2.0.9004)：
  `1489fc933b6e0b1da936ca08af462abe559dd02e`，2.0.9004。
- [cartographer_ros Jazzy发布源码](https://github.com/ros2-gbp/cartographer_ros-release/tree/release/jazzy/cartographer_ros/2.0.9003-2)：
  `220cc62d9b410d777c4bd483e9e4861e1a3325d7`，2.0.9003-2，未改ROS桥接代码。
- 两份tar.gz包含完整构建源码及原始Apache-2.0许可，core包内含修改及回归测试。
  `SOURCES.sha256`校验源包；`runtime/cartographer_multi_origin_node`是x86_64/Jazzy产物。
- 修改文件：sensor/range_data.{h,cc}、sensor/proto/sensor.proto、
  mapping/internal/2d/local_trajectory_builder_2d.cc、
  mapping/2d/probability_grid_range_data_inserter_2d.cc。
- RangeData增加可选rays字段和protobuf字段6；旧调用者的空rays沿用原逻辑。
  3D管线不改；已保存的子图栅格及地图文件格式不变。

## 重新构建与依赖

在功能包源目录运行 `bash scripts/build_multi_origin_cartographer.sh`。
需要Ubuntu24.04/Jazzy的构建工具及 `ros-jazzy-cartographer`、
`ros-jazzy-cartographer-ros` 的开发依赖（Ceres、Eigen、Lua、protobuf、absl、
PCL、gflags/glog、Cairo、gtest/gmock及ROS消息/TF/rosbag2等）。
现有328已具备；新机器先用对应package.xml运行rosdep安装缺失依赖，不能混入Conda的CMake/Python。
完成后 `colcon build --packages-select control_center_slam`。
默认使用4个低优先级编译任务，不会启动建图或发送运动命令。

回退启动参数：
`cartographer_package:=cartographer_ros cartographer_executable:=cartographer_node cartographer_configuration_basename:=multi_lidar_optimized.lua map_resolution:=0.03`。

## 验证与限制

- 新增9项多原点测试及7项原有ProbabilityGrid测试通过。
- 错误起点对照：同样60次观测，旧逻辑擦掉模拟柱子边缘，新逻辑保留；
  真正空闲位置仍能清除。另测跨雷达命中优先、重复点、越界扩图、变换裁剪和序列化。
- 35秒隔离实雷达运行：前后输入约10Hz，输出349帧匹配点云、1666次TF；
  点云间隔中位数0.100s、P95为0.111s；启动TF缓冲暖机后无持续TF故障。
  首次map→odom跨度约4.3mm/2.7mm/0.033°；安装后另测35秒仍输出349帧，
  P95间隔0.111s、最大0.122s，map→odom跨度18.7mm/9.3mm/0.497°。
  未做受控运动/静止对照，这些是当次TF变化范围，不是地图精度或长期漂移保证。
- 根因修复不能纠正已损坏旧子图；需保留旧图并新建地图绕同一柱子多视角验证。
  真实逐束雷达时间、外参标定、里程计误差及长时间回环仍影响墙厚和漂移。
  不应宣称消除了所有漏检；椅腿还受角分辨率及遮挡影响。
