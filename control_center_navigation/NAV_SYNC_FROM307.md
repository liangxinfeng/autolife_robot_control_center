# 306/328 同步307当前导航版本（2026-09-08）

基准：307取消前端25cm延伸后的control_center_navigation，不是旧的75cm长轮廓版本。

同步文件：scripts/waypoint_manager_node.py、scripts/short_distance_servo.py、config/nav2_params.yaml、launch/bringup_all.launch.py中的默认短距开关，及匹配的6份测试文件。
其余导航实现经对比一致或只包含机器编号差异，无需覆盖。

## 一致行为

- 常规MPPI Omni全向规划，横移最大0.12m/s、横向加速度±0.30m/s²。
- AMCL likelihood_field_prob与已有beamskip配置配套。
- 临近目标25cm、朝向0.35rad以内并维持0.6秒后准备交接精调；必须等Nav2终态才发送伺服命令。
- 最终地图位姿判定仍为4cm/0.05rad；受阻停车，通路稳定0.6秒后继续同一点。等待最多20秒、整段精调最多35秒，不通过反复重启Nav2无限重置。
- 不可达/感知异常/无进展不伪报完成，输出任务流程识别的failed状态，保留原VR重试当前点/主动跳过逻辑。
- 轮廓前后±0.25m、左右±0.25m，额外每侧0.04m余量，front_extension=0。局部和全局膨胀半径0.65m。此半径是代价场而非机器人尺寸。
- 与307相同，控制中心通用launch默认先Nav2再精调；VR自身launch显式short_distance_direct_start_enabled=true，保留2m内直接短距离导航。旧的候选分轴排序实现替换为307当前分轴实现，不混用两版控制逻辑。

## 保留本机数据

电池、前后激光、里程计、速度输出、语音等话题仍各自使用306/328编号。
不覆盖active_map.yaml、地图/航点库、config/waypoints.yaml、充电参数、雷达外参、全局重定位配置、语音配置、用户任务库或桌面入口。
只涉及自研control_center_navigation；不修改厂商后台导航服务，不调整机械臂/头部/腰腿遥操。

## 验证

两台分别42项回归测试通过；ROS_DOMAIN_ID=88、回环接口隔离，真实Nav2组件configure后保持INACTIVE，航点节点只使用临时空库，没有下发真机运动。
正式部署前逐文件确认源快照未改变，备份替换文件和原快照；编译后核对source/install一致、地图及保留配置哈希不变。
没有中途重启程序；停稳后关闭再开启各自~/vr306nav或~/vr328nav，控制中心导航也会使用同一后端。
实际到达精度尚需各机器人现场测试；不能把4cm软件阈值当作已测得的实物绝对误差。
