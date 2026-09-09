# 307 围观场景末端导航优化（2026-09-08）

范围：仅307，control_center_navigation 为 ~/vr307nav 和综合控制中心共享导航后端。未改306/328，未改地图、航点、机械臂或任务按键。

## 修改

- MPPI DiffDrive → Omni；横移上限0.12m/s、采样标准差0.08m/s、横向加速度上下限±0.30m/s²。保留原前进/转向限速、footprint、膨胀层、激光避障和定位连续性保护。
- AMCL likelihood_field → likelihood_field_prob，使已有 do_beamskip=true 真正生效。只影响定位观测选择，局部/全局代价地图仍接收行人障碍。Jazzy来源：https://api.nav2.org/nav2-jazzy/html/amcl__node_8cpp_source.html （createLaserObject）。
- 常规导航进入25cm、0.35rad约20°范围并维持0.6秒后请求末端精调；这是交接范围，不是成功范围。等待Nav2终态才释放其速度输出控制权，取消拒绝/异常不启动伺服，旧目标结果不能启动新精调。
- 精调通路受阻停车，连续畅通0.6秒才继续同一目标。连续等待上限20秒，整个精调仍有35秒上限；不通过反复重启Nav2重置预算。
- 最终仍要求地图位姿误差≤4cm、朝向误差≤0.05rad约2.9°，沿用分轴/稳定时间校验。TF过期拒绝精调。未收敛不伪报到达。
- 精调失败统一输出现有任务流程能识别的 failed 状态（旧 failed_precision 不被VR终态分类器识别）。保留长按A重试当前点位、长按B跳过的现有流程。
- 补齐旧代码调用但未定义的 _waypoint_pose_error，避免禁用自动精调时成功回调异常。

## 参数位置

scripts/waypoint_manager_node.py 声明ROS参数：
final_refine_handoff_distance=0.25、final_refine_handoff_yaw=0.35、final_refine_obstacle_wait=20.0、final_refine_clear_settle=0.60。
config/nav2_params.yaml 配置MPPI和AMCL。参数修改后需重启导航程序；本次没有中途重启正在运行的真机程序。

## 验证与边界

- 28项回归测试通过：受阻停车、连续清空后同目标恢复、间歇行人不重置预算、超时/无进展停止、手动接管取消等待、旧结果、取消与终态交接、精度外不成功。
- ROS_DOMAIN_ID=88、回环接口隔离：MPPI/AMCL真实插件configure成功并保持INACTIVE；航点节点临时空库初始化通过，没有发送导航目标。
- 隔离ROS仲裁器测试通过：正常速度放行→定位跳变停车→手动接管可用→显式重定位恢复。
- 扩展运行的旧 test_cmd_vel_arbiter_manual_override.py 有3项因其exec测试装载器缺少__file__失败，与本次未修改的仲裁器功能无关；使用上述真实ROS隔离测试补验，未据此修改生产仲裁器。
- 尚未进行有人/无人实机到达对比。4cm是定位估计上的判定阈值，不等于已经测得实物绝对误差4cm。
- 行人真实占用目标位置时不能保证立即精确到达；应等待让路或由操作员决定重试/跳过，不禁用避障或把未到达伪装成完成。

## 现场验证

机器人停稳后正常关闭并重开 ~/vr307nav。先无人场景验证电视区→耗材区，再在安全距离有观众时验证；不要站在目标点、底盘或外伸手臂运动范围。检查：目标附近是否进入精调，受阻是否停止，让路后是否继续同一点，失败重试是否仍是原目标。对实际位置与定位误差分别测量。

## 部署记录

- 307备份：/home/ubuntu/ros2_ws/migration_backups/nav307_crowd_sL7Ss148。
- 原工作区使用普通安装；首次尝试symlink安装遇到已有生成目录冲突，已改回AMENT_CMAKE_SYMLINK_INSTALL=OFF，编译成功，没有删除任何构建目录或用户文件。
- 源码/安装脚本与配置逐字节一致。安装目录加载的模块再跑28项相关测试通过。旧进程不重启，因此需要用户停稳后关闭再开启程序才能使用新参数和逻辑。
