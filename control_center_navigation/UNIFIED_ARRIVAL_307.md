# 307到达确认与播报统一（2026-09-14）

## 证据

语音转换节点对已发布的到达消息均记录了Forwarded TTS play command。
未播报的案例在任务层先失败：xy=0.061m；或xy=0.037m、yaw=0.061rad。
根本矛盾是Nav2成功后另取一帧TF复核，边界附近的定位/停车变化会推翻先前判定。
不能据此保证扬声器每次发声；当前修复针对已证实的“没有产生到达消息”。

## 新流程

ExactGoalSmacPlanner真实航点 → SettledMPPIController（原MPPI内限速、避障）
→ 主动停止 → SettledGoalChecker连续0.35s确认 → Nav2成功
→ 任务/界面/语音统一消费同一结果，只发一次。

- 位置成功门槛仍5cm，方向0.05rad；同时检查地图坐标误差和局部控制坐标误差。
- 内部制动瞄准2.5cm/0.035rad，为停车留余量；保持区最多5cm/0.05rad。
- 只有线速度<=0.015m/s、角速度<=0.025rad/s且连续确认才成功。
- 地图TF时间超过0.3s时拒绝继续控制；判定输入超过0.2s不能成功。
- MPPI优化前施加连续接近限速，并保留外部更严格限速；不是对优化后的轨迹随意缩放。
- 重规划同一目标不丢失停止状态；换目标/取消后状态清除。
- 仍未到位则由同一个MPPI控制器处理，没有额外发起短距离精调任务。
- 任务参数nav2_settled_arrival_authoritative必须与两个Settled插件成套启用，不能单独用于旧控制器。
- 充电专用对接流程保持原有处理。

## 验证

编译通过，84项Python测试及arrival_settle_regression、exact_goal_endpoint_regression、
settled_checker_regression三项C++专项通过。未替用户发运动目标。
物理停车误差、人员遮挡等场景仍需用户实机复测，不能把定位坐标5cm等同于实际绝对精度保证。

备份：/home/ubuntu/ros2_ws/backup_navigation/before_unified_arrival_20260914.tar.gz
重开控制中心导航模块后生效，地图与航点未更改。
