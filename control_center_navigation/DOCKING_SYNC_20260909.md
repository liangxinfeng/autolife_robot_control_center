# 回充同步：307 → 306 / 328（2026-09-09）

共享导航包 control_center_navigation 已同步桩前精调、倒车纠偏与过偏停车，保留目标机器人编号和地图航点。

本次包含关键修复：Nav2 1.3.10 倒车时错误地将控制目标投影到桩前25厘米。现在先按原充电桩坐标方向延伸目标，再翻转倒车控制朝向。仅自研导航启动本包 docking_server_compat；没有替换 /opt/ros 文件或厂商服务。

源代码与来源说明：src/docking_compat/README.md。入口：launch/bringup_navigation.launch.py。保护阈值和速度保留307验证版，未放宽碰撞及漂移保护。

使用前提：“充电桩”航点为已接触充电时保存的机器人位姿，而不是桩前等候位置。没有修改任何航点。安全退出并重新启动导航后生效；首轮真机回充必须有人监护。离线回归、控制律模拟和隔离生命周期验证不等于真机充电成功验证。

回滚备份（源包及原安装包）：
- 306：/home/ubuntu/ros2_ws/migration_backups/docking_sync307_bRiR9l
- 328：/home/ubuntu/ros2_ws/migration_backups/docking_sync307_bgrNqy
