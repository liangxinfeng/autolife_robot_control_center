# 306、328导航同步307（2026-09-08）

参照307当前 control_center_navigation 与VR导航launch，保留目标机器人编号及所有地图、航点、active_map、任务库、硬件配置。未修改V4手臂控制、RGB、控制权逻辑或厂商服务。

逐文件核对：导航运行代码、Nav2/AMCL配置、全局重定位、雷达外参与仲裁在编号归一化后已一致，仅需同步waypoint_manager_node.py的最新末端收敛逻辑，并将两台VR launch的short_distance_direct_start_enabled改为false。控制中心入口原本已为false。

- 所有目标先常规Nav2；接近目标后沿用末端精修。
- 末端位置满足径向4cm后不再追逐2cm单轴阈值。
- odom和map位置均≤4cm、方向均≤0.05rad后停车，连续稳定0.5s完成。
- XY增益1.2、Yaw增益1.5；原最高速度、加速度、避障、超时和失败语义不变。
- 坐标误差判定阈值不是实物绝对精度保证，未做真机移动验收。

两台各50项离线回归测试通过；control_center_navigation和openarmx_teleop_vr_navigation_306编译通过，安装脚本与源文件逐字节一致。完整快照diff只显示预期两处运行文件变化，地图及active_map未变化。

备份（均含source_before.tgz和install_before.tgz）：
- 306：/home/ubuntu/ros2_ws/migration_backups/nav307_latest_bGdEIPyP
- 328：/home/ubuntu/ros2_ws/migration_backups/nav307_latest_RA1HY2J1

未重启当前程序、未发送运动命令。重新开启各自控制中心导航或~/vr306nav、~/vr328nav后加载新版本。306其他zeng/厂商导航进程未处理。
