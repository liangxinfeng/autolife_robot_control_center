# 306导航启动失败修复（2026-09-14）

17:29启动日志明确报：`executable 'waypoint_manager_node.py' not found on the libexec directory`。
实际不是文件缺失：306的安装入口是指向源码的旧软链接，Windows打包同步使源码脚本失去执行权限。
编译和内容哈希检查不能发现此问题；启动抛异常后留下AMCL等节点，下一次启动就出现其他AMCL冲突。

处理：
- 恢复306源码 `scripts/waypoint_manager_node.py`、`scripts/navigation_ui_node.py` 的用户执行权限，安装软链接立即生效。
- 确认失败launch PID7777已经退出，剩余同组进程均为该轮导航节点；对其进程组发送SIGINT，退出残留AMCL等，不重启机械臂服务。
- `ros2 pkg executables control_center_navigation` 已能发现两个入口。
- 328安装入口是正常可执行的实体文件，未遇到该权限故障，本次未改变328运行状态。

后续迁移必须在目标Linux执行 `test -x` 或ROS可执行入口查询；不能只校验文件存在、哈希和编译成功。已将可执行权限检查补入本地部署验证脚本。
本次没有发起导航或运动；用户可在控制中心重新点击导航“启动”。
