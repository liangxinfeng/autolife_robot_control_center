# 导航头部RGB预览（283，2026-09-24）

- 导航地图工具栏最左侧“RGB画面”：开关地图/RGB左右并排，分隔条可拖动。
- 视频区域“放大画面”或双击视频：视频占满原地图显示区；“恢复并排”或Esc返回。
- 再次点击“RGB画面”：恢复原地图布局，保留地图数据、编辑、缩放及航点。
- 视频获得焦点后，W/S前后、A/D转向、Q/E横移、X停止、空格取消地图导航，仍沿用原控制权与速度限制。
- 切换视图、离开窗口/导航页面会清除手动按键，防止遗漏松键；不会自动发起运动。
- 默认不开视频。隐藏视频/切换页面暂停读取；退出关闭读取器。视觉服务不可用时只显示提示，不启动或重启服务。
- 后台线程读取现有 `rgbd_head_color` 共享内存，只保存最新帧，不向ROS发送图像。预览上限1280像素宽、12.5fps，不改变原生采集分辨率。超过1秒无新帧清掉旧画面。
- 默认SDK路径为 `~/miniconda3/envs/robot_env/lib/python3.12/site-packages`，可用 `AUTOLIFE_CAMERA_PYTHON_PATH` 指定已安装的兼容路径。

改动仅涉及 `scripts/navigation_ui_node.py`、新增 `scripts/head_rgb_panel.py` 及其CMake安装条目；不更改Nav2、底盘或机械臂服务。

离线测试 `test/test_nav_rgb.py` 使用FakeNode，不创建ROS控制节点；`--live`额外只读相机验证取流、暂停和恢复。使用 `QT_QPA_PLATFORM=offscreen` 运行。
