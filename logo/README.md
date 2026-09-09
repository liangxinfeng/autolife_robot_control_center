# 云蝶科技机器人胸屏 Logo 页面

这是一个完全离线的单页展示界面，视觉结构参考机器人原胸屏：深黑背景、中央蓝青色能量主体、上下边缘弱状态装饰。

## 效果

- 云蝶 Logo 保持清晰，无复制残影或扫描线
- 蝴蝶图案不保留静态底图，左右翼间歇进行三次大幅自然扇动，文字保持稳定
- 蝴蝶图案相对文字放大 16%，增强胸屏识别度
- 沿 Logo 蝶翼/无限环形态流动的粒子轨迹
- 中央多层蓝青能量波纹
- 轻触屏幕产生一次能量扩散
- 右上角可随时进入或退出全屏；全屏状态也可按 `Esc` 退出
- 自动适配竖屏和横屏，支持系统“减少动态效果”设置
- 尝试保持屏幕常亮

## 本机预览

```bash
cd /home/ubuntu/ros2_ws/src/autolife_robot_control_center/logo
python3 -m http.server 8088 --bind 127.0.0.1
```

浏览器打开：

```text
http://127.0.0.1:8088/
```

所有资源都在本目录中，不需要网络或第三方依赖。

## 一键启动

在文件管理器中双击：

```text
启动云蝶Logo.desktop
```

它会自动启动本地网页服务，并把 Logo 页面弹出到一个独立、最大化的标准 Chrome 新窗口。重复启动会复用同一个 8088 服务并再次唤起浏览器窗口，不会关闭或替换原有的 AutoLife 胸屏程序。

如果 Ubuntu 文件管理器没有直接执行 `.desktop` 文件，可右键选择“作为程序运行”，或在终端执行：

```bash
gio launch /home/ubuntu/ros2_ws/src/autolife_robot_control_center/logo/启动云蝶Logo.desktop
```

也可以直接运行底层启动脚本：

```bash
bash /home/ubuntu/ros2_ws/src/autolife_robot_control_center/logo/start_logo_display.sh
```

综合控制中心的“系统服务”页面也可以直接开启或关闭 Logo 网页。

- 点击“开启 Logo 网页”时，胸屏先切换为 `1024×768`、`Portrait Left`。
- 点击“关闭 Logo 网页”或 Logo 服务异常结束时，胸屏恢复为
  `1920×1080`、`Landscape (flipped)`。
- 显示脚本自动选择当前 primary 输出，不依赖固定的 HDMI 接口名称。

迁移整个 `autolife_robot_control_center` 到新机器人后，执行一次：

```bash
bash /home/ubuntu/ros2_ws/src/autolife_robot_control_center/logo/install_logo_service.sh
```

该脚本只安装 user systemd 单元并刷新服务列表，不会自动开启 Logo 网页。
