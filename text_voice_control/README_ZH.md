# text_voice_control 中文使用说明

## 1. 功能说明

`text_voice_control` 是一个 ROS2 文本语音控制功能包，用来复用机器人已有的
ASR/TTS 语音链路，实现“通过文本指令控制机器人语音输出”。

本包不重新实现语音识别或语音合成算法，只做接口桥接：

- 输入文本指令：订阅 `/text_voice_command`。
- 输出 TTS 指令：发布到机器人已有 `/topic_tts_{ROS_DOMAIN_ID}_{ROBOT_ID}`。
- 可选 ASR 桥接：订阅 `/topic_chat_text_{ROS_DOMAIN_ID}_{ROBOT_ID}`，识别到指定前缀后转发给 TTS。

当前机器人默认：

- `ROS_DOMAIN_ID=0`
- `ROBOT_ID=307`
- TTS 话题：`/topic_tts_0_307`
- ASR 文本话题：`/topic_chat_text_0_307`

## 2. 已实现内容

- `text_voice_control_node`
  - 常驻节点。
  - 订阅 `/text_voice_command`。
  - 支持普通文本、JSON 播放指令、停止指令。
  - 转发到机器人已有 TTS 话题。

- `say_text`
  - 命令行工具。
  - 可直接让机器人播报一段文本。
  - 可发送停止播报命令。

- `asr_tts_bridge_node`
  - 可选节点。
  - 订阅已有 ASR 文本话题。
  - 当识别文本以 `播报`、`说`、`朗读`、`请说` 开头时，把后续内容转发给 TTS。

## 3. 构建

```bash
cd ~/ros2_ws
source /opt/ros/jazzy/setup.bash
colcon build --packages-select text_voice_control
source install/setup.bash
```

如果工作空间里有重复包名目录，可以只构建 `src` 下本包：

```bash
colcon build --base-paths src/autolife_robot_control_center/text_voice_control \
  --packages-select text_voice_control
```

## 4. 启动常驻文本控制节点

```bash
cd ~/ros2_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch text_voice_control bringup.launch.py robot_id:=307 ros_domain_id:=0
```

启动后，向 `/text_voice_command` 发布文本即可播报：

```bash
ros2 topic pub --once /text_voice_command std_msgs/msg/String "{data: '你好，我是机器人'}"
```

发送停止播报：

```bash
ros2 topic pub --once /text_voice_command std_msgs/msg/String "{data: '停止播报'}"
```

也可以发送 JSON：

```bash
ros2 topic pub --once /text_voice_command std_msgs/msg/String "{data: '{\"status\":\"play\",\"text\":\"欢迎光临\"}'}"
ros2 topic pub --once /text_voice_command std_msgs/msg/String "{data: '{\"status\":\"stop\"}'}"
```

## 5. 直接命令行播报

```bash
cd ~/ros2_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 run text_voice_control say_text --text "你好，欢迎使用语音播报功能"
```

停止当前播报：

```bash
ros2 run text_voice_control say_text --stop
```

指定机器人编号：

```bash
ros2 run text_voice_control say_text --robot 283 --domain 0 --text "测试播报"
```

## 6. 启用 ASR 到 TTS 的桥接

默认 launch 不启用 ASR 桥接，避免环境里的普通说话误触发机器人播报。

需要启用时：

```bash
ros2 launch text_voice_control bringup.launch.py robot_id:=307 ros_domain_id:=0 enable_asr_bridge:=true
```

启动后，可以对机器人说：

```text
播报 欢迎光临
说 当前任务已完成
朗读 请注意安全
```

节点会把前缀后面的内容转发到 TTS。

## 7. 复用的已有机器人接口

已有 TTS 示例说明机器人支持以下话题：

```text
/topic_tts_{ROS_DOMAIN_ID}_{ROBOT_ID}
```

消息类型：

```text
std_msgs/msg/String
```

支持格式：

```json
{"status": "play", "text": "要播报的内容"}
{"status": "stop"}
```

已有 ASR 示例说明机器人识别文本来自：

```text
/topic_chat_text_{ROS_DOMAIN_ID}_{ROBOT_ID}
```

消息类型同样为：

```text
std_msgs/msg/String
```

## 8. 参数

`text_voice_control_node` 参数：

```yaml
ros_domain_id: "0"
robot_id: "307"
input_topic: "/text_voice_command"
tts_topic: ""
publish_json: true
```

`asr_tts_bridge_node` 参数：

```yaml
ros_domain_id: "0"
robot_id: "307"
asr_topic: ""
tts_topic: ""
trigger_prefixes: ["播报", "说", "朗读", "请说"]
publish_json: true
```

`tts_topic` 或 `asr_topic` 留空时，节点会自动按机器人规则生成：

```text
/topic_tts_{ROS_DOMAIN_ID}_{ROBOT_ID}
/topic_chat_text_{ROS_DOMAIN_ID}_{ROBOT_ID}
```

## 9. 调试命令

查看节点：

```bash
ros2 node list
```

查看 TTS 话题：

```bash
ros2 topic list | grep tts
```

监听 TTS 指令：

```bash
ros2 topic echo /topic_tts_0_307
```

查看 ASR 文本：

```bash
ros2 topic echo /topic_chat_text_0_307
```

## 10. 注意事项

- 机器人已有的语音服务必须运行，否则本包发布了 TTS 指令也不会真正出声。
- 若 `/topic_tts_0_307` 没有订阅者，应检查机器人 vision/voice 服务是否启动。
- 若 ASR 没有输出，需要检查机器人已有 ASR 配置、API Key、麦克风和相关服务。
- ASR 桥接默认关闭，因为它会把带前缀的识别结果变成播报动作。
- 本包只控制“说什么”和“停止说”，不负责生成音频文件、不负责选择具体 TTS 模型。
