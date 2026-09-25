# local_voice_bridge

本包替代收费 AIUI Token，覆盖第 6～10 课所需的中文语音唤醒、语音交互、语音控制、语音导航、文本理解与语音合成。

## 本地引擎

- 中文离线识别：SenseVoice INT8
- 中文离线合成：sherpa-onnx + MeloTTS `vits-melo-tts-zh_en`
- 文本理解：赛题规则与课程接口对应的本地确定性意图解析

## 保留的课程接口

- `/REIService/RecordAudio`：`std_srvs/SetBool`
- `/REIService/PcmPlayer`：`rei_voice/REIPlayer`
- `/REIService/voice_nlp`：`rei_voice/REINlp`
- `/REIService/voice_tts`：`rei_voice/REITts`
- `/REITopic/AIUIState`：`std_msgs/String`
- `/REITopic/AIUIResult`：`rei_voice/REIResult`

同时保留 `robot_audio` 课程使用的 `/voice_awake`、`/voice_collect`、`/voice_iat`、`/voice_aiui`、`/voice_tts`、`/voice_up_sync`。

## 启动

一键关闭旧的收费语音节点并启动第 6～10 课：

```bash
rosrun local_voice_bridge start_lessons_6_10.sh
```

服务组迎宾任务：

```bash
roslaunch local_voice_bridge welcome_local.launch face_mode:=any
```

服务组巡检任务：

```bash
roslaunch local_voice_bridge patrol_local.launch face_mode:=admin
```

本地识别结果发布到 `/service_group_mission/voice_command`，直接接入现有任务总控。

## 第 6～10 课对应关系

- 第六课：唤醒词“元宝”“魔力元宝”，并发布 `EVENT_WAKEUP`
- 第七课：麦克风持续监听、VAD 分段、中文离线识别、回答播报
- 第八课：解析前进、后退、左移、右移、左转、右转及距离/角度，调用 `/voice_control`
- 第九课：解析点位名称、取消导航和整体参观，调用 `/voice_nav`
- 第十课：本地规则语义服务与 MeloTTS 中文合成，不调用收费 Token

## 验证

```bash
rosrun local_voice_bridge voice_smoke_test.py
```
