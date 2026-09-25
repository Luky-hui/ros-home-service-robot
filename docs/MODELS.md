# 模型说明

## 任务二蔬菜识别

配置文件：`src/bobac3_ws/src/raicom_vision/config/vision_raicom16.yaml`

当前部署模型：

- `models/veg14_v15_newonly/veg14_v15_newonly_best.onnx`

## 任务三手机/书包识别

配置文件：`src/bobac3_ws/src/raicom_vision/config/vision_phone_bag_v5_find_service.yaml`

当前部署模型：

- `models/phone_bag_v6_combined/phone_bag_yolo11n_v6_combined_best.onnx`

## 语音相关

- `models/sensevoice/`：离线 ASR。
- `models/melo_tts/`：本地 TTS。
- `models/vad/`：语音活动检测。
- `models/sherpa-onnx-kws-zipformer-zh-en-3M-2025-12-20/`：关键词唤醒相关文件。
