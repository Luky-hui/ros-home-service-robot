---
title: face_rec功能包V1.0.0 说明
date: 2025-09-10 16:52
summary: 基于ros和face_recognition实现的人脸识别功能包。
tags:
  - ros
  - 人脸识别
---
## 1. 使用说明：
当需要添加新的人脸识别库时，只需要在face_data目录下创建一个新的目录，并将其命名为该人脸的类别名。并且向库中添加一个或多个同类别人脸样本。
## 2. 消息定义：
face_data.msg
```yaml
std_msgs/Header header
  uint32 seq
  time stamp
  string frame_id # 标签
# 人脸框数据
float64 xmin
float64 xmax
float64 ymin
float64 ymax
```
face_results.msg

```yaml
face_rec/face_data[] face_data # 人脸数据
```

## 3. 服务定义
recognition_results.srv
```yaml
int8 mode  # 分为三种模式 0:使用默认的相机话题；1:可指定相机话题；2:可指定图片识别
string str # mode为1和2的补充
---
face_rec/face_results result
string message
bool success
```
## 4. 功能说明
功能|启动文件|
--|--
上传人脸数据|encode_face.launch
实时人脸检测|face_detection.launch
人脸验证服务端|face_verification.launch
实时人脸识别|face_rec_topic.launch

