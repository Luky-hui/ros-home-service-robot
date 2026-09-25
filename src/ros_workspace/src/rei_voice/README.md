## 语音交互大模型版本

### 一、 服务接口

#### 1. /REIService/voice_tts
##### 1.1 介绍
输入一段文本，合成语音文件，输出文件路径。
服务类型为：reivoice_ai/REITts

##### 2.1 消息类型
```bash
# REITts.srv
string text         # 需要合成的文本
bool is_play        # 是否播放合成音频
---
bool success        # 服务响应结果 
string filePath     # pcm音频路径
string message      # 服务响应消息
```
#### 2. /REIService/PcmPlayer
##### 2.1 介绍
指定一段pcm音频并播放
服务类型为：reivoice_ai/REIPlayer

##### 2.2 消息类型
```bash
# REIPlayer.srv
string PcmPath      # 需要播放的pcm音频文件
---
bool success        # 服务响应结果 
string message      # 服务响应消息
```
#### 3 /REIService/RecordAudio 
##### 3.1 介绍
```bash
# std_srvs/SetBool
bool data           # 是否开启音频采集
---
bool success
string message

```

### 二、消息结果
#### 1 /REITopic/result
##### 1.1 介绍
AIUI结果事件反馈，type为当前的结果事件，包括：‘iat’、‘nlp’、‘tts’三个事件。事件顺序为‘iat‘-->’nlp’-->‘tts’。

intent 为意图列表，单个意图包含：意图名（导航意图或控制意图）、识别到的问题、技能变量。

##### 1.2 消息类型
```bash
string sid      # 会话唯一标识
string type     # 结果类型
string iat      # 语音识别结果
reivoice_ai/REIResultNlp[] intent       # 意图列表
    string name                         # 意图类型
    string query                        # 问题
    string[] slots_name                 # 技能变量名
    string[] slots_value                # 技能变量值
string anwser                           # nlp回答
```

#### 2 /REITopic/AIUIState
##### 2.1 介绍
现在可支持四种状态的反馈，包括："EVENT_WAKEUP"、"EVENT_STATE: STATE_IDLE"、"EVENT_STATE: STATE_READY"、"EVENT_STATE: STATE_WORKING"
##### 2.2 消息类型
```bash
# std_msgs/String
string data         # AIUI状态反馈
```