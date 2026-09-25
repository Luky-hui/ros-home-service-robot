voice_aiui_node共提供了三个服务：语音听写，语音理解，语音合成
1 语音合成服务：robot_tts.srv
string text
bool play
---
string audiopath
其中：
  text：要合成的文本
  play：是否播放合成的音频
  audiopath：合成音频所在的路径

2 语音听写服务：robot_iat.srv
string audiopath
---
string text
其中：
    audiopath：要听写的音频所在路径
    text：听写完成的文字信息

3 语音理解服务：robot_semanteme.srv
int32 mode
string textorpath
---
string tech
string iat
string anwser
string intent
string[] slots_name
string[] slots_value
其中：
    mode为模式：1为文字输入，2为音频输入
    textorpath:mode为1时，此处为要语义识别的文字，mode为2时，此处为音频所在的路径；
    ---
    tech：技能，system为系统技能，user为用户自定义技能
    iat：听写内容；
    anwser:语义识别后，回答的内容；
    intent：自定义意图，当tech为user时，这个有效，目前系统有两个自定义意图其中：robot_nav为机器人导航，robot_control为机器人控制
    slots_name:自定义意图所附加的语义实体，当自定义意图为robot_control的时候，slots_name里面有三个值，第一个值为：robot_control表征机器人如何运动，取值有：左转、右转、左移、右移，前进，后退等。第二个值为：number，表征移动的数据。第三个值为：unit_length表征单位，如厘米，米，度等。
               当自定义意图为：robot_nav的时候，slots_name里面有两个值，第一个值为action_move，取值为：导航到，带我去等。第二个值为：nav_pos，为具体的导航地点。
    slots_value：slots_name对应的具体值。
  
    
