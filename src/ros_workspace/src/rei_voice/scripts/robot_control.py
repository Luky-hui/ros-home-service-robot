#! /usr/bin/env python3
# -*- coding: utf-8-*

import rospy
from relative_move.srv import SetRelativeMove,SetRelativeMoveRequest
from rei_voice.msg import REIResult,REIResultNlp
from std_srvs.srv import SetBool,SetBoolRequest
from rei_voice.srv import REIPlayer,REIPlayerRequest,REINlp,REINlpRequest
from std_msgs.msg import String
import time

class voice_control():
    def __init__(self):
        self.sid = ''
        self.type = ''
        self.state = ''
        self.playtts = True 

        self.task_list = []
        rospy.init_node("voice_control")
        self.recordAudio_client = rospy.ServiceProxy("/REIService/RecordAudio",SetBool)
        self.player_client = rospy.ServiceProxy("/REIService/PcmPlayer",REIPlayer)
        self.nlp_client = rospy.ServiceProxy("/REIService/voice_nlp",REINlp)

    def addtask(self,intents):
        taskdir = {}
        for task in intents:
            taskdir[str(task.index)] = task
        for i in range(len(intents)):
            intent = taskdir[str(i)]
            if intent.name == "robot_control":
                pass
            elif intent.name == "robot_nav":
                pass
            elif intent.name == "robot_see":
                pass
            else:
                self.playtts = True

    def aiuiResult_callback(self,result):
        sid = result.sid
        self.type = result.type
        rospy.loginfo(self.type)
        if sid != self.sid:
            self.playtts = False
            self.sid = sid
            self.task_list = []
        if self.type == 'nlp':
            self.addtask(result.intent)
        if self.type == 'tts':
            if self.playtts:
                self.playAudio("./REIAIUI/audio/tts.pcm")
            
    def aiuiState_callback(self,state):
        self.state = state.data
        if self.state == "EVENT_WAKEUP":
            self.task_list = []
            self.recordAudio_client.call(SetBoolRequest(False))
            rospy.loginfo("唤醒状态")
            # 延时模拟人的停顿
            time.sleep(0.2)
            self.playAudio("./REIAIUI/audio/awake.pcm")
            time.sleep(1)
            self.recordAudio_client.call(SetBoolRequest(True))

    def playAudio(self,file):
        playMsg = REIPlayerRequest()
        playMsg.PcmPath = file
        self.player_client.call(playMsg)

    def run(self):
        self.recordAudio_client.call(SetBoolRequest(True))
        rospy.Subscriber("/REITopic/AIUIResult",REIResult,self.aiuiResult_callback,queue_size=10)
        rospy.Subscriber("/REITopic/AIUIState",String,self.aiuiState_callback,queue_size=10)
        rospy.spin()

if __name__ == "__main__":
    vc = voice_control()
    vc.run()
    