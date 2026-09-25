#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import subprocess

import rospy
from std_srvs.srv import SetBool

from rei_voice.srv import REINlp, REITts
from robot_audio.srv import Awake, robot_iat, robot_semanteme


def require(condition, message):
    if not condition:
        raise RuntimeError(message)


def main():
    rospy.init_node("local_voice_smoke_test")
    for service in (
        "/REIService/RecordAudio",
        "/REIService/voice_nlp",
        "/REIService/voice_tts",
        "/voice_awake",
        "/voice_aiui",
        "/voice_iat",
    ):
        rospy.wait_for_service(service, timeout=20.0)

    record = rospy.ServiceProxy("/REIService/RecordAudio", SetBool)
    awake = rospy.ServiceProxy("/voice_awake", Awake)
    nlp = rospy.ServiceProxy("/REIService/voice_nlp", REINlp)
    semanteme = rospy.ServiceProxy("/voice_aiui", robot_semanteme)
    iat = rospy.ServiceProxy("/voice_iat", robot_iat)
    tts = rospy.ServiceProxy("/REIService/voice_tts", REITts)

    require(record(True).success, "RecordAudio failed")
    require(awake("元宝").awake_flag, "wake word failed")
    require(nlp("可以带我去参观一下深圳馆吗").success, "REINlp failed")

    nav = semanteme(1, "带我去深圳馆")
    require(nav.intent == "robot_nav", "robot_nav intent failed")
    require(nav.slots_value == ["navigate", "深圳馆"], "robot_nav slots failed")

    control = semanteme(1, "前进三十厘米")
    require(control.intent == "robot_control", "robot_control intent failed")
    require(control.slots_value == ["前进", "30", "厘米"], "robot_control slots failed")

    guide = semanteme(1, "可以带我参观一下吗")
    require(guide.intent == "home_service_mission", "national guide intent failed")
    require(guide.slots_value == ["guide"], "national guide slots failed")

    assistant = semanteme(1, "给我推荐一下今天适合做什么菜")
    require(assistant.intent == "home_service_mission", "assistant intent failed")
    require(assistant.slots_value == ["assistant"], "assistant slots failed")

    find_object = semanteme(1, "帮我找书包")
    require(find_object.intent == "home_service_mission", "find object intent failed")
    require(
        find_object.slots_value == ["find_object", "backpack"],
        "find object slots failed",
    )

    tts_response = tts("本地语音系统测试成功。", False)
    require(tts_response.success, "TTS failed")
    require(tts_response.filePath.endswith(".wav"), "TTS path failed")

    command_tts = tts("可以带我去参观一下深圳馆吗？", False)
    command_16k = os.path.splitext(command_tts.filePath)[0] + "_16k.wav"
    subprocess.run(
        [
            "ffmpeg", "-loglevel", "error", "-y",
            "-i", command_tts.filePath,
            "-ar", "16000", "-ac", "1", "-c:a", "pcm_s16le",
            command_16k,
        ],
        check=True,
    )
    recognized = iat(command_16k).text
    require("深圳" in recognized and "参观" in recognized, "ASR round trip failed: " + recognized)
    print("LOCAL_VOICE_SMOKE_TEST_OK")


if __name__ == "__main__":
    main()
