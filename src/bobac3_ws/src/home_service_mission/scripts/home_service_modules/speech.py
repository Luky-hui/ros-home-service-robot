#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import threading

import rospy
from std_msgs.msg import String

from robot_audio.srv import robot_tts, robot_ttsRequest


class SpeechClient:
    def __init__(self, speech_topic, tts_service, check_stopped):
        self.publisher = rospy.Publisher(speech_topic, String, queue_size=10, latch=True)
        self.tts_service = tts_service
        self.tts_client = rospy.ServiceProxy(tts_service, robot_tts)
        self.check_stopped = check_stopped
        self.tts_ready = False
        self.prepared_texts = set()
        self.prepare_lock = threading.Lock()

    def wait_for_tts(self, timeout=20.0):
        if self.tts_ready:
            return True
        try:
            rospy.wait_for_service(self.tts_service, timeout=timeout)
            self.tts_ready = True
            return True
        except Exception as exc:
            rospy.logwarn("TTS service not ready after %.1fs: %s", timeout, exc)
            return False

    def speak(self, text):
        self.check_stopped()
        self.publisher.publish(String(text))
        rospy.loginfo("TTS: %s", text)
        if not self.wait_for_tts():
            return
        request = robot_ttsRequest()
        request.text = text
        request.play = True
        try:
            self.tts_client(request)
        except Exception as exc:
            rospy.logwarn("TTS service unavailable: %s", exc)

    def speak_async(self, text):
        text = text or ""
        if not text:
            return

        def worker():
            if rospy.is_shutdown():
                return
            self.speak(text)

        thread = threading.Thread(target=worker, daemon=True)
        thread.start()

    def prepare(self, text, timeout=2.0):
        text = text or ""
        if not text:
            return
        with self.prepare_lock:
            if text in self.prepared_texts:
                return
        if not self.wait_for_tts(timeout=timeout):
            return
        request = robot_ttsRequest()
        request.text = text
        request.play = False
        try:
            self.tts_client(request)
            with self.prepare_lock:
                self.prepared_texts.add(text)
            rospy.loginfo("TTS precached: %s", text)
        except Exception as exc:
            rospy.logwarn("TTS precache unavailable: %s", exc)

    def prepare_many_async(self, texts, initial_delay=0.0):
        unique_texts = []
        seen = set()
        for text in texts:
            text = text or ""
            if text and text not in seen:
                unique_texts.append(text)
                seen.add(text)
        if not unique_texts:
            return

        def worker():
            if initial_delay > 0.0:
                rospy.sleep(initial_delay)
            for text in unique_texts:
                if rospy.is_shutdown():
                    return
                self.prepare(text)

        thread = threading.Thread(target=worker, daemon=True)
        thread.start()
