#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import json
import time
from collections import deque

import rospy
from std_msgs.msg import Bool, String

from face_rec.msg import face_results

from .semantic_intent import SemanticIntentClassifier


class InteractionManager:
    def __init__(self, config, face_mode, voice_mode, status, on_stop):
        self.config = config
        self.face_mode = face_mode
        self.voice_mode = voice_mode
        self.status = status
        self.on_stop = on_stop
        self.last_face = None
        self.last_face_time = None
        self.last_command = ""
        self.command_queue = deque(maxlen=1)
        self.pending_command = ""
        self.last_intent = None
        self.last_semantic_reply = ""
        self.last_semantic_command = ""
        self.last_semantic_reply_time = 0.0
        self.emergency_stop = False
        self.require_fresh_commands = self.parse_bool(
            rospy.get_param("~require_fresh_commands", False)
        )
        self.speak_unknown_fallback = bool(rospy.get_param("~speak_unknown_fallback", True))
        self.semantic_reply_cooldown = float(rospy.get_param("~semantic_reply_cooldown", 8.0))
        semantic_config = config.optional_section("semantic_intent")
        self.semantic_intent = SemanticIntentClassifier(semantic_config)
        if not self.semantic_intent.enabled:
            self.semantic_intent = None

        interaction = config.section("interaction")
        self.face_sub = rospy.Subscriber(
            interaction["face_topic"], face_results, self.face_callback, queue_size=10
        )
        self.command_sub = rospy.Subscriber(
            interaction["command_topic"], String, self.command_callback, queue_size=10
        )
        self.intent_sub = rospy.Subscriber(
            interaction["intent_topic"], String, self.intent_callback, queue_size=10
        )
        self.stop_sub = rospy.Subscriber(
            "/raicom/emergency_stop", Bool, self.stop_callback, queue_size=10
        )

    @staticmethod
    def parse_bool(value):
        if isinstance(value, bool):
            return value
        return str(value).strip().lower() in ("1", "true", "yes", "on")

    def face_callback(self, message):
        self.last_face = message
        self.last_face_time = rospy.Time.now()

    def command_callback(self, message):
        self.last_command = message.data.strip()
        if self.last_command:
            rospy.loginfo("VOICE_COMMAND_RECOGNIZED text=%s", self.last_command)
            self.command_queue.clear()
            self.command_queue.append(self.last_command)

    def intent_callback(self, message):
        try:
            data = json.loads(message.data)
        except Exception as exc:
            rospy.logwarn("Invalid local voice intent JSON: %s", exc)
            return
        self.last_intent = data
        rospy.loginfo(
            "VOICE_INTENT_RECOGNIZED %s",
            json.dumps(data, ensure_ascii=False),
        )

    def stop_callback(self, message):
        self.emergency_stop = bool(message.data)
        if self.emergency_stop:
            self.on_stop()
            self.status.publish("emergency_stop")

    def check_stopped(self):
        if self.emergency_stop:
            raise RuntimeError("Emergency stop is active")

    def clear_commands(self, reason=""):
        self.last_command = ""
        self.pending_command = ""
        self.command_queue.clear()
        self.last_intent = None
        if reason:
            rospy.loginfo("VOICE_COMMAND_BUFFER_CLEARED reason=%s", reason)

    def classify_command(self, command, expected_mission=None):
        if not self.semantic_intent:
            return None
        try:
            result = self.semantic_intent.classify(command, expected_mission)
        except Exception as exc:
            rospy.logwarn("Semantic intent classifier failed: %s", exc)
            return None
        rospy.loginfo("SEMANTIC_INTENT %s", json.dumps(result, ensure_ascii=False))
        return result

    @staticmethod
    def semantic_accepts(result, expected_mission):
        if not result or not result.get("execute_task"):
            return False
        if expected_mission and result.get("task") != expected_mission:
            return False
        return True

    def handle_semantic_reply(self, result, reply_callback):
        if result and result.get("reply") and reply_callback:
            is_unknown_fallback = (
                result.get("interference_type") == "unknown"
                and str(result.get("source", "")).startswith("fallback")
            )
            if (
                is_unknown_fallback
                and not self.speak_unknown_fallback
            ):
                rospy.loginfo("SEMANTIC_REPLY_SUPPRESSED unknown fallback")
                return False
            now = time.time()
            reply = result["reply"]
            command = str(result.get("command", ""))
            if (
                self.last_semantic_reply == reply
                and (is_unknown_fallback or self.last_semantic_command == command)
                and now - self.last_semantic_reply_time < self.semantic_reply_cooldown
            ):
                rospy.loginfo("SEMANTIC_REPLY_SUPPRESSED duplicate")
                if is_unknown_fallback:
                    self.last_command = ""
                    self.command_queue.clear()
                return False
            self.last_semantic_reply = reply
            self.last_semantic_command = command
            self.last_semantic_reply_time = now
            reply_callback(reply)
            if is_unknown_fallback:
                self.last_command = ""
                self.command_queue.clear()
            return True
        return False

    def handle_semantic_command(self, command, expected_mission, reply_callback=None):
        result = self.classify_command(command, expected_mission)
        if not result:
            return False, None
        if self.semantic_accepts(result, expected_mission):
            return True, result
        if result.get("execute_task") and expected_mission and result.get("task") != expected_mission:
            if reply_callback:
                reply_callback("\u8bf7\u4e0b\u8fbe\u5f53\u524d\u4efb\u52a1\u5bf9\u5e94\u7684\u6307\u4ee4\u3002")
                return False, result
        self.handle_semantic_reply(result, reply_callback)
        return False, result

    def wait_for_person(self):
        if self.face_mode == "skip":
            return
        self.last_face = None
        self.last_face_time = None
        wait_started = rospy.Time.now()
        self.status.publish("waiting_for_person")
        rate = rospy.Rate(5)
        while not rospy.is_shutdown():
            self.check_stopped()
            if (
                self.last_face
                and self.last_face.face_data
                and self.last_face_time
                and self.last_face_time >= wait_started
            ):
                return
            rate.sleep()
        raise rospy.ROSInterruptException()

    def wait_for_mission(self):
        self.status.publish("waiting_for_mission_intent")
        rate = rospy.Rate(5)
        while not rospy.is_shutdown():
            self.check_stopped()
            intent = self.last_intent
            if intent and intent.get("name") == "home_service_mission":
                slots = dict(zip(intent.get("slots_name", []), intent.get("slots_value", [])))
                mission = slots.get("mission", "")
                target = slots.get("target", "")
                self.last_intent = None
                return mission, target
            if self.last_command:
                command = self.command_queue.popleft() if self.command_queue else self.last_command
                semantic = self.classify_command(command)
                if semantic and semantic.get("execute_task"):
                    mission = semantic.get("task", "")
                    target = semantic.get("target", "")
                    if mission in ("guide", "assistant", "find_object", "charge"):
                        self.pending_command = command
                        self.last_command = ""
                        return mission, target
                mission, target = self.config.parse_mission_command(command)
                if mission:
                    self.pending_command = command
                    self.last_command = ""
                    return mission, target
                if not self.command_queue:
                    self.last_command = ""
            rate.sleep()
        raise rospy.ROSInterruptException()

    def wait_for_command(self, phrases, expected_mission=None, reply_callback=None):
        if self.voice_mode == "skip":
            return phrases[0]
        if self.pending_command:
            if self.semantic_intent and expected_mission:
                accepted, _ = self.handle_semantic_command(
                    self.pending_command, expected_mission, reply_callback
                )
                if accepted:
                    command = self.pending_command
                    self.pending_command = ""
                    return command
                self.pending_command = ""
            elif self.config.command_matches(self.pending_command, phrases):
                command = self.pending_command
                self.pending_command = ""
                return command
            else:
                accepted, _ = self.handle_semantic_command(
                    self.pending_command, expected_mission, reply_callback
                )
                if accepted:
                    command = self.pending_command
                    self.pending_command = ""
                    return command
                self.pending_command = ""
        self.status.publish("waiting_for_command", phrases=phrases)
        rate = rospy.Rate(5)
        while not rospy.is_shutdown():
            self.check_stopped()
            while self.command_queue:
                command = self.command_queue.popleft()
                if self.semantic_intent and expected_mission:
                    accepted, _ = self.handle_semantic_command(
                        command, expected_mission, reply_callback
                    )
                    if accepted:
                        self.last_command = ""
                        return command
                    continue
                if self.config.command_matches(command, phrases):
                    self.last_command = ""
                    return command
                accepted, _ = self.handle_semantic_command(
                    command, expected_mission, reply_callback
                )
                if accepted:
                    self.last_command = ""
                    return command
            self.last_command = ""
            rate.sleep()
        raise rospy.ROSInterruptException()

    def wait_for_command_with_replies(self, phrases, replies, reply_callback, expected_mission=None):
        if self.voice_mode == "skip":
            return phrases[0]
        if self.pending_command:
            if self.semantic_intent and expected_mission:
                accepted, _ = self.handle_semantic_command(
                    self.pending_command, expected_mission, reply_callback
                )
                if accepted:
                    command = self.pending_command
                    self.pending_command = ""
                    return command
                self.pending_command = ""
            elif self.config.command_matches(self.pending_command, phrases):
                command = self.pending_command
                self.pending_command = ""
                return command
            else:
                accepted, _ = self.handle_semantic_command(
                    self.pending_command, expected_mission, reply_callback
                )
                if accepted:
                    command = self.pending_command
                    self.pending_command = ""
                    return command
                self.pending_command = ""
        self.status.publish("waiting_for_command", phrases=phrases)
        rate = rospy.Rate(5)
        while not rospy.is_shutdown():
            self.check_stopped()
            while self.command_queue:
                command = self.command_queue.popleft()
                replied = False
                for reply in replies:
                    if self.config.command_matches(command, reply.get("phrases", [])):
                        reply_callback(reply["speech"])
                        replied = True
                        break
                if replied:
                    continue
                if self.semantic_intent and expected_mission:
                    accepted, _ = self.handle_semantic_command(
                        command, expected_mission, reply_callback
                    )
                    if accepted:
                        self.last_command = ""
                        return command
                    continue
                if self.config.command_matches(command, phrases):
                    self.last_command = ""
                    return command
                accepted, _ = self.handle_semantic_command(
                    command, expected_mission, reply_callback
                )
                if accepted:
                    self.last_command = ""
                    return command
            self.last_command = ""
            rate.sleep()
        raise rospy.ROSInterruptException()

    def wait_for_find_target(self, reply_callback=None):
        self.status.publish("waiting_for_find_target")
        rate = rospy.Rate(5)
        while not rospy.is_shutdown():
            self.check_stopped()
            intent = self.last_intent
            if intent and intent.get("name") == "home_service_mission":
                slots = dict(zip(intent.get("slots_name", []), intent.get("slots_value", [])))
                if slots.get("mission") == "find_object" and slots.get("target"):
                    self.last_intent = None
                    return slots["target"]
            while self.command_queue:
                command = self.command_queue.popleft()
                target = self.config.parse_find_target(command)
                if target:
                    self.last_command = ""
                    return target
                semantic = self.classify_command(command, "find_object")
                if semantic and semantic.get("execute_task") and semantic.get("task") == "find_object":
                    target = semantic.get("target", "")
                    if target in self.config.section("find_object")["labels"]:
                        self.last_command = ""
                        return target
                self.handle_semantic_reply(semantic, reply_callback)
            self.last_command = ""
            rate.sleep()
        raise rospy.ROSInterruptException()
