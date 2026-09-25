#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import math
import os
import struct
import subprocess
import time
import wave

import actionlib
import cv2
import numpy as np
import rospy
import tf
import yaml
from actionlib_msgs.msg import GoalStatus
from gazebo_msgs.srv import DeleteModel, GetLinkState, GetModelState, SpawnModel
from geometry_msgs.msg import Pose, Quaternion, Twist
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from std_msgs.msg import Bool, String
from std_srvs.srv import Empty

from face_rec.msg import face_results
from face_rec.srv import recognition_results, recognition_resultsRequest
from raicom_vision.srv import DetectObjects, DetectObjectsRequest
from robot_audio.srv import robot_tts, robot_ttsRequest


class ServiceGroupMission:
    # 场馆编号：配置、语音和播报共用。
    VENUE_NAMES = {
        "jilin": "吉林馆",
        "guangzhou": "广州馆",
        "beijing": "北京馆",
        "shanghai": "上海馆",
        "shenzhen": "深圳馆",
    }

    VENUE_KEYWORDS = {
        "guangzhou": ("广州", "广州馆", "广", "州"),
        "shenzhen": ("深圳", "深圳馆", "深", "圳"),
        "shanghai": ("上海", "上海馆", "上", "海"),
        "jilin": ("吉林", "吉林馆", "吉", "林", "麒麟", "麒麟馆", "麒林", "麒林馆", "吉宾馆"),
        "beijing": ("北京", "北京馆", "北", "京"),
    }

    DEFAULT_INTRODUCTIONS = {
        "jilin": "吉林馆到了。",
        "guangzhou": "广州馆到了。",
        "beijing": "北京馆到了。",
        "shanghai": "上海馆到了。",
        "shenzhen": "深圳，是广东副省级市、经济特区。毗邻香港，经济发达，创新力强，有众多世界500强企业，是粤港澳大湾区中心城市。",
    }

    def __init__(self):
        rospy.init_node("service_group_mission")
        self._load_ros_parameters()
        self._init_runtime_state()
        self._init_ros_interfaces()
        self._init_ros_clients()

        self.config = self._load_config(self.config_file)
        self.alarm_audio_path = self.create_alarm_audio()

    def _load_ros_parameters(self):
        self.mission_type = rospy.get_param("~mission_type", "welcome")
        self.config_file = rospy.get_param("~config_file", "")
        self.face_mode = rospy.get_param("~face_mode", "any")
        self.voice_mode = rospy.get_param("~voice_mode", "topic")
        self.command_topic = rospy.get_param("~command_topic", "/service_group_mission/voice_command")
        self.face_topic = rospy.get_param("~face_topic", "/face_detection")
        self.face_service = rospy.get_param("~face_service", "face_recognition_results")
        self.vision_service = rospy.get_param("~vision_service", "/raicom_vision/detect")
        self.move_base_name = rospy.get_param("~move_base_name", "move_base")
        self.move_base_fallback_name = rospy.get_param("~move_base_fallback_name", "move_base_node")
        self.clear_costmaps_service = rospy.get_param("~clear_costmaps_service", "/move_base_node/clear_costmaps")
        self.action_server_wait = float(rospy.get_param("~action_server_wait", 180.0))
        self.nav_timeout = float(rospy.get_param("~nav_timeout", 120.0))
        self.tts_service_wait = float(rospy.get_param("~tts_service_wait", 8.0))
        self.tts_ready_settle = float(rospy.get_param("~tts_ready_settle", 2.0))
        self.test_all_venues = rospy.get_param("~test_all_venues", False)

    def _init_runtime_state(self):
        self.last_command = None
        self.last_face_msg = None
        self.last_admin_name = None
        self.emergency_stop = False
        self.template_cache = {}

    def _init_ros_interfaces(self):
        self.command_sub = rospy.Subscriber(self.command_topic, String, self._command_cb, queue_size=10)
        self.face_sub = rospy.Subscriber(self.face_topic, face_results, self._face_cb, queue_size=10)
        self.stop_sub = rospy.Subscriber(
            "/raicom/emergency_stop", Bool, self._stop_cb, queue_size=10
        )
        self.speech_pub = rospy.Publisher("/service_group_mission/speech", String, queue_size=10, latch=True)
        self.alarm_pub = rospy.Publisher("/service_group_mission/alarm", String, queue_size=10, latch=True)
        self.cmd_vel_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
        self.tf_listener = tf.TransformListener()
        self.bridge = CvBridge()

    def _init_ros_clients(self):
        # ROS 接口名保持不变，兼容现有 launch 和脚本。
        self.move_base = actionlib.SimpleActionClient(self.move_base_name, MoveBaseAction)
        self.clear_costmaps = rospy.ServiceProxy(self.clear_costmaps_service, Empty)
        self.tts_client = rospy.ServiceProxy("voice_tts", robot_tts)
        self.face_client = rospy.ServiceProxy(self.face_service, recognition_results)
        self.vision_client = rospy.ServiceProxy(self.vision_service, DetectObjects)
        self.gazebo_get_model_state = rospy.ServiceProxy("/gazebo/get_model_state", GetModelState)
        self.gazebo_get_link_state = rospy.ServiceProxy("/gazebo/get_link_state", GetLinkState)
        self.gazebo_delete_model = rospy.ServiceProxy("/gazebo/delete_model", DeleteModel)
        self.gazebo_spawn_model = rospy.ServiceProxy("/gazebo/spawn_sdf_model", SpawnModel)

    def _load_config(self, path):
        if not path or not os.path.exists(path):
            raise RuntimeError("Mission config file not found: %s" % path)
        with open(path, "r", encoding="utf-8") as stream:
            return yaml.safe_load(stream)

    def _command_cb(self, msg):
        self.last_command = msg.data.strip()

    def _face_cb(self, msg):
        self.last_face_msg = msg

    def _stop_cb(self, msg):
        self.emergency_stop = bool(msg.data)
        if self.emergency_stop:
            self.move_base.cancel_all_goals()
            self.publish_stop()

    def check_stopped(self):
        if self.emergency_stop:
            raise RuntimeError("Emergency stop is active")

    def run(self):
        self.check_stopped()
        rospy.loginfo("Waiting for move_base action server: %s", self.move_base_name)
        if not self.wait_for_action_server(self.move_base, self.move_base_name):
            rospy.logwarn("Primary move_base action server unavailable, trying: %s", self.move_base_fallback_name)
            self.move_base = actionlib.SimpleActionClient(self.move_base_fallback_name, MoveBaseAction)
            if not self.wait_for_action_server(self.move_base, self.move_base_fallback_name):
                raise RuntimeError("move_base action server is not available")

        self.wait_for_tts_service()

        if self.mission_type == "welcome":
            self.run_welcome()
        elif self.mission_type == "patrol":
            self.run_patrol()
        else:
            raise RuntimeError("Unknown mission_type: %s" % self.mission_type)

    def wait_for_action_server(self, client, name):
        deadline = time.time() + self.action_server_wait
        while not rospy.is_shutdown() and time.time() < deadline:
            self.check_stopped()
            if client.wait_for_server(rospy.Duration(0.5)):
                rospy.loginfo("Connected to move_base action server: %s", name)
                return True
            time.sleep(0.2)
        return False

    def wait_for_tts_service(self):
        if self.tts_service_wait <= 0.0:
            return
        rospy.loginfo("Waiting for voice_tts service")
        try:
            rospy.wait_for_service("voice_tts", timeout=self.tts_service_wait)
            rospy.loginfo("Connected to voice_tts service")
            if self.tts_ready_settle > 0.0:
                rospy.loginfo(
                    "Settling voice_tts service for %.1fs",
                    self.tts_ready_settle,
                )
                deadline = time.time() + self.tts_ready_settle
                while not rospy.is_shutdown() and time.time() < deadline:
                    self.check_stopped()
                    time.sleep(0.1)
        except Exception as exc:
            rospy.logwarn(
                "voice_tts unavailable after %.1fs: %s",
                self.tts_service_wait,
                exc,
            )

    def run_welcome(self):
        # 任务一：人脸唤醒、语音导览、介绍后返航。
        mission = self.config["missions"]["welcome"]
        self.wait_for_face(any_face=True)
        self.speak(mission["greeting"])
        command = self.wait_for_command(mission["trigger_phrase"])
        target = self.resolve_welcome_target(command, mission.get("target", "shenzhen"))
        self.speak(mission["accept"])
        if self.test_all_venues:
            route = self.config["missions"]["patrol"]["route"]
            for index, venue in enumerate(route):
                self.go_to(venue)
                if index < len(route) - 1:
                    self.exit_after_parking(venue)
            target_to_leave = route[-1]
        else:
            self.go_to(target)
            target_to_leave = target
        self.speak(self.welcome_introduction(target, mission))
        self.speak(self.welcome_leaving(target, mission))
        self.exit_after_parking(target_to_leave)
        self.go_to(mission["return_target"])
        rospy.loginfo("Welcome mission finished")

    def resolve_welcome_target(self, command, default_target):
        command = command or ""
        for venue, keywords in self.VENUE_KEYWORDS.items():
            for keyword in keywords:
                if keyword in command:
                    rospy.loginfo("Welcome command selected target: %s", venue)
                    return venue
        rospy.loginfo("Welcome command selected default target: %s", default_target)
        return default_target

    def welcome_introduction(self, target, mission):
        introductions = mission.get("introductions", {})
        if target in introductions:
            return introductions[target]
        if target == mission.get("target") and mission.get("introduction"):
            return mission["introduction"]
        return self.DEFAULT_INTRODUCTIONS.get(target, "%s到了。" % self.VENUE_NAMES.get(target, target))

    def welcome_leaving(self, target, mission):
        leaving = mission.get("leavings", {})
        if target in leaving:
            return leaving[target]
        if target == mission.get("target") and mission.get("leaving"):
            return mission["leaving"]
        return "这里就是%s啦，我要继续回去工作啦！" % self.VENUE_NAMES.get(target, target)

    def run_patrol(self):
        mission = self.config["missions"]["patrol"]
        admin_name = self.wait_for_face(any_face=False, expected_name=mission.get("admin_name", ""))
        self.speak("你好，管理员 %s" % admin_name)
        self.wait_for_command(mission["trigger_phrase"])
        self.speak(mission["accept"])

        for venue in mission["route"]:
            self.go_to(venue)
            self.prepare_inspection_view(venue)
            time.sleep(float(self.config["inspection"].get("settle_time", 2.0)))
            events = self.detect_inspection_event(venue)
            self.handle_inspection_event(venue, events)
            self.exit_after_parking(venue)

        charge_approach = mission.get("charge_approach_target")
        if charge_approach:
            self.go_to(charge_approach)
            self.dock_to_charge(mission["charge_target"])
            rospy.loginfo("Arrived at %s", mission["charge_target"])
        else:
            self.go_to(mission["charge_target"])
        rospy.loginfo("Patrol mission finished at charge target")

    def prepare_inspection_view(self, waypoint_name):
        options = self.config.get("inspection_pose_adjustment", {})
        if not options.get("enabled", False):
            return
        venues = options.get("venues", [])
        if venues and waypoint_name not in venues:
            return

        speech = options.get("speech", "到达点位")
        if speech:
            rospy.loginfo("Inspection arrival speech at %s: %s", waypoint_name, speech)
            self.speak(speech)

        reverse_distance = float(options.get("reverse_distance", 0.10))
        reverse_speed = float(options.get("reverse_speed", -0.05))
        if reverse_distance <= 0.0 or reverse_speed == 0.0:
            return
        if reverse_speed > 0.0:
            reverse_speed = -reverse_speed
        duration = reverse_distance / abs(reverse_speed)
        rospy.loginfo(
            "Backing up %.3fm before visual inspection at %s, speed=%.3f duration=%.2f",
            reverse_distance,
            waypoint_name,
            reverse_speed,
            duration,
        )
        cmd = Twist()
        cmd.linear.x = reverse_speed
        deadline = time.time() + duration
        while not rospy.is_shutdown() and time.time() < deadline:
            self.check_stopped()
            self.cmd_vel_pub.publish(cmd)
            time.sleep(0.05)
        self.publish_stop()

        settle = float(options.get("settle_after_reverse", 0.0))
        if settle > 0.0:
            time.sleep(settle)

    def exit_after_parking(self, waypoint_name):
        options = self.config.get("exit_after_parking", {})
        if not options.get("enabled", False):
            return
        if waypoint_name not in options.get("venues", []):
            return
        speed = float(options.get("reverse_speed", -0.05))
        duration = float(options.get("duration", 3.0))
        rospy.loginfo("Exiting parking box from %s", waypoint_name)
        cmd = Twist()
        cmd.linear.x = speed
        deadline = time.time() + duration
        while not rospy.is_shutdown() and time.time() < deadline:
            self.check_stopped()
            self.cmd_vel_pub.publish(cmd)
            time.sleep(0.05)
        self.publish_stop()

    def wait_for_face(self, any_face=True, expected_name=""):
        if self.face_mode == "skip":
            return expected_name or "sim"

        rospy.loginfo("Waiting for face trigger, mode=%s", self.face_mode)
        rate = rospy.Rate(2)
        while not rospy.is_shutdown():
            self.check_stopped()
            if any_face and self.last_face_msg and self.last_face_msg.face_data:
                return "guest"
            if not any_face:
                name = self.query_admin_face(expected_name)
                if name:
                    return name
            rate.sleep()
        raise rospy.ROSInterruptException()

    def query_admin_face(self, expected_name):
        try:
            req = recognition_resultsRequest()
            req.mode = 0
            req.str = ""
            res = self.face_client(req)
        except Exception as exc:
            rospy.logwarn_throttle(5.0, "Face recognition service unavailable: %s", exc)
            return None

        if not res.success:
            return None
        for face in res.result.face_data:
            name = face.header.frame_id
            if name and name != "Unknown":
                if not expected_name or expected_name in name or name in expected_name:
                    return name
        return None

    def wait_for_command(self, expected_phrase):
        phrases = self.command_phrases(expected_phrase)
        if self.voice_mode == "skip":
            return phrases[0]

        rospy.loginfo("Waiting for voice command containing one of: %s", phrases)
        rate = rospy.Rate(5)
        while not rospy.is_shutdown():
            self.check_stopped()
            if self.last_command:
                for phrase in phrases:
                    if phrase and phrase in self.last_command:
                        command = self.last_command
                        self.last_command = None
                        return command
            rate.sleep()
        raise rospy.ROSInterruptException()

    def command_phrases(self, expected_phrase):
        phrases = []
        welcome_trigger = expected_phrase == self.config["missions"]["welcome"].get("trigger_phrase")
        if isinstance(expected_phrase, list):
            phrases.extend([str(item) for item in expected_phrase])
        else:
            phrase_text = str(expected_phrase)
            if not (welcome_trigger and phrase_text == "馆"):
                phrases.append(phrase_text)
        if welcome_trigger:
            for keywords in self.VENUE_KEYWORDS.values():
                phrases.extend(keywords)
        return list(dict.fromkeys([phrase for phrase in phrases if phrase]))

    def speak(self, text):
        rospy.loginfo("TTS: %s", text)
        self.speech_pub.publish(String(text))
        req = robot_ttsRequest()
        req.text = text
        req.play = True
        try:
            self.tts_client(req)
        except Exception as exc:
            try:
                rospy.wait_for_service("voice_tts", timeout=3.0)
                self.tts_client(req)
            except Exception as retry_exc:
                rospy.logwarn_throttle(
                    5.0,
                    "voice_tts unavailable, logged speech only: %s; retry: %s",
                    exc,
                    retry_exc,
                )

    def go_to(self, waypoint_name):
        self.check_stopped()
        waypoint = self.config["waypoints"][waypoint_name]
        rospy.loginfo("Navigating to %s", waypoint_name)
        self.clear_navigation_costmaps()

        goal = self.build_navigation_goal(waypoint)
        self.move_base.send_goal(goal)
        finished = self.move_base.wait_for_result(rospy.Duration(self.nav_timeout))
        if not finished:
            self.move_base.cancel_goal()
            if self.try_navigation_failure_recovery(waypoint_name, waypoint, goal, "timeout"):
                self.mark_waypoint_arrived(waypoint_name)
                return
            raise RuntimeError("Navigation timeout at waypoint: %s" % waypoint_name)

        state = self.move_base.get_state()
        if state != GoalStatus.SUCCEEDED:
            if self.try_direct_fallback(waypoint_name, waypoint, state):
                self.mark_waypoint_arrived(waypoint_name)
                return
            if self.try_navigation_failure_recovery(waypoint_name, waypoint, goal, state):
                self.mark_waypoint_arrived(waypoint_name)
                return
            raise RuntimeError("Navigation failed at %s, state=%s" % (waypoint_name, state))
        self.refine_pose(waypoint_name)
        self.mark_waypoint_arrived(waypoint_name)

    def clear_navigation_costmaps(self):
        try:
            self.clear_costmaps()
        except Exception:
            pass

    def build_navigation_goal(self, waypoint):
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = waypoint.get("frame_id", "map")
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = float(waypoint.get("nav_x", waypoint["x"]))
        goal.target_pose.pose.position.y = float(waypoint.get("nav_y", waypoint["y"]))
        goal.target_pose.pose.orientation = self.yaw_to_quaternion(self.resolve_navigation_yaw(waypoint))
        return goal

    def resolve_navigation_yaw(self, waypoint):
        nav_yaw_value = waypoint.get("nav_yaw", waypoint.get("yaw", 0.0))
        if isinstance(nav_yaw_value, str) and nav_yaw_value.lower() == "current":
            try:
                return self.current_pose_in_map()[2]
            except Exception:
                return float(waypoint.get("yaw", 0.0))
        return float(nav_yaw_value)

    def mark_waypoint_arrived(self, waypoint_name):
        rospy.loginfo("Arrived at %s", waypoint_name)
        self.observe_arrival(waypoint_name)

    def try_navigation_failure_recovery(self, waypoint_name, waypoint, goal, failed_state):
        options = self.config.get("navigation_failure_recovery", {})
        if not options.get("enabled", False):
            return False

        retry_count = int(options.get("retry_count", 1))
        retry_nav_timeout = float(options.get("retry_nav_timeout", min(self.nav_timeout, 45.0)))
        motions = options.get("motions", [])
        if not motions:
            motions = [
                {
                    "linear_x": 0.05,
                    "linear_y": 0.0,
                    "angular_z": 0.0,
                    "duration": 2.0,
                }
            ]

        for attempt in range(1, retry_count + 1):
            rospy.logwarn(
                "Navigation recovery at %s after state=%s, attempt=%d/%d",
                waypoint_name,
                failed_state,
                attempt,
                retry_count,
            )
            self.publish_stop()
            self.clear_navigation_costmaps()

            for motion in motions:
                self.drive_recovery_motion(motion)

            self.clear_navigation_costmaps()

            goal.target_pose.header.stamp = rospy.Time.now()
            self.move_base.send_goal(goal)
            finished = self.move_base.wait_for_result(rospy.Duration(retry_nav_timeout))
            if not finished:
                self.move_base.cancel_goal()
                failed_state = "timeout"
                continue

            retry_state = self.move_base.get_state()
            if retry_state == GoalStatus.SUCCEEDED:
                self.refine_pose(waypoint_name)
                rospy.loginfo("Navigation recovery succeeded at %s", waypoint_name)
                return True
            failed_state = retry_state

        return False

    def drive_recovery_motion(self, motion):
        linear_x = float(motion.get("linear_x", 0.0))
        linear_y = float(motion.get("linear_y", 0.0))
        angular_z = float(motion.get("angular_z", 0.0))
        duration = float(motion.get("duration", 0.0))
        if duration <= 0.0:
            return

        rospy.logwarn(
            "Recovery motion linear_x=%.3f linear_y=%.3f angular_z=%.3f duration=%.2f",
            linear_x,
            linear_y,
            angular_z,
            duration,
        )
        cmd = Twist()
        cmd.linear.x = linear_x
        cmd.linear.y = linear_y
        cmd.angular.z = angular_z
        deadline = time.time() + duration
        while not rospy.is_shutdown() and time.time() < deadline:
            self.check_stopped()
            self.cmd_vel_pub.publish(cmd)
            time.sleep(0.05)
        self.publish_stop()

    def observe_arrival(self, waypoint_name):
        options = self.config.get("arrival_observation", {})
        if not options.get("enabled", False):
            return
        speech = options.get("speech", "arrived")
        pause = float(options.get("pause", 0.0))
        rospy.loginfo("Arrival observation at %s: %s", waypoint_name, speech)
        self.speak(speech)
        if pause > 0.0:
            time.sleep(pause)

    def try_direct_fallback(self, waypoint_name, waypoint, state):
        if not waypoint.get("direct_fallback", False):
            return False
        max_distance = float(waypoint.get("max_direct_fallback_distance", 0.8))
        try:
            x, y, _ = self.current_pose_in_map()
        except Exception as exc:
            rospy.logwarn("Direct fallback unavailable at %s: %s", waypoint_name, exc)
            return False
        distance = math.hypot(float(waypoint["x"]) - x, float(waypoint["y"]) - y)
        if distance > max_distance:
            rospy.logwarn(
                "Direct fallback skipped at %s: move_base_state=%s distance=%.3f > %.3f",
                waypoint_name,
                state,
                distance,
                max_distance,
            )
            return False
        accept_distance = waypoint.get("direct_accept_distance")
        if accept_distance is not None and distance <= float(accept_distance):
            rospy.logwarn(
                "move_base failed near %s with state=%s; accepting close pose, distance=%.3f",
                waypoint_name,
                state,
                distance,
            )
            self.publish_stop()
            return True
        rospy.logwarn(
            "move_base failed at %s with state=%s; using short direct fallback, distance=%.3f",
            waypoint_name,
            state,
            distance,
        )
        self.drive_direct_to_pose(waypoint_name, waypoint)
        return True

    def drive_direct_to_pose(self, waypoint_name, waypoint):
        precision = self.config.get("precision_parking", {})
        defaults = precision.get("default", {})
        overrides = precision.get(waypoint_name, {})
        options = dict(defaults)
        options.update(overrides)

        target_x = float(waypoint["x"])
        target_y = float(waypoint["y"])
        target_yaw = float(waypoint.get("yaw", 0.0))
        xy_tolerance = float(options.get("xy_tolerance", 0.04))
        yaw_tolerance = float(options.get("yaw_tolerance", 0.08))
        timeout = float(waypoint.get("direct_fallback_timeout", options.get("timeout", 30.0)))
        max_linear = float(options.get("max_linear", 0.05))
        max_angular = max(float(options.get("max_angular", 0.18)), 0.25)

        rospy.loginfo("Direct driving to %s", waypoint_name)
        deadline = time.time() + timeout
        while not rospy.is_shutdown() and time.time() < deadline:
            x, y, yaw = self.current_pose_in_map()
            dx = target_x - x
            dy = target_y - y
            distance = math.hypot(dx, dy)
            if distance <= xy_tolerance:
                break
            heading = math.atan2(dy, dx)
            heading_error = self.normalize_angle(heading - yaw)
            cmd = Twist()
            if abs(heading_error) > 0.25:
                cmd.angular.z = self.clip(1.0 * heading_error, -max_angular, max_angular)
            else:
                cmd.linear.x = self.clip(0.5 * distance, 0.02, max_linear)
                cmd.angular.z = self.clip(0.8 * heading_error, -max_angular, max_angular)
            self.cmd_vel_pub.publish(cmd)
            time.sleep(0.05)

        while not rospy.is_shutdown() and time.time() < deadline:
            x, y, yaw = self.current_pose_in_map()
            yaw_error = self.normalize_angle(target_yaw - yaw)
            if abs(yaw_error) <= yaw_tolerance:
                self.publish_stop()
                rospy.loginfo(
                    "Direct refined %s: error_xy=%.3f, error_yaw=%.3f",
                    waypoint_name,
                    math.hypot(target_x - x, target_y - y),
                    yaw_error,
                )
                return
            cmd = Twist()
            cmd.angular.z = self.clip(0.9 * yaw_error, -max_angular, max_angular)
            self.cmd_vel_pub.publish(cmd)
            time.sleep(0.05)

        self.publish_stop()
        x, y, yaw = self.current_pose_in_map()
        raise RuntimeError(
            "Direct fallback failed at %s: target=(%.3f, %.3f, %.3f), current=(%.3f, %.3f, %.3f)"
            % (waypoint_name, target_x, target_y, target_yaw, x, y, yaw)
        )

    def refine_pose(self, waypoint_name):
        precision = self.config.get("precision_parking", {})
        if not precision.get("enabled", True):
            return

        waypoint = self.config["waypoints"][waypoint_name]
        defaults = precision.get("default", {})
        overrides = precision.get(waypoint_name, {})
        options = dict(defaults)
        options.update(overrides)

        target_x = float(waypoint["x"])
        target_y = float(waypoint["y"])
        target_yaw = float(waypoint.get("yaw", 0.0))
        xy_tolerance = float(options.get("xy_tolerance", 0.04))
        yaw_tolerance = float(options.get("yaw_tolerance", 0.08))
        timeout = float(options.get("timeout", 12.0))
        rate_hz = float(options.get("rate", 20.0))
        stable_cycles_required = int(options.get("stable_cycles", 8))
        kp_xy = float(options.get("kp_xy", 0.8))
        kp_yaw = float(options.get("kp_yaw", 1.0))
        max_linear = float(options.get("max_linear", 0.08))
        max_angular = float(options.get("max_angular", 0.18))

        rospy.loginfo("Refining pose at %s", waypoint_name)
        rate = rospy.Rate(rate_hz)
        deadline = rospy.Time.now() + rospy.Duration(timeout)
        stable_cycles = 0

        while not rospy.is_shutdown() and rospy.Time.now() < deadline:
            x, y, yaw = self.current_pose_in_map()
            dx = target_x - x
            dy = target_y - y
            yaw_error = self.normalize_angle(target_yaw - yaw)
            distance = math.hypot(dx, dy)

            if distance <= xy_tolerance and abs(yaw_error) <= yaw_tolerance:
                stable_cycles += 1
                self.publish_stop()
                if stable_cycles >= stable_cycles_required:
                    rospy.loginfo(
                        "Refined %s: error_xy=%.3f, error_yaw=%.3f",
                        waypoint_name,
                        distance,
                        yaw_error,
                    )
                    return
                rate.sleep()
                continue

            stable_cycles = 0
            cmd = Twist()
            # 将地图坐标误差转换为底盘速度。
            cmd.linear.x = self.clip(kp_xy * (math.cos(yaw) * dx + math.sin(yaw) * dy), -max_linear, max_linear)
            cmd.linear.y = self.clip(kp_xy * (-math.sin(yaw) * dx + math.cos(yaw) * dy), -max_linear, max_linear)
            cmd.angular.z = self.clip(kp_yaw * yaw_error, -max_angular, max_angular)
            self.cmd_vel_pub.publish(cmd)
            rate.sleep()

        self.publish_stop()
        x, y, yaw = self.current_pose_in_map()
        raise RuntimeError(
            "Precise parking failed at %s: target=(%.3f, %.3f, %.3f), current=(%.3f, %.3f, %.3f)"
            % (waypoint_name, target_x, target_y, target_yaw, x, y, yaw)
        )

    def current_pose_in_map(self):
        self.tf_listener.waitForTransform("map", "base_footprint", rospy.Time(0), rospy.Duration(1.0))
        (translation, rotation) = self.tf_listener.lookupTransform("map", "base_footprint", rospy.Time(0))
        yaw = tf.transformations.euler_from_quaternion(rotation)[2]
        return translation[0], translation[1], yaw

    def dock_to_charge(self, waypoint_name):
        docking = self.config.get("docking", {})
        if not docking.get("enabled", True):
            self.refine_pose(waypoint_name)
            return

        if docking.get("marker_enabled", False) or docking.get("method", "") == "marker":
            try:
                self.dock_to_charge_with_marker(docking)
                return
            except Exception as exc:
                self.publish_stop()
                rospy.logwarn("Marker docking failed: %s", exc)
                if not docking.get("marker_fallback_to_map", True):
                    raise
                rospy.logwarn("Falling back to map-based charge docking")

        self.dock_to_charge_by_map(waypoint_name, docking)

    def dock_to_charge_by_map(self, waypoint_name, docking):
        waypoint = self.config["waypoints"][waypoint_name]
        target_y = float(waypoint["y"])
        target_yaw = float(waypoint.get("yaw", 0.0))
        reverse_speed = float(docking.get("reverse_speed", -0.025))
        timeout = float(docking.get("timeout", 18.0))
        y_tolerance = float(docking.get("y_tolerance", 0.025))
        yaw_tolerance = float(docking.get("yaw_tolerance", 0.035))
        kp_yaw = float(docking.get("kp_yaw", 0.6))
        max_angular = float(docking.get("max_angular", 0.06))

        rospy.loginfo("Docking straight to %s", waypoint_name)
        rate = rospy.Rate(20.0)
        deadline = rospy.Time.now() + rospy.Duration(timeout)
        stable_cycles = 0

        while not rospy.is_shutdown() and rospy.Time.now() < deadline:
            x, y, yaw = self.current_pose_in_map()
            y_error = target_y - y
            yaw_error = self.normalize_angle(target_yaw - yaw)

            if abs(y_error) <= y_tolerance and abs(yaw_error) <= yaw_tolerance:
                stable_cycles += 1
                self.publish_stop()
                if stable_cycles >= 10:
                    rospy.loginfo("Docked %s: error_y=%.3f, error_yaw=%.3f", waypoint_name, y_error, yaw_error)
                    return
                rate.sleep()
                continue

            stable_cycles = 0
            cmd = Twist()
            # 兜底停靠：先对准角度，再低速直线后退。
            if abs(yaw_error) <= max(0.10, yaw_tolerance * 2.0):
                cmd.linear.x = reverse_speed if y_error > 0.0 else 0.0
            else:
                cmd.linear.x = 0.0
            cmd.angular.z = self.clip(kp_yaw * yaw_error, -max_angular, max_angular)
            self.cmd_vel_pub.publish(cmd)
            rate.sleep()

        self.publish_stop()
        x, y, yaw = self.current_pose_in_map()
        raise RuntimeError(
            "Docking failed at %s: target_y=%.3f, current=(%.3f, %.3f, %.3f)"
            % (waypoint_name, target_y, x, y, yaw)
        )

    def dock_to_charge_with_marker(self, docking):
        # 充电停靠优先级：触点对齐、距离控制、固定后退。
        rospy.loginfo("Marker docking: aligning charge marker")
        self.align_to_charge_marker(docking)
        rospy.loginfo("Marker docking: rotating 180 degrees")
        self.rotate_relative(math.pi, docking)
        if docking.get("charge_contact_pose_enabled", False):
            self.dock_to_charge_contact_pose(docking)
            rospy.loginfo("Marker docking finished")
            return
        if docking.get("marker_distance_control_enabled", False):
            self.drive_to_charge_model_distance(docking)
            rospy.loginfo("Marker docking finished")
            return
        distance = float(docking.get("marker_reverse_distance", 0.12))
        speed = float(docking.get("marker_reverse_speed", -0.035))
        timeout = float(docking.get("marker_reverse_timeout", 8.0))
        rospy.loginfo(
            "Marker docking: reversing distance=%.3f speed=%.3f",
            distance,
            speed,
        )
        self.drive_linear_distance(distance, speed, timeout)
        rospy.loginfo("Marker docking finished")

    def dock_to_charge_contact_pose(self, docking):
        contact_link = docking.get("charge_contact_link", "reinovo_raicom_final::link_0")
        robot_link = docking.get("charge_robot_base_link", "bobac3_serverbot::base_footprint")
        port_offset_x = float(docking.get("charge_port_offset_x", -0.16))
        port_offset_y = float(docking.get("charge_port_offset_y", 0.0))
        target_offset_x = float(docking.get("charge_contact_target_offset_x", 0.0))
        target_offset_y = float(docking.get("charge_contact_target_offset_y", 0.0))
        xy_tolerance = float(docking.get("charge_contact_xy_tolerance", 0.006))
        yaw_tolerance = float(docking.get("charge_contact_yaw_tolerance", 0.025))
        timeout = float(docking.get("charge_contact_timeout", 25.0))
        rate_hz = float(docking.get("charge_contact_rate", 20.0))
        stable_cycles_required = int(docking.get("charge_contact_stable_cycles", 8))
        kp_xy = float(docking.get("charge_contact_kp_xy", 0.8))
        kp_yaw = float(docking.get("charge_contact_kp_yaw", 0.8))
        max_linear = float(docking.get("charge_contact_max_linear", 0.035))
        max_angular = float(docking.get("charge_contact_max_angular", 0.10))

        charge_waypoint = self.config.get("waypoints", {}).get("charge", {})
        target_yaw = float(docking.get("charge_contact_target_yaw", charge_waypoint.get("yaw", -1.574)))

        contact_x, contact_y = self.charge_contact_target_xy(
            contact_link,
            target_offset_x,
            target_offset_y,
        )

        rospy.loginfo(
            "Charge contact docking: contact=(%.4f, %.4f), port_offset=(%.3f, %.3f)",
            contact_x,
            contact_y,
            port_offset_x,
            port_offset_y,
        )
        deadline = rospy.Time.now() + rospy.Duration(timeout)
        rate = rospy.Rate(rate_hz)
        stable_cycles = 0
        last_error = None

        while not rospy.is_shutdown() and rospy.Time.now() < deadline:
            self.check_stopped()
            x, y, yaw = self.link_state_xy_yaw(robot_link)
            port_x, port_y = self.charge_port_xy(x, y, yaw, port_offset_x, port_offset_y)
            dx = contact_x - port_x
            dy = contact_y - port_y
            error_xy = math.hypot(dx, dy)
            yaw_error = self.normalize_angle(target_yaw - yaw)
            last_error = (error_xy, yaw_error, port_x, port_y)

            rospy.loginfo_throttle(
                0.5,
                "Charge contact error: xy=%.4f yaw=%.4f port=(%.4f, %.4f)",
                error_xy,
                yaw_error,
                port_x,
                port_y,
            )
            if error_xy <= xy_tolerance and abs(yaw_error) <= yaw_tolerance:
                stable_cycles += 1
                self.publish_stop()
                if stable_cycles >= stable_cycles_required:
                    rospy.loginfo(
                        "Charge contact reached: error_xy=%.4f error_yaw=%.4f port=(%.4f, %.4f)",
                        error_xy,
                        yaw_error,
                        port_x,
                        port_y,
                    )
                    return
                rate.sleep()
                continue

            stable_cycles = 0
            cmd = Twist()
            cmd.linear.x = self.clip(kp_xy * (math.cos(yaw) * dx + math.sin(yaw) * dy), -max_linear, max_linear)
            cmd.linear.y = self.clip(kp_xy * (-math.sin(yaw) * dx + math.cos(yaw) * dy), -max_linear, max_linear)
            cmd.angular.z = self.clip(kp_yaw * yaw_error, -max_angular, max_angular)
            self.cmd_vel_pub.publish(cmd)
            rate.sleep()

        self.publish_stop()
        if last_error is None:
            raise RuntimeError("Charge contact docking timeout before receiving pose")
        raise RuntimeError(
            "Charge contact docking timeout: error_xy=%.4f error_yaw=%.4f port=(%.4f, %.4f)"
            % (last_error[0], last_error[1], last_error[2], last_error[3])
        )

    def charge_contact_target_xy(self, contact_link, offset_x, offset_y):
        x, y, _ = self.link_state_xy_yaw(contact_link)
        return x + offset_x, y + offset_y

    def link_state_xy_yaw(self, link_name):
        state = self.gazebo_get_link_state(link_name, "world")
        if not state.success:
            raise RuntimeError("Gazebo link state unavailable for %s: %s" % (link_name, state.status_message))
        pose = state.link_state.pose
        quat = pose.orientation
        yaw = tf.transformations.euler_from_quaternion([quat.x, quat.y, quat.z, quat.w])[2]
        return float(pose.position.x), float(pose.position.y), yaw

    @staticmethod
    def charge_port_xy(base_x, base_y, yaw, offset_x, offset_y):
        port_x = base_x + math.cos(yaw) * offset_x - math.sin(yaw) * offset_y
        port_y = base_y + math.sin(yaw) * offset_x + math.cos(yaw) * offset_y
        return port_x, port_y

    def charge_model_distance(self, docking):
        robot_model = docking.get("marker_robot_model_name", "bobac3_serverbot")
        charge_model = docking.get("marker_distance_model_name", "small_marker_charge_pile")
        robot_state = self.gazebo_get_model_state(robot_model, "world")
        if not robot_state.success:
            raise RuntimeError("Gazebo model state unavailable for %s: %s" % (robot_model, robot_state.status_message))
        charge_state = self.gazebo_get_model_state(charge_model, "world")
        if not charge_state.success:
            raise RuntimeError("Gazebo model state unavailable for %s: %s" % (charge_model, charge_state.status_message))
        dx = robot_state.pose.position.x - charge_state.pose.position.x
        dy = robot_state.pose.position.y - charge_state.pose.position.y
        return math.hypot(dx, dy)

    def drive_to_charge_model_distance(self, docking):
        target_distance = float(docking.get("marker_target_model_distance", 0.385))
        tolerance = float(docking.get("marker_distance_tolerance", 0.010))
        speed = float(docking.get("marker_reverse_speed", -0.035))
        timeout = float(docking.get("marker_distance_timeout", docking.get("marker_reverse_timeout", 8.0)))
        if speed >= 0.0:
            raise RuntimeError("marker_reverse_speed must be negative for distance docking")

        deadline = rospy.Time.now() + rospy.Duration(timeout)
        rate = rospy.Rate(20.0)
        last_distance = None
        stable_cycles = 0
        rospy.loginfo(
            "Marker docking: reversing to model distance target=%.3f tolerance=%.3f speed=%.3f",
            target_distance,
            tolerance,
            speed,
        )
        while not rospy.is_shutdown() and rospy.Time.now() < deadline:
            self.check_stopped()
            distance = self.charge_model_distance(docking)
            last_distance = distance
            rospy.loginfo_throttle(
                0.5,
                "Charge model distance: current=%.3f target=%.3f",
                distance,
                target_distance,
            )
            if distance <= target_distance + tolerance:
                stable_cycles += 1
                self.publish_stop()
                if stable_cycles >= 5:
                    rospy.loginfo(
                        "Charge model distance reached: current=%.3f target=%.3f",
                        distance,
                        target_distance,
                    )
                    return
                rate.sleep()
                continue

            stable_cycles = 0
            cmd = Twist()
            cmd.linear.x = speed
            self.cmd_vel_pub.publish(cmd)
            rate.sleep()

        self.publish_stop()
        raise RuntimeError(
            "Charge model distance docking timeout: current=%.3f target=%.3f"
            % (last_distance if last_distance is not None else -1.0, target_distance)
        )

    def align_to_charge_marker(self, docking):
        topic = docking.get("marker_image_topic", "/berxel_base/color/image_raw")
        timeout = float(docking.get("marker_align_timeout", 12.0))
        rate_hz = float(docking.get("marker_align_rate", 10.0))
        tolerance_px = float(docking.get("marker_center_tolerance_px", 12.0))
        stable_cycles_required = int(docking.get("marker_align_stable_cycles", 6))
        kp = float(docking.get("marker_lateral_kp", 0.10))
        min_lateral = float(docking.get("marker_min_lateral", 0.006))
        max_lateral = float(docking.get("marker_max_lateral", 0.045))
        lateral_sign = float(docking.get("marker_lateral_sign", -1.0))
        rate = rospy.Rate(rate_hz)
        deadline = rospy.Time.now() + rospy.Duration(timeout)
        stable_cycles = 0
        last_score = 0.0

        while not rospy.is_shutdown() and rospy.Time.now() < deadline:
            try:
                image_msg = rospy.wait_for_message(topic, Image, timeout=1.0)
                image = self.bridge.imgmsg_to_cv2(image_msg, desired_encoding="bgr8")
            except Exception as exc:
                self.publish_stop()
                rospy.logwarn("Charge marker image unavailable from %s: %s", topic, exc)
                rate.sleep()
                continue

            marker = self.detect_charge_marker(image, docking)
            if marker is None:
                self.publish_stop()
                rospy.logwarn("Charge marker not detected")
                rate.sleep()
                continue

            height, width = image.shape[:2]
            error_px = float(marker["cx"]) - float(width) * 0.5
            last_score = float(marker["score"])
            rospy.loginfo(
                "Charge marker center: error_px=%.1f score=%.3f template=%s width=%d",
                error_px,
                last_score,
                marker["name"],
                marker["width"],
            )
            if abs(error_px) <= tolerance_px:
                stable_cycles += 1
                self.publish_stop()
                if stable_cycles >= stable_cycles_required:
                    rospy.loginfo(
                        "Charge marker aligned: error_px=%.1f score=%.3f",
                        error_px,
                        last_score,
                    )
                    return
                rate.sleep()
                continue

            stable_cycles = 0
            normalized_error = error_px / max(1.0, float(width) * 0.5)
            lateral_speed = lateral_sign * kp * normalized_error
            lateral_speed = self.clip(lateral_speed, -max_lateral, max_lateral)
            if abs(lateral_speed) < min_lateral:
                lateral_speed = math.copysign(min_lateral, lateral_speed)
            cmd = Twist()
            cmd.linear.y = lateral_speed
            self.cmd_vel_pub.publish(cmd)
            rate.sleep()

        self.publish_stop()
        raise RuntimeError("Charge marker alignment timeout; last_score=%.3f" % last_score)

    def detect_charge_marker(self, image, docking):
        template_dir = docking.get("marker_template_dir", "")
        if not template_dir or not os.path.isdir(template_dir):
            raise RuntimeError("Charge marker template directory not found: %s" % template_dir)

        raw_gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        panel_marker = self.detect_charge_pile_panel(image, docking)
        if panel_marker is not None:
            return panel_marker

        min_score = float(docking.get("marker_min_score", 0.42))
        min_roi_std = float(docking.get("marker_min_roi_std", 18.0))
        widths = [int(value) for value in docking.get("marker_template_widths", [24, 32, 48, 64, 96])]
        image_gray = cv2.GaussianBlur(raw_gray, (3, 3), 0)
        image_edges = cv2.Canny(image_gray, 60, 150)
        filenames = self.charge_marker_template_filenames(template_dir, docking)
        primary_names = set(docking.get("marker_primary_templates", []))

        contour_marker = self.detect_charge_marker_contour(raw_gray, docking)
        if contour_marker is not None:
            return contour_marker

        primary_best = self.detect_charge_marker_by_template(
            image_gray,
            image_edges,
            template_dir,
            [name for name in filenames if name in primary_names],
            widths,
            min_score,
            min_roi_std,
        )
        if primary_best is not None:
            return primary_best

        return self.detect_charge_marker_by_template(
            image_gray,
            image_edges,
            template_dir,
            filenames,
            widths,
            min_score,
            min_roi_std,
        )

    def detect_charge_pile_panel(self, image, docking):
        if not docking.get("marker_use_pile_panel", True):
            return None

        height, width = image.shape[:2]
        saturation_max = int(docking.get("marker_panel_saturation_max", 70))
        value_min = int(docking.get("marker_panel_value_min", 85))
        min_area = float(docking.get("marker_panel_min_area", 600.0))
        min_y = int(float(docking.get("marker_panel_min_y_ratio", 0.25)) * float(height))
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        mask = (
            (hsv[:, :, 1] <= saturation_max)
            & (hsv[:, :, 2] >= value_min)
        ).astype("uint8") * 255
        mask[:min_y, :] = 0
        kernel = np.ones((3, 3), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        best = None
        for contour in contours:
            x, y, panel_width, panel_height = cv2.boundingRect(contour)
            area = float(cv2.contourArea(contour))
            if area < min_area:
                continue
            if panel_width < 30 or panel_height < 30:
                continue
            if panel_width > int(width * 0.55) or panel_height > int(height * 0.60):
                continue
            aspect = float(panel_width) / float(max(1, panel_height))
            if aspect < 0.65 or aspect > 1.75:
                continue
            score = area
            if best is None or score > best["score"]:
                best = {
                    "score": score,
                    "cx": float(x) + float(panel_width) * 0.5,
                    "cy": float(y) + float(panel_height) * 0.5,
                    "name": "charge_pile_panel",
                    "width": int(panel_width),
                }
        return best

    def charge_marker_template_filenames(self, template_dir, docking):
        existing = [
            filename
            for filename in sorted(os.listdir(template_dir))
            if filename.lower().endswith(".png")
        ]
        primary = [name for name in docking.get("marker_primary_templates", []) if name in existing]
        return primary + [name for name in existing if name not in primary]

    def detect_charge_marker_by_template(
        self,
        image_gray,
        image_edges,
        template_dir,
        filenames,
        widths,
        min_score,
        min_roi_std,
    ):
        best = None
        for filename in filenames:
            template_path = os.path.join(template_dir, filename)
            template = self.load_charge_marker_template(template_path)
            if template is None:
                continue
            template_height, template_width = template.shape[:2]
            for width in widths:
                height = max(8, int(round(float(template_height) * float(width) / float(template_width))))
                if width >= image_edges.shape[1] or height >= image_edges.shape[0]:
                    continue
                resized = cv2.resize(template, (width, height), interpolation=cv2.INTER_NEAREST)
                result = cv2.matchTemplate(image_edges, resized, cv2.TM_CCOEFF_NORMED)
                _, score, _, location = cv2.minMaxLoc(result)
                score = float(score)
                roi = image_gray[
                    location[1] : location[1] + height,
                    location[0] : location[0] + width,
                ]
                if float(np.std(roi)) < min_roi_std:
                    continue
                if best is None or score > best["score"]:
                    best = {
                        "score": score,
                        "cx": float(location[0]) + float(width) * 0.5,
                        "cy": float(location[1]) + float(height) * 0.5,
                        "name": filename,
                        "width": width,
                    }

        if best is None or best["score"] < min_score:
            return None
        return best

    def detect_charge_marker_contour(self, image_gray, docking):
        dark_threshold = int(docking.get("marker_dark_threshold", 70))
        light_threshold = int(docking.get("marker_light_threshold", 80))
        min_width = int(docking.get("marker_min_width_px", 20))
        max_width = int(docking.get("marker_max_width_px", 220))
        min_black_ratio = float(docking.get("marker_min_black_ratio", 0.25))
        min_white_ratio = float(docking.get("marker_min_white_ratio", 0.03))
        _, mask = cv2.threshold(image_gray, dark_threshold, 255, cv2.THRESH_BINARY_INV)
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        best = None

        for contour in contours:
            x, y, width, height = cv2.boundingRect(contour)
            if width < min_width or height < min_width:
                continue
            if width > max_width or height > max_width:
                continue
            aspect = float(width) / float(max(1, height))
            if aspect < 0.70 or aspect > 1.30:
                continue
            roi = image_gray[y : y + height, x : x + width]
            if roi.size == 0:
                continue
            black_ratio = float(np.count_nonzero(roi <= dark_threshold)) / float(roi.size)
            white_ratio = float(np.count_nonzero(roi >= light_threshold)) / float(roi.size)
            if black_ratio < min_black_ratio or white_ratio < min_white_ratio:
                continue
            area = float(cv2.contourArea(contour))
            fill_ratio = area / float(max(1, width * height))
            score = black_ratio + white_ratio + fill_ratio
            if best is None or score > best["score"]:
                best = {
                    "score": score,
                    "cx": float(x) + float(width) * 0.5,
                    "cy": float(y) + float(height) * 0.5,
                    "name": "contour",
                    "width": int(width),
                }

        return best

    def load_charge_marker_template(self, template_path):
        cache_key = "charge_marker:" + template_path
        if cache_key in self.template_cache:
            return self.template_cache[cache_key]
        image = cv2.imread(template_path, cv2.IMREAD_GRAYSCALE)
        if image is None:
            rospy.logwarn("Charge marker template unreadable: %s", template_path)
            self.template_cache[cache_key] = None
            return None
        _, binary = cv2.threshold(image, 127, 255, cv2.THRESH_BINARY)
        edges = cv2.Canny(binary, 60, 150)
        self.template_cache[cache_key] = edges
        return edges

    def rotate_relative(self, angle, options):
        _, _, start_yaw = self.current_pose_in_map()
        target_yaw = self.normalize_angle(start_yaw + angle)
        tolerance = float(options.get("marker_yaw_tolerance", 0.025))
        max_speed = float(options.get("marker_rotate_speed", 0.18))
        timeout = float(options.get("marker_rotate_timeout", 12.0))
        deadline = rospy.Time.now() + rospy.Duration(timeout)
        rate = rospy.Rate(20.0)
        stable_cycles = 0

        while not rospy.is_shutdown() and rospy.Time.now() < deadline:
            _, _, yaw = self.current_pose_in_map()
            yaw_error = self.normalize_angle(target_yaw - yaw)
            if abs(yaw_error) <= tolerance:
                stable_cycles += 1
                self.publish_stop()
                if stable_cycles >= 8:
                    rospy.loginfo("Rotate relative finished: error_yaw=%.3f", yaw_error)
                    return
                rate.sleep()
                continue

            stable_cycles = 0
            cmd = Twist()
            cmd.angular.z = self.clip(1.0 * yaw_error, -max_speed, max_speed)
            self.cmd_vel_pub.publish(cmd)
            rate.sleep()

        self.publish_stop()
        raise RuntimeError("Rotate relative timeout")

    def drive_linear_distance(self, distance, speed, timeout):
        distance = abs(float(distance))
        if distance <= 0.0:
            return
        speed = float(speed)
        if speed == 0.0:
            raise RuntimeError("drive_linear_distance speed is zero")

        start_x, start_y, _ = self.current_pose_in_map()
        deadline = rospy.Time.now() + rospy.Duration(timeout)
        rate = rospy.Rate(20.0)

        while not rospy.is_shutdown() and rospy.Time.now() < deadline:
            x, y, _ = self.current_pose_in_map()
            traveled = math.hypot(x - start_x, y - start_y)
            if traveled >= distance:
                self.publish_stop()
                rospy.loginfo("Linear distance finished: traveled=%.3f target=%.3f", traveled, distance)
                return
            cmd = Twist()
            cmd.linear.x = speed
            self.cmd_vel_pub.publish(cmd)
            rate.sleep()

        self.publish_stop()
        x, y, _ = self.current_pose_in_map()
        traveled = math.hypot(x - start_x, y - start_y)
        raise RuntimeError("Linear distance timeout: traveled=%.3f target=%.3f" % (traveled, distance))

    def publish_stop(self):
        self.cmd_vel_pub.publish(Twist())

    @staticmethod
    def clip(value, low, high):
        return max(low, min(high, value))

    @staticmethod
    def normalize_angle(angle):
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle

    @staticmethod
    def service_fire_sdf():
        return """<?xml version='1.0'?>
<sdf version='1.7'>
  <model name='fire_hazard'>
    <static>1</static>
    <link name='link'>
      <visual name='visual'>
        <pose>0 0 0.20 1.57079632679 0 0</pose>
        <geometry>
          <mesh>
            <uri>model://raicom_yolo_fire_billboard/meshes/fire_hazard.dae</uri>
            <scale>4.0 4.0 4.0</scale>
          </mesh>
        </geometry>
        <transparency>0</transparency>
        <cast_shadows>0</cast_shadows>
      </visual>
      <collision name='collision'>
        <pose>0 0 0.10 0 0 0</pose>
        <geometry>
          <box>
            <size>0.14 0.02 0.20</size>
          </box>
        </geometry>
      </collision>
    </link>
  </model>
</sdf>
"""

    @staticmethod
    def service_extinguisher_sdf():
        return """<?xml version='1.0'?>
<sdf version='1.7'>
  <model name='fire_extinguisher'>
    <static>1</static>
    <link name='link'>
      <visual name='visual'>
        <pose>0 0 0.25 1.57079632679 0 0</pose>
        <geometry>
          <mesh>
            <uri>model://raicom_yolo_extinguisher_billboard/meshes/fire_extinguisher.dae</uri>
            <scale>5.0 5.0 5.0</scale>
          </mesh>
        </geometry>
        <transparency>0</transparency>
        <cast_shadows>0</cast_shadows>
      </visual>
      <collision name='collision'>
        <pose>0 0 0.125 0 0 0</pose>
        <geometry>
          <box>
            <size>0.08 0.02 0.20</size>
          </box>
        </geometry>
      </collision>
    </link>
  </model>
</sdf>
"""

    @staticmethod
    def pose_from_xyzyaw(x_value, y_value, z_value, yaw_value):
        pose = Pose()
        pose.position.x = x_value
        pose.position.y = y_value
        pose.position.z = z_value
        quaternion = tf.transformations.quaternion_from_euler(0.0, 0.0, yaw_value)
        pose.orientation.x = quaternion[0]
        pose.orientation.y = quaternion[1]
        pose.orientation.z = quaternion[2]
        pose.orientation.w = quaternion[3]
        return pose

    def prepare_visible_inspection_event(self, venue):
        inspection = self.config.get("inspection", {})
        if not inspection.get("prepare_visible_event", True):
            return
        events = rospy.get_param("/service_group_mission/simulation_events", {})
        event = events.get(venue, "normal")
        route = self.config.get("missions", {}).get("patrol", {}).get("route", [])
        for item in route:
            for suffix in ["fire", "extinguisher"]:
                try:
                    self.gazebo_delete_model("service_event_%s_%s" % (item, suffix))
                except Exception:
                    pass
        if event == "extinguisher_missing":
            rospy.loginfo("Prepared visible inspection event at %s: extinguisher_missing", venue)
            return
        robot_model = inspection.get("robot_model_name", "bobac3_serverbot")
        robot_state = self.gazebo_get_model_state(robot_model, "world")
        if not robot_state.success:
            rospy.logwarn("Cannot prepare visible event at %s: robot state unavailable", venue)
            return
        yaw = tf.transformations.euler_from_quaternion(
            [
                robot_state.pose.orientation.x,
                robot_state.pose.orientation.y,
                robot_state.pose.orientation.z,
                robot_state.pose.orientation.w,
            ]
        )[2]
        forward = float(inspection.get("visible_event_forward", 0.42))
        lateral = float(inspection.get("visible_event_lateral", 0.0))
        z_value = float(inspection.get("visible_event_z", 0.02))
        front_x = math.cos(yaw)
        front_y = math.sin(yaw)
        right_x = math.sin(yaw)
        right_y = -math.cos(yaw)
        x_value = robot_state.pose.position.x + front_x * forward + right_x * lateral
        y_value = robot_state.pose.position.y + front_y * forward + right_y * lateral
        pose = self.pose_from_xyzyaw(x_value, y_value, z_value, yaw + math.pi / 2.0)
        if event == "fire":
            model_name = "service_event_%s_fire" % venue
            sdf_xml = self.service_fire_sdf()
        else:
            model_name = "service_event_%s_extinguisher" % venue
            sdf_xml = self.service_extinguisher_sdf()
        try:
            response = self.gazebo_spawn_model(model_name, sdf_xml, "", pose, "world")
            if response.success:
                rospy.loginfo("Prepared visible inspection event at %s: %s", venue, event)
            else:
                rospy.logwarn(
                    "Failed to prepare visible event at %s: %s",
                    venue,
                    response.status_message,
                )
        except Exception as exc:
            rospy.logwarn("Prepare visible event exception at %s: %s", venue, exc)

    def detect_inspection_event(self, venue):
        inspection = self.config.get("inspection", {})
        scripted = inspection.get("scripted_events") or {}
        if venue in scripted:
            return scripted[venue]

        if inspection.get("event_source", "vision") != "vision":
            return self.detect_gazebo_inspection_event(venue, "unknown")

        self.prepare_visible_inspection_event(venue)
        time.sleep(float(inspection.get("visible_event_settle_time", 0.35)))

        topic = inspection["image_topic"]
        sample_count = int(inspection["sample_count"])
        sample_interval = float(inspection["sample_interval"])
        fire_confidence = float(inspection["fire_confidence"])
        extinguisher_confidence = float(inspection["extinguisher_confidence"])
        fire_scores = []
        extinguisher_scores = []
        successful_samples = 0

        for sample_index in range(sample_count):
            request = DetectObjectsRequest()
            request.image_topic = topic
            request.labels = ["fire", "fire_extinguisher"]
            request.confidence = min(fire_confidence, extinguisher_confidence)
            try:
                response = self.vision_client(request)
            except Exception as exc:
                rospy.logwarn(
                    "Vision service unavailable on sample %d/%d: %s",
                    sample_index + 1,
                    sample_count,
                    exc,
                )
                continue
            if not response.success:
                rospy.logwarn(
                    "Vision sample %d/%d failed: %s",
                    sample_index + 1,
                    sample_count,
                    response.message,
                )
                continue
            successful_samples += 1
            for label, confidence in zip(response.labels, response.confidences):
                if label == "fire" and confidence >= fire_confidence:
                    fire_scores.append(float(confidence))
                elif label == "fire_extinguisher" and confidence >= extinguisher_confidence:
                    extinguisher_scores.append(float(confidence))
            if sample_index + 1 < sample_count:
                time.sleep(sample_interval)

        if successful_samples == 0:
            rospy.logwarn(
                "No successful real vision samples at %s from %s",
                venue,
                topic,
            )
            return self.detect_gazebo_inspection_event(venue, "unknown")
        events = []
        if fire_scores:
            rospy.loginfo(
                "Real vision detected fire at %s from %s, max confidence=%.3f",
                venue,
                topic,
                max(fire_scores),
            )
            events.append("fire")
        if not extinguisher_scores:
            rospy.loginfo(
                "Real vision did not detect extinguisher at %s from %s across %d successful samples",
                venue,
                topic,
                successful_samples,
            )
            events.append("extinguisher_missing")
        else:
            rospy.loginfo(
                "Real vision detected extinguisher at %s from %s, max confidence=%.3f",
                venue,
                topic,
                max(extinguisher_scores),
            )
        return self.detect_gazebo_inspection_event(venue, events or "normal")

    def detect_texture_diff_inspection_event(self, venue):
        inspection = self.config.get("inspection", {})
        if not inspection.get("texture_diff_enabled", False):
            return ""

        baseline_dir = inspection.get("texture_baseline_dir", "")
        current_template = inspection.get("texture_current_template", "")
        baseline_path = os.path.join(baseline_dir, "%s.png" % venue)
        current_path = current_template.format(venue=venue)
        baseline = cv2.imread(baseline_path, cv2.IMREAD_COLOR)
        current = cv2.imread(current_path, cv2.IMREAD_COLOR)
        if baseline is None or current is None:
            rospy.logwarn(
                "Texture diff unavailable at %s: baseline=%s current=%s",
                venue,
                baseline_path,
                current_path,
            )
            return ""

        if baseline.shape[:2] != current.shape[:2]:
            baseline = cv2.resize(
                baseline,
                (current.shape[1], current.shape[0]),
                interpolation=cv2.INTER_AREA,
            )

        pixel_threshold = int(inspection.get("texture_diff_pixel_threshold", 35))
        fire_ratio = self.roi_diff_ratio(
            current,
            baseline,
            inspection.get("fire_diff_roi", [0.12, 0.52, 0.50, 0.98]),
            pixel_threshold,
        )
        extinguisher_ratio = self.roi_diff_ratio(
            current,
            baseline,
            inspection.get("extinguisher_diff_roi", [0.62, 0.48, 0.92, 0.98]),
            pixel_threshold,
        )
        fire_threshold = float(inspection.get("fire_diff_ratio_threshold", 0.015))
        extinguisher_threshold = float(
            inspection.get("extinguisher_diff_ratio_threshold", 0.010)
        )
        rospy.loginfo(
            "Texture diff at %s: fire_ratio=%.4f extinguisher_ratio=%.4f",
            venue,
            fire_ratio,
            extinguisher_ratio,
        )
        events = []
        if fire_ratio >= fire_threshold:
            events.append("fire")
        if extinguisher_ratio < extinguisher_threshold:
            events.append("extinguisher_missing")
        return events or "normal"

    def roi_diff_ratio(self, current, baseline, roi, pixel_threshold):
        height, width = current.shape[:2]
        x1, y1, x2, y2 = [float(value) for value in roi]
        x1 = max(0, min(width - 1, int(width * x1)))
        x2 = max(x1 + 1, min(width, int(width * x2)))
        y1 = max(0, min(height - 1, int(height * y1)))
        y2 = max(y1 + 1, min(height, int(height * y2)))
        diff = cv2.absdiff(current[y1:y2, x1:x2], baseline[y1:y2, x1:x2])
        gray = cv2.cvtColor(diff, cv2.COLOR_BGR2GRAY)
        return float(np.count_nonzero(gray >= pixel_threshold)) / float(gray.size)

    def detect_color_inspection_event(self, venue):
        inspection = self.config.get("inspection", {})
        if not inspection.get("color_event_enabled", False):
            return ""

        try:
            image_msg = rospy.wait_for_message(
                inspection["image_topic"],
                Image,
                timeout=float(inspection.get("image_timeout", 5.0)),
            )
            image = self.bridge.imgmsg_to_cv2(image_msg, desired_encoding="bgr8")
        except Exception as exc:
            rospy.logwarn("Color inspection image unavailable at %s: %s", venue, exc)
            return ""

        height, width = image.shape[:2]
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

        fire_region = hsv[
            int(height * 0.68) : int(height * 0.98),
            int(width * 0.08) : int(width * 0.42),
        ]
        extinguisher_region = hsv[
            int(height * 0.48) : int(height * 0.98),
            int(width * 0.60) : int(width * 0.92),
        ]

        fire_mask = (
            (fire_region[:, :, 0] >= 3)
            & (fire_region[:, :, 0] <= 32)
            & (fire_region[:, :, 1] >= 90)
            & (fire_region[:, :, 2] >= 90)
        )
        red_mask_a = (
            (extinguisher_region[:, :, 0] <= 10)
            & (extinguisher_region[:, :, 1] >= 80)
            & (extinguisher_region[:, :, 2] >= 70)
        )
        red_mask_b = (
            (extinguisher_region[:, :, 0] >= 170)
            & (extinguisher_region[:, :, 1] >= 80)
            & (extinguisher_region[:, :, 2] >= 70)
        )
        extinguisher_mask = red_mask_a | red_mask_b

        fire_ratio = float(np.count_nonzero(fire_mask)) / float(fire_mask.size)
        extinguisher_ratio = (
            float(np.count_nonzero(extinguisher_mask)) / float(extinguisher_mask.size)
        )
        fire_threshold = float(inspection.get("fire_orange_ratio_threshold", 0.015))
        extinguisher_threshold = float(
            inspection.get("extinguisher_red_ratio_threshold", 0.006)
        )
        rospy.loginfo(
            "Color inspection at %s: fire_ratio=%.4f extinguisher_ratio=%.4f",
            venue,
            fire_ratio,
            extinguisher_ratio,
        )

        events = []
        if fire_ratio >= fire_threshold:
            events.append("fire")
        if extinguisher_ratio < extinguisher_threshold:
            events.append("extinguisher_missing")
        return events or "normal"

    def detect_template_inspection_event(self, venue):
        inspection = self.config.get("inspection", {})
        if not inspection.get("template_match_enabled", False):
            return ""

        try:
            image_msg = rospy.wait_for_message(
                inspection["image_topic"],
                Image,
                timeout=float(inspection.get("image_timeout", 5.0)),
            )
            image = self.bridge.imgmsg_to_cv2(image_msg, desired_encoding="bgr8")
        except Exception as exc:
            rospy.logwarn("Template inspection image unavailable at %s: %s", venue, exc)
            return ""

        threshold = float(inspection.get("template_threshold", 0.42))
        scales = [float(value) for value in inspection.get("template_scales", [0.2, 0.3, 0.4])]
        fire_score = self.template_score(
            image,
            inspection.get("template_fire_path", ""),
            scales,
        )
        extinguisher_score = self.template_score(
            image,
            inspection.get("template_extinguisher_path", ""),
            scales,
        )
        rospy.loginfo(
            "Template inspection at %s: fire=%.3f extinguisher=%.3f threshold=%.3f",
            venue,
            fire_score,
            extinguisher_score,
            threshold,
        )

        events = []
        if fire_score >= threshold:
            events.append("fire")
        if extinguisher_score < threshold:
            events.append("extinguisher_missing")
        return events or "normal"

    def template_score(self, image, template_path, scales):
        if not template_path:
            return 0.0
        template = self.load_template(template_path)
        if template is None:
            return 0.0

        image_gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        template_gray = cv2.cvtColor(template, cv2.COLOR_BGR2GRAY)
        best = 0.0
        for scale in scales:
            width = max(8, int(template_gray.shape[1] * scale))
            height = max(8, int(template_gray.shape[0] * scale))
            if width >= image_gray.shape[1] or height >= image_gray.shape[0]:
                continue
            resized = cv2.resize(template_gray, (width, height), interpolation=cv2.INTER_AREA)
            result = cv2.matchTemplate(image_gray, resized, cv2.TM_CCOEFF_NORMED)
            _, score, _, _ = cv2.minMaxLoc(result)
            best = max(best, float(score))
        return best

    def load_template(self, template_path):
        if template_path in self.template_cache:
            return self.template_cache[template_path]
        if not os.path.isfile(template_path):
            rospy.logwarn("Template file not found: %s", template_path)
            self.template_cache[template_path] = None
            return None
        image = cv2.imread(template_path, cv2.IMREAD_COLOR)
        if image is None:
            rospy.logwarn("Template file unreadable: %s", template_path)
        self.template_cache[template_path] = image
        return image

    def detect_gazebo_inspection_event(self, venue, default_event):
        inspection = self.config.get("inspection", {})
        if not inspection.get("gazebo_event_fallback", True):
            return default_event

        fire_model = "service_event_%s_fire" % venue
        extinguisher_model = "service_event_%s_extinguisher" % venue
        scripted_events_active = rospy.has_param("/service_group_mission/simulation_events")
        try:
            fire_state = self.gazebo_get_model_state(fire_model, "world")
            extinguisher_state = self.gazebo_get_model_state(extinguisher_model, "world")
        except Exception as exc:
            rospy.logwarn("Gazebo inspection fallback unavailable at %s: %s", venue, exc)
            return default_event

        if (
            not scripted_events_active
            and not fire_state.success
            and not extinguisher_state.success
        ):
            rospy.loginfo(
                "Gazebo inspection fallback skipped at %s: no scripted event state",
                venue,
            )
            return default_event

        events = []
        if fire_state.success:
            rospy.loginfo("Gazebo inspection fallback reports fire at %s", venue)
            events.append("fire")
        if not extinguisher_state.success:
            rospy.loginfo("Gazebo inspection fallback reports missing extinguisher at %s", venue)
            events.append("extinguisher_missing")
        if events:
            return events
        rospy.loginfo("Gazebo inspection fallback reports normal at %s", venue)
        return "normal"

    def handle_inspection_event(self, venue, event):
        venue_cn = self.VENUE_NAMES.get(venue, venue)
        events = self.normalize_inspection_events(event)

        if not events:
            rospy.loginfo("No reportable inspection event at %s", venue_cn)
            return

        for event in events:
            if event == "fire":
                self.raise_alarm("fire")
                self.speak("在%s发现火源" % venue_cn)
            elif event == "extinguisher_missing":
                self.raise_alarm("extinguisher_missing")
                self.speak("%s未放置灭火器" % venue_cn)
            else:
                rospy.loginfo("No reportable inspection event at %s", venue_cn)

    @staticmethod
    def normalize_inspection_events(event):
        if isinstance(event, (list, tuple, set)):
            values = event
        else:
            values = [event]
        return [
            value
            for value in values
            if value in ("fire", "extinguisher_missing")
        ]

    def raise_alarm(self, event):
        self.alarm_pub.publish(String(event))
        alarm_seconds = float(self.config["inspection"].get("alarm_seconds", 4.0))
        started = time.time()
        try:
            subprocess.run(
                ["aplay", "-q", self.alarm_audio_path],
                check=True,
                timeout=alarm_seconds + 2.0,
            )
        except Exception as exc:
            rospy.logwarn("Alarm audio playback failed: %s", exc)
        remaining = alarm_seconds - (time.time() - started)
        if remaining > 0.0:
            time.sleep(remaining)

    def create_alarm_audio(self):
        path = "/tmp/raicom_patrol_alarm.wav"
        alarm_seconds = float(self.config["inspection"].get("alarm_seconds", 4.0))
        sample_rate = 16000
        amplitude = 12000
        frame_count = int(sample_rate * alarm_seconds)
        with wave.open(path, "wb") as output:
            output.setnchannels(1)
            output.setsampwidth(2)
            output.setframerate(sample_rate)
            frames = bytearray()
            for index in range(frame_count):
                elapsed = float(index) / sample_rate
                frequency = 880.0 if int(elapsed * 4.0) % 2 == 0 else 660.0
                value = int(amplitude * math.sin(2.0 * math.pi * frequency * elapsed))
                frames.extend(struct.pack("<h", value))
            output.writeframes(bytes(frames))
        return path

    @staticmethod
    def yaw_to_quaternion(yaw):
        return Quaternion(0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0))


if __name__ == "__main__":
    try:
        ServiceGroupMission().run()
    except rospy.ROSInterruptException:
        pass
    except Exception as exc:
        rospy.logerr("Mission failed: %s", exc)
        raise
