#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import hashlib
import math
import os

import rospy
from geometry_msgs.msg import Twist

from .charge_docking import ChargeDockingClient
from .config import HomeMissionConfig
from .interaction import InteractionManager
from .navigation import NavigationClient
from .recipes import RecipeBook
from .speech import SpeechClient
from .status import StatusPublisher
from .vision import VisionClient


class HomeServiceMission:
    def __init__(self):
        rospy.init_node("home_service_mission")
        self.config_file = rospy.get_param("~config_file", "")
        self.mission_type = rospy.get_param("~mission_type", "auto")
        self.face_mode = rospy.get_param("~face_mode", "any")
        self.voice_mode = rospy.get_param("~voice_mode", "topic")
        self.target_object = rospy.get_param("~target_object", "")

        self.config = HomeMissionConfig(self.config_file)
        self.status = StatusPublisher(
            "/home_service_mission/status",
            lambda: self.mission_type,
        )
        self.cmd_vel_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
        self.navigation = NavigationClient(
            self.config,
            self.cmd_vel_pub,
            self.status,
            self.check_stopped,
        )
        self.interaction = InteractionManager(
            self.config,
            self.face_mode,
            self.voice_mode,
            self.status,
            self.navigation.cancel_all_goals,
        )
        interaction_config = self.config.section("interaction")
        self.speech = SpeechClient(
            "/home_service_mission/speech",
            interaction_config["tts_service"],
            self.check_stopped,
        )
        self.vision = VisionClient(self.status, self.check_stopped)
        self.docking = ChargeDockingClient(
            self.cmd_vel_pub,
            self.status,
            self.check_stopped,
            self.config,
        )
        self.log_config_state("initial_load")

    def check_stopped(self):
        self.interaction.check_stopped()

    def config_metadata(self):
        metadata = {
            "path": self.config_file,
            "mtime": "",
            "md5": "",
        }
        try:
            metadata["mtime"] = rospy.Time.from_sec(
                os.path.getmtime(self.config_file)
            ).to_sec()
            digest = hashlib.md5()
            with open(self.config_file, "rb") as stream:
                for chunk in iter(lambda: stream.read(65536), b""):
                    digest.update(chunk)
            metadata["md5"] = digest.hexdigest()
        except Exception as exc:
            metadata["error"] = str(exc)
        return metadata

    def log_config_state(self, reason):
        metadata = self.config_metadata()
        try:
            waypoints = self.config.section("waypoints")
            navigation_config = self.config.section("navigation")
            assistant = self.config.optional_section("assistant")
            find_object = self.config.optional_section("find_object")
            rospy.loginfo(
                "HOME_CONFIG_LOADED reason=%s path=%s mtime=%s md5=%s "
                "start=(%.4f,%.4f) start_return=(%.4f,%.4f) "
                "corridor=(%.4f,%.4f) restaurant=(%.4f,%.4f) "
                "living_room=(%.4f,%.4f) bedroom=(%.4f,%.4f) "
                "return_route=%s assistant_return_route=%s find_route=%s",
                reason,
                metadata.get("path", ""),
                metadata.get("mtime", ""),
                metadata.get("md5", ""),
                float(waypoints["start"]["x"]),
                float(waypoints["start"]["y"]),
                float(waypoints["start_return"]["x"]),
                float(waypoints["start_return"]["y"]),
                float(waypoints["corridor"]["x"]),
                float(waypoints["corridor"]["y"]),
                float(waypoints["restaurant"]["x"]),
                float(waypoints["restaurant"]["y"]),
                float(waypoints["living_room"]["x"]),
                float(waypoints["living_room"]["y"]),
                float(waypoints["bedroom"]["x"]),
                float(waypoints["bedroom"]["y"]),
                navigation_config.get("return_route", []),
                assistant.get("return_route", []),
                find_object.get("route", []),
            )
            self.status.publish(
                "config_loaded",
                reason=reason,
                path=metadata.get("path", ""),
                mtime=metadata.get("mtime", ""),
                md5=metadata.get("md5", ""),
            )
        except Exception as exc:
            rospy.logwarn("HOME_CONFIG_LOG_FAILED reason=%s error=%s", reason, exc)

    def reload_config(self, reason):
        self.config = HomeMissionConfig(self.config_file)
        self.navigation.config = self.config
        self.navigation.navigation = self.config.section("navigation")
        self.interaction.config = self.config
        self.docking.config = self.config
        self.log_config_state(reason)
        self.precache_fixed_speech(reason)

    def precache_fixed_speech(self, reason):
        texts = self.collect_fixed_speech_texts()
        rospy.loginfo("TTS_PRECACHE_FIXED reason=%s count=%d", reason, len(texts))
        self.status.publish("tts_precache_fixed", reason=reason, count=len(texts))
        self.speech.prepare_many_async(texts, initial_delay=0.1)

    def collect_fixed_speech_texts(self):
        texts = []
        seen = set()
        mission_type = self.mission_type

        def add(text):
            text = text or ""
            if text and text not in seen:
                texts.append(text)
                seen.add(text)

        interaction = self.config.optional_section("interaction")
        add(interaction.get("wake_prompt", ""))

        if mission_type in ("guide", "auto"):
            add(interaction.get("accept", ""))
            guide = self.config.optional_section("guide")
            for reply in guide.get("interference_replies", []):
                add(reply.get("speech", ""))
            for text in guide.get("introductions", {}).values():
                add(text)

        if mission_type in ("assistant", "auto"):
            assistant = self.config.optional_section("assistant")
            for text in self.assistant_tts_precache_texts(assistant):
                add(text)

        if mission_type in ("find_object", "auto"):
            find_config = self.config.optional_section("find_object")
            add(find_config.get("empty_speech", ""))
            labels = find_config.get("labels", [])
            chinese_names = self.config.optional_section("chinese_names")
            for label in labels:
                target_cn = chinese_names.get(label, label)
                for key in (
                    "start_speech_template",
                    "found_speech_template",
                    "not_found_speech_template",
                ):
                    template = find_config.get(key, "")
                    if template:
                        try:
                            add(template % target_cn)
                        except Exception:
                            add(template)
                other_prefix = find_config.get("other_prefix", "")
                if other_prefix:
                    add(other_prefix + target_cn)

        finish = self.config.optional_section("finish")
        add(finish.get("start_speech", ""))
        add(finish.get("complete_speech", ""))
        return texts

    def run(self):
        if self.mission_type == "national_sequence":
            self.run_national_sequence()
            return
        self.run_current_mission(validate=True, connect=True)

    def run_current_mission(self, validate=True, connect=True):
        self.reload_config("before_mission")
        if validate:
            self.config.validate(self.mission_type)
        if connect:
            self.navigation.connect()
        self.status.publish("started")
        if self.mission_type == "charge":
            self.run_charge()
            self.status.publish("completed")
            return

        navigation_config = self.config.section("navigation")
        for waypoint in navigation_config.get("corridor_route", ["corridor"]):
            self.go_to_route_item(
                waypoint,
                default_ignore_yaw=navigation_config.get(
                    "corridor_position_only_navigation", False
                ),
                default_skip_pause=True,
            )
        direct_find_target = self.mission_type == "find_object" and bool(self.target_object)
        if not direct_find_target:
            self.interaction.wait_for_person()
            self.interaction.clear_commands("before_wake_prompt")
            self.speech.speak(self.config.section("interaction")["wake_prompt"])
            post_wake_clear_delay = float(
                self.config.section("interaction").get(
                    "post_wake_command_clear_delay",
                    0.5,
                )
            )
            if post_wake_clear_delay > 0.0:
                rospy.sleep(post_wake_clear_delay)
            self.interaction.clear_commands("after_wake_prompt")

        selected_mission = self.mission_type
        if selected_mission == "auto":
            selected_mission, selected_target = self.interaction.wait_for_mission()
            if selected_target:
                self.target_object = selected_target
        if selected_mission not in ("guide", "assistant", "find_object", "charge"):
            raise RuntimeError("Unknown home mission type: %s" % selected_mission)
        self.mission_type = selected_mission
        self.status.publish("mission_selected", selected_mission=selected_mission)

        if selected_mission == "guide":
            self.run_guide()
        elif selected_mission == "assistant":
            self.run_assistant()
        elif selected_mission == "find_object":
            self.run_find_object()
        else:
            self.run_charge()
            self.status.publish("completed")
            return

        self.reload_config("before_return")
        if self.config.section("navigation")["return_to_start"]:
            navigation_config = self.config.section("navigation")
            mission_config = self.config.optional_section(self.mission_type)
            return_route = mission_config.get(
                "return_route",
                navigation_config.get("return_route", ["start"]),
            )
            return_xy_tolerance = mission_config.get(
                "return_xy_tolerance",
                navigation_config.get("return_xy_tolerance"),
            )
            return_yaw_tolerance = mission_config.get(
                "return_yaw_tolerance",
                navigation_config.get("return_yaw_tolerance"),
            )
            return_ignore_yaw = bool(
                mission_config.get(
                    "return_ignore_yaw",
                    navigation_config.get("return_ignore_yaw", False),
                )
            )
            return_speed_multiplier = float(
                mission_config.get(
                    "return_speed_multiplier",
                    navigation_config.get("return_speed_multiplier", 1.0),
                )
            )
            return_speed_params = mission_config.get(
                "return_speed_params",
                navigation_config.get("return_speed_params", []),
            )
            old_speed_values = self.navigation.scale_dwa_parameters(
                return_speed_params,
                return_speed_multiplier,
                reason="%s_return" % self.mission_type,
            )
            self.status.publish(
                "return_route_selected",
                return_route=[self.route_item_name(item) for item in return_route],
            )
            try:
                passed_return_gate = False
                for waypoint in return_route:
                    waypoint_name = self.route_item_name(waypoint)
                    if self.should_skip_backward_return_waypoint(
                        waypoint_name,
                        passed_return_gate,
                    ):
                        rospy.logwarn(
                            "RETURN_ROUTE_GUARD mission=%s skipped=%s after=中转点2",
                            self.mission_type,
                            waypoint_name,
                        )
                        self.status.publish(
                            "return_route_guard_skipped",
                            waypoint=waypoint_name,
                            after="中转点2",
                        )
                        continue
                    self.go_to_route_item(
                        waypoint,
                        default_xy_tolerance=return_xy_tolerance,
                        default_yaw_tolerance=return_yaw_tolerance,
                        default_ignore_yaw=return_ignore_yaw,
                        default_skip_pause=(waypoint_name not in ("start", "start_return")),
                        default_precise_adjust=False,
                    )
                    if waypoint_name == "中转点2":
                        passed_return_gate = True
                self.run_return_final_adjust(mission_config)
                self.run_open_loop_motion("final_open_loop_return")
            finally:
                self.navigation.restore_dwa_parameters(
                    old_speed_values,
                    reason="%s_return" % self.mission_type,
                )
        self.status.publish("completed")

    def should_skip_backward_return_waypoint(self, waypoint_name, passed_return_gate):
        if self.mission_type not in ("guide", "assistant"):
            return False
        if not passed_return_gate:
            return False
        return waypoint_name not in ("中转点1", "start", "start_return")

    def run_return_final_adjust(self, mission_config):
        if self.mission_type != "guide":
            return
        waypoint = mission_config.get("return_final_adjust_waypoint")
        if not waypoint:
            return
        tolerance = float(mission_config.get("return_final_adjust_xy_tolerance", 0.02))
        timeout = float(mission_config.get("return_final_adjust_timeout", 6.0))
        accept_timeout = bool(mission_config.get("return_final_adjust_accept_timeout", True))
        pause = bool(mission_config.get("return_final_adjust_pause", True))
        self.status.publish(
            "return_final_adjust_started",
            waypoint=waypoint,
            target_xy_tolerance=tolerance,
            timeout=timeout,
        )
        adjusted = self.navigation.precise_arrival_adjust(
            waypoint,
            target_tolerance_override=tolerance,
            timeout_override=timeout,
            accept_timeout=accept_timeout,
        )
        self.status.publish(
            "return_final_adjust_finished",
            waypoint=waypoint,
            adjusted=bool(adjusted),
            target_xy_tolerance=tolerance,
        )
        if adjusted is False:
            return
        self.status.publish("arrived", waypoint=waypoint)
        if pause:
            self.navigation.pause_after_arrival(waypoint)

    def run_national_sequence(self):
        sequence_text = rospy.get_param(
            "~mission_sequence",
            "guide,assistant,find_object,charge",
        )
        sequence = [
            item.strip()
            for item in sequence_text.split(",")
            if item.strip()
        ]
        if not sequence:
            raise RuntimeError("National sequence is empty")
        allowed = ("guide", "assistant", "find_object", "charge")
        for mission in sequence:
            if mission not in allowed:
                raise RuntimeError("Unsupported national sequence mission: %s" % mission)
        self.navigation.connect()
        original_target = self.target_object
        between_pause = float(rospy.get_param("~sequence_between_task_pause", 0.0))
        self.status.publish("sequence_started", sequence=sequence)
        for mission in sequence:
            if rospy.is_shutdown():
                break
            self.mission_type = mission
            if mission == "find_object":
                self.target_object = rospy.get_param(
                    "~sequence_find_target",
                    original_target,
                )
            else:
                self.target_object = ""
            self.interaction.clear_commands()
            self.status.publish("sequence_task_started", selected_mission=mission)
            self.run_current_mission(validate=True, connect=False)
            self.status.publish("sequence_task_completed", selected_mission=mission)
            if between_pause > 0.0 and mission != sequence[-1]:
                rospy.sleep(between_pause)
        self.status.publish("sequence_completed")

    def go_to_route_item(
        self,
        route_item,
        default_xy_tolerance=None,
        default_yaw_tolerance=None,
        default_ignore_yaw=False,
        default_skip_pause=False,
        default_precise_adjust=None,
    ):
        if isinstance(route_item, dict):
            waypoint = route_item["waypoint"]
            xy_tolerance = route_item.get("xy_tolerance", default_xy_tolerance)
            yaw_tolerance = route_item.get("yaw_tolerance", default_yaw_tolerance)
            ignore_yaw = bool(route_item.get("ignore_yaw", default_ignore_yaw))
            skip_pause = bool(route_item.get("skip_pause", default_skip_pause))
            align_yaw = bool(route_item.get("align_yaw", False))
            align_yaw_tolerance = route_item.get("align_yaw_tolerance", yaw_tolerance)
            precise_adjust = route_item.get("precise_adjust", default_precise_adjust)
            precise_xy_tolerance = route_item.get("precise_xy_tolerance")
            precise_adjust_timeout = route_item.get("precise_adjust_timeout")
            precise_adjust_fallback_xy_tolerance = route_item.get("precise_adjust_fallback_xy_tolerance")
            accept_precise_adjust_timeout = bool(route_item.get("accept_precise_adjust_timeout", False))
            direct_adjust_waypoint = route_item.get("direct_adjust_waypoint")
            direct_adjust_xy_tolerance = route_item.get("direct_adjust_xy_tolerance")
            direct_adjust_timeout = route_item.get("direct_adjust_timeout")
            direct_adjust_accept_timeout = bool(route_item.get("direct_adjust_accept_timeout", False))
            direct_adjust_pause = bool(route_item.get("direct_adjust_pause", False))
        else:
            waypoint = route_item
            xy_tolerance = default_xy_tolerance
            yaw_tolerance = default_yaw_tolerance
            ignore_yaw = bool(default_ignore_yaw)
            skip_pause = bool(default_skip_pause)
            align_yaw = False
            align_yaw_tolerance = yaw_tolerance
            precise_adjust = default_precise_adjust
            precise_xy_tolerance = None
            precise_adjust_timeout = None
            precise_adjust_fallback_xy_tolerance = None
            accept_precise_adjust_timeout = False
            direct_adjust_waypoint = None
            direct_adjust_xy_tolerance = None
            direct_adjust_timeout = None
            direct_adjust_accept_timeout = False
            direct_adjust_pause = False
        try:
            self.navigation.go_to(
                waypoint,
                xy_tolerance=xy_tolerance,
                yaw_tolerance=yaw_tolerance,
                ignore_yaw=ignore_yaw,
                skip_pause=skip_pause,
                precise_adjust=precise_adjust,
                precise_xy_tolerance=precise_xy_tolerance,
                precise_adjust_timeout=precise_adjust_timeout,
                accept_precise_adjust_timeout=accept_precise_adjust_timeout,
            )
        except RuntimeError as exc:
            if (
                precise_adjust_fallback_xy_tolerance is None
                or "Precise adjust timeout" not in str(exc)
            ):
                raise
            self.status.publish(
                "precise_adjust_fallback_retry",
                waypoint=waypoint,
                precise_xy_tolerance=precise_xy_tolerance,
                fallback_xy_tolerance=precise_adjust_fallback_xy_tolerance,
                reason=str(exc),
            )
            self.navigation.go_to(
                waypoint,
                xy_tolerance=xy_tolerance,
                yaw_tolerance=yaw_tolerance,
                ignore_yaw=ignore_yaw,
                skip_pause=skip_pause,
                precise_adjust=precise_adjust,
                precise_xy_tolerance=precise_adjust_fallback_xy_tolerance,
                precise_adjust_timeout=precise_adjust_timeout,
                accept_precise_adjust_timeout=False,
            )
        if align_yaw:
            self.navigation.align_yaw(waypoint, tolerance=align_yaw_tolerance)
        if direct_adjust_waypoint:
            self.navigation.clear_costmaps_before_goal(direct_adjust_waypoint, 0)
            adjusted = self.navigation.precise_arrival_adjust(
                direct_adjust_waypoint,
                target_tolerance_override=direct_adjust_xy_tolerance,
                timeout_override=direct_adjust_timeout,
                accept_timeout=direct_adjust_accept_timeout,
            )
            if adjusted is False:
                return
            self.status.publish("arrived", waypoint=direct_adjust_waypoint)
            if direct_adjust_pause:
                self.navigation.pause_after_arrival(direct_adjust_waypoint)

    @staticmethod
    def route_item_name(route_item):
        if isinstance(route_item, dict):
            return route_item["waypoint"]
        return route_item

    def run_open_loop_motion(self, config_key):
        navigation = self.config.section("navigation")
        motion = navigation.get(config_key, {})
        if not motion.get("enabled", False):
            return
        twist = Twist()
        twist.linear.x = float(motion.get("linear_x", 0.0))
        twist.linear.y = float(motion.get("linear_y", 0.0))
        twist.angular.z = float(motion.get("angular_z", 0.0))
        duration = float(motion.get("duration", 0.0))
        rate = rospy.Rate(float(motion.get("rate", 10.0)))
        self.status.publish("open_loop_motion", motion=config_key, duration=duration)
        end_time = rospy.Time.now() + rospy.Duration(duration)
        while not rospy.is_shutdown() and rospy.Time.now() < end_time:
            self.check_stopped()
            self.cmd_vel_pub.publish(twist)
            rate.sleep()
        self.cmd_vel_pub.publish(Twist())

    def run_guide(self):
        guide = self.config.section("guide")
        self.interaction.wait_for_command_with_replies(
            guide["trigger_phrases"],
            guide.get("interference_replies", []),
            self.speech.speak,
            expected_mission="guide",
        )
        self.speech.speak(self.config.section("interaction")["accept"])
        for room in guide["route"]:
            for waypoint in guide.get("approach_routes", {}).get(room, []):
                self.navigation.go_to(
                    waypoint,
                    position_only=guide.get("position_only_navigation", False),
                    precise_adjust=False,
                )
            self.navigation.go_to(
                room,
                position_only=guide.get("position_only_navigation", False),
                skip_pause=True,
                precise_adjust=False,
            )
            self.navigation.precise_arrival_adjust(
                room,
                target_tolerance_override=guide.get("precise_xy_tolerance"),
                timeout_override=guide.get("precise_adjust_timeout"),
                accept_timeout=bool(guide.get("accept_precise_adjust_timeout", True)),
            )
            self.navigation.wait_until_stopped(reason="guide_intro_%s" % room)
            self.speech.speak_async(guide["introductions"][room])
            self.navigation.pause_after_arrival(room)

    def run_assistant(self):
        assistant = self.config.section("assistant")
        self.interaction.wait_for_command(
            assistant["trigger_phrases"],
            expected_mission="assistant",
            reply_callback=self.speech.speak,
        )
        self.speech.speak(assistant["start_speech"])
        if assistant.get("tts_precache_on_start", False):
            self.speech.prepare_many_async(
                self.assistant_tts_precache_texts(assistant),
                initial_delay=float(assistant.get("tts_precache_initial_delay", 0.5)),
            )
        for waypoint in assistant.get("approach_route", []):
            self.go_to_route_item(waypoint)
        self.navigation.go_to(assistant.get("target_waypoint", "kitchen"))
        vision_start_speech = assistant.get("vision_start_speech", "")
        if vision_start_speech:
            self.speech.speak(vision_start_speech)
        vision_start_delay = float(assistant.get("vision_start_delay", 0.0))
        if vision_start_delay > 0.0:
            self.status.publish("assistant_vision_delay", seconds=vision_start_delay)
            end_time = rospy.Time.now() + rospy.Duration(vision_start_delay)
            rate = rospy.Rate(10.0)
            while not rospy.is_shutdown() and rospy.Time.now() < end_time:
                self.check_stopped()
                rate.sleep()
        fallback = assistant.get("simulation_fallback", {})
        self.status.publish(
            "assistant_vision_started",
            image_topic=assistant["image_topic"],
            confidence=float(assistant["confidence"]),
            labels=list(assistant["food_labels"]),
        )
        try:
            foods = self.vision.detect_labels(
                assistant["food_labels"],
                assistant["image_topic"],
                float(assistant["confidence"]),
                int(assistant["sample_count"]),
                float(assistant["sample_interval"]),
                fallback.get("labels", []),
                bool(fallback.get("enabled", False)),
                int(assistant.get("maximum_report_count", 3)),
                int(assistant.get("minimum_consecutive_count", 1)),
            )
        except RuntimeError as exc:
            rospy.logwarn("Assistant vision unavailable, using food fill policy: %s", exc)
            self.status.publish("assistant_vision_unavailable", reason=str(exc))
            foods = []
        finally:
            self.status.publish("assistant_vision_stopped")
        recipe_book = RecipeBook(assistant)
        foods = recipe_book.ensure_minimum_foods(foods)
        if foods:
            food_names = self.food_names_for_speech(foods)
            self.status.publish(
                "assistant_foods_selected",
                labels=list(foods),
                names=food_names,
            )
            food_speech = assistant["foods_prefix"] + "、".join(food_names) + "。"
        else:
            self.speech.speak(assistant["no_food_speech"])
            self.status.publish("vision_no_food")
            return
        recipe_name, recipe_method = recipe_book.choose_recipe(foods)
        recipe_speech = assistant["recipe_template"] % (recipe_name, recipe_method)
        self.status.publish(
            "assistant_tts_before_return",
            speech=food_speech + recipe_speech,
        )
        self.speech.speak(food_speech + recipe_speech)

    def food_names_for_speech(self, foods):
        chinese_names = self.config.section("chinese_names")
        return [chinese_names.get(label, label) for label in foods]

    def assistant_tts_precache_texts(self, assistant):
        texts = []
        seen = set()

        def add(text):
            text = text or ""
            if text and text not in seen:
                texts.append(text)
                seen.add(text)

        add(assistant.get("start_speech", ""))
        add(assistant.get("vision_start_speech", ""))
        add(assistant.get("no_food_speech", ""))
        return texts

    def run_find_object(self):
        find_config = self.config.section("find_object")
        target = self.target_object
        if not target:
            target = self.interaction.wait_for_find_target(self.speech.speak)
        if target not in find_config["labels"]:
            raise RuntimeError("Unsupported target object: %s" % target)
        target_cn = self.config.section("chinese_names")[target]
        self.speech.speak(find_config["start_speech_template"] % target_cn)

        found = False
        for route_item in find_config["route"]:
            room = self.route_item_name(route_item)
            if isinstance(route_item, dict):
                for approach_item in route_item.get("approach_route", []):
                    self.go_to_route_item(approach_item)
            self.go_to_route_item(route_item)
            if isinstance(route_item, dict) and route_item.get("detect") is False:
                self.status.publish("find_transit_waypoint", waypoint=room)
                continue
            self.status.publish("find_detection_ready", waypoint=room, target=target)
            fallback = find_config.get("simulation_fallback", {}).get("room_results", {})
            try:
                self.status.publish("find_detection_started", waypoint=room, target=target)
                labels = self.vision.detect_labels(
                    find_config["labels"],
                    find_config["image_topic"],
                    float(find_config["confidence"]),
                    int(find_config["sample_count"]),
                    float(find_config["sample_interval"]),
                    fallback.get(room, []),
                    bool(find_config.get("simulation_fallback", {}).get("enabled", False)),
                )
            except RuntimeError as exc:
                rospy.logwarn("Find-object vision unavailable at %s: %s", room, exc)
                self.status.publish("vision_unavailable", room=room, reason=str(exc))
                labels = []
            speech_text = ""
            if target in labels:
                speech_text = find_config["found_speech_template"] % target_cn
                if not found:
                    self.status.publish("target_found", target=target, room=room)
                    found = True
            else:
                other_labels = [label for label in labels if label != target]
                if other_labels:
                    speech_text = (
                        find_config["other_prefix"]
                        + "、".join(self.config.section("chinese_names")[label] for label in other_labels)
                    )
                else:
                    speech_text = find_config["empty_speech"]
            self.status.publish(
                "find_detection_speech_async",
                waypoint=room,
                labels=labels,
                speech=speech_text,
            )
            self.speech.speak_async(speech_text)
        if not found:
            self.speech.speak(find_config["not_found_speech_template"] % target_cn)
            self.status.publish("target_not_found", target=target)

    def run_charge(self):
        finish = self.config.section("finish")
        pre_charge_target = finish.get("pre_charge_target", "start")
        charge_target = finish.get("charge_target", "charge")
        start_speech = finish.get("start_speech", "准备返回充电桩。")
        if bool(finish.get("start_speech_async", True)):
            self.speech.speak_async(start_speech)
        else:
            self.speech.speak(start_speech)
        self.navigation.go_to(pre_charge_target)
        docking = self.config.optional_section("docking")
        if docking.get("enabled", False):
            self.docking.run(docking)
        else:
            self.navigation.go_to(charge_target)
        self.speech.speak(finish.get("complete_speech", "开始充电，任务完成。"))
