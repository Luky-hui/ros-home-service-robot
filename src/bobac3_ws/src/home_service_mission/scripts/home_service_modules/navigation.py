#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import json
import math
import os
import time

import actionlib
import rospy
import tf
from actionlib_msgs.msg import GoalStatus, GoalStatusArray
from dynamic_reconfigure.msg import DoubleParameter
from dynamic_reconfigure.srv import Reconfigure, ReconfigureRequest
from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import Quaternion, Twist
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from std_srvs.srv import Empty


class NavigationClient:
    def __init__(self, config, cmd_vel_pub, status, check_stopped):
        self.config = config
        self.navigation = config.section("navigation")
        self.cmd_vel_pub = cmd_vel_pub
        self.status = status
        self.check_stopped = check_stopped
        self.action_server_name = self.navigation["action_server"]
        self.fallback_action_server_name = self.navigation["fallback_action_server"]
        self.move_base = actionlib.SimpleActionClient(self.action_server_name, MoveBaseAction)
        self.tf_listener = tf.TransformListener()
        self.slow_zone_active_id = ""
        self.slow_zone_saved_speed_values = {}
        self.obstacle_scan_sub = None
        self.obstacle_model_sub = None
        self.obstacle_front_distance = None
        self.obstacle_front_bearing = None
        self.obstacle_front_stamp = rospy.Time(0)
        self.obstacle_model_xy = None
        self.obstacle_model_last_xy = None
        self.obstacle_model_last_time = None
        self.obstacle_model_speed = None
        self.last_dynamic_obstacle_yield_log = 0.0
        self.virtual_wall_segments_cache = None
        self.virtual_wall_segments_cache_key = None

    def connect(self):
        wait_seconds = float(self.navigation["server_wait"])
        if self.move_base.wait_for_server(rospy.Duration(wait_seconds)):
            return
        if self.action_status_available(self.action_server_name):
            rospy.logwarn(
                "Navigation action wait timed out for %s, but status topic is alive; continuing.",
                self.action_server_name,
            )
            return
        self.move_base = actionlib.SimpleActionClient(
            self.fallback_action_server_name, MoveBaseAction
        )
        if not self.move_base.wait_for_server(rospy.Duration(wait_seconds)):
            if self.action_status_available(self.fallback_action_server_name):
                rospy.logwarn(
                    "Navigation fallback action wait timed out for %s, but status topic is alive; continuing.",
                    self.fallback_action_server_name,
                )
                return
            raise RuntimeError(
                "Navigation action servers unavailable: %s, %s"
                % (self.action_server_name, self.fallback_action_server_name)
            )

    def action_status_available(self, action_server_name):
        status_topic = action_server_name.rstrip("/") + "/status"
        try:
            rospy.wait_for_message(status_topic, GoalStatusArray, timeout=5.0)
            return True
        except rospy.ROSException:
            return False

    def cancel_all_goals(self):
        try:
            self.move_base.cancel_all_goals()
        except Exception:
            pass
        self.cmd_vel_pub.publish(Twist())

    def go_to(
        self,
        name,
        position_only=False,
        xy_tolerance=None,
        yaw_tolerance=None,
        ignore_yaw=False,
        skip_pause=False,
        precise_adjust=None,
        precise_xy_tolerance=None,
        precise_adjust_timeout=None,
        accept_precise_adjust_timeout=False,
    ):
        self.check_stopped()
        point = self.config.section("waypoints")[name]
        xy_tolerance, yaw_tolerance = self.resolve_goal_tolerances(
            point,
            position_only=position_only,
            xy_tolerance=xy_tolerance,
            yaw_tolerance=yaw_tolerance,
            ignore_yaw=ignore_yaw,
        )
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = point["frame_id"]
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = float(point["x"])
        goal.target_pose.pose.position.y = float(point["y"])
        goal.target_pose.pose.orientation = self.yaw_to_quaternion(float(point["yaw"]))
        self.status.publish(
            "navigating",
            waypoint=name,
            goal_x=float(point["x"]),
            goal_y=float(point["y"]),
            goal_yaw=float(point["yaw"]),
            xy_tolerance=xy_tolerance,
            yaw_tolerance=yaw_tolerance,
        )
        rospy.loginfo(
            "HOME_NAV_GOAL waypoint=%s goal=(%.4f,%.4f,%.4f) xy_tol=%.4f yaw_tol=%.4f",
            name,
            float(point["x"]),
            float(point["y"]),
            float(point["yaw"]),
            xy_tolerance,
            yaw_tolerance,
        )
        original_yaw_tolerance = None
        if position_only:
            original_yaw_tolerance = self.set_position_only_yaw_tolerance()
        elif ignore_yaw or bool(point.get("ignore_yaw", False)):
            self.set_configured_goal_tolerances(
                xy_tolerance=xy_tolerance,
                yaw_tolerance=yaw_tolerance,
            )
        else:
            self.set_configured_goal_tolerances(
                xy_tolerance=xy_tolerance,
                yaw_tolerance=yaw_tolerance,
            )
        use_precise_adjust = self.should_precise_adjust(name, precise_adjust)
        try:
            reached = self.send_goal_with_retry(
                name,
                point,
                goal,
                xy_tolerance,
                yaw_tolerance,
                allow_near_goal_for_precise=use_precise_adjust,
            )
        finally:
            if original_yaw_tolerance is not None:
                self.set_dwa_yaw_goal_tolerance(original_yaw_tolerance)
        if reached is False:
            return
        if use_precise_adjust:
            adjusted = self.precise_arrival_adjust(
                name,
                precise_xy_tolerance,
                timeout_override=precise_adjust_timeout,
                accept_timeout=accept_precise_adjust_timeout,
            )
            if adjusted is False:
                return
        self.status.publish("arrived", waypoint=name)
        if not skip_pause:
            self.pause_after_arrival(name)


    def should_precise_adjust(self, name, precise_adjust):
        options = self.navigation.get("precise_arrival_adjust", {})
        if precise_adjust is None:
            precise_adjust = name in options.get("waypoints", [])
        return bool(options.get("enabled", False)) and bool(precise_adjust)

    def precise_arrival_adjust(
        self,
        name,
        target_tolerance_override=None,
        timeout_override=None,
        accept_timeout=False,
    ):
        options = self.navigation.get("precise_arrival_adjust", {})
        if target_tolerance_override is None:
            target_tolerance = float(options.get("target_xy_tolerance", 0.01))
        else:
            target_tolerance = float(target_tolerance_override)
        if timeout_override is None:
            timeout = float(options.get("timeout", 20.0))
        else:
            timeout = float(timeout_override)
        max_speed = float(options.get("max_speed", 0.03))
        min_speed = float(options.get("min_speed", 0.006))
        gain = float(options.get("gain", 0.45))
        stable_cycles_required = int(options.get("stable_cycles", 5))
        map_frame = self.navigation.get("map_frame", "map")
        base_frame = self.navigation.get("base_frame", "base_footprint")
        point = self.config.section("waypoints")[name]
        self.status.publish(
            "precise_adjusting",
            waypoint=name,
            target_xy_tolerance=target_tolerance,
            timeout=timeout,
        )
        start = time.time()
        stable_cycles = 0
        last_distance = None
        rate = rospy.Rate(float(options.get("rate", 20.0)))
        try:
            while not rospy.is_shutdown() and time.time() - start < timeout:
                self.check_stopped()
                try:
                    self.tf_listener.waitForTransform(
                        map_frame, base_frame, rospy.Time(0), rospy.Duration(1.0)
                    )
                    translation, rotation = self.tf_listener.lookupTransform(
                        map_frame, base_frame, rospy.Time(0)
                    )
                except Exception as exc:
                    rospy.logwarn("Precise adjust TF failed at %s: %s", name, exc)
                    rate.sleep()
                    continue
                yaw = tf.transformations.euler_from_quaternion(rotation)[2]
                dx_map = float(point["x"]) - float(translation[0])
                dy_map = float(point["y"]) - float(translation[1])
                distance = math.hypot(dx_map, dy_map)
                last_distance = distance
                if distance <= target_tolerance:
                    stable_cycles += 1
                    self.cmd_vel_pub.publish(Twist())
                    if stable_cycles >= stable_cycles_required:
                        self.status.publish(
                            "precise_adjusted",
                            waypoint=name,
                            distance=distance,
                            target_xy_tolerance=target_tolerance,
                            seconds=time.time() - start,
                        )
                        return True
                else:
                    stable_cycles = 0
                    dx_base = math.cos(yaw) * dx_map + math.sin(yaw) * dy_map
                    dy_base = -math.sin(yaw) * dx_map + math.cos(yaw) * dy_map
                    speed = min(max_speed, max(min_speed, distance * gain))
                    scale = speed / max(distance, 1e-6)
                    twist = Twist()
                    twist.linear.x = max(-max_speed, min(max_speed, dx_base * scale))
                    twist.linear.y = max(-max_speed, min(max_speed, dy_base * scale))
                    self.cmd_vel_pub.publish(twist)
                rate.sleep()
            if accept_timeout:
                self.status.publish(
                    "precise_adjust_timeout_accepted",
                    waypoint=name,
                    last_distance=last_distance,
                    target_xy_tolerance=target_tolerance,
                    seconds=time.time() - start,
                )
                return True
            if self.soft_fail_enabled():
                rospy.logerr(
                    "Soft-fail navigation: precise adjust timeout at %s, last_distance=%s",
                    name,
                    last_distance,
                )
                self.status.publish(
                    "navigation_soft_failed",
                    waypoint=name,
                    reason="precise_adjust_timeout",
                    distance=last_distance,
                    tolerance=target_tolerance,
                )
                return False
            raise RuntimeError(
                "Precise adjust timeout at %s, last_distance=%s"
                % (name, last_distance)
            )
        finally:
            self.cmd_vel_pub.publish(Twist())

    def align_yaw(self, name, tolerance=None, timeout=None, angular_speed=None):
        self.check_stopped()
        point = self.config.section("waypoints")[name]
        target_yaw = float(point["yaw"])
        map_frame = self.navigation.get("map_frame", point.get("frame_id", "map"))
        base_frame = self.navigation.get("base_frame", "base_footprint")
        tolerance = float(
            tolerance
            if tolerance is not None
            else self.navigation.get("align_yaw_tolerance", math.radians(10.0))
        )
        timeout = float(
            timeout
            if timeout is not None
            else self.navigation.get("align_yaw_timeout", 20.0)
        )
        angular_speed = float(
            angular_speed
            if angular_speed is not None
            else self.navigation.get("align_yaw_speed", 0.25)
        )
        self.status.publish(
            "yaw_aligning",
            waypoint=name,
            yaw_tolerance=tolerance,
            target_yaw=target_yaw,
        )
        rate = rospy.Rate(20)
        deadline = time.time() + timeout
        last_error = None
        try:
            while not rospy.is_shutdown() and time.time() < deadline:
                self.check_stopped()
                try:
                    self.tf_listener.waitForTransform(
                        map_frame, base_frame, rospy.Time(0), rospy.Duration(1.0)
                    )
                    _, rotation = self.tf_listener.lookupTransform(
                        map_frame, base_frame, rospy.Time(0)
                    )
                except Exception as exc:
                    rospy.logwarn("Yaw alignment TF failed at %s: %s", name, exc)
                    rate.sleep()
                    continue
                yaw = tf.transformations.euler_from_quaternion(rotation)[2]
                error = math.atan2(
                    math.sin(target_yaw - yaw),
                    math.cos(target_yaw - yaw),
                )
                last_error = error
                if abs(error) <= tolerance:
                    self.status.publish(
                        "yaw_aligned",
                        waypoint=name,
                        yaw_error=abs(error),
                        yaw_tolerance=tolerance,
                    )
                    return True
                twist = Twist()
                speed = min(angular_speed, max(0.08, abs(error) * 0.7))
                twist.angular.z = math.copysign(speed, error)
                self.cmd_vel_pub.publish(twist)
                rate.sleep()
            if self.soft_fail_enabled():
                rospy.logerr(
                    "Soft-fail navigation: yaw alignment timeout at %s, last_error=%s",
                    name,
                    last_error,
                )
                self.status.publish(
                    "navigation_soft_failed",
                    waypoint=name,
                    reason="yaw_alignment_timeout",
                    yaw_error=last_error,
                    yaw_tolerance=tolerance,
                )
                return False
            raise RuntimeError(
                "Yaw alignment timeout at %s, last_error=%s"
                % (name, last_error)
            )
        finally:
            self.cmd_vel_pub.publish(Twist())

    def send_goal_with_retry(
        self,
        name,
        point,
        goal,
        xy_tolerance,
        yaw_tolerance,
        allow_near_goal_for_precise=False,
    ):
        retry_count = int(self.navigation.get("retry_on_abort", 0))
        retry_pause = float(self.navigation.get("retry_pause", 1.0))
        obstacle_options = self.navigation.get("dynamic_obstacle_stop", {})
        obstacle_replans = int(obstacle_options.get("replan_budget", 8))
        attempts = retry_count + 1 + max(0, obstacle_replans)
        last_state = None
        for attempt in range(attempts):
            self.check_stopped()
            self.clear_costmaps_before_goal(name, attempt)
            self.update_slow_zone_speed()
            if attempt == 0:
                self.log_dynamic_obstacle_config(name)
            self.prepare_dynamic_obstacle_before_goal(name)
            goal_start = rospy.Time.now()
            self.move_base.send_goal(goal)
            timeout = float(self.navigation["goal_timeout"])
            finished, status_state = self.wait_for_result_or_status(timeout, goal_start)
            if status_state == "obstacle_cleared":
                self.status.publish(
                    "navigation_retry",
                    waypoint=name,
                    reason="dynamic_obstacle_cleared",
                    attempt=attempt + 1,
                )
                goal.target_pose.header.stamp = rospy.Time.now()
                continue
            if not finished:
                self.move_base.cancel_goal()
                if attempt < retry_count:
                    self.status.publish("navigation_retry", waypoint=name, reason="timeout", attempt=attempt + 1)
                    rospy.sleep(retry_pause)
                    continue
                if self.should_soft_fail(name, point, xy_tolerance):
                    self.publish_navigation_soft_failed(
                        name,
                        point,
                        "timeout",
                        last_state="timeout",
                        tolerance=xy_tolerance,
                    )
                    return False
                raise RuntimeError("Navigation timeout at waypoint: %s" % name)
            last_state = status_state
            if last_state == GoalStatus.SUCCEEDED and allow_near_goal_for_precise:
                self.status.publish(
                    "move_base_succeeded_for_precise_adjust",
                    waypoint=name,
                    xy_tolerance=xy_tolerance,
                )
                return True
            if last_state == GoalStatus.SUCCEEDED and self.verify_goal_pose(
                name, point, xy_tolerance, yaw_tolerance
            ):
                return True
            if self.accept_near_goal(
                name,
                point,
                last_state,
                xy_tolerance,
                allow_near_goal_for_precise=allow_near_goal_for_precise,
            ):
                return True
            if attempt < retry_count:
                self.status.publish(
                    "navigation_retry",
                    waypoint=name,
                    reason="state_%s" % last_state,
                    attempt=attempt + 1,
                )
                rospy.sleep(retry_pause)
        if self.try_navigation_failure_recovery(
            name,
            point,
            goal,
            last_state,
            xy_tolerance,
            yaw_tolerance,
            allow_near_goal_for_precise=allow_near_goal_for_precise,
        ):
            return True
        if self.should_soft_fail(name, point, xy_tolerance):
            self.publish_navigation_soft_failed(
                name,
                point,
                "state_%s" % last_state,
                last_state=last_state,
                tolerance=xy_tolerance,
            )
            return False
        raise RuntimeError("Navigation failed at %s, state=%s" % (name, last_state))

    def try_navigation_failure_recovery(
        self,
        name,
        point,
        goal,
        failed_state,
        xy_tolerance,
        yaw_tolerance,
        allow_near_goal_for_precise=False,
    ):
        options = self.navigation.get("navigation_failure_recovery", {})
        if not bool(options.get("enabled", False)):
            return False
        retry_count = int(options.get("retry_count", 1))
        if retry_count <= 0:
            return False
        retry_timeout = float(options.get("retry_nav_timeout", min(float(self.navigation["goal_timeout"]), 45.0)))
        retry_pause = float(options.get("retry_pause", 0.2))
        motions = options.get("motions", [])
        if not motions:
            motions = [{"linear_x": 0.05, "linear_y": 0.0, "angular_z": 0.0, "duration": 1.5}]

        last_state = failed_state
        for attempt in range(1, retry_count + 1):
            self.check_stopped()
            self.status.publish(
                "navigation_failure_recovery",
                waypoint=name,
                reason="state_%s" % last_state,
                attempt=attempt,
                retry_count=retry_count,
            )
            self.cancel_all_goals()
            self.clear_costmaps_before_goal(name, attempt)
            escaped_virtual_wall = self.drive_virtual_wall_escape(name)
            if not escaped_virtual_wall:
                for motion in motions:
                    self.drive_recovery_motion(motion)
            self.clear_costmaps_before_goal(name, attempt)

            goal.target_pose.header.stamp = rospy.Time.now()
            self.move_base.send_goal(goal)
            finished, status_state = self.wait_for_result_or_status(
                retry_timeout,
                goal.target_pose.header.stamp,
            )
            if not finished:
                self.move_base.cancel_goal()
                last_state = "timeout"
            else:
                last_state = status_state
                if last_state == GoalStatus.SUCCEEDED and allow_near_goal_for_precise:
                    self.status.publish(
                        "navigation_recovery_succeeded_for_precise_adjust",
                        waypoint=name,
                        xy_tolerance=xy_tolerance,
                    )
                    return True
                if last_state == GoalStatus.SUCCEEDED and self.verify_goal_pose(
                    name,
                    point,
                    xy_tolerance,
                    yaw_tolerance,
                ):
                    self.status.publish("navigation_recovery_succeeded", waypoint=name)
                    return True
                if self.accept_near_goal(
                    name,
                    point,
                    last_state,
                    xy_tolerance,
                    allow_near_goal_for_precise=allow_near_goal_for_precise,
                ):
                    self.status.publish("navigation_recovery_succeeded", waypoint=name)
                    return True
            if retry_pause > 0.0 and attempt < retry_count:
                rospy.sleep(retry_pause)
        return False

    def drive_recovery_motion(self, motion):
        linear_x = float(motion.get("linear_x", 0.0))
        linear_y = float(motion.get("linear_y", 0.0))
        angular_z = float(motion.get("angular_z", 0.0))
        duration = float(motion.get("duration", 0.0))
        if duration <= 0.0:
            return
        self.status.publish(
            "navigation_recovery_motion",
            linear_x=linear_x,
            linear_y=linear_y,
            angular_z=angular_z,
            duration=duration,
        )
        twist = Twist()
        twist.linear.x = linear_x
        twist.linear.y = linear_y
        twist.angular.z = angular_z
        rate = rospy.Rate(float(motion.get("rate", 20.0)))
        deadline = time.time() + duration
        try:
            while not rospy.is_shutdown() and time.time() < deadline:
                self.check_stopped()
                self.cmd_vel_pub.publish(twist)
                rate.sleep()
        finally:
            self.cmd_vel_pub.publish(Twist())

    def drive_virtual_wall_escape(self, waypoint):
        options = self.navigation.get("navigation_failure_recovery", {}).get(
            "virtual_wall_escape", {}
        )
        if not bool(options.get("enabled", False)):
            return False
        pose = self.current_robot_pose()
        if pose is None:
            self.status.publish(
                "virtual_wall_escape_skipped",
                waypoint=waypoint,
                reason="pose_unavailable",
            )
            return False
        segments = self.virtual_wall_segments()
        if not segments:
            self.status.publish(
                "virtual_wall_escape_skipped",
                waypoint=waypoint,
                reason="segments_unavailable",
            )
            return False
        robot_x, robot_y, robot_yaw = pose
        nearest = self.nearest_segment_escape_vector(robot_x, robot_y, segments)
        if nearest is None:
            self.status.publish(
                "virtual_wall_escape_skipped",
                waypoint=waypoint,
                reason="nearest_segment_unavailable",
            )
            return False
        distance, escape_x_map, escape_y_map = nearest
        engage_distance = float(options.get("engage_distance", 0.28))
        if distance > engage_distance:
            self.status.publish(
                "virtual_wall_escape_skipped",
                waypoint=waypoint,
                reason="not_near_virtual_wall",
                distance=distance,
                engage_distance=engage_distance,
            )
            return False
        speed = float(options.get("speed", 0.08))
        duration = float(options.get("duration", 1.0))
        if speed <= 0.0 or duration <= 0.0:
            return False
        escape_norm = math.hypot(escape_x_map, escape_y_map)
        if escape_norm <= 1e-6:
            return False
        escape_x_map /= escape_norm
        escape_y_map /= escape_norm
        twist = Twist()
        twist.linear.x = (
            math.cos(robot_yaw) * escape_x_map + math.sin(robot_yaw) * escape_y_map
        ) * speed
        twist.linear.y = (
            -math.sin(robot_yaw) * escape_x_map + math.cos(robot_yaw) * escape_y_map
        ) * speed
        max_axis_speed = float(options.get("max_axis_speed", speed))
        twist.linear.x = max(-max_axis_speed, min(max_axis_speed, twist.linear.x))
        twist.linear.y = max(-max_axis_speed, min(max_axis_speed, twist.linear.y))
        rate = rospy.Rate(float(options.get("rate", 20.0)))
        deadline = time.time() + duration
        self.status.publish(
            "virtual_wall_escape",
            waypoint=waypoint,
            distance=distance,
            engage_distance=engage_distance,
            linear_x=twist.linear.x,
            linear_y=twist.linear.y,
            duration=duration,
        )
        rospy.logwarn(
            "VIRTUAL_WALL_ESCAPE waypoint=%s distance=%.3f engage=%.3f vx=%.3f vy=%.3f duration=%.2f",
            waypoint,
            distance,
            engage_distance,
            twist.linear.x,
            twist.linear.y,
            duration,
        )
        try:
            while not rospy.is_shutdown() and time.time() < deadline:
                self.check_stopped()
                self.cmd_vel_pub.publish(twist)
                rate.sleep()
        finally:
            self.cmd_vel_pub.publish(Twist())
        return True

    def virtual_wall_segments(self):
        forbidden_zones = self.config.optional_section("forbidden_zones")
        json_file = forbidden_zones.get("virtual_wall_file", "")
        cache_key = "|".join(
            [
                str(json_file),
                str(os.path.getmtime(json_file)) if json_file and os.path.exists(json_file) else "",
                str(len(forbidden_zones.get("zones", []))),
            ]
        )
        if self.virtual_wall_segments_cache_key == cache_key:
            return self.virtual_wall_segments_cache or []
        segments = []
        for zone in forbidden_zones.get("zones", []):
            points = zone.get("points", [])
            vertices = self.points_to_xy(points)
            if len(vertices) >= 2:
                for index in range(len(vertices)):
                    segments.append((vertices[index], vertices[(index + 1) % len(vertices)]))
        if json_file and os.path.exists(json_file):
            try:
                with open(json_file, "r", encoding="utf-8") as stream:
                    data = json.load(stream)
                for wall in data.get("vws", []):
                    vertices = self.points_to_xy(wall.get("points", []))
                    if len(vertices) >= 2:
                        for index in range(len(vertices) - 1):
                            segments.append((vertices[index], vertices[index + 1]))
            except Exception as exc:
                rospy.logwarn("Read virtual wall file failed: %s", exc)
        self.virtual_wall_segments_cache = segments
        self.virtual_wall_segments_cache_key = cache_key
        return segments

    @staticmethod
    def points_to_xy(points):
        vertices = []
        for point in points:
            try:
                vertices.append((float(point["x"]), float(point["y"])))
            except (KeyError, TypeError, ValueError):
                return []
        return vertices

    @staticmethod
    def nearest_segment_escape_vector(x, y, segments):
        best = None
        for start, end in segments:
            x1, y1 = start
            x2, y2 = end
            sx = x2 - x1
            sy = y2 - y1
            length_sq = sx * sx + sy * sy
            if length_sq <= 1e-9:
                continue
            t = ((x - x1) * sx + (y - y1) * sy) / length_sq
            t = max(0.0, min(1.0, t))
            closest_x = x1 + t * sx
            closest_y = y1 + t * sy
            escape_x = x - closest_x
            escape_y = y - closest_y
            distance = math.hypot(escape_x, escape_y)
            if distance <= 1e-6:
                normal_x = -sy
                normal_y = sx
                normal_norm = math.hypot(normal_x, normal_y)
                if normal_norm <= 1e-6:
                    continue
                escape_x = normal_x / normal_norm
                escape_y = normal_y / normal_norm
            if best is None or distance < best[0]:
                best = (distance, escape_x, escape_y)
        return best

    def wait_for_result_or_status(self, timeout, goal_start):
        poll_interval = float(self.navigation.get("action_status_poll_interval", 0.2))
        deadline = time.time() + timeout
        last_state = None
        try:
            while not rospy.is_shutdown() and time.time() < deadline:
                self.update_slow_zone_speed()
                if self.dynamic_obstacle_in_front():
                    self.move_base.cancel_goal()
                    self.cmd_vel_pub.publish(Twist())
                    if self.wait_for_dynamic_obstacle_clear():
                        return False, "obstacle_cleared"
                    return False, "obstacle_wait_timeout"
                if self.move_base.wait_for_result(rospy.Duration(poll_interval)):
                    return True, self.move_base.get_state()
                last_state = self.move_base.get_state()
                if last_state in (
                    GoalStatus.SUCCEEDED,
                    GoalStatus.ABORTED,
                    GoalStatus.REJECTED,
                    GoalStatus.PREEMPTED,
                    GoalStatus.RECALLED,
                    GoalStatus.LOST,
                ):
                    return True, last_state
                status_state = self.latest_action_status_since(goal_start)
                if status_state in (
                    GoalStatus.SUCCEEDED,
                    GoalStatus.ABORTED,
                    GoalStatus.REJECTED,
                    GoalStatus.PREEMPTED,
                    GoalStatus.RECALLED,
                    GoalStatus.LOST,
                ):
                    rospy.logwarn(
                        "Using %s/status fallback for move_base result: state=%s",
                        self.action_server_name,
                        status_state,
                    )
                    return True, status_state
            return False, last_state
        finally:
            self.apply_slow_zone(None)

    def ensure_obstacle_scan_subscriber(self):
        if self.obstacle_scan_sub is not None:
            return
        options = self.navigation.get("dynamic_obstacle_stop", {})
        topic = options.get("scan_topic", "/scan")
        self.obstacle_scan_sub = rospy.Subscriber(
            topic,
            LaserScan,
            self.obstacle_scan_callback,
            queue_size=1,
        )
        rospy.loginfo("DYNAMIC_OBSTACLE_MONITOR scan_topic=%s", topic)

    def log_dynamic_obstacle_config(self, waypoint):
        options = self.navigation.get("dynamic_obstacle_stop", {})
        rospy.loginfo(
            "DYNAMIC_OBSTACLE_CONFIG waypoint=%s enabled=%s source=%s mode=%s target=%s robot=%s engage=%.3f stop=%.3f resume=%.3f yield_while_close=%s continuous=%s",
            waypoint,
            bool(options.get("enabled", False)),
            str(options.get("source", "gazebo_model")),
            str(options.get("check_mode", "directional")),
            str(options.get("target_model", "cylinder_obstacle")),
            str(options.get("robot_model", "bobac3_serverbot")),
            float(options.get("engage_distance", options.get("stop_distance", 0.42))),
            float(options.get("stop_distance", 0.42)),
            float(options.get("resume_distance", 0.60)),
            bool(options.get("yield_while_close", True)),
            bool(options.get("continuous_yield", bool(options.get("yield_while_close", True)))),
        )

    def prepare_dynamic_obstacle_before_goal(self, waypoint):
        options = self.navigation.get("dynamic_obstacle_stop", {})
        if not bool(options.get("enabled", False)):
            return
        source = str(options.get("source", "gazebo_model")).strip().lower()
        if source == "laser_scan":
            self.ensure_obstacle_scan_subscriber()
        else:
            self.ensure_obstacle_model_subscriber()
        sample_wait = float(options.get("startup_sample_wait", 0.08))
        if sample_wait > 0.0:
            rospy.sleep(sample_wait)
        rospy.loginfo(
            "DYNAMIC_OBSTACLE_SAMPLE waypoint=%s distance=%s bearing=%s keep=%.3f resume=%.3f",
            waypoint,
            self.obstacle_front_distance,
            self.obstacle_front_bearing,
            float(options.get("stop_distance", 0.42)),
            float(options.get("resume_distance", 0.60)),
        )
        if self.dynamic_obstacle_in_front():
            self.cmd_vel_pub.publish(Twist())
            self.wait_for_dynamic_obstacle_clear()

    def ensure_obstacle_model_subscriber(self):
        if self.obstacle_model_sub is not None:
            return
        options = self.navigation.get("dynamic_obstacle_stop", {})
        topic = options.get("model_states_topic", "/gazebo/model_states")
        self.obstacle_model_sub = rospy.Subscriber(
            topic,
            ModelStates,
            self.obstacle_model_callback,
            queue_size=1,
        )
        rospy.loginfo(
            "DYNAMIC_OBSTACLE_MONITOR model_topic=%s target_model=%s",
            topic,
            options.get("target_model", "cylinder_obstacle"),
        )

    def obstacle_scan_callback(self, msg):
        options = self.navigation.get("dynamic_obstacle_stop", {})
        center = float(options.get("front_angle", 0.0))
        window = float(options.get("angle_window", 0.35))
        min_valid = int(options.get("min_valid_ranges", 3))
        values = []
        angle = msg.angle_min
        for value in msg.ranges:
            if abs(self.angle_diff(angle, center)) <= window:
                if math.isfinite(value) and value >= msg.range_min and value <= msg.range_max:
                    values.append(float(value))
            angle += msg.angle_increment
        if len(values) < min_valid:
            self.obstacle_front_distance = None
        else:
            self.obstacle_front_distance = min(values)
        self.obstacle_front_stamp = msg.header.stamp if msg.header.stamp else rospy.Time.now()

    def obstacle_model_callback(self, msg):
        options = self.navigation.get("dynamic_obstacle_stop", {})
        target_model = str(options.get("target_model", "cylinder_obstacle"))
        robot_model = str(options.get("robot_model", "bobac3_serverbot"))
        if target_model not in msg.name:
            self.obstacle_front_distance = None
            self.obstacle_front_bearing = None
            self.obstacle_front_stamp = rospy.Time.now()
            self.obstacle_model_xy = None
            self.obstacle_model_speed = None
            return
        if robot_model in msg.name:
            robot_gazebo_pose = msg.pose[msg.name.index(robot_model)]
            robot_x = float(robot_gazebo_pose.position.x)
            robot_y = float(robot_gazebo_pose.position.y)
            robot_q = robot_gazebo_pose.orientation
            robot_yaw = tf.transformations.euler_from_quaternion(
                [robot_q.x, robot_q.y, robot_q.z, robot_q.w]
            )[2]
        else:
            robot_pose = self.current_robot_pose()
            if robot_pose is None:
                return
            robot_x, robot_y, robot_yaw = robot_pose
        pose = msg.pose[msg.name.index(target_model)]
        now = rospy.Time.now()
        obstacle_x = float(pose.position.x)
        obstacle_y = float(pose.position.y)
        if self.obstacle_model_xy is not None and self.obstacle_model_last_time is not None:
            dt = max(1e-3, (now - self.obstacle_model_last_time).to_sec())
            moved = math.hypot(
                obstacle_x - float(self.obstacle_model_xy[0]),
                obstacle_y - float(self.obstacle_model_xy[1]),
            )
            self.obstacle_model_speed = moved / dt
        self.obstacle_model_last_xy = self.obstacle_model_xy
        self.obstacle_model_xy = (obstacle_x, obstacle_y)
        self.obstacle_model_last_time = now
        dx = obstacle_x - robot_x
        dy = obstacle_y - robot_y
        distance = math.hypot(dx, dy)
        bearing = self.angle_diff(math.atan2(dy, dx), robot_yaw)
        self.obstacle_front_distance = distance
        self.obstacle_front_bearing = bearing
        self.obstacle_front_stamp = now

    def dynamic_obstacle_in_front(self):
        options = self.navigation.get("dynamic_obstacle_stop", {})
        if not bool(options.get("enabled", False)):
            return False
        source = str(options.get("source", "gazebo_model")).strip().lower()
        if source == "laser_scan":
            self.ensure_obstacle_scan_subscriber()
        else:
            self.ensure_obstacle_model_subscriber()
        if self.obstacle_front_distance is None:
            return False
        stale_after = float(options.get("stale_after", 0.6))
        if (rospy.Time.now() - self.obstacle_front_stamp).to_sec() > stale_after:
            return False
        check_mode = str(options.get("check_mode", "directional")).strip().lower()
        if source != "laser_scan" and check_mode != "center_distance":
            angle_window = float(options.get("angle_window", 0.35))
            if self.obstacle_front_bearing is None or abs(self.obstacle_front_bearing) > angle_window:
                return False
        engage_distance = float(
            options.get("engage_distance", options.get("stop_distance", 0.42))
        )
        return self.obstacle_front_distance <= engage_distance

    def wait_for_dynamic_obstacle_clear(self):
        options = self.navigation.get("dynamic_obstacle_stop", {})
        engage_distance = float(
            options.get("engage_distance", options.get("stop_distance", 0.42))
        )
        resume_distance = float(options.get("resume_distance", 0.60))
        stable_time = float(options.get("clear_stable_time", 0.6))
        max_wait = float(options.get("max_wait", 45.0))
        poll_rate = rospy.Rate(float(options.get("wait_rate", 10.0)))
        start = time.time()
        clear_since = None
        obstacle_still_since = None
        last_yield_time = 0.0
        last_cancel_time = 0.0
        continuous_yield = bool(
            options.get("continuous_yield", bool(options.get("yield_while_close", True)))
        )
        self.status.publish(
            "dynamic_obstacle_stop",
            distance=self.obstacle_front_distance,
            bearing=self.obstacle_front_bearing,
            obstacle_speed=self.obstacle_model_speed,
            engage_distance=engage_distance,
            stop_distance=float(options.get("stop_distance", 0.42)),
            resume_distance=resume_distance,
            check_mode=str(options.get("check_mode", "directional")),
        )
        rospy.logwarn(
            "DYNAMIC_OBSTACLE_STOP distance=%s bearing=%s engage_distance=%.3f resume_distance=%.3f mode=%s",
            self.obstacle_front_distance,
            self.obstacle_front_bearing,
            engage_distance,
            resume_distance,
            str(options.get("check_mode", "directional")),
        )
        self.clear_costmaps_before_goal("dynamic_obstacle", 0)
        if bool(options.get("yield_on_block", True)) and not continuous_yield:
            self.yield_away_from_dynamic_obstacle(
                options,
                self.obstacle_front_distance,
                self.obstacle_front_bearing,
                self.obstacle_model_speed,
                reason="blocked",
            )
            last_yield_time = time.time()
        while not rospy.is_shutdown():
            self.check_stopped()
            yielded_this_cycle = False
            distance = self.obstacle_front_distance
            bearing = self.obstacle_front_bearing
            obstacle_speed = self.obstacle_model_speed
            fresh = (
                distance is not None
                and (rospy.Time.now() - self.obstacle_front_stamp).to_sec()
                <= float(options.get("stale_after", 0.6))
            )
            source = str(options.get("source", "gazebo_model")).strip().lower()
            check_mode = str(options.get("check_mode", "directional")).strip().lower()
            angle_clear = True
            if source != "laser_scan" and check_mode != "center_distance" and bearing is not None:
                angle_clear = abs(bearing) > float(options.get("angle_window", 0.35))
            if (not fresh) or distance >= resume_distance or angle_clear:
                if clear_since is None:
                    clear_since = time.time()
                if time.time() - clear_since >= stable_time:
                    self.status.publish(
                        "dynamic_obstacle_clear",
                        distance=distance,
                        bearing=bearing,
                        waited=time.time() - start,
                    )
                    rospy.loginfo(
                        "DYNAMIC_OBSTACLE_CLEAR distance=%s bearing=%s waited=%.2f",
                        distance,
                        bearing,
                        time.time() - start,
                    )
                    self.clear_costmaps_before_goal("dynamic_obstacle", 1)
                    return True
            else:
                clear_since = None
                if bool(options.get("yield_while_close", True)):
                    if continuous_yield:
                        now = time.time()
                        if now - last_cancel_time >= float(options.get("cancel_refresh_interval", 0.15)):
                            try:
                                self.move_base.cancel_all_goals()
                            except Exception:
                                pass
                            last_cancel_time = now
                        yielded_this_cycle = self.publish_dynamic_obstacle_yield_once(
                            options,
                            distance,
                            bearing,
                            obstacle_speed,
                            reason="keep_distance_continuous",
                        )
                        obstacle_still_since = None
                    elif time.time() - last_yield_time >= float(options.get("yield_cooldown", 0.8)):
                        self.yield_away_from_dynamic_obstacle(
                            options,
                            distance,
                            bearing,
                            obstacle_speed,
                            reason="keep_distance",
                        )
                        last_yield_time = time.time()
                        obstacle_still_since = None
                elif self.should_yield_from_stuck_obstacle(options, obstacle_speed):
                    if obstacle_still_since is None:
                        obstacle_still_since = time.time()
                    elif (
                        time.time() - obstacle_still_since
                        >= float(options.get("yield_stationary_time", 1.0))
                        and time.time() - last_yield_time
                        >= float(options.get("yield_cooldown", 1.5))
                    ):
                        self.yield_away_from_dynamic_obstacle(
                            options,
                            distance,
                            bearing,
                            obstacle_speed,
                            reason="stationary",
                        )
                        last_yield_time = time.time()
                        obstacle_still_since = None
                else:
                    obstacle_still_since = None
            if not yielded_this_cycle:
                self.cmd_vel_pub.publish(Twist())
            if max_wait > 0.0 and time.time() - start > max_wait:
                self.status.publish(
                    "dynamic_obstacle_wait_timeout",
                    distance=distance,
                    waited=time.time() - start,
                )
                rospy.logwarn(
                    "DYNAMIC_OBSTACLE_WAIT_TIMEOUT distance=%s waited=%.2f",
                    distance,
                    time.time() - start,
                )
                return False
            poll_rate.sleep()

    def should_yield_from_stuck_obstacle(self, options, obstacle_speed):
        if not bool(options.get("yield_when_stationary", True)):
            return False
        if obstacle_speed is None:
            return False
        return obstacle_speed <= float(options.get("stationary_speed", 0.015))

    def build_dynamic_obstacle_yield_twist(self, options, bearing):
        if bearing is None:
            return None
        speed = float(options.get("yield_speed", 0.08))
        if speed <= 0.0:
            return None
        twist = Twist()
        if bool(options.get("guard_reverse_only", True)):
            twist.linear.x = -speed
            twist.linear.y = (
                -math.sin(bearing)
                * speed
                * float(options.get("guard_lateral_scale", 0.0))
            )
        else:
            twist.linear.x = -math.cos(bearing) * speed
            twist.linear.y = -math.sin(bearing) * speed
        return twist

    def publish_dynamic_obstacle_yield_once(
        self,
        options,
        distance,
        bearing,
        obstacle_speed,
        reason="keep_distance_continuous",
    ):
        twist = self.build_dynamic_obstacle_yield_twist(options, bearing)
        if twist is None:
            return False
        self.cmd_vel_pub.publish(twist)
        now = time.time()
        if now - self.last_dynamic_obstacle_yield_log >= 1.0:
            self.last_dynamic_obstacle_yield_log = now
            self.status.publish(
                "dynamic_obstacle_yield",
                reason=reason,
                distance=distance,
                bearing=bearing,
                obstacle_speed=obstacle_speed,
                linear_x=twist.linear.x,
                linear_y=twist.linear.y,
                duration=0.0,
            )
            rospy.logwarn(
                "DYNAMIC_OBSTACLE_YIELD reason=%s distance=%s bearing=%s obstacle_speed=%s vx=%.3f vy=%.3f continuous=true",
                reason,
                distance,
                bearing,
                obstacle_speed,
                twist.linear.x,
                twist.linear.y,
            )
        return True

    def yield_away_from_dynamic_obstacle(
        self,
        options,
        distance,
        bearing,
        obstacle_speed,
        reason="blocked",
    ):
        twist = self.build_dynamic_obstacle_yield_twist(options, bearing)
        duration = float(options.get("yield_duration", 0.45))
        if twist is None or duration <= 0.0:
            return
        self.status.publish(
            "dynamic_obstacle_yield",
            reason=reason,
            distance=distance,
            bearing=bearing,
            obstacle_speed=obstacle_speed,
            linear_x=twist.linear.x,
            linear_y=twist.linear.y,
            duration=duration,
        )
        rospy.logwarn(
            "DYNAMIC_OBSTACLE_YIELD reason=%s distance=%s bearing=%s obstacle_speed=%s vx=%.3f vy=%.3f duration=%.2f",
            reason,
            distance,
            bearing,
            obstacle_speed,
            twist.linear.x,
            twist.linear.y,
            duration,
        )
        rate = rospy.Rate(float(options.get("yield_rate", 20.0)))
        deadline = time.time() + duration
        try:
            while not rospy.is_shutdown() and time.time() < deadline:
                self.check_stopped()
                self.cmd_vel_pub.publish(twist)
                rate.sleep()
        finally:
            self.cmd_vel_pub.publish(Twist())

    @staticmethod
    def angle_diff(a, b):
        return math.atan2(math.sin(a - b), math.cos(a - b))

    def update_slow_zone_speed(self):
        slow_zones = self.config.optional_section("slow_zones")
        if not bool(slow_zones.get("enabled", False)):
            self.apply_slow_zone(None)
            return
        pose = self.current_robot_xy()
        if pose is None:
            return
        x, y = pose
        selected = None
        for zone in slow_zones.get("zones", []):
            if not bool(zone.get("enabled", True)):
                continue
            points = zone.get("points", [])
            if self.point_inside_polygon(x, y, points):
                selected = zone
                break
        self.apply_slow_zone(selected)

    def wait_until_stopped(self, reason="", stable_time=None, timeout=None):
        options = self.navigation.get("wait_until_stopped", {})
        if stable_time is None:
            stable_time = float(options.get("stable_time", 0.35))
        if timeout is None:
            timeout = float(options.get("timeout", 1.5))
        odom_topic = options.get("odom_topic", "/odom")
        linear_threshold = float(options.get("linear_threshold", 0.015))
        angular_threshold = float(options.get("angular_threshold", 0.04))
        self.status.publish(
            "wait_until_stopped_started",
            reason=reason,
            stable_time=stable_time,
            timeout=timeout,
        )
        self.cmd_vel_pub.publish(Twist())
        start = time.time()
        stable_since = None
        while not rospy.is_shutdown() and time.time() - start <= timeout:
            self.check_stopped()
            self.cmd_vel_pub.publish(Twist())
            try:
                msg = rospy.wait_for_message(odom_topic, Odometry, timeout=0.2)
                vx = float(msg.twist.twist.linear.x)
                vy = float(msg.twist.twist.linear.y)
                wz = float(msg.twist.twist.angular.z)
                stopped = (
                    math.hypot(vx, vy) <= linear_threshold
                    and abs(wz) <= angular_threshold
                )
            except Exception:
                rospy.sleep(stable_time)
                self.status.publish(
                    "wait_until_stopped_no_odom",
                    reason=reason,
                    odom_topic=odom_topic,
                )
                return True
            if stopped:
                if stable_since is None:
                    stable_since = time.time()
                if time.time() - stable_since >= stable_time:
                    self.status.publish("wait_until_stopped_done", reason=reason)
                    return True
            else:
                stable_since = None
        self.status.publish("wait_until_stopped_timeout", reason=reason)
        return False

    def current_robot_xy(self):
        map_frame = self.navigation.get("map_frame", "map")
        base_frame = self.navigation.get("base_frame", "base_footprint")
        try:
            self.tf_listener.waitForTransform(
                map_frame, base_frame, rospy.Time(0), rospy.Duration(0.05)
            )
            translation, _ = self.tf_listener.lookupTransform(
                map_frame, base_frame, rospy.Time(0)
            )
            return float(translation[0]), float(translation[1])
        except Exception:
            return None

    def current_robot_pose(self):
        map_frame = self.navigation.get("map_frame", "map")
        base_frame = self.navigation.get("base_frame", "base_footprint")
        try:
            self.tf_listener.waitForTransform(
                map_frame, base_frame, rospy.Time(0), rospy.Duration(0.05)
            )
            translation, rotation = self.tf_listener.lookupTransform(
                map_frame, base_frame, rospy.Time(0)
            )
            yaw = tf.transformations.euler_from_quaternion(rotation)[2]
            return float(translation[0]), float(translation[1]), yaw
        except Exception:
            return None

    def apply_slow_zone(self, zone):
        zone_id = ""
        if zone:
            zone_id = str(zone.get("id", "slow_zone"))
        if zone_id == self.slow_zone_active_id:
            return
        if self.slow_zone_active_id:
            old_id = self.slow_zone_active_id
            self.restore_dwa_parameters(
                self.slow_zone_saved_speed_values,
                reason="slow_zone_exit_%s" % old_id,
            )
            rospy.loginfo("SLOW_ZONE_EXIT id=%s", old_id)
            self.status.publish("slow_zone_exit", zone_id=old_id)
            self.slow_zone_active_id = ""
            self.slow_zone_saved_speed_values = {}
        if not zone:
            return
        slow_zones = self.config.optional_section("slow_zones")
        speed_values = zone.get("speed_values", slow_zones.get("speed_values", {}))
        if speed_values:
            saved = self.limit_dwa_parameters(
                speed_values,
                reason="slow_zone_enter_%s" % zone_id,
            )
        else:
            params = zone.get("speed_params", slow_zones.get("speed_params", []))
            multiplier = float(zone.get("multiplier", slow_zones.get("default_multiplier", 0.45)))
            saved = self.scale_dwa_parameters(
                params,
                multiplier,
                reason="slow_zone_enter_%s" % zone_id,
            )
        if not saved:
            return
        self.slow_zone_active_id = zone_id
        self.slow_zone_saved_speed_values = saved
        rospy.loginfo("SLOW_ZONE_ENTER id=%s speed_values=%s", zone_id, speed_values)
        self.status.publish("slow_zone_enter", zone_id=zone_id, speed_values=speed_values)

    @staticmethod
    def point_inside_polygon(x, y, points):
        if len(points) < 3:
            return False
        vertices = []
        for point in points:
            try:
                vertices.append((float(point["x"]), float(point["y"])))
            except (KeyError, TypeError, ValueError):
                return False
        inside = False
        j = len(vertices) - 1
        for i, vertex in enumerate(vertices):
            xi, yi = vertex
            xj, yj = vertices[j]
            if ((yi > y) != (yj > y)):
                cross_x = (xj - xi) * (y - yi) / (yj - yi + 1e-12) + xi
                if x < cross_x:
                    inside = not inside
            j = i
        return inside

    def latest_action_status_since(self, goal_start):
        status_topic = self.action_server_name.rstrip("/") + "/status"
        try:
            message = rospy.wait_for_message(
                status_topic,
                GoalStatusArray,
                timeout=float(self.navigation.get("action_status_wait_timeout", 0.05)),
            )
        except rospy.ROSException:
            return None
        latest = None
        for status in message.status_list:
            if status.goal_id.stamp < goal_start - rospy.Duration(1.0):
                continue
            if latest is None or status.goal_id.stamp > latest.goal_id.stamp:
                latest = status
        if latest is None:
            return None
        return latest.status

    def verify_goal_pose(self, name, point, xy_tolerance, yaw_tolerance):
        map_frame = self.navigation.get("map_frame", point.get("frame_id", "map"))
        base_frame = self.navigation.get("base_frame", "base_footprint")
        try:
            self.tf_listener.waitForTransform(
                map_frame, base_frame, rospy.Time(0), rospy.Duration(2.0)
            )
            translation, rotation = self.tf_listener.lookupTransform(
                map_frame, base_frame, rospy.Time(0)
            )
        except Exception as exc:
            rospy.logwarn("Goal precision check failed at %s: %s", name, exc)
            return False
        yaw = tf.transformations.euler_from_quaternion(rotation)[2]
        goal_yaw = float(point["yaw"])
        dx = float(translation[0]) - float(point["x"])
        dy = float(translation[1]) - float(point["y"])
        distance = math.hypot(dx, dy)
        yaw_error = math.atan2(math.sin(yaw - goal_yaw), math.cos(yaw - goal_yaw))
        yaw_error_abs = abs(yaw_error)
        self.status.publish(
            "goal_precision_check",
            waypoint=name,
            distance=distance,
            xy_tolerance=xy_tolerance,
            yaw_error=yaw_error_abs,
            yaw_tolerance=yaw_tolerance,
        )
        if distance <= xy_tolerance and yaw_error_abs <= yaw_tolerance:
            return True
        rospy.logwarn(
            "Goal precision check rejected %s: robot=(%.4f, %.4f, %.4f) goal=(%.4f, %.4f, %.4f) distance=%.4f/%.4f yaw_error=%.4f/%.4f",
            name,
            float(translation[0]),
            float(translation[1]),
            yaw,
            float(point["x"]),
            float(point["y"]),
            goal_yaw,
            distance,
            xy_tolerance,
            yaw_error_abs,
            yaw_tolerance,
        )
        return False

    def clear_costmaps_before_goal(self, name, attempt):
        if not bool(self.navigation.get("clear_costmaps_before_goal", False)):
            return
        service_name = self.navigation.get(
            "clear_costmaps_service",
            "/move_base_node/clear_costmaps",
        )
        timeout = float(self.navigation.get("clear_costmaps_wait", 5.0))
        try:
            rospy.wait_for_service(service_name, timeout=timeout)
            clear_costmaps = rospy.ServiceProxy(service_name, Empty)
            clear_costmaps()
            self.status.publish("costmaps_cleared", waypoint=name, attempt=attempt + 1)
        except Exception as exc:
            rospy.logwarn("Clear costmaps before %s failed: %s", name, exc)

    def accept_near_goal(
        self,
        name,
        point,
        state,
        xy_tolerance,
        allow_near_goal_for_precise=False,
    ):
        if not bool(self.navigation.get("accept_aborted_within_goal_tolerance", False)):
            return False
        map_frame = self.navigation.get("map_frame", point.get("frame_id", "map"))
        base_frame = self.navigation.get("base_frame", "base_footprint")
        tolerance = float(xy_tolerance)
        if allow_near_goal_for_precise:
            precise_options = self.navigation.get("precise_arrival_adjust", {})
            tolerance = float(
                precise_options.get(
                    "start_xy_tolerance",
                    self.navigation.get("near_goal_xy_tolerance", xy_tolerance),
                )
            )
        try:
            self.tf_listener.waitForTransform(
                map_frame, base_frame, rospy.Time(0), rospy.Duration(2.0)
            )
            translation, _ = self.tf_listener.lookupTransform(
                map_frame, base_frame, rospy.Time(0)
            )
        except Exception as exc:
            rospy.logwarn("Near-goal check failed at %s: %s", name, exc)
            return False
        dx = float(translation[0]) - float(point["x"])
        dy = float(translation[1]) - float(point["y"])
        distance = math.hypot(dx, dy)
        rospy.logwarn(
            "Near-goal check at %s after move_base state=%s: robot=(%.3f, %.3f) goal=(%.3f, %.3f) distance=%.3f tolerance=%.3f",
            name,
            state,
            float(translation[0]),
            float(translation[1]),
            float(point["x"]),
            float(point["y"]),
            distance,
            tolerance,
        )
        if distance <= tolerance:
            self.status.publish(
                "arrived_near_goal",
                waypoint=name,
                move_base_state=state,
                distance=distance,
                tolerance=tolerance,
            )
            rospy.logwarn(
                "Accepted near-goal arrival at %s after move_base state=%s: distance=%.3f tolerance=%.3f",
                name,
                state,
                distance,
                tolerance,
            )
            return True
        return False

    def soft_fail_enabled(self):
        value = os.environ.get("RAICOM_NAVIGATION_SOFT_FAIL")
        if value is None:
            value = self.navigation.get("soft_fail_on_navigation_error", False)
        if isinstance(value, bool):
            return value
        return str(value).strip().lower() in ("1", "true", "yes", "on")

    def navigation_distance_to_goal(self, point):
        map_frame = self.navigation.get("map_frame", point.get("frame_id", "map"))
        base_frame = self.navigation.get("base_frame", "base_footprint")
        try:
            self.tf_listener.waitForTransform(
                map_frame, base_frame, rospy.Time(0), rospy.Duration(2.0)
            )
            translation, _ = self.tf_listener.lookupTransform(
                map_frame, base_frame, rospy.Time(0)
            )
            return math.hypot(
                float(translation[0]) - float(point["x"]),
                float(translation[1]) - float(point["y"]),
            )
        except Exception as exc:
            rospy.logwarn("Navigation distance check failed: %s", exc)
            return None

    def should_soft_fail(self, name, point, tolerance):
        if not self.soft_fail_enabled():
            return False
        max_distance = self.navigation.get("soft_fail_max_distance")
        if max_distance is None:
            return True
        distance = self.navigation_distance_to_goal(point)
        if distance is None:
            self.status.publish(
                "navigation_soft_fail_rejected",
                waypoint=name,
                reason="distance_unavailable",
                tolerance=tolerance,
                max_distance=float(max_distance),
            )
            return False
        limit = max(float(max_distance), float(tolerance or 0.0))
        if distance <= limit:
            return True
        self.status.publish(
            "navigation_soft_fail_rejected",
            waypoint=name,
            reason="too_far",
            distance=distance,
            tolerance=tolerance,
            max_distance=limit,
        )
        rospy.logerr(
            "Rejecting soft-fail at %s: distance=%s exceeds max_distance=%s.",
            name,
            distance,
            limit,
        )
        return False

    def publish_navigation_soft_failed(
        self,
        name,
        point,
        reason,
        last_state=None,
        tolerance=None,
    ):
        distance = self.navigation_distance_to_goal(point)
        rospy.logerr(
            "Soft-fail navigation at %s: reason=%s state=%s distance=%s tolerance=%s; continuing mission.",
            name,
            reason,
            last_state,
            distance,
            tolerance,
        )
        self.status.publish(
            "navigation_soft_failed",
            waypoint=name,
            reason=reason,
            move_base_state=last_state,
            distance=distance,
            tolerance=tolerance,
        )

    def set_position_only_yaw_tolerance(self):
        param_name = self.navigation.get(
            "dwa_yaw_goal_tolerance_param",
            "/move_base_node/DWAPlannerROS/yaw_goal_tolerance",
        )
        old_value = rospy.get_param(param_name, None)
        new_value = float(
            self.navigation.get("position_only_yaw_goal_tolerance", math.pi)
        )
        self.status.publish(
            "navigation_position_only_yaw_tolerance",
            yaw_goal_tolerance=new_value,
        )
        self.set_dwa_yaw_goal_tolerance(new_value)
        return old_value

    def resolve_goal_tolerances(
        self,
        point,
        position_only=False,
        xy_tolerance=None,
        yaw_tolerance=None,
        ignore_yaw=False,
    ):
        if xy_tolerance is None:
            xy_tolerance = point.get(
                "xy_tolerance",
                self.navigation.get("goal_xy_tolerance", 0.01),
            )
        if yaw_tolerance is None:
            if position_only or ignore_yaw or bool(point.get("ignore_yaw", False)):
                yaw_tolerance = math.pi
            else:
                yaw_tolerance = point.get(
                    "yaw_tolerance",
                    self.navigation.get("goal_yaw_tolerance", math.radians(10.0)),
                )
        return float(xy_tolerance), float(yaw_tolerance)

    def set_configured_goal_tolerances(self, xy_tolerance=None, yaw_tolerance=None):
        if yaw_tolerance is None:
            yaw_tolerance = float(
                self.navigation.get("goal_yaw_tolerance", math.radians(10.0))
            )
        if xy_tolerance is None:
            xy_tolerance = float(self.navigation.get("goal_xy_tolerance", 0.01))
        self.status.publish(
            "navigation_goal_tolerances",
            xy_goal_tolerance=xy_tolerance,
            yaw_goal_tolerance=yaw_tolerance,
        )
        service_name = self.navigation.get(
            "dwa_reconfigure_service",
            "/move_base_node/DWAPlannerROS/set_parameters",
        )
        rospy.wait_for_service(service_name, timeout=10.0)
        request = ReconfigureRequest()
        request.config.doubles.append(
            DoubleParameter(name="xy_goal_tolerance", value=xy_tolerance)
        )
        request.config.doubles.append(
            DoubleParameter(name="yaw_goal_tolerance", value=yaw_tolerance)
        )
        service = rospy.ServiceProxy(service_name, Reconfigure)
        service(request)

    def set_dwa_yaw_goal_tolerance(self, value):
        service_name = self.navigation.get(
            "dwa_reconfigure_service",
            "/move_base_node/DWAPlannerROS/set_parameters",
        )
        rospy.wait_for_service(service_name, timeout=10.0)
        request = ReconfigureRequest()
        request.config.doubles.append(
            DoubleParameter(name="yaw_goal_tolerance", value=float(value))
        )
        service = rospy.ServiceProxy(service_name, Reconfigure)
        service(request)

    def scale_dwa_parameters(self, names, multiplier, reason=""):
        multiplier = float(multiplier)
        if multiplier <= 0.0 or abs(multiplier - 1.0) < 1e-6:
            return {}
        names = [str(name).strip() for name in names if str(name).strip()]
        if not names:
            return {}
        service_name = self.navigation.get(
            "dwa_reconfigure_service",
            "/move_base_node/DWAPlannerROS/set_parameters",
        )
        namespace = service_name
        if namespace.endswith("/set_parameters"):
            namespace = namespace[: -len("/set_parameters")]
        old_values = {}
        new_values = {}
        for name in names:
            param_name = namespace.rstrip("/") + "/" + name
            old_value = rospy.get_param(param_name, None)
            if old_value is None:
                rospy.logwarn("DWA speed parameter missing: %s", param_name)
                continue
            try:
                old_value = float(old_value)
            except (TypeError, ValueError):
                rospy.logwarn("DWA speed parameter is not numeric: %s=%s", param_name, old_value)
                continue
            old_values[name] = old_value
            new_values[name] = old_value * multiplier
        if not new_values:
            return {}
        try:
            self.set_dwa_double_parameters(new_values)
        except Exception as exc:
            rospy.logwarn("DWA speed scale failed: %s", exc)
            self.status.publish(
                "dwa_speed_scale_failed",
                reason=reason,
                error=str(exc),
            )
            return {}
        self.status.publish(
            "dwa_speed_scaled",
            reason=reason,
            multiplier=multiplier,
            old_values=old_values,
            new_values=new_values,
        )
        return old_values

    def restore_dwa_parameters(self, old_values, reason=""):
        if not old_values:
            return
        try:
            self.set_dwa_double_parameters(old_values)
        except Exception as exc:
            rospy.logwarn("DWA speed restore failed: %s", exc)
            self.status.publish(
                "dwa_speed_restore_failed",
                reason=reason,
                error=str(exc),
            )
            return
        self.status.publish(
            "dwa_speed_restored",
            reason=reason,
            values=old_values,
        )

    def limit_dwa_parameters(self, values, reason=""):
        values = {
            str(name).strip(): float(value)
            for name, value in values.items()
            if str(name).strip()
        }
        if not values:
            return {}
        service_name = self.navigation.get(
            "dwa_reconfigure_service",
            "/move_base_node/DWAPlannerROS/set_parameters",
        )
        namespace = service_name
        if namespace.endswith("/set_parameters"):
            namespace = namespace[: -len("/set_parameters")]
        old_values = {}
        new_values = {}
        for name, target_value in values.items():
            param_name = namespace.rstrip("/") + "/" + name
            old_value = rospy.get_param(param_name, None)
            if old_value is None:
                rospy.logwarn("DWA speed parameter missing: %s", param_name)
                continue
            try:
                old_value = float(old_value)
            except (TypeError, ValueError):
                rospy.logwarn("DWA speed parameter is not numeric: %s=%s", param_name, old_value)
                continue
            old_values[name] = old_value
            new_values[name] = min(old_value, target_value)
        if not new_values:
            return {}
        try:
            self.set_dwa_double_parameters(new_values)
        except Exception as exc:
            rospy.logwarn("DWA speed limit failed: %s", exc)
            self.status.publish(
                "dwa_speed_limit_failed",
                reason=reason,
                error=str(exc),
            )
            return {}
        self.status.publish(
            "dwa_speed_limited",
            reason=reason,
            old_values=old_values,
            new_values=new_values,
        )
        return old_values

    def set_dwa_double_parameters(self, values):
        service_name = self.navigation.get(
            "dwa_reconfigure_service",
            "/move_base_node/DWAPlannerROS/set_parameters",
        )
        rospy.wait_for_service(service_name, timeout=10.0)
        request = ReconfigureRequest()
        for name, value in values.items():
            request.config.doubles.append(
                DoubleParameter(name=str(name), value=float(value))
            )
        service = rospy.ServiceProxy(service_name, Reconfigure)
        service(request)

    def pause_after_arrival(self, name):
        options = self.config.section("arrival_pause")
        if not options.get("enabled", False):
            return
        if name not in options.get("waypoints", []):
            return
        seconds = float(options.get("seconds", 5.0))
        self.status.publish("arrival_pause", waypoint=name, seconds=seconds)
        deadline = time.time() + seconds
        while not rospy.is_shutdown() and time.time() < deadline:
            self.check_stopped()
            time.sleep(0.1)

    @staticmethod
    def yaw_to_quaternion(yaw):
        return Quaternion(0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0))
