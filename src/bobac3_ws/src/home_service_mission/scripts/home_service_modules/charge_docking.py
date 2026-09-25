#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import math

import rospy
import tf
from ar_pose.srv import Track, TrackRequest
from ar_track_alvar_msgs.msg import AlvarMarkers
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan


class ChargeDockingClient:
    def __init__(self, cmd_vel_pub, status, check_stopped, config=None):
        self.cmd_vel_pub = cmd_vel_pub
        self.status = status
        self.check_stopped = check_stopped
        self.config = config
        self.tf_listener = tf.TransformListener()

    def run(self, docking):
        if not docking.get("enabled", False):
            self.status.publish("docking_skipped")
            return
        method = docking.get("method", "ar_pose")
        if method in ("map_pose", "map", "provincial_map"):
            self.dock_to_charge_by_map(docking)
            self.publish_stop()
            self.status.publish("docking_completed")
            return
        if method != "ar_pose":
            raise RuntimeError("Unsupported docking method: %s" % method)
        if docking.get("pre_align_yaw", False):
            self.align_absolute_yaw(
                float(docking.get("pre_align_target_yaw", 0.0)),
                float(docking.get("pre_align_yaw_tolerance", 0.17453292519943295)),
                float(docking.get("pre_align_timeout", 20.0)),
                float(docking.get("pre_align_speed", 0.2)),
                int(docking.get("pre_align_stable_cycles", 6)),
                docking,
            )
        if docking.get("direct_marker_docking", False):
            if docking.get("marker_lateral_align", False):
                self.align_marker_lateral(docking)
        elif docking.get("use_track_service", True):
            self.track_ar_pose(docking)
        reverse_distance = float(docking.get("reverse_distance", 0.0))
        if docking.get("reverse_distance_from_marker", False):
            reverse_distance = self.reverse_distance_from_marker(docking)
        if docking.get("rotate_after_track", True):
            self.rotate_relative(float(docking.get("rotate_angle", math.pi)), docking)
        if docking.get("reverse_after_rotate", True):
            if docking.get("reverse_distance_from_marker", False) and docking.get(
                "reverse_with_marker_feedback", True
            ):
                self.drive_to_marker_distance(
                    docking,
                    reverse_distance,
                    float(docking.get("reverse_speed", -0.035)),
                    float(docking.get("reverse_timeout", 20.0)),
                )
            else:
                self.drive_linear_distance(
                    reverse_distance,
                    float(docking.get("reverse_speed", -0.035)),
                    float(docking.get("reverse_timeout", 20.0)),
                    docking,
                )
        self.publish_stop()
        self.status.publish("docking_completed")

    def dock_to_charge_by_map(self, docking):
        if self.config is None:
            raise RuntimeError("Map docking requires mission config")
        waypoint_name = docking.get(
            "map_charge_waypoint",
            docking.get("charge_waypoint", "charge"),
        )
        waypoints = self.config.section("waypoints")
        if waypoint_name not in waypoints:
            raise RuntimeError("Map docking waypoint is not configured: %s" % waypoint_name)
        waypoint = waypoints[waypoint_name]
        target_y = float(docking.get("map_target_y", waypoint["y"]))
        target_yaw = float(docking.get("map_target_yaw", waypoint.get("yaw", 0.0)))
        y_tolerance = float(docking.get("map_y_tolerance", docking.get("xy_tolerance", 0.03)))
        yaw_tolerance = float(docking.get("map_yaw_tolerance", docking.get("yaw_tolerance", 0.05)))
        reverse_speed = -abs(float(docking.get("map_reverse_speed", docking.get("reverse_speed", -0.025))))
        angular_gain = float(docking.get("map_angular_gain", docking.get("angular_gain", 0.8)))
        max_angular = abs(float(docking.get("map_max_angular", docking.get("max_angular", 0.08))))
        timeout = float(docking.get("map_timeout", docking.get("reverse_timeout", 18.0)))
        stable_required = int(docking.get("map_stable_cycles", docking.get("stable_cycles", 10)))
        deadline = rospy.Time.now() + rospy.Duration(timeout)
        stable_cycles = 0
        rate = rospy.Rate(float(docking.get("map_rate", 20.0)))
        self.status.publish(
            "docking_map_reverse",
            waypoint=waypoint_name,
            target_y=target_y,
            target_yaw=target_yaw,
        )

        while not rospy.is_shutdown() and rospy.Time.now() < deadline:
            self.check_stopped()
            _, y, yaw = self.current_pose(docking)
            y_error = target_y - y
            yaw_error = self.normalize_angle(target_yaw - yaw)
            if abs(y_error) <= y_tolerance and abs(yaw_error) <= yaw_tolerance:
                stable_cycles += 1
                self.publish_stop()
                if stable_cycles >= stable_required:
                    self.status.publish(
                        "docking_map_reached",
                        y_error=y_error,
                        yaw_error=yaw_error,
                    )
                    return
                rate.sleep()
                continue
            stable_cycles = 0
            cmd = Twist()
            if abs(y_error) > y_tolerance:
                cmd.linear.x = reverse_speed
            cmd.angular.z = self.clip(angular_gain * yaw_error, -max_angular, max_angular)
            self.cmd_vel_pub.publish(cmd)
            rate.sleep()

        self.publish_stop()
        raise RuntimeError("Map docking timeout")

    def track_ar_pose(self, docking):
        service_name = docking.get("track_service", "/track")
        wait_seconds = float(docking.get("service_wait", 10.0))
        self.status.publish("docking_ar_wait", service=service_name)
        rospy.wait_for_service(service_name, timeout=wait_seconds)
        client = rospy.ServiceProxy(service_name, Track)
        request = TrackRequest()
        request.ar_id = int(docking.get("ar_id", 0))
        request.goal_dist = float(docking.get("goal_dist", 0.35))
        self.status.publish(
            "docking_ar_track",
            service=service_name,
            ar_id=request.ar_id,
            goal_dist=request.goal_dist,
        )
        response = client(request)
        if not response.success:
            raise RuntimeError("AR docking failed: %s" % response.message)

    def reverse_distance_from_marker(self, docking):
        marker_distance = self.read_marker_distance(docking)
        final_distance = float(docking.get("final_marker_distance", 0.10))
        fallback = float(docking.get("reverse_distance_fallback", 0.25))
        if marker_distance is None:
            reverse_distance = fallback
            reverse_offset = 0.0
            self.status.publish(
                "docking_reverse_distance_fallback",
                distance=reverse_distance,
            )
        else:
            reverse_offset = float(docking.get("reverse_distance_offset", 0.0))
            reverse_distance = marker_distance - final_distance - reverse_offset
        reverse_min = float(docking.get("reverse_distance_min", 0.0))
        reverse_max = float(docking.get("reverse_distance_max", 0.70))
        reverse_distance = self.clip(reverse_distance, reverse_min, reverse_max)
        self.status.publish(
            "docking_reverse_distance",
            marker_distance=marker_distance if marker_distance is not None else -1.0,
            final_marker_distance=final_distance,
            reverse_distance_offset=reverse_offset,
            distance=reverse_distance,
        )
        return reverse_distance

    def read_marker_distance(self, docking):
        marker = self.read_marker_pose(docking)
        if marker is None:
            return None
        position = marker.pose.pose.position
        axis = docking.get("marker_distance_axis", "x")
        if axis == "x":
            return abs(float(position.x))
        if axis == "y":
            return abs(float(position.y))
        if axis == "z":
            return abs(float(position.z))
        return math.sqrt(
            float(position.x) ** 2
            + float(position.y) ** 2
            + float(position.z) ** 2
        )

    def read_marker_pose(self, docking):
        topic = docking.get("marker_topic", "/base_camera/ar_pose_marker")
        ar_id = int(docking.get("ar_id", 0))
        timeout = float(docking.get("marker_wait", 2.0))
        deadline = rospy.Time.now() + rospy.Duration(timeout)
        while not rospy.is_shutdown() and rospy.Time.now() < deadline:
            self.check_stopped()
            try:
                message = rospy.wait_for_message(topic, AlvarMarkers, timeout=0.5)
            except rospy.ROSException:
                continue
            for marker in message.markers:
                if marker.id == ar_id:
                    return marker
        rospy.logwarn("AR marker unavailable on %s for id %s", topic, ar_id)
        return None

    def align_marker_lateral(self, docking):
        axis = docking.get("marker_lateral_axis", "y")
        tolerance = float(docking.get("marker_lateral_tolerance", 0.05))
        kp = float(docking.get("marker_lateral_kp", 0.4))
        max_speed = abs(float(docking.get("marker_lateral_max_speed", 0.04)))
        timeout = float(docking.get("marker_align_timeout", 10.0))
        stable_cycles_required = int(docking.get("marker_align_stable_cycles", 6))
        deadline = rospy.Time.now() + rospy.Duration(timeout)
        stable_cycles = 0
        rate = rospy.Rate(20.0)
        self.status.publish(
            "docking_marker_lateral_align",
            axis=axis,
            tolerance=tolerance,
        )

        while not rospy.is_shutdown() and rospy.Time.now() < deadline:
            self.check_stopped()
            marker = self.read_marker_pose(docking)
            if marker is None:
                rate.sleep()
                continue
            position = marker.pose.pose.position
            if axis == "x":
                error = float(position.x)
            elif axis == "z":
                error = float(position.z)
            else:
                error = float(position.y)
            if abs(error) <= tolerance:
                stable_cycles += 1
                self.publish_stop()
                if stable_cycles >= stable_cycles_required:
                    self.status.publish(
                        "docking_marker_lateral_aligned",
                        error=error,
                    )
                    return
                rate.sleep()
                continue
            stable_cycles = 0
            cmd = Twist()
            cmd.linear.y = self.clip(kp * error, -max_speed, max_speed)
            self.cmd_vel_pub.publish(cmd)
            rate.sleep()

        self.publish_stop()
        raise RuntimeError("Docking marker lateral align timeout")

    def align_absolute_yaw(
        self,
        target_yaw,
        tolerance,
        timeout,
        max_speed,
        stable_cycles_required,
        docking,
    ):
        deadline = rospy.Time.now() + rospy.Duration(timeout)
        rate = rospy.Rate(20.0)
        stable_cycles = 0
        max_speed = abs(max_speed)
        self.status.publish(
            "docking_pre_align_yaw",
            target_yaw=target_yaw,
            tolerance=tolerance,
        )

        while not rospy.is_shutdown() and rospy.Time.now() < deadline:
            self.check_stopped()
            _, _, yaw = self.current_pose(docking)
            yaw_error = self.normalize_angle(target_yaw - yaw)
            if abs(yaw_error) <= tolerance:
                stable_cycles += 1
                self.publish_stop()
                if stable_cycles >= stable_cycles_required:
                    self.status.publish(
                        "docking_pre_yaw_aligned",
                        yaw=yaw,
                        yaw_error=yaw_error,
                    )
                    return
                rate.sleep()
                continue
            stable_cycles = 0
            cmd = Twist()
            cmd.angular.z = self.clip(yaw_error, -max_speed, max_speed)
            self.cmd_vel_pub.publish(cmd)
            rate.sleep()

        self.publish_stop()
        raise RuntimeError("Docking pre-align yaw timeout")

    def rotate_relative(self, angle, docking):
        _, _, start_yaw = self.current_pose(docking)
        target_yaw = self.normalize_angle(start_yaw + angle)
        tolerance = float(docking.get("yaw_tolerance", 0.035))
        max_speed = abs(float(docking.get("rotate_speed", 0.20)))
        timeout = float(docking.get("rotate_timeout", 20.0))
        stable_cycles_required = int(docking.get("stable_cycles", 8))
        deadline = rospy.Time.now() + rospy.Duration(timeout)
        rate = rospy.Rate(20.0)
        stable_cycles = 0
        self.status.publish("docking_rotate", target_yaw=target_yaw)

        while not rospy.is_shutdown() and rospy.Time.now() < deadline:
            self.check_stopped()
            _, _, yaw = self.current_pose(docking)
            yaw_error = self.normalize_angle(target_yaw - yaw)
            if abs(yaw_error) <= tolerance:
                stable_cycles += 1
                self.publish_stop()
                if stable_cycles >= stable_cycles_required:
                    return
                rate.sleep()
                continue
            stable_cycles = 0
            cmd = Twist()
            cmd.angular.z = self.clip(yaw_error, -max_speed, max_speed)
            self.cmd_vel_pub.publish(cmd)
            rate.sleep()

        self.publish_stop()
        raise RuntimeError("Docking rotate timeout")

    def drive_to_marker_distance(self, docking, fallback_distance, speed, timeout):
        target_distance = float(docking.get("final_marker_distance", 0.05))
        distance_axis = docking.get("marker_distance_axis", "x")
        lateral_axis = docking.get("marker_lateral_axis", "y")
        lateral_kp = float(docking.get("marker_lateral_kp", 0.4))
        lateral_max_speed = abs(float(docking.get("marker_lateral_max_speed", 0.04)))
        max_reverse_speed = abs(speed)
        min_reverse_speed = abs(float(docking.get("reverse_min_speed", 0.01)))
        distance_kp = float(docking.get("reverse_distance_kp", 0.6))
        max_travel = float(docking.get("reverse_distance_max", fallback_distance))
        marker_missing_grace = float(docking.get("reverse_marker_missing_grace", 1.0))
        lost_success_travel_margin = float(
            docking.get("reverse_marker_lost_success_travel_margin", 0.04)
        )
        lost_success_distance_margin = float(
            docking.get("reverse_marker_lost_success_distance_margin", 0.04)
        )
        lost_stop_distance = float(
            docking.get("reverse_marker_lost_stop_distance", 0.25)
        )
        lost_stop_as_success = bool(
            docking.get("reverse_marker_lost_stop_as_success", True)
        )
        expected_travel_margin = float(
            docking.get("reverse_marker_expected_travel_stop_margin", 0.02)
        )
        timeout_stop_as_success = bool(
            docking.get("reverse_marker_timeout_stop_as_success", True)
        )
        odom_fallback_enabled = bool(docking.get("reverse_marker_odom_fallback", True))
        stable_cycles_required = int(docking.get("reverse_marker_stop_stable_cycles", 4))
        yaw_correction = bool(docking.get("reverse_yaw_correction", True))
        yaw_target = float(docking.get("pre_align_target_yaw", 0.0))
        yaw_kp = float(docking.get("reverse_yaw_kp", 0.8))
        yaw_max_speed = abs(float(docking.get("reverse_yaw_max_speed", 0.08)))
        start_x, start_y, _ = self.current_pose(docking)
        deadline = rospy.Time.now() + rospy.Duration(timeout)
        missing_since = None
        stable_cycles = 0
        last_marker_distance = None
        marker_missing_reported = False
        odom_fallback_reported = False
        odom_fallback_start_traveled = None
        odom_fallback_start_marker_distance = None
        rate = rospy.Rate(20.0)
        self.status.publish(
            "docking_reverse_marker_feedback",
            target_distance=target_distance,
            max_travel=max_travel,
        )

        while not rospy.is_shutdown() and rospy.Time.now() < deadline:
            self.check_stopped()
            x, y, yaw = self.current_pose(docking)
            traveled = math.hypot(x - start_x, y - start_y)
            if traveled >= max_travel:
                self.publish_stop()
                self.status.publish(
                    "docking_reverse_marker_max_travel",
                    traveled=traveled,
                    max_travel=max_travel,
                )
                return
            if traveled >= max(0.0, fallback_distance - expected_travel_margin):
                self.publish_stop()
                self.status.publish(
                    "docking_reverse_expected_travel_stop",
                    traveled=traveled,
                    fallback_distance=fallback_distance,
                    expected_travel_margin=expected_travel_margin,
                    last_marker_distance=(
                        last_marker_distance
                        if last_marker_distance is not None
                        else -1.0
                    ),
                )
                return

            marker = self.read_marker_pose_once(docking, timeout=0.10)
            if marker is None:
                self.publish_stop()
                stable_cycles = 0
                now = rospy.Time.now()
                if missing_since is None:
                    missing_since = now
                    marker_missing_reported = False
                if not marker_missing_reported:
                    self.status.publish(
                        "docking_reverse_marker_missing",
                        traveled=traveled,
                        fallback_distance=fallback_distance,
                        last_marker_distance=(
                            last_marker_distance
                            if last_marker_distance is not None
                            else -1.0
                        ),
                    )
                    marker_missing_reported = True
                if (
                    docking.get("reverse_marker_lost_lidar_fallback", True)
                    and last_marker_distance is not None
                    and last_marker_distance
                    <= float(docking.get("reverse_marker_lost_lidar_start_distance", 0.30))
                ):
                    remaining_timeout = max(
                        0.0,
                        (deadline - rospy.Time.now()).to_sec(),
                    )
                    if self.drive_to_rear_lidar_distance(
                        docking,
                        speed,
                        remaining_timeout,
                    ):
                        return
                if (
                    last_marker_distance is not None
                    and last_marker_distance
                    <= target_distance + lost_success_distance_margin
                ):
                    self.status.publish(
                        "docking_reverse_marker_lost_near_target",
                        last_marker_distance=last_marker_distance,
                        target_distance=target_distance,
                    )
                    return
                if (
                    last_marker_distance is not None
                    and last_marker_distance <= lost_stop_distance
                ):
                    self.publish_stop()
                    self.status.publish(
                        "docking_reverse_marker_lost_safety_stop",
                        last_marker_distance=last_marker_distance,
                        target_distance=target_distance,
                        lost_stop_distance=lost_stop_distance,
                        traveled=traveled,
                    )
                    if lost_stop_as_success:
                        return
                    raise RuntimeError("Docking AR marker lost near target")
                if traveled >= max(0.0, fallback_distance - lost_success_travel_margin):
                    self.status.publish(
                        "docking_reverse_marker_lost_after_expected_travel",
                        traveled=traveled,
                        fallback_distance=fallback_distance,
                    )
                    return
                if (now - missing_since).to_sec() >= marker_missing_grace:
                    if last_marker_distance is None:
                        raise RuntimeError("Docking AR marker lost during reverse")
                    if not odom_fallback_enabled:
                        if docking.get("reverse_marker_lost_lidar_fallback", True):
                            remaining_timeout = max(
                                0.0,
                                (deadline - rospy.Time.now()).to_sec(),
                            )
                            if self.drive_to_rear_lidar_distance(
                                docking,
                                speed,
                                remaining_timeout,
                            ):
                                return
                        extra_travel = float(
                            docking.get("reverse_marker_lost_extra_travel", 0.0)
                        )
                        extra_start_distance = float(
                            docking.get("reverse_marker_lost_extra_start_distance", 0.26)
                        )
                        if extra_travel > 0.0 and last_marker_distance <= extra_start_distance:
                            extra_speed = float(
                                docking.get("reverse_marker_lost_extra_speed", speed)
                            )
                            extra_timeout = float(
                                docking.get("reverse_marker_lost_extra_timeout", 2.0)
                            )
                            self.status.publish(
                                "docking_reverse_marker_lost_extra_travel",
                                distance=extra_travel,
                                speed=extra_speed,
                                last_marker_distance=last_marker_distance,
                            )
                            self.drive_linear_distance(
                                extra_travel,
                                extra_speed,
                                extra_timeout,
                                docking,
                            )
                        self.publish_stop()
                        self.status.publish(
                            "docking_reverse_marker_lost_stop",
                            traveled=traveled,
                            fallback_distance=fallback_distance,
                            last_marker_distance=last_marker_distance,
                        )
                        return
                    if not odom_fallback_reported:
                        self.status.publish(
                            "docking_reverse_marker_lost_odom_fallback",
                            traveled=traveled,
                            fallback_distance=fallback_distance,
                            last_marker_distance=last_marker_distance,
                        )
                        odom_fallback_reported = True
                    if odom_fallback_start_traveled is None:
                        odom_fallback_start_traveled = traveled
                        odom_fallback_start_marker_distance = last_marker_distance
                    fallback_travel = max(0.0, traveled - odom_fallback_start_traveled)
                    estimated_marker_distance = max(
                        0.0,
                        odom_fallback_start_marker_distance - fallback_travel,
                    )
                    remaining_by_marker = max(
                        0.0,
                        estimated_marker_distance - target_distance,
                    )
                    remaining_by_odom = max(
                        0.0,
                        fallback_distance - traveled,
                    )
                    remaining = min(remaining_by_marker, remaining_by_odom)
                    if estimated_marker_distance <= target_distance + lost_success_distance_margin:
                        self.status.publish(
                            "docking_reverse_odom_fallback_reached",
                            traveled=traveled,
                            fallback_distance=fallback_distance,
                            last_marker_distance=last_marker_distance,
                            estimated_marker_distance=estimated_marker_distance,
                        )
                        return
                    reverse_speed = self.clip(
                        remaining * distance_kp,
                        min_reverse_speed,
                        max_reverse_speed,
                    )
                    cmd = Twist()
                    cmd.linear.x = -reverse_speed if speed < 0.0 else reverse_speed
                    if yaw_correction:
                        yaw_error = self.normalize_angle(yaw_target - yaw)
                        cmd.angular.z = self.clip(
                            yaw_kp * yaw_error,
                            -yaw_max_speed,
                            yaw_max_speed,
                        )
                    self.cmd_vel_pub.publish(cmd)
                    rate.sleep()
                    continue
                rate.sleep()
                continue
            missing_since = None
            marker_missing_reported = False
            odom_fallback_reported = False
            odom_fallback_start_traveled = None
            odom_fallback_start_marker_distance = None

            position = marker.pose.pose.position
            marker_distance = self.marker_axis_value(position, distance_axis, absolute=True)
            last_marker_distance = marker_distance
            lateral_error = self.marker_axis_value(position, lateral_axis, absolute=False)
            if marker_distance <= target_distance:
                stable_cycles += 1
                self.publish_stop()
                if stable_cycles >= stable_cycles_required:
                    self.status.publish(
                        "docking_reverse_marker_reached",
                        marker_distance=marker_distance,
                        target_distance=target_distance,
                    )
                    return
                rate.sleep()
                continue
            stable_cycles = 0

            remaining = marker_distance - target_distance
            reverse_speed = self.clip(remaining * distance_kp, min_reverse_speed, max_reverse_speed)
            cmd = Twist()
            cmd.linear.x = -reverse_speed if speed < 0.0 else reverse_speed
            cmd.linear.y = self.clip(
                lateral_kp * lateral_error,
                -lateral_max_speed,
                lateral_max_speed,
            )
            if yaw_correction:
                yaw_error = self.normalize_angle(yaw_target - yaw)
                cmd.angular.z = self.clip(yaw_kp * yaw_error, -yaw_max_speed, yaw_max_speed)
            self.cmd_vel_pub.publish(cmd)
            rate.sleep()

        self.publish_stop()
        self.status.publish(
            "docking_reverse_marker_timeout_stop",
            timeout=timeout,
            last_marker_distance=(
                last_marker_distance
                if last_marker_distance is not None
                else -1.0
            ),
        )
        if timeout_stop_as_success:
            return
        raise RuntimeError("Docking marker-feedback reverse timeout")

    def read_marker_pose_once(self, docking, timeout=0.10):
        topic = docking.get("marker_topic", "/base_camera/ar_pose_marker")
        ar_id = int(docking.get("ar_id", 0))
        try:
            message = rospy.wait_for_message(topic, AlvarMarkers, timeout=timeout)
        except rospy.ROSException:
            return None
        for marker in message.markers:
            if marker.id == ar_id:
                return marker
        return None

    def marker_axis_value(self, position, axis, absolute=False):
        if axis == "x":
            value = float(position.x)
        elif axis == "y":
            value = float(position.y)
        elif axis == "z":
            value = float(position.z)
        else:
            value = math.sqrt(
                float(position.x) ** 2
                + float(position.y) ** 2
                + float(position.z) ** 2
            )
        if absolute:
            return abs(value)
        return value

    def drive_to_rear_lidar_distance(self, docking, speed, timeout):
        topic = docking.get("reverse_lidar_topic", "/scan")
        stop_distance = float(docking.get("reverse_lidar_stop_distance", 0.15))
        tolerance = float(docking.get("reverse_lidar_stop_tolerance", 0.01))
        rear_angle = float(docking.get("reverse_lidar_rear_angle", math.pi))
        angle_window = float(docking.get("reverse_lidar_angle_window", 0.35))
        min_speed = abs(float(docking.get("reverse_lidar_min_speed", 0.015)))
        max_speed = abs(float(docking.get("reverse_lidar_max_speed", abs(speed))))
        kp = float(docking.get("reverse_lidar_kp", 0.8))
        sample_timeout = float(docking.get("reverse_lidar_sample_timeout", 0.2))
        timeout = min(float(docking.get("reverse_lidar_timeout", timeout)), timeout)
        timeout_stop_as_success = bool(
            docking.get("reverse_lidar_timeout_stop_as_success", True)
        )
        deadline = rospy.Time.now() + rospy.Duration(max(0.1, timeout))
        rate = rospy.Rate(20.0)
        self.status.publish(
            "docking_reverse_lidar_fallback",
            topic=topic,
            stop_distance=stop_distance,
            rear_angle=rear_angle,
            angle_window=angle_window,
        )

        while not rospy.is_shutdown() and rospy.Time.now() < deadline:
            self.check_stopped()
            rear_distance = self.read_lidar_distance(
                topic,
                rear_angle,
                angle_window,
                sample_timeout,
            )
            if rear_distance is None:
                self.publish_stop()
                self.status.publish("docking_reverse_lidar_unavailable", topic=topic)
                return False
            if rear_distance <= stop_distance + tolerance:
                self.publish_stop()
                self.status.publish(
                    "docking_reverse_lidar_reached",
                    rear_distance=rear_distance,
                    stop_distance=stop_distance,
                )
                return True
            remaining = rear_distance - stop_distance
            reverse_speed = self.clip(remaining * kp, min_speed, max_speed)
            cmd = Twist()
            cmd.linear.x = -reverse_speed if speed < 0.0 else reverse_speed
            self.cmd_vel_pub.publish(cmd)
            rate.sleep()

        self.publish_stop()
        self.status.publish(
            "docking_reverse_lidar_timeout_stop",
            stop_distance=stop_distance,
        )
        return timeout_stop_as_success

    def read_lidar_distance(self, topic, center_angle, angle_window, timeout):
        try:
            scan = rospy.wait_for_message(topic, LaserScan, timeout=timeout)
        except rospy.ROSException:
            return None
        best = None
        half_window = abs(angle_window) * 0.5
        for index, distance in enumerate(scan.ranges):
            if not math.isfinite(distance):
                continue
            if distance < scan.range_min or distance > scan.range_max:
                continue
            angle = scan.angle_min + index * scan.angle_increment
            if self.angle_distance(angle, center_angle) > half_window:
                continue
            if best is None or distance < best:
                best = float(distance)
        return best

    def drive_linear_distance(self, distance, speed, timeout, docking):
        distance = abs(distance)
        if distance <= 0.0:
            return
        if speed == 0.0:
            raise RuntimeError("Docking reverse speed is zero")
        start_x, start_y, _ = self.current_pose(docking)
        deadline = rospy.Time.now() + rospy.Duration(timeout)
        rate = rospy.Rate(20.0)
        self.status.publish("docking_reverse", distance=distance, speed=speed)

        while not rospy.is_shutdown() and rospy.Time.now() < deadline:
            self.check_stopped()
            x, y, _ = self.current_pose(docking)
            traveled = math.hypot(x - start_x, y - start_y)
            if traveled >= distance:
                self.publish_stop()
                return
            cmd = Twist()
            cmd.linear.x = speed
            self.cmd_vel_pub.publish(cmd)
            rate.sleep()

        self.publish_stop()
        raise RuntimeError("Docking reverse timeout")

    def current_pose(self, docking):
        map_frame = docking.get("map_frame", "map")
        base_frame = docking.get("base_frame", "base_footprint")
        self.tf_listener.waitForTransform(
            map_frame,
            base_frame,
            rospy.Time(0),
            rospy.Duration(3.0),
        )
        translation, rotation = self.tf_listener.lookupTransform(
            map_frame,
            base_frame,
            rospy.Time(0),
        )
        yaw = tf.transformations.euler_from_quaternion(rotation)[2]
        return float(translation[0]), float(translation[1]), yaw

    def publish_stop(self):
        self.cmd_vel_pub.publish(Twist())

    @staticmethod
    def clip(value, low, high):
        return max(low, min(high, value))

    @staticmethod
    def angle_distance(a, b):
        return abs(math.atan2(math.sin(a - b), math.cos(a - b)))

    @staticmethod
    def normalize_angle(angle):
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle
