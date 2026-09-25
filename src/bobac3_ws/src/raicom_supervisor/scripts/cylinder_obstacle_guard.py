#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import math
import time

import rospy
import tf
import yaml
from actionlib_msgs.msg import GoalID
from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import Twist


class CylinderObstacleGuard:
    def __init__(self):
        self.config_file = rospy.get_param("~config_file", "")
        self.options = self.load_options(self.config_file)
        self.enabled = bool(self.options.get("enabled", True))
        self.target_model = str(self.options.get("target_model", "cylinder_obstacle"))
        self.robot_model = str(self.options.get("robot_model", "bobac3_serverbot"))
        self.engage_distance = float(
            self.options.get("guard_engage_distance", self.options.get("engage_distance", 0.65))
        )
        self.resume_distance = float(
            self.options.get("guard_resume_distance", self.options.get("resume_distance", 0.75))
        )
        self.yield_speed = float(
            self.options.get("guard_yield_speed", self.options.get("yield_speed", 0.12))
        )
        self.reverse_only = bool(self.options.get("guard_reverse_only", True))
        self.lateral_scale = float(self.options.get("guard_lateral_scale", 0.0))
        self.rate_hz = float(self.options.get("guard_rate", 50.0))
        self.cancel_rate_hz = float(self.options.get("guard_cancel_rate", 10.0))
        self.stale_after = float(self.options.get("stale_after", 0.6))

        self.cmd_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
        self.cancel_pubs = [
            rospy.Publisher("/move_base/cancel", GoalID, queue_size=1),
            rospy.Publisher("/move_base_node/cancel", GoalID, queue_size=1),
        ]
        self.last_cancel_time = 0.0
        self.last_log_time = 0.0
        self.last_sample_time = 0.0
        self.distance = None
        self.bearing = None
        self.active = False

        topic = str(self.options.get("model_states_topic", "/gazebo/model_states"))
        rospy.Subscriber(topic, ModelStates, self.model_states_cb, queue_size=1)
        rospy.logwarn(
            "CYLINDER_OBSTACLE_GUARD started enabled=%s target=%s robot=%s engage=%.3f resume=%.3f speed=%.3f rate=%.1f reverse_only=%s lateral_scale=%.2f",
            self.enabled,
            self.target_model,
            self.robot_model,
            self.engage_distance,
            self.resume_distance,
            self.yield_speed,
            self.rate_hz,
            self.reverse_only,
            self.lateral_scale,
        )

    def load_options(self, path):
        if not path:
            return {}
        try:
            with open(path, encoding="utf-8") as f:
                data = yaml.safe_load(f) or {}
            return data.get("navigation", {}).get("dynamic_obstacle_stop", {}) or {}
        except Exception as exc:
            rospy.logwarn("CYLINDER_OBSTACLE_GUARD config load failed: %s", exc)
            return {}

    @staticmethod
    def angle_diff(a, b):
        return math.atan2(math.sin(a - b), math.cos(a - b))

    def model_states_cb(self, msg):
        if self.target_model not in msg.name or self.robot_model not in msg.name:
            self.distance = None
            self.bearing = None
            return
        robot_pose = msg.pose[msg.name.index(self.robot_model)]
        target_pose = msg.pose[msg.name.index(self.target_model)]
        robot_x = float(robot_pose.position.x)
        robot_y = float(robot_pose.position.y)
        obstacle_x = float(target_pose.position.x)
        obstacle_y = float(target_pose.position.y)
        q = robot_pose.orientation
        robot_yaw = tf.transformations.euler_from_quaternion([q.x, q.y, q.z, q.w])[2]
        dx = obstacle_x - robot_x
        dy = obstacle_y - robot_y
        self.distance = math.hypot(dx, dy)
        self.bearing = self.angle_diff(math.atan2(dy, dx), robot_yaw)
        self.last_sample_time = time.time()

    def publish_cancel(self):
        if self.cancel_rate_hz <= 0.0:
            return
        now = time.time()
        if now - self.last_cancel_time < 1.0 / max(1.0, self.cancel_rate_hz):
            return
        self.last_cancel_time = now
        msg = GoalID()
        msg.stamp = rospy.Time.now()
        for pub in self.cancel_pubs:
            pub.publish(msg)

    def build_avoid_twist(self):
        if self.bearing is None or self.yield_speed <= 0.0:
            return None
        twist = Twist()
        if self.reverse_only:
            twist.linear.x = -self.yield_speed
            twist.linear.y = -math.sin(self.bearing) * self.yield_speed * self.lateral_scale
        else:
            twist.linear.x = -math.cos(self.bearing) * self.yield_speed
            twist.linear.y = -math.sin(self.bearing) * self.yield_speed
        return twist

    def log_active(self, twist):
        now = time.time()
        if now - self.last_log_time < 0.5:
            return
        self.last_log_time = now
        rospy.logwarn(
            "CYLINDER_OBSTACLE_GUARD active distance=%s bearing=%s vx=%.3f vy=%.3f",
            "%.3f" % self.distance if self.distance is not None else "None",
            "%.3f" % self.bearing if self.bearing is not None else "None",
            twist.linear.x,
            twist.linear.y,
        )

    def spin(self):
        rate = rospy.Rate(self.rate_hz)
        while not rospy.is_shutdown():
            if not self.enabled:
                rate.sleep()
                continue
            fresh = (
                self.distance is not None
                and time.time() - self.last_sample_time <= self.stale_after
            )
            if fresh and (self.active or self.distance <= self.engage_distance):
                self.active = self.distance < self.resume_distance
                if self.active:
                    twist = self.build_avoid_twist()
                    if twist is not None:
                        self.publish_cancel()
                        self.cmd_pub.publish(twist)
                        self.log_active(twist)
            else:
                self.active = False
            rate.sleep()


def main():
    rospy.init_node("cylinder_obstacle_guard")
    CylinderObstacleGuard().spin()


if __name__ == "__main__":
    main()
