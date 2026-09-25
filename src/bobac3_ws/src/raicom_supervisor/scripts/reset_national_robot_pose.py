#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import argparse
import math

import rospy
import yaml
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import SetModelState
from geometry_msgs.msg import PoseWithCovarianceStamped
from std_srvs.srv import Empty


DEFAULT_CONFIG = "/home/robot/bobac3_ws/src/home_service_mission/config/national_home_waypoints.yaml"


def parse_args():
    parser = argparse.ArgumentParser(description="Reset national simulation robot pose.")
    parser.add_argument("--config", default=DEFAULT_CONFIG)
    parser.add_argument("--waypoint", default="start")
    parser.add_argument("--model-name", default="bobac3_serverbot")
    parser.add_argument("--z", type=float, default=0.01255)
    parser.add_argument("--frame-id", default="map")
    parser.add_argument("--gazebo-frame", default="world")
    parser.add_argument("--publish-count", type=int, default=10)
    return parser.parse_args(rospy.myargv()[1:])


def load_waypoint(path, name):
    with open(path, "r", encoding="utf-8") as stream:
        config = yaml.safe_load(stream)
    point = config["waypoints"].get(name)
    if not point:
        raise RuntimeError("Waypoint not found in config: %s" % name)
    return point


def yaw_to_quaternion(yaw):
    return 0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0)


def main():
    args = parse_args()
    rospy.init_node("reset_national_robot_pose", anonymous=True)
    point = load_waypoint(args.config, args.waypoint)
    x = float(point["x"])
    y = float(point["y"])
    yaw = float(point["yaw"])
    qx, qy, qz, qw = yaw_to_quaternion(yaw)

    rospy.wait_for_service("/gazebo/set_model_state", timeout=10.0)
    set_model_state = rospy.ServiceProxy("/gazebo/set_model_state", SetModelState)
    state = ModelState()
    state.model_name = args.model_name
    state.reference_frame = args.gazebo_frame
    state.pose.position.x = x
    state.pose.position.y = y
    state.pose.position.z = args.z
    state.pose.orientation.x = qx
    state.pose.orientation.y = qy
    state.pose.orientation.z = qz
    state.pose.orientation.w = qw
    response = set_model_state(state)
    if not response.success:
        raise RuntimeError(response.status_message)

    initial_pub = rospy.Publisher("/initialpose", PoseWithCovarianceStamped, queue_size=1, latch=True)
    initial_pose = PoseWithCovarianceStamped()
    initial_pose.header.frame_id = args.frame_id
    initial_pose.pose.pose.position.x = x
    initial_pose.pose.pose.position.y = y
    initial_pose.pose.pose.orientation.x = qx
    initial_pose.pose.pose.orientation.y = qy
    initial_pose.pose.pose.orientation.z = qz
    initial_pose.pose.pose.orientation.w = qw
    initial_pose.pose.covariance[0] = 0.01
    initial_pose.pose.covariance[7] = 0.01
    initial_pose.pose.covariance[35] = 0.02
    for _ in range(max(1, args.publish_count)):
        initial_pose.header.stamp = rospy.Time.now()
        initial_pub.publish(initial_pose)
        rospy.sleep(0.1)

    try:
        rospy.wait_for_service("/move_base_node/clear_costmaps", timeout=5.0)
        rospy.ServiceProxy("/move_base_node/clear_costmaps", Empty)()
    except Exception as exc:
        rospy.logwarn("clear_costmaps skipped: %s", exc)

    print(
        "RESET_NATIONAL_ROBOT_POSE_DONE waypoint=%s x=%.4f y=%.4f yaw=%.4f"
        % (args.waypoint, x, y, yaw)
    )


if __name__ == "__main__":
    main()
