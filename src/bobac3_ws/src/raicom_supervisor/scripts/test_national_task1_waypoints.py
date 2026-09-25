#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import argparse
import csv
import json
import math
import os
import time

import actionlib
import rospy
import tf
import yaml
from actionlib_msgs.msg import GoalStatus
from dynamic_reconfigure.msg import DoubleParameter
from dynamic_reconfigure.srv import Reconfigure, ReconfigureRequest
from geometry_msgs.msg import Quaternion, Twist
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal


DEFAULT_CONFIG = "/home/robot/bobac3_ws/src/home_service_mission/config/national_home_waypoints.yaml"
DEFAULT_OUTPUT = "/tmp/raicom_task1_waypoint_test.csv"
DEFAULT_ROUTE = "corridor,restaurant,kitchen,living_room,bedroom,start"
SCORE_BOX_SIZE_METERS = 0.50


def yaw_to_quaternion(yaw):
    return Quaternion(0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0))


def quaternion_to_yaw(rotation):
    return tf.transformations.euler_from_quaternion(rotation)[2]


def normalize_angle(angle):
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


def load_config(path):
    with open(path, "r", encoding="utf-8") as stream:
        return yaml.safe_load(stream)


def lookup_pose(listener, map_frame, base_frame):
    listener.waitForTransform(map_frame, base_frame, rospy.Time(0), rospy.Duration(8.0))
    translation, rotation = listener.lookupTransform(map_frame, base_frame, rospy.Time(0))
    return float(translation[0]), float(translation[1]), quaternion_to_yaw(rotation)


def send_goal(client, point, yaw=None):
    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = point["frame_id"]
    goal.target_pose.header.stamp = rospy.Time.now()
    goal.target_pose.pose.position.x = float(point["x"])
    goal.target_pose.pose.position.y = float(point["y"])
    if yaw is None:
        yaw = float(point["yaw"])
    goal.target_pose.pose.orientation = yaw_to_quaternion(float(yaw))
    client.send_goal(goal)


def set_dwa_double(name, value):
    service_name = "/move_base_node/DWAPlannerROS/set_parameters"
    rospy.wait_for_service(service_name, timeout=10.0)
    request = ReconfigureRequest()
    request.config.doubles.append(DoubleParameter(name=name, value=float(value)))
    service = rospy.ServiceProxy(service_name, Reconfigure)
    service(request)


def inside_axis_aligned_box(dx, dy, box_size):
    half = box_size / 2.0
    return abs(dx) <= half and abs(dy) <= half


def clear_of_box_edge(dx, dy, box_size, robot_radius):
    half = box_size / 2.0
    margin = half - robot_radius
    return abs(dx) <= margin and abs(dy) <= margin


def parse_args():
    parser = argparse.ArgumentParser(description="Test national task-1 waypoint stop accuracy.")
    parser.add_argument("--config", default=DEFAULT_CONFIG)
    parser.add_argument("--route", default=DEFAULT_ROUTE)
    parser.add_argument("--output", default=DEFAULT_OUTPUT)
    parser.add_argument("--map-frame", default="map")
    parser.add_argument("--base-frame", default="base_footprint")
    parser.add_argument("--action-server", default="/move_base")
    parser.add_argument("--goal-timeout", type=float, default=120.0)
    parser.add_argument("--settle-seconds", type=float, default=5.0)
    parser.add_argument("--robot-radius", type=float, default=0.18)
    parser.add_argument("--score-box-size", type=float, default=SCORE_BOX_SIZE_METERS)
    parser.add_argument(
        "--position-only",
        action="store_true",
        help="Test waypoint center accuracy without requiring the final stored yaw.",
    )
    parser.add_argument(
        "--yaw-goal-tolerance-override",
        type=float,
        default=0.0,
        help="Temporarily set DWAPlannerROS yaw_goal_tolerance while this test runs.",
    )
    return parser.parse_args(rospy.myargv()[1:])


def main():
    args = parse_args()
    rospy.init_node("test_national_task1_waypoints", anonymous=True)
    config = load_config(args.config)
    waypoints = config["waypoints"]
    route = [name.strip() for name in args.route.split(",") if name.strip()]
    for name in route:
        if name not in waypoints:
            raise RuntimeError("Waypoint not found in config: %s" % name)

    cmd_vel_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
    listener = tf.TransformListener()
    client = actionlib.SimpleActionClient(args.action_server, MoveBaseAction)
    if not client.wait_for_server(rospy.Duration(30.0)):
        raise RuntimeError("move_base action server is not ready: %s" % args.action_server)

    original_yaw_tolerance = None
    if args.position_only and args.yaw_goal_tolerance_override <= 0.0:
        args.yaw_goal_tolerance_override = math.pi
    if args.yaw_goal_tolerance_override > 0.0:
        original_yaw_tolerance = rospy.get_param(
            "/move_base_node/DWAPlannerROS/yaw_goal_tolerance", None
        )
        print(
            "DWA_YAW_GOAL_TOLERANCE_SET old=%s new=%.4f"
            % (original_yaw_tolerance, args.yaw_goal_tolerance_override),
            flush=True,
        )
        set_dwa_double("yaw_goal_tolerance", args.yaw_goal_tolerance_override)

    os.makedirs(os.path.dirname(args.output), exist_ok=True)
    fields = [
        "position_only",
        "yaw_goal_tolerance_override",
        "waypoint",
        "goal_x",
        "goal_y",
        "goal_yaw",
        "final_x",
        "final_y",
        "final_yaw",
        "dx",
        "dy",
        "distance_error",
        "yaw_error",
        "move_base_state",
        "center_inside_50cm_box",
        "estimated_no_line_touch",
        "edge_clearance_x",
        "edge_clearance_y",
    ]
    try:
        with open(args.output, "w", encoding="utf-8", newline="") as stream:
            writer = csv.DictWriter(stream, fieldnames=fields)
            writer.writeheader()
            for name in route:
                point = waypoints[name]
                goal_x = float(point["x"])
                goal_y = float(point["y"])
                goal_yaw = float(point["yaw"])
                requested_yaw = goal_yaw
                if args.position_only:
                    current_x, current_y, current_yaw = lookup_pose(
                        listener, args.map_frame, args.base_frame
                    )
                    requested_yaw = current_yaw
                    print(
                        "POSITION_ONLY_REFERENCE current=(%.4f, %.4f, %.4f)"
                        % (current_x, current_y, current_yaw),
                        flush=True,
                    )
                print(
                    "TEST_WAYPOINT_START %s goal=(%.4f, %.4f, %.4f) requested_yaw=%.4f"
                    % (name, goal_x, goal_y, goal_yaw, requested_yaw),
                    flush=True,
                )
                send_goal(client, point, yaw=requested_yaw)
                if not client.wait_for_result(rospy.Duration(args.goal_timeout)):
                    client.cancel_goal()
                    state = "TIMEOUT"
                else:
                    state = GoalStatus.to_string(client.get_state())
                cmd_vel_pub.publish(Twist())
                settle_until = time.time() + args.settle_seconds
                while not rospy.is_shutdown() and time.time() < settle_until:
                    cmd_vel_pub.publish(Twist())
                    rospy.sleep(0.2)
                final_x, final_y, final_yaw = lookup_pose(listener, args.map_frame, args.base_frame)
                dx = final_x - goal_x
                dy = final_y - goal_y
                yaw_error = normalize_angle(final_yaw - goal_yaw)
                distance_error = math.hypot(dx, dy)
                half = args.score_box_size / 2.0
                edge_clearance_x = half - abs(dx) - args.robot_radius
                edge_clearance_y = half - abs(dy) - args.robot_radius
                row = {
                    "position_only": args.position_only,
                    "yaw_goal_tolerance_override": args.yaw_goal_tolerance_override,
                    "waypoint": name,
                    "goal_x": goal_x,
                    "goal_y": goal_y,
                    "goal_yaw": goal_yaw,
                    "final_x": final_x,
                    "final_y": final_y,
                    "final_yaw": final_yaw,
                    "dx": dx,
                    "dy": dy,
                    "distance_error": distance_error,
                    "yaw_error": yaw_error,
                    "move_base_state": state,
                    "center_inside_50cm_box": inside_axis_aligned_box(dx, dy, args.score_box_size),
                    "estimated_no_line_touch": clear_of_box_edge(
                        dx, dy, args.score_box_size, args.robot_radius
                    ),
                    "edge_clearance_x": edge_clearance_x,
                    "edge_clearance_y": edge_clearance_y,
                }
                writer.writerow(row)
                stream.flush()
                print(
                    "TEST_WAYPOINT_RESULT " + json.dumps(row, ensure_ascii=False, sort_keys=True),
                    flush=True,
                )
    finally:
        if original_yaw_tolerance is not None:
            print(
                "DWA_YAW_GOAL_TOLERANCE_RESTORE value=%s" % original_yaw_tolerance,
                flush=True,
            )
            set_dwa_double("yaw_goal_tolerance", original_yaw_tolerance)
    print("TASK1_WAYPOINT_TEST_DONE output=%s" % args.output, flush=True)


if __name__ == "__main__":
    main()
