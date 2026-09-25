#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import argparse
import math
import os

import rospy
import tf
import yaml


POINT_NAMES = [
    "start",
    "corridor",
    "restaurant",
    "kitchen",
    "living_room",
    "bedroom",
    "charge",
]


def quaternion_to_yaw(quaternion):
    return tf.transformations.euler_from_quaternion(quaternion)[2]


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("name", choices=POINT_NAMES)
    parser.add_argument(
        "--config",
        default="/home/robot/bobac3_ws/src/home_service_mission/config/home_waypoints.yaml",
    )
    parser.add_argument("--map-frame", default="map")
    parser.add_argument("--base-frame", default="base_footprint")
    arguments = parser.parse_args(rospy.myargv()[1:])

    if not os.path.isfile(arguments.config):
        raise RuntimeError("Config file not found: %s" % arguments.config)

    rospy.init_node("capture_home_waypoint", anonymous=True)
    listener = tf.TransformListener()
    listener.waitForTransform(
        arguments.map_frame,
        arguments.base_frame,
        rospy.Time(0),
        rospy.Duration(10.0),
    )
    translation, rotation = listener.lookupTransform(
        arguments.map_frame,
        arguments.base_frame,
        rospy.Time(0),
    )
    yaw = quaternion_to_yaw(rotation)

    with open(arguments.config, "r", encoding="utf-8") as stream:
        config = yaml.safe_load(stream)
    point = config["waypoints"][arguments.name]
    point["frame_id"] = arguments.map_frame
    point["x"] = round(float(translation[0]), 4)
    point["y"] = round(float(translation[1]), 4)
    point["yaw"] = round(float(yaw), 4)
    temporary = arguments.config + ".tmp"
    with open(temporary, "w", encoding="utf-8") as stream:
        yaml.safe_dump(config, stream, allow_unicode=True, sort_keys=False)
    os.replace(temporary, arguments.config)
    print(
        "HOME_WAYPOINT_SAVED name=%s x=%.4f y=%.4f yaw=%.4f"
        % (arguments.name, point["x"], point["y"], point["yaw"])
    )


if __name__ == "__main__":
    main()
