#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import sys

import yaml


def main():
    config_file = "/home/robot/bobac3_ws/src/home_service_mission/config/home_waypoints.yaml"
    for argument in sys.argv[1:]:
        if argument.startswith("_config_file:="):
            config_file = argument.split(":=", 1)[1]
    if not os.path.isfile(config_file):
        print("HOME_CONFIG_NOT_READY missing_file=" + config_file)
        return 2
    with open(config_file, "r", encoding="utf-8") as stream:
        config = yaml.safe_load(stream)
    missing = []
    for name, point in config["waypoints"].items():
        if point.get("x") is None or point.get("y") is None or point.get("yaw") is None:
            missing.append(name)
    if missing:
        print("HOME_CONFIG_NOT_READY missing_points=" + ",".join(missing))
        return 2
    print("HOME_CONFIG_OK points=" + ",".join(config["waypoints"].keys()))
    return 0


if __name__ == "__main__":
    sys.exit(main())
