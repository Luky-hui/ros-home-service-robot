#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import argparse
import math
import os
import random
import shutil
import time

import cv2
import numpy as np
import rospy
import yaml
from cv_bridge import CvBridge
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import DeleteModel, GetModelState, SetModelState, SpawnModel
from geometry_msgs.msg import Pose
from sensor_msgs.msg import Image
from tf.transformations import quaternion_from_euler


CONFIG_FILE = "/home/robot/bobac3_ws/src/service_group_mission/config/service_group_waypoints.yaml"
IMAGE_TOPIC = "/berxel_base/color/image_raw"
ROBOT_MODEL = "bobac3_serverbot"
EVENT_MODEL_NAMES = [
    "service_event_beijing_fire",
    "service_event_beijing_extinguisher",
    "service_event_guangzhou_fire",
    "service_event_guangzhou_extinguisher",
    "service_event_jilin_fire",
    "service_event_jilin_extinguisher",
    "service_event_shanghai_fire",
    "service_event_shanghai_extinguisher",
    "service_event_shenzhen_fire",
    "service_event_shenzhen_extinguisher",
]


def make_pose(x_value, y_value, z_value, yaw_value):
    pose = Pose()
    pose.position.x = x_value
    pose.position.y = y_value
    pose.position.z = z_value
    qx, qy, qz, qw = quaternion_from_euler(0.0, 0.0, yaw_value)
    pose.orientation.x = qx
    pose.orientation.y = qy
    pose.orientation.z = qz
    pose.orientation.w = qw
    return pose


def fire_sdf(scale_value):
    return """<?xml version='1.0'?>
<sdf version='1.7'>
  <model name='fire_hazard'>
    <static>1</static>
    <link name='link'>
      <visual name='visual'>
        <pose>0 0 0 1.57079632679 0 0</pose>
        <geometry>
          <mesh>
            <uri>model://rei_2025raicom/fire_hazard/meshes/fire_hazard.dae</uri>
            <scale>{scale} {scale} {scale}</scale>
          </mesh>
        </geometry>
        <transparency>0</transparency>
        <cast_shadows>0</cast_shadows>
      </visual>
      <collision name='collision'>
        <geometry>
          <box>
            <size>0.14 0.02 0.20</size>
          </box>
        </geometry>
      </collision>
    </link>
  </model>
</sdf>
""".format(scale=scale_value)


def extinguisher_sdf(scale_value):
    return """<?xml version='1.0'?>
<sdf version='1.7'>
  <model name='fire_extinguisher'>
    <static>1</static>
    <link name='link'>
      <visual name='visual'>
        <pose>0 0 0 1.57079632679 0 0</pose>
        <geometry>
          <mesh>
            <uri>model://rei_2025raicom/fire_extinguisher/meshes/fire_extinguisher.dae</uri>
            <scale>{scale} {scale} {scale}</scale>
          </mesh>
        </geometry>
        <transparency>0</transparency>
        <cast_shadows>0</cast_shadows>
      </visual>
      <collision name='collision'>
        <geometry>
          <box>
            <size>0.08 0.02 0.20</size>
          </box>
        </geometry>
      </collision>
    </link>
  </model>
</sdf>
""".format(scale=scale_value)


def delete_events(delete_model):
    for model_name in EVENT_MODEL_NAMES:
        try:
            delete_model(model_name)
        except Exception:
            pass


def set_robot_pose(set_model_state, get_model_state, waypoint, rng):
    state = get_model_state(ROBOT_MODEL, "world")
    yaw_value = float(waypoint["yaw"]) + rng.uniform(-0.055, 0.055)
    x_value = float(waypoint.get("nav_x", waypoint["x"])) + rng.uniform(-0.045, 0.045)
    y_value = float(waypoint.get("nav_y", waypoint["y"])) + rng.uniform(-0.045, 0.045)
    model_state = ModelState()
    model_state.model_name = ROBOT_MODEL
    model_state.reference_frame = "world"
    model_state.pose = make_pose(x_value, y_value, state.pose.position.z, yaw_value)
    result = set_model_state(model_state)
    if not result.success:
        raise RuntimeError("set_model_state failed: %s" % result.status_message)
    return yaw_value


def event_pose(waypoint, rng, kind):
    yaw_value = float(waypoint["yaw"])
    front_x = math.cos(yaw_value)
    front_y = math.sin(yaw_value)
    right_x = math.sin(yaw_value)
    right_y = -math.cos(yaw_value)
    if kind == "fire":
        forward = rng.uniform(0.30, 0.42)
        lateral = rng.uniform(-0.20, 0.05)
        z_value = rng.uniform(0.10, 0.15)
        object_yaw = yaw_value + math.pi / 2.0
    else:
        forward = rng.uniform(0.34, 0.48)
        lateral = rng.uniform(0.02, 0.24)
        z_value = rng.uniform(0.09, 0.15)
        object_yaw = yaw_value + math.pi / 2.0
    x_value = float(waypoint["x"]) + front_x * forward + right_x * lateral
    y_value = float(waypoint["y"]) + front_y * forward + right_y * lateral
    object_yaw += rng.uniform(-0.06, 0.06)
    return make_pose(x_value, y_value, z_value, object_yaw)


def wait_image(bridge, image_topic):
    image_msg = None
    for _ in range(4):
        image_msg = rospy.wait_for_message(image_topic, Image, timeout=5.0)
        rospy.sleep(0.05)
    return bridge.imgmsg_to_cv2(image_msg, desired_encoding="bgr8")


def diff_bbox(baseline, current):
    diff = cv2.absdiff(baseline, current)
    gray = cv2.cvtColor(diff, cv2.COLOR_BGR2GRAY)
    _, mask = cv2.threshold(gray, 12, 255, cv2.THRESH_BINARY)
    kernel = np.ones((5, 5), dtype=np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
    component_count, _, stats, _ = cv2.connectedComponentsWithStats(mask, 8)
    boxes = []
    for index in range(1, component_count):
        x_value, y_value, width, height, area = stats[index]
        if area < 80:
            continue
        if width < 8 or height < 8:
            continue
        boxes.append((x_value, y_value, x_value + width, y_value + height, area))
    if not boxes:
        return None
    x1 = min(box[0] for box in boxes)
    y1 = min(box[1] for box in boxes)
    x2 = max(box[2] for box in boxes)
    y2 = max(box[3] for box in boxes)
    height, width = current.shape[:2]
    pad = 4
    x1 = max(0, x1 - pad)
    y1 = max(0, y1 - pad)
    x2 = min(width - 1, x2 + pad)
    y2 = min(height - 1, y2 + pad)
    if x2 <= x1 or y2 <= y1:
        return None
    return x1, y1, x2, y2


def yolo_line(class_id, bbox, image_shape):
    height, width = image_shape[:2]
    x1, y1, x2, y2 = bbox
    cx = ((x1 + x2) / 2.0) / float(width)
    cy = ((y1 + y2) / 2.0) / float(height)
    bw = (x2 - x1) / float(width)
    bh = (y2 - y1) / float(height)
    return "%d %.6f %.6f %.6f %.6f" % (class_id, cx, cy, bw, bh)


def split_name(index, train_ratio):
    return "train" if (index % 10) < int(train_ratio * 10.0) else "val"


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--out", default="/home/robot/ros_workspace/datasets/raicom_service_yolo")
    parser.add_argument("--samples-per-kind", type=int, default=18)
    parser.add_argument("--seed", type=int, default=20260702)
    parser.add_argument("--clean", action="store_true")
    args = parser.parse_args()

    rospy.init_node("collect_service_yolo_dataset", anonymous=True)
    rng = random.Random(args.seed)
    with open(CONFIG_FILE, "r", encoding="utf-8") as stream:
        config = yaml.safe_load(stream)
    venues = list(config["missions"]["patrol"]["route"])
    waypoints = config["waypoints"]

    if args.clean and os.path.isdir(args.out):
        shutil.rmtree(args.out)
    for split in ["train", "val"]:
        os.makedirs(os.path.join(args.out, "images", split), exist_ok=True)
        os.makedirs(os.path.join(args.out, "labels", split), exist_ok=True)

    rospy.wait_for_service("/gazebo/delete_model", timeout=15.0)
    rospy.wait_for_service("/gazebo/spawn_sdf_model", timeout=15.0)
    rospy.wait_for_service("/gazebo/set_model_state", timeout=15.0)
    rospy.wait_for_service("/gazebo/get_model_state", timeout=15.0)
    delete_model = rospy.ServiceProxy("/gazebo/delete_model", DeleteModel)
    spawn_model = rospy.ServiceProxy("/gazebo/spawn_sdf_model", SpawnModel)
    set_model_state = rospy.ServiceProxy("/gazebo/set_model_state", SetModelState)
    get_model_state = rospy.ServiceProxy("/gazebo/get_model_state", GetModelState)
    bridge = CvBridge()

    sample_index = 0
    kept = 0
    for venue in venues:
        waypoint = waypoints[venue]
        for kind in ["fire", "extinguisher", "empty"]:
            for _ in range(args.samples_per_kind):
                delete_events(delete_model)
                set_robot_pose(set_model_state, get_model_state, waypoint, rng)
                time.sleep(0.30)
                baseline = wait_image(bridge, IMAGE_TOPIC)
                label_lines = []
                if kind != "empty":
                    model_name = "service_event_%s_%s" % (
                        venue,
                        "fire" if kind == "fire" else "extinguisher",
                    )
                    sdf_xml = (
                        fire_sdf(rng.uniform(3.4, 4.4))
                        if kind == "fire"
                        else extinguisher_sdf(rng.uniform(4.0, 5.4))
                    )
                    response = spawn_model(
                        model_name,
                        sdf_xml,
                        "",
                        event_pose(waypoint, rng, kind),
                        "world",
                    )
                    if not response.success:
                        raise RuntimeError(response.status_message)
                    time.sleep(0.25)
                image = wait_image(bridge, IMAGE_TOPIC)
                if kind != "empty":
                    bbox = diff_bbox(baseline, image)
                    if bbox is None:
                        rospy.logwarn("skip %s %s: no diff bbox", venue, kind)
                        delete_events(delete_model)
                        continue
                    label_lines.append(yolo_line(0 if kind == "fire" else 1, bbox, image.shape))

                split = split_name(sample_index, 0.8)
                image_name = "%05d_%s_%s.jpg" % (sample_index, venue, kind)
                label_name = image_name.replace(".jpg", ".txt")
                cv2.imwrite(os.path.join(args.out, "images", split, image_name), image)
                with open(os.path.join(args.out, "labels", split, label_name), "w", encoding="utf-8") as stream:
                    stream.write("\n".join(label_lines))
                    if label_lines:
                        stream.write("\n")
                sample_index += 1
                kept += 1
                delete_events(delete_model)
                rospy.loginfo("saved %s %s %s", split, venue, kind)

    data_yaml = os.path.join(args.out, "data.yaml")
    with open(data_yaml, "w", encoding="utf-8") as stream:
        stream.write("path: %s\n" % args.out)
        stream.write("train: images/train\n")
        stream.write("val: images/val\n")
        stream.write("names:\n")
        stream.write("  0: fire\n")
        stream.write("  1: fire_extinguisher\n")
    print("DATASET", args.out, "IMAGES", kept, "YAML", data_yaml)


if __name__ == "__main__":
    main()
