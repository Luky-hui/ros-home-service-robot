#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import math
import os
import random

import rospy
import yaml
from gazebo_msgs.srv import DeleteModel, SpawnModel
from geometry_msgs.msg import Pose
from tf.transformations import quaternion_from_euler


MODEL_ROOT = "/home/robot/.gazebo/models/rei_2025raicom"
PRECOMPUTED_EVENTS_FILE = "/tmp/raicom_service_events.yaml"
EVENT_MODEL_NAMES = {
    "fire": "fire_hazard",
    "extinguisher": "fire_extinguisher",
}
DEFAULT_ROUTE = ["jilin", "guangzhou", "beijing", "shanghai", "shenzhen"]
DEFAULT_EVENTS = {
    "jilin": "fire",
    "guangzhou": "normal",
    "beijing": "extinguisher_missing",
    "shanghai": "normal",
    "shenzhen": "normal",
}


def load_yaml(path):
    with open(path, "r", encoding="utf-8") as stream:
        return yaml.safe_load(stream)


def read_sdf(model_name):
    sdf_path = os.path.join(MODEL_ROOT, model_name, "model.sdf")
    with open(sdf_path, "r", encoding="utf-8") as stream:
        return stream.read()


def make_fire_sdf():
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


def make_extinguisher_sdf():
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


def scale_visual_mesh(sdf_xml, scale):
    scale_text = "%.3f %.3f %.3f" % (scale, scale, scale)
    return sdf_xml.replace("<scale>1 1 1</scale>", "<scale>%s</scale>" % scale_text, 1)


def make_pose(x, y, z, yaw):
    pose = Pose()
    pose.position.x = x
    pose.position.y = y
    pose.position.z = z
    qx, qy, qz, qw = quaternion_from_euler(0.0, 0.0, yaw)
    pose.orientation.x = qx
    pose.orientation.y = qy
    pose.orientation.z = qz
    pose.orientation.w = qw
    return pose


def object_pose(waypoint, forward, lateral, z, yaw_offset):
    yaw = float(waypoint["yaw"])
    front_x = math.cos(yaw)
    front_y = math.sin(yaw)
    right_x = math.sin(yaw)
    right_y = -math.cos(yaw)
    x = float(waypoint["x"]) + front_x * forward + right_x * lateral
    y = float(waypoint["y"]) + front_y * forward + right_y * lateral
    return make_pose(x, y, z, yaw + yaw_offset)


def resolve_events(route):
    reuse_precomputed = rospy.get_param("~reuse_precomputed_events", True)
    if reuse_precomputed and os.path.exists(PRECOMPUTED_EVENTS_FILE):
        with open(PRECOMPUTED_EVENTS_FILE, "r", encoding="utf-8") as stream:
            precomputed = yaml.safe_load(stream) or {}
        if all(venue in precomputed for venue in route):
            rospy.loginfo("Using precomputed service events from %s", PRECOMPUTED_EVENTS_FILE)
            return {venue: precomputed.get(venue, "normal") for venue in route}

    mode = rospy.get_param("~event_mode", "random")
    if mode == "fixed":
        return dict(DEFAULT_EVENTS)

    if mode == "random":
        seed = rospy.get_param("~random_seed", "")
        rng = random.Random(str(seed)) if seed != "" else random.Random()
        events = {venue: "normal" for venue in route}
        if not route:
            return events
        fire_venue = rng.choice(route)
        missing_choices = [venue for venue in route if venue != fire_venue] or list(route)
        missing_venue = rng.choice(missing_choices)
        events[fire_venue] = "fire"
        events[missing_venue] = "extinguisher_missing"
        for venue in route:
            if venue in (fire_venue, missing_venue):
                continue
            events[venue] = rng.choice(["normal", "normal", "fire", "extinguisher_missing"])
        return events

    raise ValueError("Unsupported event_mode: %s" % mode)


def delete_existing(delete_model, route):
    names = []
    for venue in route:
        names.append("service_event_%s_fire" % venue)
        names.append("service_event_%s_extinguisher" % venue)
    for name in names:
        try:
            delete_model(name)
        except rospy.ServiceException:
            pass


def spawn(spawn_model, model_name, sdf_xml, pose):
    response = spawn_model(model_name, sdf_xml, "", pose, "world")
    if not response.success:
        raise RuntimeError("%s: %s" % (model_name, response.status_message))
    rospy.loginfo("Spawned %s", model_name)


def main():
    rospy.init_node("spawn_service_events")

    config_file = rospy.get_param(
        "~config_file",
        "/home/robot/bobac3_ws/src/service_group_mission/config/service_group_waypoints.yaml",
    )
    config = load_yaml(config_file)
    route = list(config["missions"]["patrol"].get("route", DEFAULT_ROUTE))
    waypoints = config["waypoints"]

    fire_sdf = make_fire_sdf()
    extinguisher_sdf = make_extinguisher_sdf()

    forward = float(rospy.get_param("~forward", 0.62))
    fire_forward = float(rospy.get_param("~fire_forward", 0.35))
    extinguisher_forward = float(rospy.get_param("~extinguisher_forward", forward))
    fire_lateral = float(rospy.get_param("~fire_lateral", -0.15))
    extinguisher_lateral = float(rospy.get_param("~extinguisher_lateral", 0.15))
    fire_z = float(rospy.get_param("~fire_z", 0.02))
    extinguisher_z = float(rospy.get_param("~extinguisher_z", 0.02))
    fire_yaw_offset = float(rospy.get_param("~fire_yaw_offset", math.pi / 2.0))
    extinguisher_yaw_offset = float(
        rospy.get_param("~extinguisher_yaw_offset", math.pi / 2.0)
    )

    events = resolve_events(route)

    rospy.wait_for_service("/gazebo/delete_model", timeout=20.0)
    rospy.wait_for_service("/gazebo/spawn_sdf_model", timeout=20.0)
    delete_model = rospy.ServiceProxy("/gazebo/delete_model", DeleteModel)
    spawn_model = rospy.ServiceProxy("/gazebo/spawn_sdf_model", SpawnModel)

    delete_existing(delete_model, route)

    for venue in route:
        event = events.get(venue, "normal")
        waypoint = waypoints[venue]
        rospy.loginfo("Service event %s: %s", venue, event)

        if event == "fire":
            spawn(
                spawn_model,
                "service_event_%s_fire" % venue,
                fire_sdf,
                object_pose(
                    waypoint, fire_forward, fire_lateral, fire_z, fire_yaw_offset
                ),
            )
        elif event == "normal":
            spawn(
                spawn_model,
                "service_event_%s_extinguisher" % venue,
                extinguisher_sdf,
                object_pose(
                    waypoint,
                    extinguisher_forward,
                    extinguisher_lateral,
                    extinguisher_z,
                    extinguisher_yaw_offset,
                ),
            )
        elif event == "extinguisher_missing":
            continue
        else:
            raise ValueError("Unsupported service event: %s" % event)

    rospy.set_param("/service_group_mission/simulation_events", events)
    rospy.loginfo("Service inspection events spawned: %s", events)


if __name__ == "__main__":
    main()
