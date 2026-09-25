#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import SetModelState
from geometry_msgs.msg import Twist
from tf.transformations import quaternion_from_euler


SCENE_POSES = {
    # name: (x, y, z, roll, pitch, yaw)
    "service_group_map": (1.5, 1.05, 0.0, 0.0, 0.0, 0.0),
    "enclosure": (3.0, 2.57, 0.005, -0.000522, -0.000522, -1.57),
    "jilin": (2.967, 2.2, 0.25, 1.57, 0.0, -1.57),
    "guangzhou": (2.967, 1.2, 0.25, 1.57, 0.0, -1.57),
    "beijing": (2.967, 0.2, 0.25, 1.57, 0.0, -1.57),
    "shanghai": (1.467, 2.2, 0.25, 1.57, 0.0, -1.57),
    "shenzhen": (1.467, 1.2, 0.25, 1.57, 0.0, -1.57),
    "reinovo_raicom_final": (1.6, 1.245, 0.0, 0.0, 0.0, 0.0),
}


ROBOT_POSE = {
    "bobac3_serverbot": (1.485, 1.254, 0.02, 0.0, 0.0, 0.0),
}


def make_state(model_name, pose_tuple):
    x, y, z, roll, pitch, yaw = pose_tuple
    qx, qy, qz, qw = quaternion_from_euler(roll, pitch, yaw)

    state = ModelState()
    state.model_name = model_name
    state.reference_frame = "world"
    state.pose.position.x = x
    state.pose.position.y = y
    state.pose.position.z = z
    state.pose.orientation.x = qx
    state.pose.orientation.y = qy
    state.pose.orientation.z = qz
    state.pose.orientation.w = qw
    state.twist = Twist()
    return state


def main():
    rospy.init_node("reset_service_scene")
    include_robot = rospy.get_param("~include_robot", False)

    rospy.wait_for_service("/gazebo/set_model_state", timeout=10.0)
    set_model_state = rospy.ServiceProxy("/gazebo/set_model_state", SetModelState)

    poses = dict(SCENE_POSES)
    if include_robot:
        poses.update(ROBOT_POSE)

    failed = []
    for model_name, pose in poses.items():
        try:
            response = set_model_state(make_state(model_name, pose))
            if not response.success:
                failed.append("%s: %s" % (model_name, response.status_message))
                rospy.logwarn("Reset failed for %s: %s", model_name, response.status_message)
            else:
                rospy.loginfo("Reset %s", model_name)
        except rospy.ServiceException as exc:
            failed.append("%s: %s" % (model_name, exc))
            rospy.logwarn("Reset service error for %s: %s", model_name, exc)

    if failed:
        raise RuntimeError("Some models failed to reset: " + "; ".join(failed))

    rospy.loginfo("Service-group scene reset finished")


if __name__ == "__main__":
    main()
