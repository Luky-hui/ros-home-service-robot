#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import math

import rospy
from gazebo_msgs.srv import GetModelState
from face_rec.msg import face_data, face_results


def make_face_message():
    message = face_results()
    data = face_data()
    data.header.stamp = rospy.Time.now()
    data.header.frame_id = "sim_face_trigger"
    data.xmin = 220.0
    data.xmax = 420.0
    data.ymin = 80.0
    data.ymax = 320.0
    message.face_data.append(data)
    return message


def main():
    rospy.init_node("sim_face_trigger")
    person_model = rospy.get_param("~person_model", "service_group")
    robot_model = rospy.get_param("~robot_model", "bobac3_serverbot")
    trigger_distance = float(rospy.get_param("~trigger_distance", 1.2))
    publish_when_missing = bool(rospy.get_param("~publish_when_missing", False))
    rate_hz = float(rospy.get_param("~rate", 5.0))

    publisher = rospy.Publisher("/face_detection", face_results, queue_size=10)
    rospy.wait_for_service("/gazebo/get_model_state", timeout=30.0)
    get_model_state = rospy.ServiceProxy("/gazebo/get_model_state", GetModelState)
    rate = rospy.Rate(rate_hz)

    while not rospy.is_shutdown():
        try:
            person = get_model_state(person_model, "world")
            robot = get_model_state(robot_model, "world")
            should_publish = False
            if person.success and robot.success:
                dx = person.pose.position.x - robot.pose.position.x
                dy = person.pose.position.y - robot.pose.position.y
                distance = math.hypot(dx, dy)
                should_publish = distance <= trigger_distance
                rospy.loginfo_throttle(
                    2.0,
                    "sim face trigger distance %.3f m, enabled=%s",
                    distance,
                    should_publish,
                )
            else:
                should_publish = publish_when_missing
            if should_publish:
                publisher.publish(make_face_message())
        except Exception as exc:
            rospy.logwarn_throttle(2.0, "sim face trigger skipped: %s", exc)
        rate.sleep()


if __name__ == "__main__":
    main()
