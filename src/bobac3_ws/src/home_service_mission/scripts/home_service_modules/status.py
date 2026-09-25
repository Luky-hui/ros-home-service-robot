#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import json

import rospy
from std_msgs.msg import String


class StatusPublisher:
    def __init__(self, topic, mission_getter):
        self.publisher = rospy.Publisher(topic, String, queue_size=10, latch=True)
        self.mission_getter = mission_getter

    def publish(self, state, **details):
        message = {"state": state, "mission": self.mission_getter()}
        message.update(details)
        self.publisher.publish(String(json.dumps(message, ensure_ascii=False, sort_keys=True)))
        rospy.loginfo("HOME_STATUS %s", message)
