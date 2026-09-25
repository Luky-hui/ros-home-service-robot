#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import threading

import rospy
from actionlib_msgs.msg import GoalID
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool, String
from std_srvs.srv import SetBool, SetBoolResponse


class RaicomSupervisor:
    def __init__(self):
        rospy.init_node("raicom_supervisor")
        self.lock = threading.Lock()
        self.stopped = False
        self.stop_pub = rospy.Publisher(
            "/raicom/emergency_stop", Bool, queue_size=10, latch=True
        )
        self.state_pub = rospy.Publisher(
            "/raicom/supervisor_state", String, queue_size=10, latch=True
        )
        self.cmd_vel_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
        self.move_base_cancel_pub = rospy.Publisher(
            "/move_base/cancel", GoalID, queue_size=10
        )
        self.move_base_node_cancel_pub = rospy.Publisher(
            "/move_base_node/cancel", GoalID, queue_size=10
        )
        self.service = rospy.Service(
            "/raicom/emergency_stop/set", SetBool, self.set_stop
        )
        self.timer = rospy.Timer(rospy.Duration(0.05), self.enforce_stop)
        self.publish_state()

    def set_stop(self, request):
        with self.lock:
            self.stopped = bool(request.data)
            if self.stopped:
                self.cancel_navigation()
                self.cmd_vel_pub.publish(Twist())
            self.publish_state()
        message = "emergency_stop_active" if self.stopped else "emergency_stop_released"
        return SetBoolResponse(success=True, message=message)

    def cancel_navigation(self):
        cancel = GoalID()
        self.move_base_cancel_pub.publish(cancel)
        self.move_base_node_cancel_pub.publish(cancel)

    def enforce_stop(self, _event):
        with self.lock:
            if self.stopped:
                self.cmd_vel_pub.publish(Twist())
                self.stop_pub.publish(Bool(True))

    def publish_state(self):
        self.stop_pub.publish(Bool(self.stopped))
        self.state_pub.publish(
            String("emergency_stop_active" if self.stopped else "ready")
        )


if __name__ == "__main__":
    RaicomSupervisor()
    rospy.spin()
