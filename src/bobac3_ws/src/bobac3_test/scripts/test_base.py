#!/usr/bin/env python3
# coding: utf-8

import rospy
from nav_msgs.msg import Odometry
import time
import sys

class OdomCounter:
    def __init__(self):
        self.count = 0
        self.start_time = None
        self.last_msg_time = None
        rospy.init_node('odom_frequency_test', anonymous=True)
        rospy.Subscriber("/odom", Odometry, self.callback)
        rospy.loginfo("已启动底盘 /odom 话题监听节点...")

    def callback(self, msg):
        if self.start_time is None:
            self.start_time = time.time()
        self.count += 1
        self.last_msg_time = time.time()  # 记录最后一次收到消息的时间

    def run(self, duration=10):
        rospy.loginfo("准备统计 /odom 发布频率（共 %d 秒）..." % duration)
        rospy.sleep(1)
        rospy.loginfo("开始统计...")

        max_no_msg_sec = 3  # 连续几秒无消息则判定失败
        last_count = 0

        for i in range(1, duration + 1):
            rospy.sleep(1)
            now_count = self.count
            delta = now_count - last_count
            last_count = now_count

            # 检查是否超时无消息
            if self.last_msg_time is not None:
                no_msg_time = time.time() - self.last_msg_time
                if no_msg_time > max_no_msg_sec:
                    rospy.logerr(f"❌ 连续超过{max_no_msg_sec}秒未接收到 /odom 消息，测试失败！")
                    sys.exit(1)
            else:
                # 尚未收到任何消息，等同超时
                rospy.logerr(f"❌ 尚未接收到任何 /odom 消息，测试失败！")
                sys.exit(1)

            rospy.loginfo("[第 %d 秒] 当前帧数：%d（+%d/s）", i, now_count, delta)

        total_time = time.time() - self.start_time
        fps = self.count / total_time if total_time > 0 else 0
        rospy.loginfo("统计结束 ✅")
        rospy.loginfo("总帧数：%d，平均频率：%.2f Hz", self.count, fps)

if __name__ == '__main__':
    try:
        tester = OdomCounter()
        tester.run()
    except rospy.ROSInterruptException:
        pass
