#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from sensor_msgs.msg import LaserScan
import time
import sys

class LidarTester:
    def __init__(self):
        rospy.init_node("lidar_tester", anonymous=True)

        self.front_count = 0
        self.rear_count = 0
        self.prev_front_count = 0
        self.prev_rear_count = 0

        rospy.Subscriber("/front_lidar/scan", LaserScan, self.front_callback)
        rospy.Subscriber("/rear_lidar/scan", LaserScan, self.rear_callback)

        rospy.loginfo("已启动前后激光雷达帧率测试节点...")

    def front_callback(self, msg):
        self.front_count += 1

    def rear_callback(self, msg):
        self.rear_count += 1

    def run(self, duration_sec=11):
        warmup_time = 1
        start_time = time.time()
        rate = rospy.Rate(1)

        front_start = 0
        rear_start = 0
        printed_seconds = 0
        warmup_done = False
        no_data_counter = 0
        max_no_data_seconds = 3

        while not rospy.is_shutdown():
            elapsed = time.time() - start_time
            if elapsed >= duration_sec:
                break

            if elapsed >= warmup_time and not warmup_done:
                rospy.loginfo("开始正式统计帧数（已跳过前 1 秒）...")
                front_start = self.front_count
                rear_start = self.rear_count
                warmup_done = True
                printed_seconds = 0

            if warmup_done:
                front_delta = self.front_count - self.prev_front_count
                rear_delta = self.rear_count - self.prev_rear_count

                if front_delta == 0 and rear_delta == 0:
                    no_data_counter += 1
                else:
                    no_data_counter = 0

                if no_data_counter >= max_no_data_seconds:
                    rospy.logerr(f"❌ 连续 {max_no_data_seconds} 秒未接收到激光雷达数据，测试失败！")
                    sys.exit(1)

                rospy.loginfo(f"[第 {printed_seconds + 1} 秒] /front_lidar 当前帧数：{self.front_count}（+{front_delta}/s），/rear_lidar 当前帧数：{self.rear_count}（+{rear_delta}/s）")
                printed_seconds += 1

            self.prev_front_count = self.front_count
            self.prev_rear_count = self.rear_count

            rate.sleep()

        rospy.loginfo("测试结束 ✅")
        total_front = self.front_count - front_start
        total_rear = self.rear_count - rear_start
        rospy.loginfo(f"/front_lidar 总共接收到帧数：{total_front}，平均帧率：{total_front / 10:.2f} FPS")
        rospy.loginfo(f"/rear_lidar 总共接收到帧数：{total_rear}，平均帧率：{total_rear / 10:.2f} FPS")


if __name__ == "__main__":
    tester = LidarTester()
    tester.run(duration_sec=11)
