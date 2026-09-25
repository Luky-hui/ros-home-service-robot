#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from sensor_msgs.msg import Image
import time
import sys

class DualCameraTester:
    def __init__(self):
        rospy.init_node("dual_camera_tester", anonymous=True)

        self.base_count = 0
        self.prev_base_count = 0
        self.head_count = 0
        self.prev_head_count = 0

        rospy.Subscriber("/base_camera/image_raw", Image, self.base_callback)
        rospy.Subscriber("/head_camera/image_raw", Image, self.head_callback)

        rospy.loginfo("已启动基础摄像头和头部摄像头图像接收测试节点...")

    def base_callback(self, msg):
        self.base_count += 1

    def head_callback(self, msg):
        self.head_count += 1

    def run(self, duration_sec=11):
        warmup_time = 1  # 跳过前1秒启动阶段
        start_time = time.time()

        rate = rospy.Rate(1)  # 每秒打印一次
        printed_seconds = 0
        warmup_done = False

        base_start = 0
        head_start = 0

        # 连续多少秒没有接收到帧时判定失败
        max_no_data_seconds = 3
        no_data_counter = 0

        while not rospy.is_shutdown():
            elapsed = time.time() - start_time
            if elapsed >= duration_sec:
                break

            if elapsed >= warmup_time and not warmup_done:
                rospy.loginfo("开始正式统计帧数（已跳过前 1 秒）...")
                base_start = self.base_count
                head_start = self.head_count
                warmup_done = True
                printed_seconds = 0

            if warmup_done:
                base_delta = self.base_count - self.prev_base_count
                head_delta = self.head_count - self.prev_head_count

                # 判断是否有新数据
                if base_delta == 0 and head_delta == 0:
                    no_data_counter += 1
                else:
                    no_data_counter = 0  # 收到数据，重置计数

                if no_data_counter >= max_no_data_seconds:
                    rospy.logerr(f"❌ 连续{max_no_data_seconds}秒未接收到任何图像数据，测试失败！")
                    sys.exit(1)

                rospy.loginfo(f"[第 {printed_seconds + 1} 秒] /base_camera 当前帧数：{self.base_count}（+{base_delta}/s），/head_camera 当前帧数：{self.head_count}（+{head_delta}/s）")
                printed_seconds += 1

            self.prev_base_count = self.base_count
            self.prev_head_count = self.head_count

            rate.sleep()

        rospy.loginfo("测试结束 ✅")
        total_base = self.base_count - base_start
        total_head = self.head_count - head_start
        rospy.loginfo(f"/base_camera 总共接收到帧数：{total_base}，平均帧率：{total_base / 10:.2f} FPS")
        rospy.loginfo(f"/head_camera 总共接收到帧数：{total_head}，平均帧率：{total_head / 10:.2f} FPS")


if __name__ == "__main__":
    tester = DualCameraTester()
    tester.run(duration_sec=11)
