#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from sensor_msgs.msg import Image
import time
import sys

class BerxelBaseColorTester:
    def __init__(self):
        rospy.init_node("berxel_base_color_tester", anonymous=True)

        self.rgb_count = 0
        self.prev_rgb_count = 0

        # 订阅 berxel_base_color 的 RGB 图像话题
        self.rgb_topic = "/bobac3_base/rgb/rgb_raw"
        rospy.Subscriber(self.rgb_topic, Image, self.rgb_callback)

        rospy.loginfo(f"已启动 Berxel Base Color 图像接收测试节点，订阅话题: {self.rgb_topic}")

    def rgb_callback(self, msg):
        self.rgb_count += 1

    def run(self, duration_sec=11):
        warmup_time = 1  # 跳过前1秒启动阶段
        start_time = time.time()

        rate = rospy.Rate(1)  # 每秒打印一次
        printed_seconds = 0
        warmup_done = False

        rgb_base = 0

        max_no_data_seconds = 3  # 连续几秒无数据则判失败
        no_data_counter = 0

        while not rospy.is_shutdown():
            elapsed = time.time() - start_time
            if elapsed >= duration_sec:
                break

            if elapsed >= warmup_time and not warmup_done:
                rospy.loginfo("开始正式统计帧数（已跳过前 1 秒）...")
                rgb_base = self.rgb_count
                warmup_done = True
                printed_seconds = 0

            if warmup_done:
                rgb_delta = self.rgb_count - self.prev_rgb_count

                if rgb_delta == 0:
                    no_data_counter += 1
                else:
                    no_data_counter = 0

                if no_data_counter >= max_no_data_seconds:
                    rospy.logerr(f"❌ 连续{max_no_data_seconds}秒未接收到任何图像数据，测试失败！")
                    sys.exit(1)

                rospy.loginfo(f"[第 {printed_seconds + 1} 秒] 当前 RGB 帧数：{self.rgb_count}（+{rgb_delta}/s）")
                printed_seconds += 1

            self.prev_rgb_count = self.rgb_count

            rate.sleep()

        rospy.loginfo("测试结束 ✅")
        total_rgb = self.rgb_count - rgb_base
        rospy.loginfo(f"总共接收到 RGB 图像帧数：{total_rgb}，平均帧率：{total_rgb / 10:.2f} FPS")

if __name__ == "__main__":
    tester = BerxelBaseColorTester()
    tester.run(duration_sec=11)  # 实际统计后10秒
