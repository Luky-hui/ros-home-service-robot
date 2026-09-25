#! /usr/bin/env python3
# -*- coding: utf-8 -*

import os
import cv2
import numpy as np
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import face_recognition
from face_rec.msg import face_data, face_results
import PIL
from PIL import ImageFont,ImageDraw

class Face_Rec():
    def __init__(self):
        """初始化所有参数"""
        rospy.init_node("face_rec_topic") # 初始化节点
        self.tolerance = rospy.get_param('~tolerance', '')  # 人脸比对容差值
        self.face_data = rospy.get_param('~face_data', '') # 人脸图片目录
        camera_topic = rospy.get_param('~camera_topic', '') # 人脸图片目录
        self.pub_data = rospy.Publisher("/face_detection", face_results, queue_size=10) # 发布人脸数据
        self.pub_img = rospy.Publisher("/image_raw/face_datection", Image, queue_size=10) # 设置发布话题名、类型、设置队列个数
        self.sub_img = rospy.Subscriber(camera_topic,Image,self.image_callback) # 订阅摄像头节点
        rospy.spin()

    def image_callback(self, image):        
        bridgr = CvBridge()
        # 将消息转为bgr格式
        frame = bridgr.imgmsg_to_cv2(image,'bgr8')
        
        process_this_frame = True 
        # 将视频帧大小调整为1/4大小，以加快人脸识别处理速度
        small_frame = cv2.resize(frame, (0, 0), fx=0.25, fy=0.25)
        # 将图像从 BGR 颜色（OpenCV 使用）转换为 RGB 颜色（face_recognition使用）
        rgb_small_frame = cv2.cvtColor(small_frame, cv2.COLOR_BGR2RGB)
        if process_this_frame == True: # 间隔
            # 查找当前视频帧中的所有人脸
            face_locations = face_recognition.face_locations(rgb_small_frame)
            if face_locations:
                rospy.loginfo_throttle(1.0, "检测到的人脸数:%s", len(face_locations))
        # 定义发布消息
        process_this_frame = not process_this_frame 
        results = face_results()
        for (top, right, bottom, left) in face_locations:
            # 在面部画一个框，放大备份人脸位置，因为我们检测到的帧被缩放到1/4大小
            top *= 4
            right *=4
            bottom *=4
            left *=4
            cv2.rectangle(frame, (left,top),(right,bottom),(100,100,255),2)
            # 定义发布消息
            data = face_data()
            data.header.stamp = image.header.stamp
            data.xmin = left
            data.xmax = right
            data.ymin = top
            data.ymax = bottom
            results.face_data.append(data)
        img = bridgr.cv2_to_imgmsg(frame,"bgr8") # 把OpenCV图像转换为ROS消息
        self.pub_data.publish(results)
        self.pub_img.publish(img) # 发布图像到话题

if __name__=="__main__":
    Face_Rec()
