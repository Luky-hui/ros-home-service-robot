#! /usr/bin/env python3
# -*- coding: utf-8 -*

import os
import cv2
import numpy as np
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import face_recognition
from face_rec.srv import *
from face_rec.msg import face_data,face_results
import PIL
from PIL import ImageFont,ImageDraw

class Face_Rec():
    def __init__(self):
        """初始化所有参数"""
        rospy.init_node("face_rec_service") # 初始化节点
        self.known_face_encodings = np.load('face_encodeing.npz')['encoding']
        self.known_face_names = np.load('face_encodeing.npz')['labels']
        self.face_data = rospy.get_param('~face_data', '') # 人脸图片目录
        self.tolerance = rospy.get_param('~tolerance', '')  # 人脸比对容差值
        self.camera_topic = rospy.get_param('~camera_topic', '') # 人脸图片目录
        self.image_encoding = rospy.get_param('~image_encoding', '') # 输入图像编码格式
        self.bridgr = CvBridge()
        self.face_recsrv = rospy.Service('face_recognition_results', recognition_results,self.detect_callback) 
        rospy.spin()

    def detect_callback(self, req):
        results = face_results()
        if req.mode == 1:
            try:
                image = rospy.wait_for_message(req.str,Image)
                frame = self.bridgr.imgmsg_to_cv2(image,self.image_encoding)
                return self.generate_srv(frame)
            except:
                rospy.logwarn(f"Camera topic {req.str} subscription failed")
                return recognition_resultsResponse(results,"Camera topic subscription failed",False)
        elif req.mode == 0:
            image = rospy.wait_for_message(self.camera_topic,Image,timeout=None)
            frame = self.bridgr.imgmsg_to_cv2(image,self.image_encoding)
            return self.generate_srv(frame)
        elif req.mode ==2:
            try:
                rospy.loginfo("Image path :%s",req.str)
                frame = cv2.imread(req.str,1)
                if(frame.date==None):
                    rospy.logwarn("Image path is empty")
                    return recognition_resultsResponse(results,"Image path is empty",False)
                else:  
                    return self.generate_srv(frame)
            except:
                rospy.logwarn("Image path error")
                return recognition_resultsResponse(results,"Image path error",False)

        else:
            return recognition_resultsResponse(results,False)
    
    def generate_srv(self,frame):
        print('-----------------------------------------------')
        print('Start recognizing faces.')
        face_locations = [] # 检测到的未知人脸列表
        face_encodings = [] # 未知人脸编码列表
        face_names = [] # 实时标签列表列表
        process_this_frame = True 
        results = face_results()
        # 将视频帧大小调整为1/4大小，以加快人脸识别处理速度
        small_frame = cv2.resize(frame, (0, 0), fx=0.25, fy=0.25)
        # 将图像从 BGR 颜色（OpenCV 使用）转换为 RGB 颜色（face_recognition使用）
        rgb_small_frame = cv2.cvtColor(small_frame, cv2.COLOR_BGR2RGB)
        if process_this_frame == True: # 间隔
            # 查找当前视频帧中的所有人脸
            face_locations = face_recognition.face_locations(rgb_small_frame)       
            # 把查找到人脸进行编码
            face_encodings = face_recognition.face_encodings(rgb_small_frame,face_locations)
            face_names = []
            rospy.loginfo("Number of faces detected:%s",len(face_encodings))
            for face_encoding in face_encodings:
                # 将检测到的人脸和已知人脸库中的图片比较
                name = "Unknown"
                # 计算检测到的人脸和已知人脸的误差
                face_distances = face_recognition.face_distance(self.known_face_encodings,face_encoding)
                matches = face_recognition.compare_faces(self.known_face_encodings, face_encoding)
                # 得到误差最小的人脸在已知列表中的位置
                best_match_index = np.argmin(face_distances)
                matches = face_recognition.compare_faces(self.known_face_encodings, face_encoding,self.tolerance)
                if matches[best_match_index]: # 如果对比结果为True
                    name = self.known_face_names[best_match_index] # 为检测到的人脸做标签
                face_names.append(name)

        process_this_frame = not process_this_frame       
        
        for (top, right, bottom, left), name in zip(face_locations, face_names):
            data = face_data()
            data.header.frame_id = name
            data.xmin = left*4
            data.xmax = right*4
            data.ymin = top*4
            data.ymax = bottom*4
            rospy.loginfo("detected:%s",name)
            results.face_data.append(data)
        print('-----------------------------------------------')
        return recognition_resultsResponse(results,"",True)

if __name__=="__main__":
    Face_Rec()
