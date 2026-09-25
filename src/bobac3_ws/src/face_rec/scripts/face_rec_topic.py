#! /usr/bin/env python3
# -*- coding: utf-8 -*

import os
import cv2
import numpy as np
import rospy
from sensor_msgs.msg import Image,CameraInfo
from cv_bridge import CvBridge
import face_recognition
from face_rec.msg import face_data, face_results
import PIL
from PIL import ImageFont,ImageDraw
from threading import Thread

class Face_Rec():
    def __init__(self):
        """初始化所有参数"""
        rospy.init_node("face_rec_topic") # 初始化节点
        self.tolerance = rospy.get_param('~tolerance', '')  # 人脸比对容差值
        self.face_data = rospy.get_param('~face_data', '') # 人脸图片目录
        self.image_encoding = rospy.get_param('~image_encoding', '') # 输入图像编码格式
        self.box = []
        self.box_sub = []
        self.bridgr = CvBridge()
        camera_topic = rospy.get_param('~camera_topic', '') # 人脸图片目录
        self.known_face_names = np.load('face_encodeing.npz')['labels']
        print(self.known_face_names)
        self.known_face_encodings = np.load('face_encodeing.npz')['encoding']
        self.pub_data = rospy.Publisher("/face_detection", face_results, queue_size=10) # 发布人脸数据
        self.pub_img = rospy.Publisher("/image_raw/face_rec", Image, queue_size=10) # 设置发布话题名、类型、设置队列个数
        self.sub_img = rospy.Subscriber(camera_topic,Image,self.image_callback) # 订阅摄像头节点
        self.sub_test = rospy.Subscriber("/head_camera/camera_info",CameraInfo,self.test_callback)
        rospy.spin()


    def test_callback(self,data):
        box = self.box
        i_index = 0
        for rec in box:
            rospy.sleep(1)
            if rec["label"] == "未检测" or rec["label"]=="等待检测" or rec["label"]=="unknow":
                face_encodings = face_recognition.face_encodings(rec["image"],[rec["box"]])
                rospy.loginfo("检测到的人脸数:%s",len(face_encodings))
                face_encoding = face_encodings[0]
                print('-----------------1------------------')
                # 将检测到的人脸和已知人脸库中的图片比较
                name = "unknow"
                # 计算检测到的人脸和已知人脸的误差
                face_distances = face_recognition.face_distance(self.known_face_encodings,face_encoding)
                print('-----------------2------------------')
                matches = face_recognition.compare_faces(self.known_face_encodings, face_encoding)
                # 得到误差最小的人脸在已知列表中的位置
                best_match_index = np.argmin(face_distances)
                matches = face_recognition.compare_faces(self.known_face_encodings, face_encoding,self.tolerance)
                print(matches)
                if matches[best_match_index]: # 如果对比结果为True
                    name = self.known_face_names[best_match_index] # 为检测到的人脸做标签
                print(f"-----------------{name}-----------------------")
                box[i_index]["label"]=name
                # print(box)
                i_index += 1
        self.box_sub = box

    def image_callback(self, image): 
        # 将消息转为bgr格式
        frame = self.bridgr.imgmsg_to_cv2(image,self.image_encoding)
        # 将视频帧大小调整为1/4大小，以加快人脸识别处理速度
        small_frame = cv2.resize(frame, (0, 0), fx=0.25, fy=0.25)
        # 将图像从 BGR 颜色（OpenCV 使用）转换为 RGB 颜色（face_recognition使用）
        rgb_small_frame = cv2.cvtColor(small_frame, cv2.COLOR_BGR2RGB)
        # if process_this_frame == True: # 间隔
            # 查找当前视频帧中的所有人脸
        face_locations = face_recognition.face_locations(rgb_small_frame) 
            # rospy.loginfo("检测到的人脸数:%s",len(face_locations))
        # 定义发布消息
        # process_this_frame = not process_this_frame 
        self.image_rendering(frame,face_locations,rgb_small_frame)
        
    def computing_center(self,top, right, bottom, left):
        x = top-bottom
        y = right-left
        return x,y

    def image_rendering(self,frame,face_locations,rgb):
        box_sub = self.box_sub
        results = face_results()
        draw = dict()
        id = rospy.Time.now()
        print(len(face_locations))
        print(len(self.box))
        if len(face_locations) == 0:
            self.box=[]
        if len(face_locations) >= len(self.box):
            print("----------------------------------------")
            for (top, right, bottom, left) in face_locations:
                # 在面部画一个框，放大备份人脸位置，因为我们检测到的帧被缩放到1/4大小
                x,y = self.computing_center(top, right, bottom, left)
                i = 0
                sw = False
                for box in self.box:
                    top1, right1, bottom1, left1 = box["box"]
                    x1,y1 = self.computing_center(top1, right1, bottom1, left1) 
                    if abs(x-x1) < 20 and abs(y-y1)< 20:
                        self.box[i]["box"]=(top,right,bottom,left)#[top,right,bottom,left]
                        self.box[i]["image"]=rgb
                        for sub in box_sub:
                            if sub["id"] == box['id']:
                                self.box[i]["label"]=sub["label"]
                            else:
                                # self.box[i]["box"]=(top,right,bottom,left)#[top,right,bottom,left]
                                self.box[i]["label"]="等待检测"
                                # self.box.pop(i)
                        sw = True
                    # else:
                    
                    # elif box["id"] == box['id']:
                    #     self.box.pop(i)
                    i += 1
                if sw != True:
                    draw["id"] = id
                    draw["box"]=(top,right,bottom,left)#[top,right,bottom,left]
                    draw["label"]="未检测"
                    draw["image"] = rgb
                    self.box.append(draw)
                    # id_list = [id["id"] for id in self.box]
                    # if 
                    
        for box in self.box:
            top3, right3, bottom3, left3 = box["box"]
            top3 *= 4
            right3 *=4
            bottom3 *=4
            left3 *=4
            cv2.rectangle(frame, (left3,top3),(right3,bottom3),(100,100,255),2)
            # 在人脸下方写上标签
            cv2.rectangle(frame, (left3, bottom3 - 35), (right3, bottom3), (0, 0, 255), cv2.FILLED)
            frame = self.paint_chinese_opencv(frame, str(box["label"]), (left3 + 6, bottom3 - 40), (255,255,255))

        img = self.bridgr.cv2_to_imgmsg(frame,self.image_encoding) # 把OpenCV图像转换为ROS消息
        self.pub_img.publish(img) # 发布图像到话题

    def paint_chinese_opencv(self,im,chinese,pos,color):
        img_PIL = PIL.Image.fromarray(cv2.cvtColor(im,cv2.COLOR_BGR2RGB))
        font = ImageFont.truetype('NotoSansCJK-Bold.ttc',30)
        fillColor = color #(255,0,0)
        position = pos #(100,100)
        draw = ImageDraw.Draw(img_PIL)
        draw.text(position,chinese,font=font,fill=fillColor)
        img = cv2.cvtColor(np.asarray(img_PIL),cv2.COLOR_RGB2BGR)
        return img


if __name__=="__main__":
    Face_Rec()





# #! /usr/bin/env python3
# # -*- coding: utf-8 -*

# import os
# import cv2
# import numpy as np
# import rospy
# from sensor_msgs.msg import Image
# from cv_bridge import CvBridge
# import face_recognition
# from face_rec.msg import face_data, face_results
# import PIL
# from PIL import ImageFont,ImageDraw

# class Face_Rec():
#     def __init__(self):
#         """初始化所有参数"""
#         rospy.init_node("face_rec_topic") # 初始化节点
#         self.known_face_encodings = np.load('face_encodeing.npz')['encoding']
#         self.known_face_names = np.load('face_encodeing.npz')['labels']
#         self.tolerance = rospy.get_param('~tolerance', '')  # 人脸比对容差值
#         self.face_data = rospy.get_param('~face_data', '') # 人脸图片目录
#         camera_topic = rospy.get_param('~camera_topic', '') # 人脸图片目录
#         self.pub_data = rospy.Publisher("/face_results", face_results, queue_size=10) # 发布人脸数据
#         self.pub_img = rospy.Publisher("/camera/face_recognition", Image, queue_size=10) # 设置发布话题名、类型、设置队列个数       
#         self.sub_img = rospy.Subscriber(camera_topic,Image,self.image_callback) # 订阅摄像头节点
#         rospy.spin()
        
#     def image_callback(self, image):
#         self.bridgr = CvBridge()
#         # 将消息转为bgr格式
#         frame = self.bridgr.imgmsg_to_cv2(image,'bgr8')
#         face_locations = [] # 检测到的未知人脸列表
#         face_encodings = [] # 未知人脸编码列表
#         face_names = [] # 实时标签列表列表
#         process_this_frame = True 
#         # 将视频帧大小调整为1/4大小，以加快人脸识别处理速度
#         small_frame = cv2.resize(frame, (0, 0), fx=0.25, fy=0.25)
#         # 将图像从 BGR 颜色（OpenCV 使用）转换为 RGB 颜色（face_recognition使用）
#         rgb_small_frame = cv2.cvtColor(small_frame, cv2.COLOR_BGR2RGB)
#         if process_this_frame == True: # 间隔
#             # 查找当前视频帧中的所有人脸
#             face_locations = face_recognition.face_locations(rgb_small_frame)       
#             # 把查找到人脸进行编码
#             face_encodings = face_recognition.face_encodings(rgb_small_frame,face_locations)
#             face_names = []
#             rospy.loginfo("检测到的人脸数:%s",len(face_encodings))
#             for face_encoding in face_encodings:
#                 # 将检测到的人脸和已知人脸库中的图片比较
#                 name = "Unknown"
#                 # 计算检测到的人脸和已知人脸的误差
#                 face_distances = face_recognition.face_distance(self.known_face_encodings,face_encoding)
#                 matches = face_recognition.compare_faces(self.known_face_encodings, face_encoding,self.tolerance)
#                 # 得到误差最小的人脸在已知列表中的位置
#                 best_match_index = np.argmin(face_distances)
#                 print(face_distances.shape)
#                 # print(len(matches))
#                 if matches[best_match_index]: # 如果对比结果为True
#                     name = self.known_face_names[best_match_index] # 为检测到的人脸做标签
#                 face_names.append(name)
#         process_this_frame = not process_this_frame

#         # 定义发布消息
#         results = face_results()
#         results.num = len(face_names)

#         for (top, right, bottom, left), name in zip(face_locations, face_names):
#             # 在面部画一个框，放大备份人脸位置，因为我们检测到的帧被缩放到1/4大小
#             top *= 4
#             right *=4
#             bottom *=4
#             left *=4
#             cv2.rectangle(frame, (left,top),(right,bottom),(0,0,255),2)
#             # 在人脸下方写上标签
#             cv2.rectangle(frame, (left, bottom - 35), (right, bottom), (0, 0, 255), cv2.FILLED)
#             font = cv2.FONT_HERSHEY_DUPLEX
#             # cv2.putText(frame,name,(left - 10, bottom - 10), font, 1.0,(255,255,255), 1)
#             frame = self.paint_chinese_opencv(frame, name, (left + 6, bottom - 40), (255,255,255))

#             # 定义发布消息
#             data = face_data()
#             data.name = name
#             data.xmin = left
#             data.xmax = right
#             data.ymin = top
#             data.ymax = bottom
#             results.face_data.append(data)

#         img = self.bridgr.cv2_to_imgmsg(frame,"bgr8") # 把OpenCV图像转换为ROS消息
#         self.pub_data.publish(results)
#         self.pub_img.publish(img) # 发布图像到话题


#     def face_load(self):
#         """加载图像并学习如何识别它，并添加到已知人脸库"""
#         for name in os.listdir(self.face_data):
#             rospy.loginfo("添加'%s'的人脸数据",name)
#             file = os.path.join(self.face_data,name)
#             for img in os.listdir(file):
#                 new_image = face_recognition.load_image_file(os.path.join(file,img)) # 将图片转换为numpy数组
#                 new_face_encoding = face_recognition.face_encodings(new_image) # 得到人脸编码
#                 if len(new_face_encoding) > 0:
#                     self.known_face_encodings.append(new_face_encoding) # 添加到已知人脸库
#                     self.known_face_names.append(name) # 添加人脸名称
#                 else:
#                     rospy.loginfo(f'图像{self.face_data}{img}未检测到人脸数据，请更换图像')

#     def paint_chinese_opencv(self,im,chinese,pos,color):
#         img_PIL = PIL.Image.fromarray(cv2.cvtColor(im,cv2.COLOR_BGR2RGB))
#         font = ImageFont.truetype('NotoSansCJK-Bold.ttc',30)
#         fillColor = color #(255,0,0)
#         position = pos #(100,100)
#         draw = ImageDraw.Draw(img_PIL)
#         draw.text(position,chinese,font=font,fill=fillColor)
    
#         img = cv2.cvtColor(np.asarray(img_PIL),cv2.COLOR_RGB2BGR)
#         return img


# if __name__=="__main__":
#     Face_Rec()
