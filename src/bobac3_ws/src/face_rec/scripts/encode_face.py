#! /usr/bin/env python3
# -*- coding: utf-8 -*

import numpy as np
import os
import face_recognition
import rospy

def face_load(face_dir):
    """加载图像并学习如何识别它，并添加到已知人脸库"""
    face_encodings = []
    face_names = []
    for name in os.listdir(face_dir):
        file = os.path.join(face_dir,name)
        rospy.loginfo("进入目录'%s'",file)
        rospy.loginfo("添加'%s'的人脸数据",name)
        for img in os.listdir(file):
            path = os.path.join(file,img)
            for type in ['jpg', 'png', 'jpeg', 'tif', 'bmp','_..no']:
                if img.split('.')[-1] == type:
                    break
            if type != '_..no':
                new_image = face_recognition.load_image_file(os.path.join(file,img)) # 将图片转换为numpy数组
                new_face_encoding = face_recognition.face_encodings(new_image) # 得到人脸编码
                if len(new_face_encoding) == 1:
                    face_encodings.append(new_face_encoding[0]) # 添加到已知人脸库
                    face_names.append(name) # 添加人脸名称
                    rospy.loginfo("'%s':成功添加'%s'的人脸数据",img,name)
                elif len(new_face_encoding) > 0:
                    rospy.logerr("'%s':中检测到多个人脸",img)
                else:
                    rospy.logerr("'%s':未检测到人脸数据",img)
            else:
                rospy.logerr("'%s':错误的文件格式",img)
    return face_encodings,face_names

if __name__=="__main__":
    rospy.init_node("encode_face")
    face_encodings,face_names = face_load(rospy.get_param("~face_data",''))
    np.savez("face_encodeing.npz",encoding=face_encodings,labels=face_names)