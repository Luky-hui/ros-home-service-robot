#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import json
import os
import math
from geometry_msgs.msg import PoseWithCovarianceStamped
from visualization_msgs.msg import MarkerArray, Marker
from std_srvs.srv import Empty, EmptyResponse
from tf.transformations import euler_from_quaternion

class PoseSaver:
    def __init__(self):
        rospy.init_node('pose_saver', anonymous=True)
        
        
        # 参数
        map_file = rospy.get_param('~map_file', 'src/bobac/maps/saved_poses.json')
        self.json_file = map_file.replace(".yaml", "_point.json")
        
        # 当前位姿
        self.current_pose = None
        
        # 缓存的位姿数据
        self.cached_poses = []
        
        # 订阅amcl_pose
        self.pose_sub = rospy.Subscriber('/amcl_pose', PoseWithCovarianceStamped, self.pose_callback)
        
        # 发布标记
        self.marker_pub = rospy.Publisher('/saved_poses_markers', MarkerArray, queue_size=1, latch=True)
        
        # 服务
        self.save_service = rospy.Service('/save_pose', Empty, self.save_pose_callback)
        self.clear_service = rospy.Service('/clear_poses', Empty, self.clear_poses_callback)
        self.load_service = rospy.Service('/load_poses', Empty, self.load_poses_callback)
        
        # 定时发布标记
        self.marker_timer = rospy.Timer(rospy.Duration(10.0), self.publish_markers)
        
        rospy.loginfo("位姿保存器已启动")
        rospy.loginfo("JSON文件: %s", self.json_file)
        rospy.loginfo("服务:")
        rospy.loginfo("  /save_pose - 保存当前位姿")
        rospy.loginfo("  /clear_poses - 清除所有位姿")
        rospy.loginfo("  /load_poses - 重新加载位姿")
        
        # 加载现有位姿
        self.load_poses()
        
    def pose_callback(self, msg):
        """接收AMCL位姿"""
        self.current_pose = msg
        
    def quaternion_to_yaw(self, orientation):
        """四元数转yaw角"""
        quaternion = (
            orientation.x,
            orientation.y,
            orientation.z,
            orientation.w
        )
        euler = euler_from_quaternion(quaternion)
        return euler[2]  # yaw角
        
    def load_poses(self):
        """从JSON文件加载位姿"""
        try:
            # 确保目录存在
            os.makedirs(os.path.dirname(self.json_file), exist_ok=True)
            
            if os.path.exists(self.json_file):
                with open(self.json_file, 'r', encoding='utf-8') as f:
                    data = json.load(f)
                    self.cached_poses = data.get('points', [])
                    rospy.loginfo("加载了 %d 个已保存的位姿", len(self.cached_poses))
                    # 打印加载的位姿信息
                    for pose in self.cached_poses:
                        rospy.loginfo("位姿 ID=%s: x=%.3f, y=%.3f, theta=%.3f", 
                                    pose['id'], pose['x'], pose['y'], pose['theta'])
            else:
                self.cached_poses = []
                rospy.loginfo("JSON文件不存在，将创建新文件")
                
        except Exception as e:
            rospy.logerr("加载位姿失败: %s", str(e))
            self.cached_poses = []
            
    def save_poses(self, points):
        """保存位姿到JSON文件"""
        try:
            # 确保目录存在
            os.makedirs(os.path.dirname(self.json_file), exist_ok=True)
            
            data = {"points": points}
            with open(self.json_file, 'w', encoding='utf-8') as f:
                json.dump(data, f, indent=3, ensure_ascii=False)
            
            # 更新缓存
            self.cached_poses = points
            rospy.loginfo("位姿已保存到: %s", self.json_file)
            return True
        except Exception as e:
            rospy.logerr("保存位姿失败: %s", str(e))
            return False
            
    def get_existing_poses(self):
        """获取现有位姿"""
        return self.cached_poses[:]
            
    def save_pose_callback(self, req):
        """保存位姿服务回调"""
        if self.current_pose is None:
            rospy.logwarn("没有接收到当前位姿，无法保存")
            return EmptyResponse()
            
        # 获取现有位姿
        existing_poses = self.get_existing_poses()
        
        # 确定新的ID
        if existing_poses:
            max_id = max(int(point['id']) for point in existing_poses)
            new_id = str(max_id + 1)
        else:
            new_id = "0"
            
        # 提取位姿信息
        pose = self.current_pose.pose.pose
        x = pose.position.x
        y = pose.position.y
        theta = self.quaternion_to_yaw(pose.orientation)
        
        # 创建新的位姿点
        new_point = {
            "id": new_id,
            "point_type": 0,
            "theta": theta,
            "x": x,
            "y": y
        }
        
        # 添加到现有位姿
        existing_poses.append(new_point)
        
        # 保存到文件
        if self.save_poses(existing_poses):
            rospy.loginfo("保存位姿 ID=%s: x=%.3f, y=%.3f, theta=%.3f", 
                         new_id, x, y, theta)
        
        return EmptyResponse()
        
    def clear_poses_callback(self, req):
        """清除位姿服务回调"""
        try:
            if self.save_poses([]):
                rospy.loginfo("已清除所有保存的位姿")
        except Exception as e:
            rospy.logerr("清除位姿失败: %s", str(e))
        return EmptyResponse()
        
    def load_poses_callback(self, req):
        """重新加载位姿服务回调"""
        self.load_poses()
        return EmptyResponse()
        
    def publish_markers(self, event):
        """发布位姿标记"""
        poses = self.cached_poses
        
        if not poses:
            # 如果没有位姿，发布空的标记数组
            marker_array = MarkerArray()
            self.marker_pub.publish(marker_array)
            return
            
        marker_array = MarkerArray()
        
        rospy.loginfo("准备发布 %d 个位姿的标记", len(poses))
        
        # 添加当前位姿的标记
        for i, pose_data in enumerate(poses):
            pose_id = pose_data['id']
            rospy.loginfo("发布标记 - 位姿 ID=%s: x=%.3f, y=%.3f, theta=%.3f", 
                          pose_id, pose_data['x'], pose_data['y'], pose_data['theta'])
            
            # 位置标记 (球体)
            marker = Marker()
            marker.header.frame_id = "map"
            marker.header.stamp = rospy.Time.now()
            marker.ns = "saved_poses"
            marker.id = i * 3  # 简化ID分配
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            
            marker.pose.position.x = pose_data['x']
            marker.pose.position.y = pose_data['y']
            marker.pose.position.z = 0.0
            marker.pose.orientation.w = 1.0
            
            marker.scale.x = 0.4
            marker.scale.y = 0.4
            marker.scale.z = 0.4
            
            marker.color.r = 0.0
            marker.color.g = 1.0
            marker.color.b = 0.0
            marker.color.a = 1.0
            
            marker.lifetime = rospy.Duration(0)
            marker_array.markers.append(marker)
            
            # 方向箭头
            arrow = Marker()
            arrow.header.frame_id = "map"
            arrow.header.stamp = rospy.Time.now()
            arrow.ns = "saved_poses"
            arrow.id = i * 3 + 1
            arrow.type = Marker.ARROW
            arrow.action = Marker.ADD
            
            arrow.pose.position.x = pose_data['x']
            arrow.pose.position.y = pose_data['y']
            arrow.pose.position.z = 0.0
            
            # 从yaw角创建四元数
            theta = pose_data['theta']
            arrow.pose.orientation.x = 0.0
            arrow.pose.orientation.y = 0.0
            arrow.pose.orientation.z = math.sin(theta / 2.0)
            arrow.pose.orientation.w = math.cos(theta / 2.0)
            
            arrow.scale.x = 0.6  # 长度
            arrow.scale.y = 0.1  # 宽度
            arrow.scale.z = 0.1  # 高度
            
            arrow.color.r = 1.0
            arrow.color.g = 0.0
            arrow.color.b = 0.0
            arrow.color.a = 1.0
            
            arrow.lifetime = rospy.Duration(0)
            marker_array.markers.append(arrow)
            
            # ID文本标记
            text = Marker()
            text.header.frame_id = "map"
            text.header.stamp = rospy.Time.now()
            text.ns = "saved_poses"
            text.id = i * 3 + 2
            text.type = Marker.TEXT_VIEW_FACING
            text.action = Marker.ADD
            
            text.pose.position.x = pose_data['x']
            text.pose.position.y = pose_data['y']
            text.pose.position.z = 0.6
            text.pose.orientation.w = 1.0
            
            text.scale.z = 0.4  # 文字大小
            
            text.color.r = 1.0
            text.color.g = 1.0
            text.color.b = 1.0
            text.color.a = 1.0
            
            text.text = "ID:" + str(pose_id)  # 确保使用JSON中的ID字段
            print(text.text)
            text.lifetime = rospy.Duration(0)
            marker_array.markers.append(text)
        
        # 清除可能的旧标记 (在添加新标记之后)
        current_marker_count = len(poses) * 3
        for i in range(current_marker_count, current_marker_count + 50):  # 只清除可能的旧标记
            delete_marker = Marker()
            delete_marker.header.frame_id = "map"
            delete_marker.header.stamp = rospy.Time.now()
            delete_marker.ns = "saved_poses"
            delete_marker.id = i
            delete_marker.action = Marker.DELETE
            marker_array.markers.append(delete_marker)
        
        rospy.loginfo("发布了 %d 个位姿的标记 (共 %d 个标记)", len(poses), current_marker_count)
        self.marker_pub.publish(marker_array)

if __name__ == '__main__':
    try:
        pose_saver = PoseSaver()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
