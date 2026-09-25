#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import json
import yaml
import os
import actionlib
import threading
from geometry_msgs.msg import PoseStamped, Point
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from visualization_msgs.msg import MarkerArray, Marker
from std_srvs.srv import Empty, EmptyResponse
from std_msgs.msg import Int32, String
from actionlib_msgs.msg import GoalStatus
import tf.transformations as tf_trans

class MultiPointNavigator:
    def __init__(self):
        rospy.init_node('multi_point_navigator', anonymous=True)
        
        # 参数
        self.poses_json_file = rospy.get_param('~poses_json_file', 'src/bobac/maps/saved_poses.json')
        self.nav_yaml_file = rospy.get_param('~mnav_yaml_file', 'src/bobac/maps/navigation_sequence.yaml')
        self.timeout = rospy.get_param('~navigation_timeout', 300.0)  # 每个点的导航超时时间
        map_file_path = rospy.get_param('~map_file', '$(find bobac)/maps/virtual_walls.json')
        self.poses_json_file = map_file_path.replace(".yaml", "_point.json")

        # 状态变量
        self.poses_dict = {}  # 存储所有位姿点 {id: pose_data}
        self.nav_sequence = []  # 导航序列
        self.current_goal_index = 0  # 当前导航点索引
        self.is_navigating = False  # 是否正在导航
        self.is_paused = False  # 是否暂停
        self.navigation_thread = None
        
        # MoveBase Action Client
        self.move_base_client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        rospy.loginfo("等待move_base服务器...")
        if not self.move_base_client.wait_for_server(rospy.Duration(10.0)):
            rospy.logerr("无法连接到move_base服务器!")
            return
        rospy.loginfo("已连接到move_base服务器")
        
        # Clear costmaps service client
        rospy.loginfo("等待clear_costmaps服务...")
        try:
            rospy.wait_for_service('/move_base/clear_costmaps', timeout=5.0)
            self.clear_costmaps_client = rospy.ServiceProxy('/move_base/clear_costmaps', Empty)
            rospy.loginfo("已连接到clear_costmaps服务")
        except rospy.ROSException:
            rospy.logwarn("无法连接到clear_costmaps服务，将跳过代价地图清除")
            self.clear_costmaps_client = None
        
        # 发布器
        self.marker_pub = rospy.Publisher('/navigation_markers', MarkerArray, queue_size=1, latch=True)
        self.status_pub = rospy.Publisher('/navigation_status', String, queue_size=1)
        self.current_goal_pub = rospy.Publisher('/current_navigation_goal', Int32, queue_size=1)
        
        # 服务
        self.start_service = rospy.Service('/start_navigation', Empty, self.start_navigation_callback)
        self.pause_service = rospy.Service('/pause_navigation', Empty, self.pause_navigation_callback)
        self.resume_service = rospy.Service('/resume_navigation', Empty, self.resume_navigation_callback)
        self.stop_service = rospy.Service('/stop_navigation', Empty, self.stop_navigation_callback)
        self.reload_service = rospy.Service('/reload_navigation_files', Empty, self.reload_files_callback)
        
        # 定时器
        self.marker_timer = rospy.Timer(rospy.Duration(2.0), self.publish_markers)
        self.status_timer = rospy.Timer(rospy.Duration(1.0), self.publish_status)
        
        rospy.loginfo("多点导航器已启动")
        rospy.loginfo("位姿文件: %s", self.poses_json_file)
        rospy.loginfo("导航序列文件: %s", self.nav_yaml_file)
        rospy.loginfo("服务:")
        rospy.loginfo("  /start_navigation - 开始导航")
        rospy.loginfo("  /pause_navigation - 暂停导航")
        rospy.loginfo("  /resume_navigation - 恢复导航")
        rospy.loginfo("  /stop_navigation - 停止导航")
        rospy.loginfo("  /reload_navigation_files - 重新加载文件")
        
        # 加载文件
        self.load_poses()
        self.load_navigation_sequence()
        
    def load_poses(self):
        """从JSON文件加载位姿"""
        try:
            if os.path.exists(self.poses_json_file):
                with open(self.poses_json_file, 'r', encoding='utf-8') as f:
                    data = json.load(f)
                    points = data.get('points', [])
                    
                self.poses_dict = {}
                for point in points:
                    self.poses_dict[point['id']] = point
                    
                rospy.loginfo("加载了 %d 个位姿点", len(self.poses_dict))
                for pose_id in sorted(self.poses_dict.keys(), key=int):
                    pose = self.poses_dict[pose_id]
                    rospy.loginfo("  ID=%s: x=%.2f, y=%.2f, theta=%.2f", 
                                 pose_id, pose['x'], pose['y'], pose['theta'])
            else:
                rospy.logwarn("位姿文件不存在: %s", self.poses_json_file)
                self.poses_dict = {}
                
        except Exception as e:
            rospy.logerr("加载位姿文件失败: %s", str(e))
            self.poses_dict = {}
            
    def load_navigation_sequence(self):
        """从YAML文件加载导航序列"""
        try:
            if os.path.exists(self.nav_yaml_file):
                with open(self.nav_yaml_file, 'r', encoding='utf-8') as f:
                    data = yaml.safe_load(f)
                    
                self.nav_sequence = data.get('mnav_points', [])
                rospy.loginfo("加载导航序列: %s", self.nav_sequence)
                
                # 验证序列中的点是否存在
                missing_points = []
                for point_id in self.nav_sequence:
                    if str(point_id) not in self.poses_dict:
                        missing_points.append(point_id)
                        
                if missing_points:
                    rospy.logwarn("导航序列中缺失的位姿点: %s", missing_points)
                    
            else:
                rospy.logwarn("导航序列文件不存在: %s", self.nav_yaml_file)
                rospy.loginfo("创建示例导航序列文件...")
                self.create_example_nav_file()
                
        except Exception as e:
            rospy.logerr("加载导航序列失败: %s", str(e))
            self.nav_sequence = []
            
    def create_example_nav_file(self):
        """创建示例导航序列文件"""
        try:
            os.makedirs(os.path.dirname(self.nav_yaml_file), exist_ok=True)
            
            # 使用现有位姿点的前几个ID作为示例
            example_points = []
            if self.poses_dict:
                sorted_ids = sorted(self.poses_dict.keys(), key=int)
                example_points = [int(id_str) for id_str in sorted_ids[:min(3, len(sorted_ids))]]
            else:
                example_points = [0, 1, 2]  # 默认示例
                
            example_data = {
                'mnav_points': example_points,
                'description': '导航点序列，按照列表顺序依次导航到对应ID的位姿点'
            }
            
            with open(self.nav_yaml_file, 'w', encoding='utf-8') as f:
                yaml.dump(example_data, f, allow_unicode=True, default_flow_style=False)
                
            rospy.loginfo("已创建示例导航序列文件: %s", self.nav_yaml_file)
            rospy.loginfo("示例序列: %s", example_points)
            
        except Exception as e:
            rospy.logerr("创建示例文件失败: %s", str(e))
            
    def create_pose_goal(self, pose_data):
        """从位姿数据创建MoveBase目标"""
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        
        # 位置
        goal.target_pose.pose.position.x = pose_data['x']
        goal.target_pose.pose.position.y = pose_data['y']
        goal.target_pose.pose.position.z = 0.0
        
        # 方向（从yaw角转四元数）
        theta = pose_data['theta']
        qx, qy, qz, qw = tf_trans.quaternion_from_euler(0, 0, theta)
        goal.target_pose.pose.orientation.x = qx
        goal.target_pose.pose.orientation.y = qy
        goal.target_pose.pose.orientation.z = qz
        goal.target_pose.pose.orientation.w = qw
        
        return goal
        
    def navigation_worker(self):
        """导航工作线程"""
        rospy.loginfo("开始多点导航，共 %d 个点", len(self.nav_sequence))
        
        self.current_goal_index = 0
        success_count = 0
        
        for i, point_id in enumerate(self.nav_sequence):
            if not self.is_navigating:
                rospy.loginfo("导航被停止")
                break
                
            # 检查暂停状态
            while self.is_paused and self.is_navigating:
                rospy.loginfo("导航已暂停，等待恢复...")
                rospy.sleep(1.0)
                
            if not self.is_navigating:
                break
                
            self.current_goal_index = i
            point_id_str = str(point_id)
            
            if point_id_str not in self.poses_dict:
                rospy.logwarn("跳过不存在的位姿点 ID=%s", point_id)
                continue
                
            pose_data = self.poses_dict[point_id_str]
            rospy.loginfo("导航到点 %d/%d: ID=%s (x=%.2f, y=%.2f, theta=%.2f)", 
                         i+1, len(self.nav_sequence), point_id, 
                         pose_data['x'], pose_data['y'], pose_data['theta'])
            
            # 创建并发送目标
            goal = self.create_pose_goal(pose_data)
            self.move_base_client.send_goal(goal)
            
            # 等待结果
            success = self.move_base_client.wait_for_result(rospy.Duration(self.timeout))
            
            if success:
                state = self.move_base_client.get_state()
                if state == GoalStatus.SUCCEEDED:
                    rospy.loginfo("成功到达点 ID=%s", point_id)
                    success_count += 1
                else:
                    rospy.logwarn("到达点 ID=%s 失败，状态: %d", point_id, state)
            else:
                rospy.logwarn("导航到点 ID=%s 超时", point_id)
                self.move_base_client.cancel_goal()
                
            # 到达后短暂停留
            if self.is_navigating:
                rospy.sleep(1.0)
                
        # 导航完成
        self.is_navigating = False
        self.is_paused = False
        
        if success_count == len(self.nav_sequence):
            rospy.loginfo("多点导航完成！成功到达所有 %d 个点", success_count)
        else:
            rospy.loginfo("多点导航结束。成功: %d/%d", success_count, len(self.nav_sequence))
            
    def start_navigation_callback(self, req):
        """开始导航服务回调"""
        if self.is_navigating:
            rospy.logwarn("导航已在进行中")
            return EmptyResponse()
        
        # 重新加载配置文件
        rospy.loginfo("重新加载配置文件...")
        self.load_poses()
        self.load_navigation_sequence()
            
        if not self.nav_sequence:
            rospy.logwarn("导航序列为空，无法开始导航")
            return EmptyResponse()
            
        if not self.poses_dict:
            rospy.logwarn("没有加载位姿点，无法开始导航")
            return EmptyResponse()
        
        # 清除代价地图
        if self.clear_costmaps_client:
            try:
                rospy.loginfo("清除代价地图...")
                self.clear_costmaps_client()
                rospy.loginfo("代价地图已清除")
            except rospy.ServiceException as e:
                rospy.logwarn("清除代价地图失败: %s", str(e))
        else:
            rospy.logwarn("clear_costmaps服务不可用，跳过代价地图清除")
            
        self.is_navigating = True
        self.is_paused = False
        self.navigation_thread = threading.Thread(target=self.navigation_worker)
        self.navigation_thread.daemon = True
        self.navigation_thread.start()
        
        rospy.loginfo("开始多点导航")
        return EmptyResponse()
        
    def pause_navigation_callback(self, req):
        """暂停导航服务回调"""
        if not self.is_navigating:
            rospy.logwarn("没有正在进行的导航")
            return EmptyResponse()
            
        self.is_paused = True
        self.move_base_client.cancel_goal()
        rospy.loginfo("导航已暂停")
        return EmptyResponse()
        
    def resume_navigation_callback(self, req):
        """恢复导航服务回调"""
        if not self.is_navigating:
            rospy.logwarn("没有正在进行的导航")
            return EmptyResponse()
            
        if not self.is_paused:
            rospy.logwarn("导航没有暂停")
            return EmptyResponse()
            
        self.is_paused = False
        rospy.loginfo("导航已恢复")
        return EmptyResponse()
        
    def stop_navigation_callback(self, req):
        """停止导航服务回调"""
        if not self.is_navigating:
            rospy.logwarn("没有正在进行的导航")
            return EmptyResponse()
            
        self.is_navigating = False
        self.is_paused = False
        self.move_base_client.cancel_goal()
        rospy.loginfo("导航已停止")
        return EmptyResponse()
        
    def reload_files_callback(self, req):
        """重新加载文件服务回调"""
        self.load_poses()
        self.load_navigation_sequence()
        rospy.loginfo("已重新加载位姿和导航序列文件")
        return EmptyResponse()
        
    def publish_status(self, event):
        """发布导航状态"""
        if self.is_navigating:
            if self.is_paused:
                status = "PAUSED"
            else:
                status = "NAVIGATING"
        else:
            status = "IDLE"
            
        status_msg = String()
        status_msg.data = status
        self.status_pub.publish(status_msg)
        
        # 发布当前目标索引
        goal_msg = Int32()
        goal_msg.data = self.current_goal_index
        self.current_goal_pub.publish(goal_msg)
        
    def publish_markers(self, event):
        """发布导航路径标记"""
        marker_array = MarkerArray()
        
        if not self.nav_sequence or not self.poses_dict:
            self.marker_pub.publish(marker_array)
            return
            
        # 路径线条
        path_marker = Marker()
        path_marker.header.frame_id = "map"
        path_marker.header.stamp = rospy.Time.now()
        path_marker.ns = "navigation_path"
        path_marker.id = 0
        path_marker.type = Marker.LINE_STRIP
        path_marker.action = Marker.ADD
        
        path_marker.scale.x = 0.1  # 线宽
        path_marker.color.r = 0.0
        path_marker.color.g = 0.0
        path_marker.color.b = 1.0
        path_marker.color.a = 0.8
        path_marker.lifetime = rospy.Duration(0)
        
        # 添加路径点
        for point_id in self.nav_sequence:
            point_id_str = str(point_id)
            if point_id_str in self.poses_dict:
                pose_data = self.poses_dict[point_id_str]
                point = Point()
                point.x = pose_data['x']
                point.y = pose_data['y']
                point.z = 0.1
                path_marker.points.append(point)
                
        if path_marker.points:
            marker_array.markers.append(path_marker)
            
        # 导航点标记
        for i, point_id in enumerate(self.nav_sequence):
            point_id_str = str(point_id)
            if point_id_str not in self.poses_dict:
                continue
                
            pose_data = self.poses_dict[point_id_str]
            
            # 点标记
            marker = Marker()
            marker.header.frame_id = "map"
            marker.header.stamp = rospy.Time.now()
            marker.ns = "navigation_points"
            marker.id = i
            marker.type = Marker.CYLINDER
            marker.action = Marker.ADD
            
            marker.pose.position.x = pose_data['x']
            marker.pose.position.y = pose_data['y']
            marker.pose.position.z = 0.0
            marker.pose.orientation.w = 1.0
            
            marker.scale.x = 0.5
            marker.scale.y = 0.5
            marker.scale.z = 0.1
            
            # 颜色表示状态
            if self.is_navigating and i == self.current_goal_index:
                # 当前目标：黄色
                marker.color.r = 1.0
                marker.color.g = 1.0
                marker.color.b = 0.0
            elif self.is_navigating and i < self.current_goal_index:
                # 已完成：绿色
                marker.color.r = 0.0
                marker.color.g = 1.0
                marker.color.b = 0.0
            else:
                # 待执行：蓝色
                marker.color.r = 0.0
                marker.color.g = 0.0
                marker.color.b = 1.0
                
            marker.color.a = 0.8
            marker.lifetime = rospy.Duration(0)
            marker_array.markers.append(marker)
            
            # 序号文本
            text_marker = Marker()
            text_marker.header.frame_id = "map"
            text_marker.header.stamp = rospy.Time.now()
            text_marker.ns = "navigation_points"
            text_marker.id = i + 1000
            text_marker.type = Marker.TEXT_VIEW_FACING
            text_marker.action = Marker.ADD
            
            text_marker.pose.position.x = pose_data['x']
            text_marker.pose.position.y = pose_data['y']
            text_marker.pose.position.z = 0.8
            text_marker.pose.orientation.w = 1.0
            
            text_marker.scale.z = 0.4
            text_marker.color.r = 1.0
            text_marker.color.g = 1.0
            text_marker.color.b = 1.0
            text_marker.color.a = 1.0
            
            text_marker.text = f"{i+1}:{point_id}"
            text_marker.lifetime = rospy.Duration(0)
            marker_array.markers.append(text_marker)
            
        # 清除多余标记
        for i in range(len(self.nav_sequence), len(self.nav_sequence) + 50):
            delete_marker = Marker()
            delete_marker.header.frame_id = "map"
            delete_marker.ns = "navigation_points"
            delete_marker.id = i
            delete_marker.action = Marker.DELETE
            marker_array.markers.append(delete_marker)
            
            delete_text = Marker()
            delete_text.header.frame_id = "map"
            delete_text.ns = "navigation_points"
            delete_text.id = i + 1000
            delete_text.action = Marker.DELETE
            marker_array.markers.append(delete_text)
            
        self.marker_pub.publish(marker_array)

if __name__ == '__main__':
    try:
        navigator = MultiPointNavigator()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
