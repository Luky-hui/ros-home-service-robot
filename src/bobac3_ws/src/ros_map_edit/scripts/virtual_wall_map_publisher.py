#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import yaml
import json
import numpy as np
import os
from nav_msgs.msg import OccupancyGrid, MapMetaData
from geometry_msgs.msg import Pose, Point, Quaternion
from std_msgs.msg import Header, ColorRGBA
from visualization_msgs.msg import MarkerArray, Marker
import tf.transformations as tf_trans
import math

class LineIterator:
    """Bresenham线段迭代器，用于在栅格地图上画线"""
    def __init__(self, x0, y0, x1, y1):
        self.x0, self.y0 = int(x0), int(y0)
        self.x1, self.y1 = int(x1), int(y1)
        self.dx = abs(self.x1 - self.x0)
        self.dy = abs(self.y1 - self.y0)
        self.sx = 1 if self.x0 < self.x1 else -1
        self.sy = 1 if self.y0 < self.y1 else -1
        self.err = self.dx - self.dy
        self.x, self.y = self.x0, self.y0
        self.finished = False
    
    def is_valid(self):
        return not self.finished
    
    def advance(self):
        if self.x == self.x1 and self.y == self.y1:
            self.finished = True
            return
        
        e2 = 2 * self.err
        if e2 > -self.dy:
            self.err -= self.dy
            self.x += self.sx
        if e2 < self.dx:
            self.err += self.dx
            self.y += self.sy
    
    def get_x(self):
        return self.x
    
    def get_y(self):
        return self.y

class VirtualWallMapPublisher:
    def __init__(self):
        rospy.init_node('virtual_wall_map_publisher', anonymous=True)
        
        # 获取参数
        map_file = rospy.get_param('~map_file', '') 
        self.map_yaml_file = map_file
        self.virtual_walls_file = map_file.replace('.yaml', '.json')
        
        # 解析$(find package)格式的路径
        self.map_yaml_file = self.resolve_ros_path(self.map_yaml_file)
        self.virtual_walls_file = self.resolve_ros_path(self.virtual_walls_file)
        
        rospy.loginfo(f"Map YAML file: {self.map_yaml_file}")
        rospy.loginfo(f"Virtual walls file: {self.virtual_walls_file}")
        
        # 发布器
        self.map_publisher = rospy.Publisher(
            '/virtual_wall_map',
            OccupancyGrid,
            queue_size=1,
            latch=True
        )
        
        # 发布MarkerArray用于可视化
        self.marker_publisher = rospy.Publisher(
            '/virtual_wall_map_markers',
            MarkerArray,
            queue_size=10
        )
        
        # 地图信息
        self.map_info = None
        self.virtual_walls = []
        
        # 加载地图和虚拟墙数据
        self.load_map_info()
        self.load_virtual_walls()
        
        # 发布虚拟墙地图
        self.publish_virtual_wall_map()
        self.publish_markers()
        
        # 定时重新发布（可选）
        self.timer = rospy.Timer(rospy.Duration(5.0), self.timer_callback)
        
        rospy.loginfo("Virtual Wall Map Publisher initialized")
    
    def resolve_ros_path(self, path):
        """解析ROS路径中的$(find package)格式"""
        if '$(find' in path:
            import rospkg
            rospack = rospkg.RosPack()
            try:
                start = path.find('$(find ') + 7
                end = path.find(')', start)
                package_name = path[start:end]
                package_path = rospack.get_path(package_name)
                resolved_path = path.replace('$(find ' + package_name + ')', package_path)
                return resolved_path
            except Exception as e:
                rospy.logwarn(f"Failed to resolve ROS path {path}: {e}")
                return path
        return path
    
    def load_map_info(self):
        """从YAML文件加载地图信息"""
        try:
            if not os.path.exists(self.map_yaml_file):
                rospy.logerr(f"Map YAML file not found: {self.map_yaml_file}")
                return
            
            with open(self.map_yaml_file, 'r') as f:
                map_data = yaml.safe_load(f)
            
            # 创建MapMetaData
            self.map_info = MapMetaData()
            self.map_info.resolution = map_data.get('resolution', 0.05)
            self.map_info.width = 0
            self.map_info.height = 0
            
            # 获取图像文件路径
            image_file = map_data.get('image', '')
            if not os.path.isabs(image_file):
                # 相对路径，相对于YAML文件目录
                yaml_dir = os.path.dirname(self.map_yaml_file)
                image_file = os.path.join(yaml_dir, image_file)
            
            # 读取图像获取尺寸
            if os.path.exists(image_file):
                from PIL import Image
                with Image.open(image_file) as img:
                    self.map_info.width = img.width
                    self.map_info.height = img.height
            
            # 设置原点
            origin = map_data.get('origin', [0.0, 0.0, 0.0])
            self.map_info.origin = Pose()
            self.map_info.origin.position.x = origin[0]
            self.map_info.origin.position.y = origin[1]
            self.map_info.origin.position.z = 0.0
            
            # 设置方向（从yaw角度创建四元数）
            yaw = origin[2] if len(origin) > 2 else 0.0
            quat = tf_trans.quaternion_from_euler(0, 0, yaw)
            self.map_info.origin.orientation = Quaternion(
                x=quat[0], y=quat[1], z=quat[2], w=quat[3]
            )
            
            self.map_info.map_load_time = rospy.Time.now()
            
            rospy.loginfo(f"Loaded map info: {self.map_info.width}x{self.map_info.height}, "
                         f"resolution: {self.map_info.resolution}")
            
        except Exception as e:
            rospy.logerr(f"Failed to load map info: {e}")
    
    def load_virtual_walls(self):
        """从JSON文件加载虚拟墙数据"""
        try:
            if not os.path.exists(self.virtual_walls_file):
                rospy.logwarn(f"Virtual walls file not found: {self.virtual_walls_file}")
                self.virtual_walls = []
                return
            
            with open(self.virtual_walls_file, 'r', encoding='utf-8') as f:
                data = json.load(f)
            
            self.virtual_walls = data.get('vws', [])
            rospy.loginfo(f"Loaded {len(self.virtual_walls)} virtual walls")
            
        except Exception as e:
            rospy.logerr(f"Failed to load virtual walls: {e}")
            self.virtual_walls = []
    
    def world_to_map(self, world_x, world_y):
        """将世界坐标转换为地图坐标"""
        if self.map_info is None:
            return 0, 0
        
        # 考虑地图原点偏移
        map_x = (world_x - self.map_info.origin.position.x) / self.map_info.resolution
        map_y = (world_y - self.map_info.origin.position.y) / self.map_info.resolution
        
        return int(map_x), int(map_y)
    
    def map_index(self, x, y):
        """计算地图数组索引"""
        if (x < 0 or x >= self.map_info.width or 
            y < 0 or y >= self.map_info.height):
            return -1
        
        # 修正Y方向，不进行翻转
        return y * self.map_info.width + x
    
    def draw_letter_patterns(self, msg):
        """在地图左上角绘制REINOVO文字（垂直显示）"""
        # 定义每个字母的7x5像素图案 (高x宽)
        letter_patterns = {
            'R': [
                [1,1,1,1,0],
                [1,0,0,0,1],
                [1,0,0,0,1],
                [1,1,1,1,0],
                [1,0,1,0,0],
                [1,0,0,1,0],
                [1,0,0,0,1]
            ],
            'E': [
                [1,1,1,1,1],
                [1,0,0,0,0],
                [1,0,0,0,0],
                [1,1,1,1,0],
                [1,0,0,0,0],
                [1,0,0,0,0],
                [1,1,1,1,1]
            ],
            'I': [
                [1,1,1,1,1],
                [0,0,1,0,0],
                [0,0,1,0,0],
                [0,0,1,0,0],
                [0,0,1,0,0],
                [0,0,1,0,0],
                [1,1,1,1,1]
            ],
            'N': [
                [1,0,0,0,1],
                [1,1,0,0,1],
                [1,0,1,0,1],
                [1,0,0,1,1],
                [1,0,0,0,1],
                [1,0,0,0,1],
                [1,0,0,0,1]
            ],
            'O': [
                [0,1,1,1,0],
                [1,0,0,0,1],
                [1,0,0,0,1],
                [1,0,0,0,1],
                [1,0,0,0,1],
                [1,0,0,0,1],
                [0,1,1,1,0]
            ],
            'V': [
                [1,0,0,0,1],
                [1,0,0,0,1],
                [1,0,0,0,1],
                [1,0,0,0,1],
                [0,1,0,1,0],
                [0,1,0,1,0],
                [0,0,1,0,0]
            ]
        }
        
        text = "REINOVO"
        letter_width = 5  # 每个字母宽度
        letter_height = 7
        letter_spacing = 8  # 字母间距（垂直方向）
        
        # 垂直显示文字，从左上角开始
        # 修正坐标系：在OccupancyGrid中，Y=0在底部，Y=height-1在顶部
        start_x = 2  # 距离左边界2像素
        start_y = 2  # 距离顶部2像素
        
        for letter_idx, letter in enumerate(text):
            if letter in letter_patterns:
                pattern = letter_patterns[letter]
                letter_start_x = start_x + letter_idx * letter_spacing  # 横向排列
                for row in range(letter_height):
                    for col in range(letter_width):
                        if pattern[row][col] == 1:
                            map_x = letter_start_x + col
                            map_y = self.map_info.height - start_y - row - 1
                            if (0 <= map_x < self.map_info.width and 0 <= map_y < self.map_info.height):
                                idx = self.map_index(map_x, map_y)
                                if idx >= 0 and idx < len(msg.data):
                                    msg.data[idx] = 0
        
        rospy.loginfo(f"Drew REINOVO text vertically at top-left position (x={start_x}, y={start_y}), map size: {self.map_info.width}x{self.map_info.height}")
    
    def draw_thick_line(self, msg, start_x, start_y, end_x, end_y, thickness=3):
        """绘制有厚度的线条，用于虚拟墙"""
        # 使用Bresenham算法绘制基本线条
        line_iter = LineIterator(start_x, start_y, end_x, end_y)
        
        # 收集所有线条上的点
        line_points = []
        while line_iter.is_valid():
            x = line_iter.get_x()
            y = line_iter.get_y()
            line_points.append((x, y))
            line_iter.advance()
        
        # 为每个点添加厚度
        map_size = len(msg.data)
        radius = thickness // 2
        
        for center_x, center_y in line_points:
            # 在每个中心点周围绘制正方形区域
            for dx in range(-radius, radius + 1):
                for dy in range(-radius, radius + 1):
                    x = center_x + dx
                    y = center_y + dy
                    
                    # 检查边界
                    if (0 <= x < self.map_info.width and 0 <= y < self.map_info.height):
                        idx = self.map_index(x, y)
                        if 0 <= idx < map_size:
                            msg.data[idx] = 100  # 设为障碍物
        
        return len(line_points)
    
    def publish_virtual_wall_map(self):
        """发布虚拟墙地图"""
        if self.map_info is None:
            rospy.logerr("Map info not loaded, cannot publish virtual wall map")
            return
        
        # 创建OccupancyGrid消息
        msg = OccupancyGrid()
        msg.header = Header()
        msg.header.frame_id = "map"
        msg.header.stamp = rospy.Time.now()
        msg.info = self.map_info
        
        # 初始化地图数据（全部设为未知）
        map_size = self.map_info.width * self.map_info.height
        msg.data = [-1] * map_size

        rospy.loginfo(f"Map size: {map_size}")
        
        # 在地图左上角绘制REINOVO文字
        self.draw_letter_patterns(msg)
        
        # 绘制虚拟墙 - 使用厚线条
        wall_count = 0
        total_points_drawn = 0
        
        # 根据地图分辨率计算虚拟墙厚度
        # 分辨率越小（地图越精细），厚度应该越大
        if self.map_info.resolution <= 0.025:  # 高分辨率地图
            wall_thickness = 7
        elif self.map_info.resolution <= 0.05:  # 中等分辨率地图
            wall_thickness = 5
        else:  # 低分辨率地图
            wall_thickness = 3
        
        rospy.loginfo(f"Using wall thickness: {wall_thickness} pixels (resolution: {self.map_info.resolution})")
        
        for wall in self.virtual_walls:
            points = wall.get('points', [])
            if len(points) >= 2:
                # 获取起点和终点的世界坐标
                start_world_x = points[0]['x']
                start_world_y = points[0]['y']
                end_world_x = points[1]['x']
                end_world_y = points[1]['y']
                
                # 转换为地图坐标
                start_map_x, start_map_y = self.world_to_map(start_world_x, start_world_y)
                end_map_x, end_map_y = self.world_to_map(end_world_x, end_world_y)
                
                rospy.logdebug(f"Wall {wall_count}: "
                              f"World ({start_world_x:.3f}, {start_world_y:.3f}) -> "
                              f"({end_world_x:.3f}, {end_world_y:.3f}), "
                              f"Map ({start_map_x}, {start_map_y}) -> ({end_map_x}, {end_map_y})")
                
                # 使用厚线条绘制虚拟墙
                points_drawn = self.draw_thick_line(msg, start_map_x, start_map_y, 
                                                  end_map_x, end_map_y, wall_thickness)
                total_points_drawn += points_drawn
                
                rospy.loginfo(f"Drew virtual wall {wall_count} with {points_drawn} base points, "
                             f"thickness: {wall_thickness} pixels")
                wall_count += 1

        rospy.loginfo(f"Published virtual wall map with {wall_count} walls, "
                     f"total base points: {total_points_drawn}")
        self.map_publisher.publish(msg)
    
    def publish_markers(self):
        """发布可视化标记"""
        marker_array = MarkerArray()
        
        # 删除所有现有标记
        delete_marker = Marker()
        delete_marker.action = Marker.DELETEALL
        marker_array.markers.append(delete_marker)
        
        # 为每个虚拟墙创建线条标记
        for i, wall in enumerate(self.virtual_walls):
            if len(wall.get('points', [])) >= 2:
                marker = Marker()
                marker.header.frame_id = "map"
                marker.header.stamp = rospy.Time.now()
                marker.ns = "virtual_wall_lines"
                marker.id = i
                marker.type = Marker.LINE_STRIP
                marker.action = Marker.ADD
                
                # 设置线条属性
                marker.scale.x = 0.15  # 线条宽度，比虚拟墙管理器稍粗一点
                marker.color = ColorRGBA(0.8, 0.0, 0.8, 1.0)  # 紫色，区别于虚拟墙管理器
                
                # 添加点
                for point in wall['points']:
                    p = Point()
                    p.x = point['x']
                    p.y = point['y']
                    p.z = 0.1  # 稍微抬高一点避免重叠
                    marker.points.append(p)
                
                marker_array.markers.append(marker)
                
                # 为每个端点添加球形标记
                for j, point in enumerate(wall['points']):
                    point_marker = Marker()
                    point_marker.header.frame_id = "map"
                    point_marker.header.stamp = rospy.Time.now()
                    point_marker.ns = "virtual_wall_map_points"
                    point_marker.id = i * 100 + j  # 确保ID唯一
                    point_marker.type = Marker.SPHERE
                    point_marker.action = Marker.ADD
                    
                    point_marker.pose.position.x = point['x']
                    point_marker.pose.position.y = point['y']
                    point_marker.pose.position.z = 0.1
                    point_marker.pose.orientation.w = 1.0
                    
                    point_marker.scale.x = 0.25
                    point_marker.scale.y = 0.25
                    point_marker.scale.z = 0.25
                    point_marker.color = ColorRGBA(0.0, 0.8, 0.8, 1.0)  # 青色
                    
                    marker_array.markers.append(point_marker)
        
        self.marker_publisher.publish(marker_array)
    
    def timer_callback(self, event):
        """定时器回调，重新加载和发布"""
        # 重新加载虚拟墙（地图信息通常不变）
        self.load_virtual_walls()
        # self.publish_virtual_wall_map()
        self.publish_markers()
    
    def run(self):
        """运行节点"""
        rospy.loginfo("Virtual Wall Map Publisher is running...")
        rospy.spin()

if __name__ == '__main__':
    try:
        publisher = VirtualWallMapPublisher()
        publisher.run()
    except rospy.ROSInterruptException:
        rospy.loginfo("Virtual Wall Map Publisher shutting down") 