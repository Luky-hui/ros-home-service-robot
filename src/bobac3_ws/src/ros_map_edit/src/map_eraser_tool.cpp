#include "ros_map_edit/map_eraser_tool.h"
#include "ros_map_edit/tool_manager.h"
#include <rviz/viewport_mouse_event.h>
#include <rviz/visualization_manager.h>
#include <rviz/geometry.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <OgrePlane.h>
#include <OgreVector3.h>
#include <cmath>

namespace ros_map_edit
{

MapEraserTool::MapEraserTool()
  : tf_listener_(tf_buffer_)
  , map_received_(false)
  , mouse_pressed_(false)
  , brush_size_(1.0)
  , brush_mode_(ERASE_TO_FREE)
{
  // 注册到工具管理器
  ToolManager::getInstance().registerMapEraserTool(this);
}

MapEraserTool::~MapEraserTool()
{
}

void MapEraserTool::onInitialize()
{
  // Initialize ROS
  ros::NodeHandle nh;
  map_sub_ = nh.subscribe("map", 1, &MapEraserTool::mapCallback, this);
  map_pub_ = nh.advertise<nav_msgs::OccupancyGrid>("map_edited", 1, true);

  // Setup only brush size property - 改为像素单位
  brush_size_property_ = new rviz::FloatProperty("Brush Size", 5.0,
                                                 "Size of the square eraser brush (NxN pixels)",
                                                 getPropertyContainer(), SLOT(updateProperties()), this);
  brush_size_property_->setMin(1.0);
  brush_size_property_->setMax(10.0);
}

void MapEraserTool::activate()
{
  setStatus("黑白橡皮擦 - 左键画黑色(障碍物), 右键画白色(自由空间), 拖拽连续画");
  updateProperties();
}

void MapEraserTool::deactivate()
{
  mouse_pressed_ = false;
}

int MapEraserTool::processMouseEvent(rviz::ViewportMouseEvent& event)
{
  if (!map_received_)
  {
    setStatus("等待地图数据...");
    return Render;
  }

  if (event.leftDown())
  {
    mouse_pressed_ = true;
    brush_mode_ = ERASE_TO_OCCUPIED; // 左键画黑色
    Ogre::Vector3 intersection;
    Ogre::Plane ground_plane(Ogre::Vector3::UNIT_Z, 0.0f);
    
    if (rviz::getPointOnPlaneFromWindowXY(event.viewport,
                                          ground_plane,
                                          event.x, event.y, intersection))
    {
      geometry_msgs::Point point;
      point.x = intersection.x;
      point.y = intersection.y;
      point.z = 0.0;
      
      eraseAtPoint(point);
    }
  }
  else if (event.rightDown())
  {
    mouse_pressed_ = true;
    brush_mode_ = ERASE_TO_FREE; // 右键画白色
    Ogre::Vector3 intersection;
    Ogre::Plane ground_plane(Ogre::Vector3::UNIT_Z, 0.0f);
    
    if (rviz::getPointOnPlaneFromWindowXY(event.viewport,
                                          ground_plane,
                                          event.x, event.y, intersection))
    {
      geometry_msgs::Point point;
      point.x = intersection.x;
      point.y = intersection.y;
      point.z = 0.0;
      
      eraseAtPoint(point);
    }
  }
  else if (event.leftUp() || event.rightUp())
  {
    mouse_pressed_ = false;
  }
  else if (event.type == QEvent::MouseMove && mouse_pressed_)
  {
    Ogre::Vector3 intersection;
    Ogre::Plane ground_plane(Ogre::Vector3::UNIT_Z, 0.0f);
    
    if (rviz::getPointOnPlaneFromWindowXY(event.viewport,
                                          ground_plane,
                                          event.x, event.y, intersection))
    {
      geometry_msgs::Point point;
      point.x = intersection.x;
      point.y = intersection.y;
      point.z = 0.0;
      
      eraseAtPoint(point);
    }
  }

  return Render;
}

void MapEraserTool::updateProperties()
{
  brush_size_ = brush_size_property_->getFloat();
  int size = static_cast<int>(brush_size_);
  
  QString status_msg = "黑白橡皮擦 - 笔刷: " + QString::number(size) + "x" + QString::number(size) + "像素, 左键:黑色 右键:白色";
  setStatus(status_msg);
}

void MapEraserTool::mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg)
{
  current_map_ = *msg;
  map_received_ = true;
}

void MapEraserTool::eraseAtPoint(const geometry_msgs::Point& point)
{
  if (!map_received_)
    return;

  // Convert world coordinates to map coordinates
  int map_x = static_cast<int>((point.x - current_map_.info.origin.position.x) / current_map_.info.resolution);
  int map_y = static_cast<int>((point.y - current_map_.info.origin.position.y) / current_map_.info.resolution);
  
  // 画笔大小表示正方形的边长（像素）
  int brush_size = static_cast<int>(brush_size_);
  
  // 确保画笔大小至少为1
  if (brush_size < 1) {
    brush_size = 1;
  }
  
  // Determine the value to paint
  int8_t paint_value;
  switch (brush_mode_)
  {
    case ERASE_TO_FREE:
      paint_value = 0;   // Free space
      break;
    case ERASE_TO_OCCUPIED:
      paint_value = 100; // Occupied space
      break;
    case ERASE_TO_UNKNOWN:
    default:
      paint_value = -1;  // Unknown space
      break;
  }
  
  // 计算正方形画笔的范围
  // 对于奇数大小：以点击位置为中心
  // 对于偶数大小：向左上角偏移
  int half_size = brush_size / 2;
  int start_x = map_x - half_size;
  int start_y = map_y - half_size;
  
  // 如果是奇数大小，保持中心对齐
  if (brush_size % 2 == 1) {
    // 奇数：中心对齐，例如3x3时从(x-1,y-1)到(x+1,y+1)
    start_x = map_x - half_size;
    start_y = map_y - half_size;
  } else {
    // 偶数：向左上角偏移，例如2x2时从(x,y)到(x+1,y+1)
    start_x = map_x;
    start_y = map_y;
  }
  
  // Apply square brush
  for (int dy = 0; dy < brush_size; ++dy)
  {
    for (int dx = 0; dx < brush_size; ++dx)
    {
      int target_x = start_x + dx;
      int target_y = start_y + dy;
      
      // Check bounds
      if (target_x >= 0 && target_x < static_cast<int>(current_map_.info.width) &&
          target_y >= 0 && target_y < static_cast<int>(current_map_.info.height))
      {
        int index = target_y * current_map_.info.width + target_x;
        if (index >= 0 && index < static_cast<int>(current_map_.data.size()))
        {
          current_map_.data[index] = paint_value;
        }
      }
    }
  }
  
  publishModifiedMap();
}

void MapEraserTool::paintAtPoint(const geometry_msgs::Point& point)
{
  eraseAtPoint(point); // Same implementation
}

geometry_msgs::Point MapEraserTool::screenToMap(int screen_x, int screen_y)
{
  // This would need viewport transformation - simplified for now
  geometry_msgs::Point point;
  point.x = point.y = point.z = 0.0;
  return point;
}

void MapEraserTool::publishModifiedMap()
{
  current_map_.header.stamp = ros::Time::now();
  current_map_.header.frame_id = "map";
  
  // 发布到编辑地图话题
  map_pub_.publish(current_map_);
  
  // 同时更新原始地图话题（如果可能）
  ros::NodeHandle nh;
  ros::Publisher original_map_pub = nh.advertise<nav_msgs::OccupancyGrid>("map", 1, true);
  original_map_pub.publish(current_map_);
  
  // 记录发布信息 - 更新为像素单位
  int size = static_cast<int>(brush_size_);
  setStatus("地图已修改并发布 - 笔刷: " + QString::number(size) + "x" + QString::number(size) + "像素");
}

void MapEraserTool::loadMap(const std::string& filename)
{
  // Implementation would load from PGM/YAML files
  setStatus("Map loading from file not yet implemented");
}

void MapEraserTool::saveMap(const std::string& filename)
{
  // Implementation would save to PGM/YAML files
  setStatus("Map saving to file not yet implemented");
}

nav_msgs::OccupancyGrid MapEraserTool::getCurrentMap() const
{
  return current_map_;
}

} // end namespace ros_map_edit

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(ros_map_edit::MapEraserTool, rviz::Tool) 