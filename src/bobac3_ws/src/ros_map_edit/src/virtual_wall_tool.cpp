#include "ros_map_edit/virtual_wall_tool.h"
#include "ros_map_edit/tool_manager.h"
#include <rviz/viewport_mouse_event.h>
#include <rviz/visualization_manager.h>
#include <rviz/geometry.h>
#include <rviz/ogre_helpers/line.h>
#include <OgrePlane.h>
#include <OgreVector3.h>
#include <QColorDialog>
#include <jsoncpp/json/json.h>
#include <fstream>
#include <QTimer>

namespace ros_map_edit
{

VirtualWallTool::VirtualWallTool()
  : marker_id_counter_(0)
  , drawing_wall_(false)
{
  // 注册到工具管理器
  ToolManager::getInstance().registerVirtualWallTool(this);
}

VirtualWallTool::~VirtualWallTool()
{
}

void VirtualWallTool::onInitialize()
{
  // Initialize ROS publisher
  ros::NodeHandle nh;
  marker_pub_ = nh.advertise<visualization_msgs::MarkerArray>("virtual_walls_markers", 1);

  // Setup properties
  color_property_ = new rviz::ColorProperty("Color", QColor(255, 0, 0),
                                            "Color of virtual walls",
                                            getPropertyContainer(), SLOT(updateProperties()), this);

  line_width_property_ = new rviz::FloatProperty("Line Width", 0.1,
                                                "Width of virtual wall lines",
                                                getPropertyContainer(), SLOT(updateProperties()), this);
  line_width_property_->setMin(0.01);
  line_width_property_->setMax(1.0);

  show_points_property_ = new rviz::BoolProperty("Show Points", true,
                                                "Show individual points on walls",
                                                getPropertyContainer(), SLOT(updateProperties()), this);

  // 自动加载保存的虚拟墙文件
  autoLoadVirtualWalls();
}

void VirtualWallTool::autoLoadVirtualWalls()
{
  // 首先清空现有的虚拟墙，发布空的markers
  virtual_walls_.clear();
  current_wall_points_.clear();
  drawing_wall_ = false;
  updateMarkers(); // 这会发布空的marker数组
  
  // 从参数服务器获取当前地图文件路径
  ros::NodeHandle nh;
  std::string map_file;
  if (nh.getParam("/map_server/map_file", map_file))
  {
    // 构造对应的虚拟墙文件路径
    std::string vw_file = map_file;
    size_t dot_pos = vw_file.find_last_of(".");
    if (dot_pos != std::string::npos)
    {
      vw_file = vw_file.substr(0, dot_pos) + ".json";
    }
    else
    {
      vw_file += ".json";
    }
    
    // 检查文件是否存在
    std::ifstream test_file(vw_file);
    if (test_file.good())
    {
      test_file.close();
      ROS_INFO("找到对应的虚拟墙文件，正在加载: %s", vw_file.c_str());
      
      // 延迟加载以确保RViz完全初始化
      QTimer::singleShot(1000, [this, vw_file]() {
        loadVirtualWalls(vw_file);
        setStatus("虚拟墙已加载: " + QString::fromStdString(vw_file));
      });
    }
    else
    {
      ROS_INFO("当前地图没有对应的虚拟墙文件: %s，发布空的虚拟墙", vw_file.c_str());
      setStatus("当前地图无虚拟墙文件，显示为空");
    }
  }
  else
  {
    ROS_INFO("未找到地图文件参数，发布空的虚拟墙");
    setStatus("未加载地图，虚拟墙为空");
  }
}

void VirtualWallTool::activate()
{
  setStatus("虚拟墙工具 - 左键点击两个点创建墙体，右键取消");
  updateMarkers();
}

void VirtualWallTool::deactivate()
{
  drawing_wall_ = false;
  current_wall_points_.clear();
}

int VirtualWallTool::processMouseEvent(rviz::ViewportMouseEvent& event)
{
  if (event.leftDown())
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
      
      addPoint(point);
      
      // 限制为两个点，第二个点后自动完成
      if (current_wall_points_.size() >= 2)
      {
        finishCurrentWall();
      }
      else
      {
        drawing_wall_ = true;
      }
    }
  }
  else if (event.rightDown() && drawing_wall_)
  {
    // 右键取消当前墙体
    current_wall_points_.clear();
    drawing_wall_ = false;
    updateMarkers();
    setStatus("已取消当前虚拟墙");
  }

  return Render;
}

void VirtualWallTool::updateProperties()
{
  updateMarkers();
}

void VirtualWallTool::addPoint(const geometry_msgs::Point& point)
{
  current_wall_points_.push_back(point);
  updateMarkers();
}

void VirtualWallTool::finishCurrentWall()
{
  if (current_wall_points_.size() >= 2)
  {
    VirtualWall wall;
    wall.points = current_wall_points_;
    wall.id = "wall_" + std::to_string(virtual_walls_.size());
    virtual_walls_.push_back(wall);
  }
  
  current_wall_points_.clear();
  drawing_wall_ = false;
  updateMarkers();
}

void VirtualWallTool::updateMarkers()
{
  marker_array_.markers.clear();
  marker_id_counter_ = 0;

  QColor color = color_property_->getColor();
  float line_width = line_width_property_->getFloat();
  bool show_points = show_points_property_->getBool();

  // Draw existing walls
  for (const auto& wall : virtual_walls_)
  {
    if (wall.points.size() >= 2)
    {
      visualization_msgs::Marker line_marker;
      line_marker.header.frame_id = "map";
      line_marker.header.stamp = ros::Time::now();
      line_marker.ns = "virtual_walls";
      line_marker.id = marker_id_counter_++;
      line_marker.type = visualization_msgs::Marker::LINE_STRIP;
      line_marker.action = visualization_msgs::Marker::ADD;
      line_marker.pose.orientation.w = 1.0;
      line_marker.scale.x = line_width;
      line_marker.color.r = color.redF();
      line_marker.color.g = color.greenF();
      line_marker.color.b = color.blueF();
      line_marker.color.a = 1.0;

      for (const auto& point : wall.points)
      {
        line_marker.points.push_back(point);
      }

      marker_array_.markers.push_back(line_marker);

      // Show points if enabled
      if (show_points)
      {
        for (const auto& point : wall.points)
        {
          visualization_msgs::Marker point_marker;
          point_marker.header.frame_id = "map";
          point_marker.header.stamp = ros::Time::now();
          point_marker.ns = "virtual_wall_points";
          point_marker.id = marker_id_counter_++;
          point_marker.type = visualization_msgs::Marker::SPHERE;
          point_marker.action = visualization_msgs::Marker::ADD;
          point_marker.pose.position = point;
          point_marker.pose.orientation.w = 1.0;
          point_marker.scale.x = point_marker.scale.y = point_marker.scale.z = 0.05;
          point_marker.color.r = color.redF();
          point_marker.color.g = color.greenF();
          point_marker.color.b = color.blueF();
          point_marker.color.a = 1.0;

          marker_array_.markers.push_back(point_marker);
        }
      }
    }
  }

  // Draw current wall being created
  if (current_wall_points_.size() >= 2)
  {
    visualization_msgs::Marker current_line;
    current_line.header.frame_id = "map";
    current_line.header.stamp = ros::Time::now();
    current_line.ns = "current_wall";
    current_line.id = marker_id_counter_++;
    current_line.type = visualization_msgs::Marker::LINE_STRIP;
    current_line.action = visualization_msgs::Marker::ADD;
    current_line.pose.orientation.w = 1.0;
    current_line.scale.x = line_width;
    current_line.color.r = 1.0;
    current_line.color.g = 1.0;
    current_line.color.b = 0.0; // Yellow for current wall
    current_line.color.a = 0.8;

    for (const auto& point : current_wall_points_)
    {
      current_line.points.push_back(point);
    }

    marker_array_.markers.push_back(current_line);
  }

  publishMarkers();
}

void VirtualWallTool::publishMarkers()
{
  marker_pub_.publish(marker_array_);
}

void VirtualWallTool::loadVirtualWalls(const std::string& filename)
{
  std::ifstream file(filename);
  if (!file.is_open())
  {
    setStatus("Failed to open virtual walls file: " + QString::fromStdString(filename));
    return;
  }

  Json::Value root;
  file >> root;

  virtual_walls_.clear();
  
  if (root.isMember("vws") && root["vws"].isArray())
  {
    for (const auto& vw_json : root["vws"])
    {
      if (vw_json.isMember("points") && vw_json["points"].isArray())
      {
        VirtualWall wall;
        wall.id = "wall_" + std::to_string(virtual_walls_.size());
        
        for (const auto& point_json : vw_json["points"])
        {
          geometry_msgs::Point point;
          point.x = point_json["x"].asDouble();
          point.y = point_json["y"].asDouble();
          point.z = 0.0;
          wall.points.push_back(point);
        }
        
        if (wall.points.size() >= 2)
        {
          virtual_walls_.push_back(wall);
        }
      }
    }
  }

  updateMarkers();
  setStatus("Loaded " + QString::number(virtual_walls_.size()) + " virtual walls");
}

void VirtualWallTool::saveVirtualWalls(const std::string& filename)
{
  Json::Value root;
  Json::Value vws_array(Json::arrayValue);

  for (const auto& wall : virtual_walls_)
  {
    if (wall.points.size() >= 2)
    {
      Json::Value wall_json;
      Json::Value points_array(Json::arrayValue);
      
      for (const auto& point : wall.points)
      {
        Json::Value point_json;
        point_json["x"] = point.x;
        point_json["y"] = point.y;
        points_array.append(point_json);
      }
      
      wall_json["points"] = points_array;
      vws_array.append(wall_json);
    }
  }

  root["vws"] = vws_array;

  std::ofstream file(filename);
  if (file.is_open())
  {
    Json::StreamWriterBuilder builder;
    builder["indentation"] = "   ";
    std::unique_ptr<Json::StreamWriter> writer(builder.newStreamWriter());
    writer->write(root, &file);
    
    setStatus("Saved " + QString::number(virtual_walls_.size()) + " virtual walls");
  }
  else
  {
    setStatus("Failed to save virtual walls file: " + QString::fromStdString(filename));
  }
}

void VirtualWallTool::clearVirtualWalls()
{
  virtual_walls_.clear();
  current_wall_points_.clear();
  drawing_wall_ = false;
  updateMarkers();
  setStatus("Cleared all virtual walls");
}

std::vector<VirtualWall> VirtualWallTool::getVirtualWalls() const
{
  return virtual_walls_;
}

void VirtualWallTool::setVirtualWalls(const std::vector<VirtualWall>& walls)
{
  virtual_walls_ = walls;
  current_wall_points_.clear();
  drawing_wall_ = false;
  updateMarkers();
  setStatus("Set " + QString::number(virtual_walls_.size()) + " virtual walls");
}

void VirtualWallTool::loadVirtualWallsForMap(const std::string& map_file_path)
{
  // 新增方法：根据地图文件路径加载对应的虚拟墙文件
  virtual_walls_.clear();
  current_wall_points_.clear();
  drawing_wall_ = false;
  
  // 构造虚拟墙文件路径
  std::string vw_file = map_file_path;
  size_t dot_pos = vw_file.find_last_of(".");
  if (dot_pos != std::string::npos)
  {
    vw_file = vw_file.substr(0, dot_pos) + ".json";
  }
  else
  {
    vw_file += ".json";
  }
  
  // 检查文件是否存在
  std::ifstream test_file(vw_file);
  if (test_file.good())
  {
    test_file.close();
    ROS_INFO("加载虚拟墙文件: %s", vw_file.c_str());
    loadVirtualWalls(vw_file);
  }
  else
  {
    ROS_INFO("地图文件 %s 没有对应的虚拟墙文件，显示为空", map_file_path.c_str());
    updateMarkers(); // 发布空的markers
  }
}

} // end namespace ros_map_edit

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(ros_map_edit::VirtualWallTool, rviz::Tool) 