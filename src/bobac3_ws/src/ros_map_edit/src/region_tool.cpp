#include "ros_map_edit/region_tool.h"
#include "ros_map_edit/tool_manager.h"
#include <rviz/viewport_mouse_event.h>
#include <rviz/visualization_manager.h>
#include <rviz/geometry.h>
#include <OgrePlane.h>
#include <OgreVector3.h>
#include <jsoncpp/json/json.h>
#include <fstream>
#include <sstream>
#include <QTimer>

namespace ros_map_edit
{

RegionTool::RegionTool()
  : marker_id_counter_(0)
  , region_id_counter_(1)
  , drawing_region_(false)
{
  // 注册到工具管理器
  ToolManager::getInstance().registerRegionTool(this);
}

RegionTool::~RegionTool()
{
}

void RegionTool::onInitialize()
{
  // Initialize ROS publisher
  ros::NodeHandle nh;
  marker_pub_ = nh.advertise<visualization_msgs::MarkerArray>("region_markers", 1);

  // Setup properties
  color_property_ = new rviz::ColorProperty("Color", QColor(0, 255, 0),
                                            "Color of region polygons",
                                            getPropertyContainer(), SLOT(updateProperties()), this);

  alpha_property_ = new rviz::FloatProperty("Alpha", 0.5,
                                            "Transparency of region polygons",
                                            getPropertyContainer(), SLOT(updateProperties()), this);
  alpha_property_->setMin(0.0);
  alpha_property_->setMax(1.0);

  region_type_property_ = new rviz::IntProperty("Region Type", 0,
                                                "Type identifier for the region",
                                                getPropertyContainer(), SLOT(updateProperties()), this);
  region_type_property_->setMin(0);
  region_type_property_->setMax(10);

  region_param_property_ = new rviz::FloatProperty("Region Parameter", 1.0,
                                                   "Parameter value for the region",
                                                   getPropertyContainer(), SLOT(updateProperties()), this);

  frame_id_property_ = new rviz::StringProperty("Frame ID", "map",
                                                "Frame ID for the regions",
                                                getPropertyContainer(), SLOT(updateProperties()), this);

  show_points_property_ = new rviz::BoolProperty("Show Points", true,
                                                "Show individual points on regions",
                                                getPropertyContainer(), SLOT(updateProperties()), this);

  // 自动加载保存的区域文件
  autoLoadRegions();
}

void RegionTool::activate()
{
  setStatus("Region Tool - Left click to add polygon points, right click to finish region");
  updateMarkers();
}

void RegionTool::deactivate()
{
  drawing_region_ = false;
  current_region_points_.clear();
}

int RegionTool::processMouseEvent(rviz::ViewportMouseEvent& event)
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
      drawing_region_ = true;
    }
  }
  else if (event.rightDown() && drawing_region_)
  {
    finishCurrentRegion();
  }

  return Render;
}

void RegionTool::updateProperties()
{
  updateMarkers();
}

void RegionTool::addPoint(const geometry_msgs::Point& point)
{
  current_region_points_.push_back(point);
  updateMarkers();
}

void RegionTool::finishCurrentRegion()
{
  if (current_region_points_.size() >= 3)
  {
    Region region;
    region.points = current_region_points_;
    region.id = generateRegionId();
    region.frame_id = frame_id_property_->getStdString();
    region.type = region_type_property_->getInt();
    region.param = region_param_property_->getFloat();
    regions_.push_back(region);
  }
  
  current_region_points_.clear();
  drawing_region_ = false;
  updateMarkers();
}

std::string RegionTool::generateRegionId()
{
  return "region" + std::to_string(region_id_counter_++);
}

void RegionTool::updateMarkers()
{
  marker_array_.markers.clear();
  marker_id_counter_ = 0;

  QColor color = color_property_->getColor();
  float alpha = alpha_property_->getFloat();

  // Draw existing regions
  for (const auto& region : regions_)
  {
    if (region.points.size() >= 3)
    {
      // Create filled polygon
      visualization_msgs::Marker polygon_marker;
      polygon_marker.header.frame_id = region.frame_id;
      polygon_marker.header.stamp = ros::Time::now();
      polygon_marker.ns = "regions";
      polygon_marker.id = marker_id_counter_++;
      polygon_marker.type = visualization_msgs::Marker::TRIANGLE_LIST;
      polygon_marker.action = visualization_msgs::Marker::ADD;
      polygon_marker.pose.orientation.w = 1.0;
      polygon_marker.scale.x = polygon_marker.scale.y = polygon_marker.scale.z = 1.0;
      polygon_marker.color.r = color.redF();
      polygon_marker.color.g = color.greenF();
      polygon_marker.color.b = color.blueF();
      polygon_marker.color.a = alpha;

      // Simple triangulation for convex polygons
      for (size_t i = 1; i < region.points.size() - 1; ++i)
      {
        polygon_marker.points.push_back(region.points[0]);
        polygon_marker.points.push_back(region.points[i]);
        polygon_marker.points.push_back(region.points[i + 1]);
      }

      marker_array_.markers.push_back(polygon_marker);

      // Create outline
      visualization_msgs::Marker outline_marker;
      outline_marker.header.frame_id = region.frame_id;
      outline_marker.header.stamp = ros::Time::now();
      outline_marker.ns = "region_outlines";
      outline_marker.id = marker_id_counter_++;
      outline_marker.type = visualization_msgs::Marker::LINE_STRIP;
      outline_marker.action = visualization_msgs::Marker::ADD;
      outline_marker.pose.orientation.w = 1.0;
      outline_marker.scale.x = 0.02;
      outline_marker.color.r = color.redF();
      outline_marker.color.g = color.greenF();
      outline_marker.color.b = color.blueF();
      outline_marker.color.a = 1.0;

      for (const auto& point : region.points)
      {
        outline_marker.points.push_back(point);
      }
      // Close the polygon
      outline_marker.points.push_back(region.points[0]);

      marker_array_.markers.push_back(outline_marker);

      // Add region label
      visualization_msgs::Marker text_marker;
      text_marker.header.frame_id = region.frame_id;
      text_marker.header.stamp = ros::Time::now();
      text_marker.ns = "region_labels";
      text_marker.id = marker_id_counter_++;
      text_marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
      text_marker.action = visualization_msgs::Marker::ADD;
      
      // Calculate centroid
      geometry_msgs::Point centroid;
      centroid.x = centroid.y = centroid.z = 0.0;
      for (const auto& point : region.points)
      {
        centroid.x += point.x;
        centroid.y += point.y;
      }
      centroid.x /= region.points.size();
      centroid.y /= region.points.size();
      centroid.z = 0.1;
      
      text_marker.pose.position = centroid;
      text_marker.pose.orientation.w = 1.0;
      text_marker.scale.z = 0.1;
      text_marker.color.r = 1.0;
      text_marker.color.g = 1.0;
      text_marker.color.b = 1.0;
      text_marker.color.a = 1.0;
      text_marker.text = region.id + " (T:" + std::to_string(region.type) + ")";

      marker_array_.markers.push_back(text_marker);
    }
  }

  // Draw current region being created
  if (current_region_points_.size() >= 2)
  {
    visualization_msgs::Marker current_outline;
    current_outline.header.frame_id = frame_id_property_->getStdString();
    current_outline.header.stamp = ros::Time::now();
    current_outline.ns = "current_region";
    current_outline.id = marker_id_counter_++;
    current_outline.type = visualization_msgs::Marker::LINE_STRIP;
    current_outline.action = visualization_msgs::Marker::ADD;
    current_outline.pose.orientation.w = 1.0;
    current_outline.scale.x = 0.03;
    current_outline.color.r = 1.0;
    current_outline.color.g = 1.0;
    current_outline.color.b = 0.0; // Yellow for current region
    current_outline.color.a = 1.0;

    for (const auto& point : current_region_points_)
    {
      current_outline.points.push_back(point);
    }

    marker_array_.markers.push_back(current_outline);
  }

  publishMarkers();
}

void RegionTool::publishMarkers()
{
  marker_pub_.publish(marker_array_);
}

void RegionTool::loadRegions(const std::string& filename)
{
  std::ifstream file(filename);
  if (!file.is_open())
  {
    setStatus("Failed to open regions file: " + QString::fromStdString(filename));
    return;
  }

  Json::Value root;
  file >> root;

  regions_.clear();
  
  if (root.isMember("regions") && root["regions"].isArray())
  {
    for (const auto& region_json : root["regions"])
    {
      Region region;
      region.id = region_json["id"].asString();
      region.frame_id = region_json["frame_id"].asString();
      region.type = region_json["type"].asInt();
      region.param = region_json["param"].asDouble();
      
      if (region_json.isMember("points") && region_json["points"].isArray())
      {
        for (const auto& point_json : region_json["points"])
        {
          geometry_msgs::Point point;
          point.x = point_json["x"].asDouble();
          point.y = point_json["y"].asDouble();
          point.z = point_json["z"].asDouble();
          region.points.push_back(point);
        }
        
        if (region.points.size() >= 3)
        {
          regions_.push_back(region);
        }
      }
    }
  }

  updateMarkers();
  setStatus("Loaded " + QString::number(regions_.size()) + " regions");
}

void RegionTool::saveRegions(const std::string& filename)
{
  Json::Value root;
  Json::Value regions_array(Json::arrayValue);

  for (const auto& region : regions_)
  {
    if (region.points.size() >= 3)
    {
      Json::Value region_json;
      region_json["id"] = region.id;
      region_json["frame_id"] = region.frame_id;
      region_json["type"] = region.type;
      region_json["param"] = region.param;
      
      Json::Value points_array(Json::arrayValue);
      for (const auto& point : region.points)
      {
        Json::Value point_json;
        point_json["x"] = point.x;
        point_json["y"] = point.y;
        point_json["z"] = point.z;
        points_array.append(point_json);
      }
      
      region_json["points"] = points_array;
      regions_array.append(region_json);
    }
  }

  root["regions"] = regions_array;

  std::ofstream file(filename);
  if (file.is_open())
  {
    Json::StreamWriterBuilder builder;
    builder["indentation"] = "  ";
    std::unique_ptr<Json::StreamWriter> writer(builder.newStreamWriter());
    writer->write(root, &file);
    
    setStatus("Saved " + QString::number(regions_.size()) + " regions");
  }
  else
  {
    setStatus("Failed to save regions file: " + QString::fromStdString(filename));
  }
}

void RegionTool::clearRegions()
{
  regions_.clear();
  current_region_points_.clear();
  drawing_region_ = false;
  updateMarkers();
  setStatus("Cleared all regions");
}

std::vector<Region> RegionTool::getRegions() const
{
  return regions_;
}

void RegionTool::setRegions(const std::vector<Region>& regions)
{
  regions_ = regions;
  current_region_points_.clear();
  drawing_region_ = false;
  updateMarkers();
  setStatus("Set " + QString::number(regions_.size()) + " regions");
}

void RegionTool::autoLoadRegions()
{
  // 首先清空现有的区域，发布空的markers
  regions_.clear();
  current_region_points_.clear();
  drawing_region_ = false;
  updateMarkers(); // 这会发布空的marker数组
  
  // 从参数服务器获取当前地图文件路径
  ros::NodeHandle nh;
  std::string map_file;
  if (nh.getParam("/map_server/map_file", map_file))
  {
    // 构造对应的区域文件路径
    std::string region_file = map_file;
    size_t dot_pos = region_file.find_last_of(".");
    if (dot_pos != std::string::npos)
    {
      region_file = region_file.substr(0, dot_pos) + "_region.json";
    }
    else
    {
      region_file += "_region.json";
    }
    
    // 检查文件是否存在
    std::ifstream test_file(region_file);
    if (test_file.good())
    {
      test_file.close();
      ROS_INFO("找到对应的区域文件，正在加载: %s", region_file.c_str());
      
      // 延迟加载以确保RViz完全初始化
      QTimer::singleShot(1000, [this, region_file]() {
        loadRegions(region_file);
        setStatus("区域已加载: " + QString::fromStdString(region_file));
      });
    }
    else
    {
      ROS_INFO("当前地图没有对应的区域文件: %s，发布空的区域", region_file.c_str());
      setStatus("当前地图无区域文件，显示为空");
    }
  }
  else
  {
    ROS_INFO("未找到地图文件参数，发布空的区域");
    setStatus("未加载地图，区域为空");
  }
}

void RegionTool::loadRegionsForMap(const std::string& map_file_path)
{
  // 新增方法：根据地图文件路径加载对应的区域文件
  regions_.clear();
  current_region_points_.clear();
  drawing_region_ = false;
  
  // 构造区域文件路径
  std::string region_file = map_file_path;
  size_t dot_pos = region_file.find_last_of(".");
  if (dot_pos != std::string::npos)
  {
    region_file = region_file.substr(0, dot_pos) + "_region.json";
  }
  else
  {
    region_file += "_region.json";
  }
  
  // 检查文件是否存在
  std::ifstream test_file(region_file);
  if (test_file.good())
  {
    test_file.close();
    ROS_INFO("加载区域文件: %s", region_file.c_str());
    loadRegions(region_file);
  }
  else
  {
    ROS_INFO("地图文件 %s 没有对应的区域文件，显示为空", map_file_path.c_str());
    updateMarkers(); // 发布空的markers
  }
}

} // end namespace ros_map_edit

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(ros_map_edit::RegionTool, rviz::Tool) 