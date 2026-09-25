#include "ros_map_edit/map_file_manager.h"
#include <fstream>
#include <sstream>
#include <iostream>
#include <yaml-cpp/yaml.h>
#include <ros/ros.h>
#include <visualization_msgs/MarkerArray.h>
#include <curl/curl.h>
#include <sys/stat.h>
#include <unistd.h>

namespace ros_map_edit
{

MapFileManager::MapFileManager() : curl_initialized_(false)
{
  initCurl();
}

MapFileManager::~MapFileManager()
{
  cleanupCurl();
}

bool MapFileManager::loadVirtualWalls(const std::string& filename, std::vector<VirtualWall>& walls)
{
  try
  {
    std::ifstream file(filename);
    if (!file.is_open())
    {
      last_error_ = "Failed to open file: " + filename;
      return false;
    }

    Json::Value root;
    file >> root;

    walls.clear();
    
    if (root.isMember("vws") && root["vws"].isArray())
    {
      for (const auto& vw_json : root["vws"])
      {
        if (vw_json.isMember("points") && vw_json["points"].isArray())
        {
          VirtualWall wall;
          wall.id = "wall_" + std::to_string(walls.size());
          
          for (const auto& point_json : vw_json["points"])
          {
            geometry_msgs::Point point = jsonToPoint(point_json);
            wall.points.push_back(point);
          }
          
          if (wall.points.size() >= 2)
          {
            walls.push_back(wall);
          }
        }
      }
    }

    return true;
  }
  catch (const std::exception& e)
  {
    last_error_ = "Error parsing JSON: " + std::string(e.what());
    return false;
  }
}

bool MapFileManager::saveVirtualWalls(const std::string& filename, const std::vector<VirtualWall>& walls)
{
  try
  {
    Json::Value root;
    Json::Value vws_array(Json::arrayValue);

    for (const auto& wall : walls)
    {
      if (wall.points.size() >= 2)
      {
        Json::Value wall_json;
        Json::Value points_array(Json::arrayValue);
        
        for (const auto& point : wall.points)
        {
          points_array.append(pointToJson(point));
        }
        
        wall_json["points"] = points_array;
        vws_array.append(wall_json);
      }
    }

    root["vws"] = vws_array;

    std::ofstream file(filename);
    if (!file.is_open())
    {
      last_error_ = "Failed to create file: " + filename;
      return false;
    }

    Json::StreamWriterBuilder builder;
    builder["indentation"] = "   ";
    std::unique_ptr<Json::StreamWriter> writer(builder.newStreamWriter());
    writer->write(root, &file);
    
    return true;
  }
  catch (const std::exception& e)
  {
    last_error_ = "Error writing JSON: " + std::string(e.what());
    return false;
  }
}

bool MapFileManager::loadRegions(const std::string& filename, std::vector<Region>& regions)
{
  try
  {
    std::ifstream file(filename);
    if (!file.is_open())
    {
      last_error_ = "Failed to open file: " + filename;
      return false;
    }

    Json::Value root;
    file >> root;

    regions.clear();
    
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
            geometry_msgs::Point point = jsonToPoint(point_json);
            region.points.push_back(point);
          }
          
          if (region.points.size() >= 3)
          {
            regions.push_back(region);
          }
        }
      }
    }

    return true;
  }
  catch (const std::exception& e)
  {
    last_error_ = "Error parsing JSON: " + std::string(e.what());
    return false;
  }
}

bool MapFileManager::saveRegions(const std::string& filename, const std::vector<Region>& regions)
{
  try
  {
    Json::Value root;
    Json::Value regions_array(Json::arrayValue);

    for (const auto& region : regions)
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
          points_array.append(pointToJson(point));
        }
        
        region_json["points"] = points_array;
        regions_array.append(region_json);
      }
    }

    root["regions"] = regions_array;

    std::ofstream file(filename);
    if (!file.is_open())
    {
      last_error_ = "Failed to create file: " + filename;
      return false;
    }

    Json::StreamWriterBuilder builder;
    builder["indentation"] = "  ";
    std::unique_ptr<Json::StreamWriter> writer(builder.newStreamWriter());
    writer->write(root, &file);
    
    return true;
  }
  catch (const std::exception& e)
  {
    last_error_ = "Error writing JSON: " + std::string(e.what());
    return false;
  }
}

bool MapFileManager::loadMap(const std::string& filename, nav_msgs::OccupancyGrid& map)
{
  // Determine file type from extension
  std::string ext = filename.substr(filename.find_last_of(".") + 1);
  
  if (ext == "yaml" || ext == "yml")
  {
    return loadYAML(filename, map);
  }
  else if (ext == "pgm")
  {
    return loadPGM(filename, map);
  }
  else
  {
    last_error_ = "Unsupported file format: " + ext;
    return false;
  }
}

bool MapFileManager::saveMap(const std::string& filename, const nav_msgs::OccupancyGrid& map)
{
  // Determine file type from extension
  std::string ext = filename.substr(filename.find_last_of(".") + 1);
  
  if (ext == "yaml" || ext == "yml")
  {
    return saveYAML(filename, map);
  }
  else if (ext == "pgm")
  {
    return savePGM(filename, map);
  }
  else
  {
    last_error_ = "Unsupported file format: " + ext;
    return false;
  }
}

bool MapFileManager::saveMapImage(const std::string& filename, const nav_msgs::OccupancyGrid& map)
{
  return savePGM(filename, map);
}

bool MapFileManager::saveMapFiles(const std::string& yaml_filename)
{
  // 尝试从map_edited话题获取当前编辑的地图
  ros::NodeHandle nh;
  nav_msgs::OccupancyGridConstPtr map_msg = 
    ros::topic::waitForMessage<nav_msgs::OccupancyGrid>("map_edited", nh, ros::Duration(1.0));
  
  if (!map_msg)
  {
    // 如果没有编辑后的地图，尝试获取原始地图
    map_msg = ros::topic::waitForMessage<nav_msgs::OccupancyGrid>("map", nh, ros::Duration(5.0));
    
    if (!map_msg)
    {
      last_error_ = "无法获取地图数据，请确保地图话题正在发布";
      return false;
    }
  }
  
  return saveYAML(yaml_filename, *map_msg);
}

bool MapFileManager::saveMapFiles(const std::string& yaml_filename, const nav_msgs::OccupancyGrid& map)
{
  // 直接保存传入的地图数据
  return saveYAML(yaml_filename, map);
}

bool MapFileManager::saveVirtualWallsFile(const std::string& filename)
{
  // 尝试从虚拟墙话题获取数据
  ros::NodeHandle nh;
  visualization_msgs::MarkerArrayConstPtr walls_msg = 
    ros::topic::waitForMessage<visualization_msgs::MarkerArray>("virtual_walls_markers", nh, ros::Duration(1.0));
  
  // 创建基本的空文件结构
  Json::Value root;
  root["vws"] = Json::Value(Json::arrayValue);
  
  if (walls_msg && !walls_msg->markers.empty())
  {
    // 转换MarkerArray到VirtualWall格式
    std::vector<VirtualWall> walls;
    
    for (const auto& marker : walls_msg->markers)
    {
      if (marker.type == visualization_msgs::Marker::LINE_STRIP && 
          marker.points.size() >= 2)
      {
        VirtualWall current_wall;
        current_wall.points.clear();
        for (const auto& point : marker.points)
        {
          geometry_msgs::Point p;
          p.x = point.x;
          p.y = point.y;
          p.z = point.z;
          current_wall.points.push_back(p);
        }
        current_wall.id = "wall_" + std::to_string(marker.id);
        walls.push_back(current_wall);
      }
    }
    
    return saveVirtualWalls(filename, walls);
  }
  
  // 保存空文件
  std::ofstream file(filename);
  if (!file.is_open())
  {
    last_error_ = "Failed to create virtual walls file: " + filename;
    return false;
  }

  Json::StreamWriterBuilder builder;
  builder["indentation"] = "   ";
  std::unique_ptr<Json::StreamWriter> writer(builder.newStreamWriter());
  writer->write(root, &file);
  
  return true;
}

bool MapFileManager::saveVirtualWallsFile(const std::string& filename, const std::vector<VirtualWall>& walls)
{
  // 直接保存传入的虚拟墙数据
  return saveVirtualWalls(filename, walls);
}

bool MapFileManager::saveRegionsFile(const std::string& filename)
{
  // 尝试从区域话题获取数据
  ros::NodeHandle nh;
  visualization_msgs::MarkerArrayConstPtr regions_msg = 
    ros::topic::waitForMessage<visualization_msgs::MarkerArray>("region_markers", nh, ros::Duration(1.0));
  
  // 创建基本的空文件结构
  Json::Value root;
  root["regions"] = Json::Value(Json::arrayValue);
  
  if (regions_msg && !regions_msg->markers.empty())
  {
    // 转换MarkerArray到Region格式
    std::vector<Region> regions;
    
    for (const auto& marker : regions_msg->markers)
    {
      if (marker.type == visualization_msgs::Marker::TRIANGLE_LIST && 
          marker.points.size() >= 3)
      {
        Region region;
        region.id = "region_" + std::to_string(marker.id);
        region.frame_id = marker.header.frame_id;
        region.type = 0;
        region.param = 1.0;
        
        // 从三角形列表重构多边形（简化处理）
        std::set<std::pair<double, double>> unique_points;
        for (const auto& point : marker.points)
        {
          unique_points.insert({point.x, point.y});
        }
        
        for (const auto& unique_point : unique_points)
        {
          geometry_msgs::Point p;
          p.x = unique_point.first;
          p.y = unique_point.second;
          p.z = 0.0;
          region.points.push_back(p);
        }
        
        regions.push_back(region);
      }
    }
    
    return saveRegions(filename, regions);
  }
  
  // 保存空文件
  std::ofstream file(filename);
  if (!file.is_open())
  {
    last_error_ = "Failed to create regions file: " + filename;
    return false;
  }

  Json::StreamWriterBuilder builder;
  builder["indentation"] = "  ";
  std::unique_ptr<Json::StreamWriter> writer(builder.newStreamWriter());
  writer->write(root, &file);
  
  return true;
}

bool MapFileManager::saveRegionsFile(const std::string& filename, const std::vector<Region>& regions)
{
  // 直接保存传入的区域数据
  return saveRegions(filename, regions);
}

Json::Value MapFileManager::pointToJson(const geometry_msgs::Point& point)
{
  Json::Value json;
  json["x"] = point.x;
  json["y"] = point.y;
  json["z"] = point.z;
  return json;
}

geometry_msgs::Point MapFileManager::jsonToPoint(const Json::Value& json)
{
  geometry_msgs::Point point;
  point.x = json["x"].asDouble();
  point.y = json["y"].asDouble();
  point.z = json.isMember("z") ? json["z"].asDouble() : 0.0;
  return point;
}

bool MapFileManager::loadPGMWithParams(const std::string& filename, nav_msgs::OccupancyGrid& map,
                                     double occupied_thresh, double free_thresh, bool negate)
{
  std::ifstream file(filename, std::ios::binary);
  if (!file.is_open())
  {
    last_error_ = "Failed to open PGM file: " + filename;
    return false;
  }

  std::string format;
  int width, height, maxval;
  
  // 读取PGM文件头
  file >> format;
  if (format != "P5")
  {
    last_error_ = "Only P5 PGM format is supported";
    return false;
  }
  
  // 跳过注释行
  std::string line;
  std::getline(file, line); // 跳过format行的剩余部分
  while (std::getline(file, line) && line[0] == '#') {
    // 跳过注释行
  }
  
  // 解析宽度和高度（可能在同一行或不同行）
  std::istringstream iss(line);
  if (!(iss >> width)) {
    last_error_ = "Failed to read width from PGM file";
    return false;
  }
  
  if (!(iss >> height)) {
    // 如果高度不在同一行，读取下一行
    if (!std::getline(file, line) || !(std::istringstream(line) >> height)) {
      last_error_ = "Failed to read height from PGM file";
      return false;
    }
  }
  
  // 读取最大值
  file >> maxval;
  if (maxval != 255) {
    last_error_ = "Only 8-bit PGM files (maxval 255) are supported";
    return false;
  }
  
  file.ignore(1); // 跳过最后一个换行符
  
  // 设置地图信息
  map.info.width = width;
  map.info.height = height;
  // 如果resolution还未设置（即为0），设置默认值
  if (map.info.resolution == 0.0) {
    map.info.resolution = 0.05; // 默认分辨率，通常由YAML文件提供
    map.info.origin.position.x = 0.0;
    map.info.origin.position.y = 0.0;
    map.info.origin.position.z = 0.0;
    map.info.origin.orientation.w = 1.0;
  }
  
  map.data.resize(width * height);
  
  // 读取图像数据
  std::vector<uint8_t> image_data(width * height);
  file.read(reinterpret_cast<char*>(image_data.data()), width * height);
  
  if (!file) {
    last_error_ = "Failed to read image data from PGM file";
    return false;
  }
  
  // 根据ROS map_server的标准转换PGM数据到占用栅格
  // 参考ROS wiki: http://wiki.ros.org/map_server#Value_Interpretation
  // 注意：需要翻转Y轴以匹配ROS地图坐标系（Y轴向上）
  for (int y = 0; y < height; ++y)
  {
    for (int x = 0; x < width; ++x)
    {
      // 图像坐标系到地图坐标系的转换：翻转Y轴
      int image_index = y * width + x;  // 图像中的索引（Y轴向下）
      int map_index = (height - 1 - y) * width + x;  // 地图中的索引（Y轴向上）
      
      uint8_t pixel = image_data[image_index];
      
      // 转换像素值到概率 (根据negate标志)
      double p;
      if (negate) {
        p = pixel / 255.0;
      } else {
        p = (255 - pixel) / 255.0;
      }
      
      // 根据阈值转换为占用栅格值
      if (p > occupied_thresh) {
        map.data[map_index] = 100;  // 占用
      } else if (p < free_thresh) {
        map.data[map_index] = 0;    // 自由
      } else {
        map.data[map_index] = -1;   // 未知
      }
    }
  }
  
  return true;
}

bool MapFileManager::loadPGM(const std::string& filename, nav_msgs::OccupancyGrid& map)
{
  // 使用默认参数调用带参数的版本
  return loadPGMWithParams(filename, map, 0.65, 0.196, false);
}

bool MapFileManager::savePGM(const std::string& filename, const nav_msgs::OccupancyGrid& map)
{
  std::ofstream file(filename, std::ios::binary);
  if (!file.is_open())
  {
    last_error_ = "Failed to create PGM file: " + filename;
    return false;
  }

  // 写入PGM文件头
  file << "P5\n";
  file << "# Created by ros_map_edit\n";
  file << map.info.width << " " << map.info.height << "\n";
  file << "255\n";
  
  // 准备图像数据
  std::vector<uint8_t> image_data(map.info.width * map.info.height);
  
  // 根据ROS map_server的标准转换占用栅格到PGM数据
  const double occupied_thresh = 0.65;
  const double free_thresh = 0.196;
  const bool negate = false;
  
  // 注意：需要翻转Y轴以匹配图像坐标系（Y轴向下）
  for (int y = 0; y < static_cast<int>(map.info.height); ++y)
  {
    for (int x = 0; x < static_cast<int>(map.info.width); ++x)
    {
      // 地图坐标系到图像坐标系的转换：翻转Y轴
      int map_index = (map.info.height - 1 - y) * map.info.width + x;  // 地图中的索引（Y轴向上）
      int image_index = y * map.info.width + x;  // 图像中的索引（Y轴向下）
      
      int8_t cell = map.data[map_index];
      uint8_t pixel;
      
      if (cell == 0) {
        // 自由空间 -> 根据negate转换
        pixel = negate ? 0 : 254;
      } else if (cell == 100) {
        // 占用空间 -> 根据negate转换  
        pixel = negate ? 254 : 0;
      } else {
        // 未知空间 -> 灰色
        pixel = 205;
      }
      
      image_data[image_index] = pixel;
    }
  }
  
  // 写入图像数据
  file.write(reinterpret_cast<char*>(image_data.data()), image_data.size());
  
  if (!file) {
    last_error_ = "Failed to write image data to PGM file";
    return false;
  }
  
  return true;
}

bool MapFileManager::loadYAML(const std::string& filename, nav_msgs::OccupancyGrid& map)
{
  try
  {
    ROS_INFO("正在解析YAML文件: %s", filename.c_str());
    YAML::Node config = YAML::LoadFile(filename);
    
    // 解析地图参数
    map.info.resolution = config["resolution"].as<double>();
    
    // 解析origin数组
    if (config["origin"] && config["origin"].IsSequence() && config["origin"].size() >= 2)
    {
      map.info.origin.position.x = config["origin"][0].as<double>();
      map.info.origin.position.y = config["origin"][1].as<double>();
      map.info.origin.position.z = 0.0;
      map.info.origin.orientation.w = 1.0;
    }
    else
    {
      last_error_ = "Invalid or missing origin field in YAML file";
      return false;
    }
    
    // 解析PGM解释参数，使用默认值如果不存在
    double occupied_thresh = 0.65;
    double free_thresh = 0.196;
    bool negate = false;
    
    if (config["occupied_thresh"].IsDefined())
    {
      occupied_thresh = config["occupied_thresh"].as<double>();
    }
    
    if (config["free_thresh"].IsDefined())
    {
      free_thresh = config["free_thresh"].as<double>();
    }
    
    if (config["negate"].IsDefined())
    {
      // negate字段可能是布尔值或整数，需要兼容处理
      try {
        negate = config["negate"].as<bool>();
      } catch (const YAML::BadConversion&) {
        // 如果无法转换为布尔值，尝试作为整数处理
        int negate_int = config["negate"].as<int>();
        negate = (negate_int != 0);
      }
    }
    
    ROS_INFO("YAML解析成功 - 分辨率: %.3f, 原点: [%.2f, %.2f], occupied_thresh: %.3f, free_thresh: %.3f, negate: %s", 
             map.info.resolution, map.info.origin.position.x, map.info.origin.position.y,
             occupied_thresh, free_thresh, negate ? "true" : "false");
    
    // 获取图像文件名
    if (!config["image"].IsDefined())
    {
      last_error_ = "Missing image field in YAML file";
      return false;
    }
    
    std::string image_file = config["image"].as<std::string>();
    ROS_INFO("YAML中指定的图像文件: %s", image_file.c_str());
    
    // 构建PGM文件的完整路径
    std::string pgm_filename;
    
    // 如果image_file是绝对路径，直接使用
    if (image_file[0] == '/' || (image_file.size() > 1 && image_file[1] == ':'))
    {
      pgm_filename = image_file;
    }
    else
    {
      // 相对路径：与YAML文件同目录
      size_t last_slash = filename.find_last_of("/\\");
      if (last_slash != std::string::npos)
      {
        std::string dir = filename.substr(0, last_slash + 1);
        pgm_filename = dir + image_file;
      }
      else
      {
        // YAML文件在当前目录
        pgm_filename = image_file;
      }
    }
    
    ROS_INFO("构建的PGM文件路径: %s", pgm_filename.c_str());
    
    // 检查PGM文件是否存在
    std::ifstream test_file(pgm_filename);
    if (!test_file.good())
    {
      last_error_ = "PGM文件不存在: " + pgm_filename;
      ROS_ERROR("PGM文件不存在: %s", pgm_filename.c_str());
      
      // 尝试在几个可能的位置查找
      std::vector<std::string> search_paths = {
        image_file,  // 原始名称
        "maps/" + image_file,  // maps目录
        "../maps/" + image_file,  // 上级maps目录
        "src/ros_map_edit/maps/" + image_file  // 完整相对路径
      };
      
      for (const auto& search_path : search_paths)
      {
        std::ifstream search_file(search_path);
        if (search_file.good())
        {
          search_file.close();
          pgm_filename = search_path;
          ROS_INFO("在替代路径找到PGM文件: %s", pgm_filename.c_str());
          break;
        }
      }
    }
    else
    {
      test_file.close();
    }
    
    // 加载PGM文件，使用YAML文件中的参数
    ROS_INFO("尝试加载PGM文件: %s", pgm_filename.c_str());
    bool success = loadPGMWithParams(pgm_filename, map, occupied_thresh, free_thresh, negate);
    
    if (success)
    {
      ROS_INFO("PGM文件加载成功: %dx%d像素", map.info.width, map.info.height);
    }
    else
    {
      ROS_ERROR("PGM文件加载失败: %s", last_error_.c_str());
    }
    
    return success;
  }
  catch (const std::exception& e)
  {
    last_error_ = "Error parsing YAML: " + std::string(e.what());
    ROS_ERROR("YAML解析错误: %s", e.what());
    return false;
  }
}

bool MapFileManager::saveYAML(const std::string& filename, const nav_msgs::OccupancyGrid& map)
{
  try
  {
    std::ofstream file(filename);
    if (!file.is_open())
    {
      last_error_ = "Failed to create YAML file: " + filename;
      return false;
    }

    // Generate PGM filename
    std::string base = filename.substr(0, filename.find_last_of("."));
    std::string pgm_filename = base + ".pgm";
    std::string pgm_basename = pgm_filename.substr(pgm_filename.find_last_of("/\\") + 1);
    
    // Save PGM file
    if (!savePGM(pgm_filename, map))
    {
      return false;
    }
    
    // Write YAML
    file << "image: " << pgm_basename << "\n";
    file << "resolution: " << map.info.resolution << "\n";
    file << "origin: [" << map.info.origin.position.x << ", " 
         << map.info.origin.position.y << ", 0.0]\n";
    file << "negate: 0\n";
    file << "occupied_thresh: 0.65\n";
    file << "free_thresh: 0.196\n";
    
    return true;
  }
  catch (const std::exception& e)
  {
    last_error_ = "Error writing YAML: " + std::string(e.what());
    return false;
  }
}

// ====================
// FTP功能实现
// ====================

// 用于curl写入文件的回调函数
static size_t WriteFileCallback(void *contents, size_t size, size_t nmemb, void *userp)
{
  std::ofstream* file = static_cast<std::ofstream*>(userp);
  size_t total_size = size * nmemb;
  file->write(static_cast<const char*>(contents), total_size);
  return file->good() ? total_size : 0;
}

// 用于curl读取文件的回调函数
static size_t ReadFileCallback(void *contents, size_t size, size_t nmemb, void *userp)
{
  std::ifstream* file = static_cast<std::ifstream*>(userp);
  size_t total_size = size * nmemb;
  file->read(static_cast<char*>(contents), total_size);
  return file->gcount();
}

// 用于curl获取目录列表的回调函数
static size_t WriteStringCallback(void *contents, size_t size, size_t nmemb, void *userp)
{
  std::string* str = static_cast<std::string*>(userp);
  size_t total_size = size * nmemb;
  str->append(static_cast<const char*>(contents), total_size);
  return total_size;
}

void MapFileManager::setFTPConfig(const FTPConfig& config)
{
  ftp_config_ = config;
}

FTPConfig MapFileManager::getFTPConfig() const
{
  return ftp_config_;
}

bool MapFileManager::initCurl()
{
  if (curl_initialized_)
    return true;
    
  CURLcode result = curl_global_init(CURL_GLOBAL_DEFAULT);
  if (result != CURLE_OK)
  {
    last_error_ = "Failed to initialize CURL: " + std::string(curl_easy_strerror(result));
    return false;
  }
  
  curl_initialized_ = true;
  return true;
}

void MapFileManager::cleanupCurl()
{
  if (curl_initialized_)
  {
    curl_global_cleanup();
    curl_initialized_ = false;
  }
}

std::string MapFileManager::buildFTPURL(const std::string& remote_path) const
{
  std::string url = "ftp://";
  if (!ftp_config_.username.empty())
  {
    url += ftp_config_.username;
    if (!ftp_config_.password.empty())
    {
      url += ":" + ftp_config_.password;
    }
    url += "@";
  }
  
  url += ftp_config_.host;
  
  if (ftp_config_.port != 21)
  {
    url += ":" + std::to_string(ftp_config_.port);
  }
  
  // 确保路径以/开头
  std::string full_path = ftp_config_.remote_dir;
  if (full_path.back() != '/')
    full_path += "/";
  full_path += remote_path;
  
  url += full_path;
  
  return url;
}

bool MapFileManager::testFTPConnection()
{
  if (!curl_initialized_)
  {
    last_error_ = "CURL not initialized";
    return false;
  }
  
  CURL* curl = curl_easy_init();
  if (!curl)
  {
    last_error_ = "Failed to initialize CURL handle";
    return false;
  }
  
  std::string url = buildFTPURL("");
  std::string response;
  
  curl_easy_setopt(curl, CURLOPT_URL, url.c_str());
  curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, WriteStringCallback);
  curl_easy_setopt(curl, CURLOPT_WRITEDATA, &response);
  curl_easy_setopt(curl, CURLOPT_TIMEOUT, static_cast<long>(ftp_config_.timeout));
  
  // libcurl默认使用被动模式，如果需要强制使用PASV而不是EPSV，可以关闭EPSV
  if (ftp_config_.use_passive)
  {
    curl_easy_setopt(curl, CURLOPT_FTP_USE_EPSV, 0L);
  }
  
  // 只是列出目录来测试连接
  curl_easy_setopt(curl, CURLOPT_DIRLISTONLY, 1L);
  
  CURLcode result = curl_easy_perform(curl);
  curl_easy_cleanup(curl);
  
  if (result != CURLE_OK)
  {
    std::string error_msg = std::string(curl_easy_strerror(result));
    
    // 提供更友好的错误提示
    if (result == CURLE_COULDNT_CONNECT)
    {
      last_error_ = "无法连接到FTP服务器 " + ftp_config_.host + ":" + std::to_string(ftp_config_.port) + 
                   "\n请检查:\n• 服务器地址是否正确\n• 服务器是否运行\n• 网络连接是否正常\n• 防火墙设置";
    }
    else if (result == CURLE_LOGIN_DENIED)
    {
      last_error_ = "FTP登录失败，请检查用户名和密码";
    }
    else if (result == CURLE_OPERATION_TIMEDOUT)
    {
      last_error_ = "连接超时，请检查网络连接或增加超时时间";
    }
    else
    {
      last_error_ = "FTP连接测试失败: " + error_msg;
    }
    return false;
  }
  
  return true;
}

bool MapFileManager::listFTPDirectory(std::vector<std::string>& file_list, const std::string& remote_dir)
{
  if (!curl_initialized_)
  {
    last_error_ = "CURL not initialized";
    return false;
  }
  
  CURL* curl = curl_easy_init();
  if (!curl)
  {
    last_error_ = "Failed to initialize CURL handle";
    return false;
  }
  
  std::string url = buildFTPURL(remote_dir);
  std::string response;
  
  curl_easy_setopt(curl, CURLOPT_URL, url.c_str());
  curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, WriteStringCallback);
  curl_easy_setopt(curl, CURLOPT_WRITEDATA, &response);
  curl_easy_setopt(curl, CURLOPT_TIMEOUT, static_cast<long>(ftp_config_.timeout));
  curl_easy_setopt(curl, CURLOPT_DIRLISTONLY, 1L);
  
  if (ftp_config_.use_passive)
  {
    curl_easy_setopt(curl, CURLOPT_FTP_USE_EPSV, 0L);
  }
  
  CURLcode result = curl_easy_perform(curl);
  curl_easy_cleanup(curl);
  
  if (result != CURLE_OK)
  {
    std::string error_msg = std::string(curl_easy_strerror(result));
    
    // 提供更友好的错误提示
    if (result == CURLE_REMOTE_ACCESS_DENIED)
    {
      last_error_ = "访问远程目录被拒绝\n请检查:\n• 目录路径是否正确\n• 用户权限是否足够\n• 目录是否存在";
    }
    else if (result == CURLE_FTP_ACCEPT_FAILED)
    {
      last_error_ = "FTP数据连接失败，请检查被动模式设置";
    }
    else if (result == CURLE_LOGIN_DENIED)
    {
      last_error_ = "FTP登录失败，请检查用户名和密码";
    }
    else
    {
      last_error_ = "获取FTP目录列表失败: " + error_msg + "\n目标路径: " + url;
    }
    return false;
  }
  
  // 解析响应，按行分割文件名
  file_list.clear();
  std::istringstream iss(response);
  std::string line;
  while (std::getline(iss, line))
  {
    if (!line.empty())
    {
      // 移除行末的回车符
      if (line.back() == '\r')
        line.pop_back();
      file_list.push_back(line);
    }
  }
  
  return true;
}

bool MapFileManager::createFTPDirectory(const std::string& remote_dir)
{
  if (!curl_initialized_)
  {
    last_error_ = "CURL not initialized";
    return false;
  }
  
  CURL* curl = curl_easy_init();
  if (!curl)
  {
    last_error_ = "Failed to initialize CURL handle";
    return false;
  }
  
  std::string url = buildFTPURL("");
  std::string mkdir_cmd = "MKD " + remote_dir;
  
  struct curl_slist* commands = nullptr;
  commands = curl_slist_append(commands, mkdir_cmd.c_str());
  
  curl_easy_setopt(curl, CURLOPT_URL, url.c_str());
  curl_easy_setopt(curl, CURLOPT_QUOTE, commands);
  curl_easy_setopt(curl, CURLOPT_TIMEOUT, static_cast<long>(ftp_config_.timeout));
  
  if (ftp_config_.use_passive)
  {
    curl_easy_setopt(curl, CURLOPT_FTP_USE_EPSV, 0L);
  }
  
  CURLcode result = curl_easy_perform(curl);
  
  curl_slist_free_all(commands);
  curl_easy_cleanup(curl);
  
  if (result != CURLE_OK)
  {
    last_error_ = "Failed to create FTP directory: " + std::string(curl_easy_strerror(result));
    return false;
  }
  
  return true;
}

bool MapFileManager::downloadFileFromFTP(const std::string& remote_filename, const std::string& local_filename)
{
  if (!curl_initialized_)
  {
    last_error_ = "CURL not initialized";
    return false;
  }
  
  CURL* curl = curl_easy_init();
  if (!curl)
  {
    last_error_ = "Failed to initialize CURL handle";
    return false;
  }
  
  std::ofstream file(local_filename, std::ios::binary);
  if (!file.is_open())
  {
    last_error_ = "Failed to create local file: " + local_filename;
    curl_easy_cleanup(curl);
    return false;
  }
  
  std::string url = buildFTPURL(remote_filename);
  
  curl_easy_setopt(curl, CURLOPT_URL, url.c_str());
  curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, WriteFileCallback);
  curl_easy_setopt(curl, CURLOPT_WRITEDATA, &file);
  curl_easy_setopt(curl, CURLOPT_TIMEOUT, static_cast<long>(ftp_config_.timeout));
  
  if (ftp_config_.use_passive)
  {
    curl_easy_setopt(curl, CURLOPT_FTP_USE_EPSV, 0L);
  }
  
  CURLcode result = curl_easy_perform(curl);
  curl_easy_cleanup(curl);
  file.close();
  
  if (result != CURLE_OK)
  {
    last_error_ = "Failed to download file from FTP: " + std::string(curl_easy_strerror(result));
    // 删除可能部分下载的文件
    unlink(local_filename.c_str());
    return false;
  }
  
  return true;
}

bool MapFileManager::uploadFileToFTP(const std::string& local_filename, const std::string& remote_filename)
{
  if (!curl_initialized_)
  {
    last_error_ = "CURL not initialized";
    return false;
  }
  
  // 检查本地文件是否存在
  struct stat file_info;
  if (stat(local_filename.c_str(), &file_info) != 0)
  {
    last_error_ = "Local file does not exist: " + local_filename;
    return false;
  }
  
  CURL* curl = curl_easy_init();
  if (!curl)
  {
    last_error_ = "Failed to initialize CURL handle";
    return false;
  }
  
  std::ifstream file(local_filename, std::ios::binary);
  if (!file.is_open())
  {
    last_error_ = "Failed to open local file: " + local_filename;
    curl_easy_cleanup(curl);
    return false;
  }
  
  std::string url = buildFTPURL(remote_filename);
  
  // 启用详细输出用于调试
  curl_easy_setopt(curl, CURLOPT_VERBOSE, 1L);
  curl_easy_setopt(curl, CURLOPT_URL, url.c_str());
  curl_easy_setopt(curl, CURLOPT_UPLOAD, 1L);
  curl_easy_setopt(curl, CURLOPT_READFUNCTION, ReadFileCallback);
  curl_easy_setopt(curl, CURLOPT_READDATA, &file);
  curl_easy_setopt(curl, CURLOPT_INFILESIZE_LARGE, static_cast<curl_off_t>(file_info.st_size));
  curl_easy_setopt(curl, CURLOPT_TIMEOUT, static_cast<long>(ftp_config_.timeout));
  
  if (ftp_config_.use_passive)
  {
    curl_easy_setopt(curl, CURLOPT_FTP_USE_EPSV, 0L);
  }
  
  // 尝试创建目录结构
  if (ftp_config_.remote_dir != "/" && !ftp_config_.remote_dir.empty())
  {
    struct curl_slist* commands = nullptr;
    std::string mkdir_cmd = "MKD " + ftp_config_.remote_dir;
    commands = curl_slist_append(commands, mkdir_cmd.c_str());
    curl_easy_setopt(curl, CURLOPT_QUOTE, commands);
  }
  
  CURLcode result = curl_easy_perform(curl);
  
  // 获取HTTP响应码
  long response_code;
  curl_easy_getinfo(curl, CURLINFO_RESPONSE_CODE, &response_code);
  
  curl_easy_cleanup(curl);
  file.close();
  
  if (result != CURLE_OK)
  {
    std::string error_msg = std::string(curl_easy_strerror(result));
    
    // 提供更详细的错误信息
    if (result == CURLE_UPLOAD_FAILED)
    {
      last_error_ = "FTP上传失败 - 可能的原因:\n"
                   "• FTP用户没有写权限\n"
                   "• 目录不存在或无法创建\n"
                   "• 磁盘空间不足\n"
                   "详细错误: " + error_msg + "\n"
                   "响应码: " + std::to_string(response_code) + "\n"
                   "URL: " + url;
    }
    else if (result == CURLE_REMOTE_ACCESS_DENIED)
    {
      last_error_ = "FTP访问被拒绝 - FTP用户oryxbot没有足够权限\n"
                   "请联系FTP管理员授予写权限或使用其他目录\n"
                   "目标路径: " + url;
    }
    else if (result == CURLE_FTP_ACCEPT_FAILED)
    {
      last_error_ = "FTP数据连接失败 - 请检查被动模式设置\n"
                   "详细错误: " + error_msg;
    }
    else
    {
      last_error_ = "FTP上传失败: " + error_msg + "\n响应码: " + std::to_string(response_code);
    }
    return false;
  }
  
  return true;
}

bool MapFileManager::loadMapFromFTP(const std::string& remote_filename, nav_msgs::OccupancyGrid& map)
{
  // 创建临时文件
  std::string temp_filename = "/tmp/ros_map_edit_temp_" + std::to_string(getpid()) + "_map.yaml";
  
  // 从FTP下载文件
  if (!downloadFileFromFTP(remote_filename, temp_filename))
  {
    return false;
  }
  
  // 加载本地临时文件
  bool result = loadMap(temp_filename, map);
  
  // 清理临时文件
  unlink(temp_filename.c_str());
  
  return result;
}

bool MapFileManager::saveMapToFTP(const std::string& remote_filename, const nav_msgs::OccupancyGrid& map)
{
  // 创建临时文件
  std::string temp_filename = "/tmp/ros_map_edit_temp_" + std::to_string(getpid()) + "_map.yaml";
  
  // 保存到本地临时文件
  if (!saveMap(temp_filename, map))
  {
    return false;
  }
  
  // 上传到FTP
  bool result = uploadFileToFTP(temp_filename, remote_filename);
  
  // 清理临时文件
  unlink(temp_filename.c_str());
  
  // 如果是YAML文件，还需要上传对应的PGM文件
  if (result && (remote_filename.find(".yaml") != std::string::npos || remote_filename.find(".yml") != std::string::npos))
  {
    std::string base = temp_filename.substr(0, temp_filename.find_last_of("."));
    std::string temp_pgm = base + ".pgm";
    
    std::string remote_base = remote_filename.substr(0, remote_filename.find_last_of("."));
    std::string remote_pgm = remote_base + ".pgm";
    
    // 检查PGM文件是否存在
    struct stat file_info;
    if (stat(temp_pgm.c_str(), &file_info) == 0)
    {
      result = uploadFileToFTP(temp_pgm, remote_pgm);
      unlink(temp_pgm.c_str());
    }
  }
  
  return result;
}

bool MapFileManager::loadVirtualWallsFromFTP(const std::string& remote_filename, std::vector<VirtualWall>& walls)
{
  // 创建临时文件
  std::string temp_filename = "/tmp/ros_map_edit_temp_" + std::to_string(getpid()) + "_walls.json";
  
  // 从FTP下载文件
  if (!downloadFileFromFTP(remote_filename, temp_filename))
  {
    return false;
  }
  
  // 加载本地临时文件
  bool result = loadVirtualWalls(temp_filename, walls);
  
  // 清理临时文件
  unlink(temp_filename.c_str());
  
  return result;
}

bool MapFileManager::saveVirtualWallsToFTP(const std::string& remote_filename, const std::vector<VirtualWall>& walls)
{
  // 创建临时文件
  std::string temp_filename = "/tmp/ros_map_edit_temp_" + std::to_string(getpid()) + "_walls.json";
  
  // 保存到本地临时文件
  if (!saveVirtualWalls(temp_filename, walls))
  {
    return false;
  }
  
  // 上传到FTP
  bool result = uploadFileToFTP(temp_filename, remote_filename);
  
  // 清理临时文件
  unlink(temp_filename.c_str());
  
  return result;
}

bool MapFileManager::loadRegionsFromFTP(const std::string& remote_filename, std::vector<Region>& regions)
{
  // 创建临时文件
  std::string temp_filename = "/tmp/ros_map_edit_temp_" + std::to_string(getpid()) + "_regions.json";
  
  // 从FTP下载文件
  if (!downloadFileFromFTP(remote_filename, temp_filename))
  {
    return false;
  }
  
  // 加载本地临时文件
  bool result = loadRegions(temp_filename, regions);
  
  // 清理临时文件
  unlink(temp_filename.c_str());
  
  return result;
}

bool MapFileManager::saveRegionsToFTP(const std::string& remote_filename, const std::vector<Region>& regions)
{
  // 创建临时文件
  std::string temp_filename = "/tmp/ros_map_edit_temp_" + std::to_string(getpid()) + "_regions.json";
  
  // 保存到本地临时文件
  if (!saveRegions(temp_filename, regions))
  {
    return false;
  }
  
  // 上传到FTP
  bool result = uploadFileToFTP(temp_filename, remote_filename);
  
  // 清理临时文件
  unlink(temp_filename.c_str());
  
  return result;
}

bool MapFileManager::saveMapFilesToFTP(const std::string& remote_basename)
{
  // 尝试从map_edited话题获取当前编辑的地图
  ros::NodeHandle nh;
  nav_msgs::OccupancyGridConstPtr map_msg = 
    ros::topic::waitForMessage<nav_msgs::OccupancyGrid>("map_edited", nh, ros::Duration(1.0));
  
  if (!map_msg)
  {
    // 如果没有编辑后的地图，尝试获取原始地图
    map_msg = ros::topic::waitForMessage<nav_msgs::OccupancyGrid>("map", nh, ros::Duration(5.0));
    
    if (!map_msg)
    {
      last_error_ = "无法获取地图数据，请确保地图话题正在发布";
      return false;
    }
  }
  
  return saveMapFilesToFTP(remote_basename, *map_msg);
}

bool MapFileManager::saveMapFilesToFTP(const std::string& remote_basename, const nav_msgs::OccupancyGrid& map)
{
  // 保存地图文件
  std::string yaml_filename = remote_basename + ".yaml";
  if (!saveMapToFTP(yaml_filename, map))
  {
    return false;
  }
  
  // 尝试保存虚拟墙文件
  std::string walls_filename = remote_basename + "_virtual_walls.json";
  ros::NodeHandle nh;
  visualization_msgs::MarkerArrayConstPtr walls_msg = 
    ros::topic::waitForMessage<visualization_msgs::MarkerArray>("virtual_walls_markers", nh, ros::Duration(1.0));
  
  if (walls_msg && !walls_msg->markers.empty())
  {
    // 转换MarkerArray到VirtualWall格式
    std::vector<VirtualWall> walls;
    
    for (const auto& marker : walls_msg->markers)
    {
      if (marker.type == visualization_msgs::Marker::LINE_STRIP && 
          marker.points.size() >= 2)
      {
        VirtualWall current_wall;
        current_wall.points.clear();
        for (const auto& point : marker.points)
        {
          geometry_msgs::Point p;
          p.x = point.x;
          p.y = point.y;
          p.z = point.z;
          current_wall.points.push_back(p);
        }
        current_wall.id = "wall_" + std::to_string(marker.id);
        walls.push_back(current_wall);
      }
    }
    
    if (!walls.empty())
    {
      saveVirtualWallsToFTP(walls_filename, walls);
    }
  }
  
  // 尝试保存区域文件
  std::string regions_filename = remote_basename + "_regions.json";
  visualization_msgs::MarkerArrayConstPtr regions_msg = 
    ros::topic::waitForMessage<visualization_msgs::MarkerArray>("region_markers", nh, ros::Duration(1.0));
  
  if (regions_msg && !regions_msg->markers.empty())
  {
    // 转换MarkerArray到Region格式
    std::vector<Region> regions;
    
    for (const auto& marker : regions_msg->markers)
    {
      if (marker.type == visualization_msgs::Marker::TRIANGLE_LIST && 
          marker.points.size() >= 3)
      {
        Region region;
        region.id = "region_" + std::to_string(marker.id);
        region.frame_id = marker.header.frame_id;
        region.type = 0;
        region.param = 1.0;
        
        // 从三角形列表重构多边形（简化处理）
        std::set<std::pair<double, double>> unique_points;
        for (const auto& point : marker.points)
        {
          unique_points.insert({point.x, point.y});
        }
        
        for (const auto& unique_point : unique_points)
        {
          geometry_msgs::Point p;
          p.x = unique_point.first;
          p.y = unique_point.second;
          p.z = 0.0;
          region.points.push_back(p);
        }
        
        regions.push_back(region);
      }
    }
    
    if (!regions.empty())
    {
      saveRegionsToFTP(regions_filename, regions);
    }
  }
  
  return true;
}

std::string MapFileManager::getPackageMapsDir() const
{
  // 使用rospack命令获取包路径
  std::string command = "rospack find ros_map_edit";
  FILE* pipe = popen(command.c_str(), "r");
  if (!pipe)
  {
    return "";
  }
  
  char buffer[256];
  std::string package_path = "";
  if (fgets(buffer, sizeof(buffer), pipe) != nullptr)
  {
    package_path = std::string(buffer);
    // 移除换行符
    if (!package_path.empty() && package_path.back() == '\n')
    {
      package_path.pop_back();
    }
  }
  pclose(pipe);
  
  if (package_path.empty())
  {
    return "";
  }
  
  return package_path + "/maps";
}

bool MapFileManager::downloadMapFromFTP(const std::string& remote_filename, nav_msgs::OccupancyGrid& map, std::string& local_filepath)
{
  // 获取maps目录路径
  std::string maps_dir = getPackageMapsDir();
  if (maps_dir.empty())
  {
    last_error_ = "无法找到ros_map_edit包路径";
    return false;
  }
  
  // 确保maps目录存在
  std::string mkdir_cmd = "mkdir -p " + maps_dir;
  system(mkdir_cmd.c_str());
  
  // 构建本地文件路径
  std::string filename = remote_filename;
  size_t pos = filename.find_last_of("/");
  if (pos != std::string::npos)
  {
    filename = filename.substr(pos + 1);
  }
  
  local_filepath = maps_dir + "/" + filename;
  
  // 下载文件到本地
  if (!downloadFileFromFTP(remote_filename, local_filepath))
  {
    return false;
  }
  
  // 如果是YAML文件，还需要下载对应的PGM文件
  if (filename.find(".yaml") != std::string::npos || filename.find(".yml") != std::string::npos)
  {
    std::string base_name = filename.substr(0, filename.find_last_of("."));
    std::string remote_pgm = remote_filename.substr(0, remote_filename.find_last_of(".")) + ".pgm";
    std::string local_pgm = maps_dir + "/" + base_name + ".pgm";
    
    // 尝试下载PGM文件
    downloadFileFromFTP(remote_pgm, local_pgm);
  }
  
  // 加载本地文件
  bool result = loadMap(local_filepath, map);
  
  return result;
}

bool MapFileManager::downloadVirtualWallsFromFTP(const std::string& remote_filename, std::vector<VirtualWall>& walls, std::string& local_filepath)
{
  // 获取maps目录路径
  std::string maps_dir = getPackageMapsDir();
  if (maps_dir.empty())
  {
    last_error_ = "无法找到ros_map_edit包路径";
    return false;
  }
  
  // 构建本地文件路径
  std::string filename = remote_filename;
  size_t pos = filename.find_last_of("/");
  if (pos != std::string::npos)
  {
    filename = filename.substr(pos + 1);
  }
  
  local_filepath = maps_dir + "/" + filename;
  
  // 下载文件到本地
  if (!downloadFileFromFTP(remote_filename, local_filepath))
  {
    return false;
  }
  
  // 加载本地文件
  bool result = loadVirtualWalls(local_filepath, walls);
  
  return result;
}

bool MapFileManager::downloadRegionsFromFTP(const std::string& remote_filename, std::vector<Region>& regions, std::string& local_filepath)
{
  // 获取maps目录路径
  std::string maps_dir = getPackageMapsDir();
  if (maps_dir.empty())
  {
    last_error_ = "无法找到ros_map_edit包路径";
    return false;
  }
  
  // 构建本地文件路径
  std::string filename = remote_filename;
  size_t pos = filename.find_last_of("/");
  if (pos != std::string::npos)
  {
    filename = filename.substr(pos + 1);
  }
  
  local_filepath = maps_dir + "/" + filename;
  
  // 下载文件到本地
  if (!downloadFileFromFTP(remote_filename, local_filepath))
  {
    return false;
  }
  
  // 加载本地文件
  bool result = loadRegions(local_filepath, regions);
  
  return result;
}

} // end namespace ros_map_edit 