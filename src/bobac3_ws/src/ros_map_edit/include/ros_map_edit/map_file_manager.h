#ifndef MAP_FILE_MANAGER_H
#define MAP_FILE_MANAGER_H

#include <string>
#include <vector>
#include <jsoncpp/json/json.h>
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/Point.h>
#include "virtual_wall_tool.h"
#include "region_tool.h"

namespace ros_map_edit
{

// FTP连接配置结构体
struct FTPConfig
{
  std::string host;        // FTP服务器地址
  int port;                // FTP端口，默认21
  std::string username;    // 用户名
  std::string password;    // 密码
  std::string remote_dir;  // 远程目录，默认"/"
  bool use_passive;        // 是否使用被动模式，默认true
  int timeout;             // 连接超时时间（秒），默认30
  
  FTPConfig() : port(21), remote_dir("/"), use_passive(true), timeout(30) {}
};

class MapFileManager
{
public:
  MapFileManager();
  virtual ~MapFileManager();

  // Virtual walls I/O
  bool loadVirtualWalls(const std::string& filename, std::vector<VirtualWall>& walls);
  bool saveVirtualWalls(const std::string& filename, const std::vector<VirtualWall>& walls);

  // Regions I/O
  bool loadRegions(const std::string& filename, std::vector<Region>& regions);
  bool saveRegions(const std::string& filename, const std::vector<Region>& regions);

  // Map I/O
  bool loadMap(const std::string& filename, nav_msgs::OccupancyGrid& map);
  bool saveMap(const std::string& filename, const nav_msgs::OccupancyGrid& map);
  bool saveMapImage(const std::string& filename, const nav_msgs::OccupancyGrid& map);

  // 一键保存所有文件
  bool saveMapFiles(const std::string& yaml_filename);
  bool saveMapFiles(const std::string& yaml_filename, const nav_msgs::OccupancyGrid& map);
  bool saveVirtualWallsFile(const std::string& filename);
  bool saveVirtualWallsFile(const std::string& filename, const std::vector<VirtualWall>& walls);
  bool saveRegionsFile(const std::string& filename);
  bool saveRegionsFile(const std::string& filename, const std::vector<Region>& regions);

  // FTP功能
  void setFTPConfig(const FTPConfig& config);
  FTPConfig getFTPConfig() const;
  
  // FTP地图操作
  bool loadMapFromFTP(const std::string& remote_filename, nav_msgs::OccupancyGrid& map);
  bool saveMapToFTP(const std::string& remote_filename, const nav_msgs::OccupancyGrid& map);
  
  // FTP下载到本地maps文件夹
  bool downloadMapFromFTP(const std::string& remote_filename, nav_msgs::OccupancyGrid& map, std::string& local_filepath);
  bool downloadVirtualWallsFromFTP(const std::string& remote_filename, std::vector<VirtualWall>& walls, std::string& local_filepath);
  bool downloadRegionsFromFTP(const std::string& remote_filename, std::vector<Region>& regions, std::string& local_filepath);
  
  // FTP虚拟墙操作
  bool loadVirtualWallsFromFTP(const std::string& remote_filename, std::vector<VirtualWall>& walls);
  bool saveVirtualWallsToFTP(const std::string& remote_filename, const std::vector<VirtualWall>& walls);
  
  // FTP区域操作
  bool loadRegionsFromFTP(const std::string& remote_filename, std::vector<Region>& regions);
  bool saveRegionsToFTP(const std::string& remote_filename, const std::vector<Region>& regions);
  
  // FTP一键保存操作
  bool saveMapFilesToFTP(const std::string& remote_basename);
  bool saveMapFilesToFTP(const std::string& remote_basename, const nav_msgs::OccupancyGrid& map);
  
  // FTP工具方法
  bool testFTPConnection();
  bool listFTPDirectory(std::vector<std::string>& file_list, const std::string& remote_dir = "");
  bool createFTPDirectory(const std::string& remote_dir);

  // Utility functions
  std::string getLastError() const { return last_error_; }
  
  // 获取包的maps文件夹路径
  std::string getPackageMapsDir() const;
  
private:
  // JSON helpers
  Json::Value pointToJson(const geometry_msgs::Point& point);
  geometry_msgs::Point jsonToPoint(const Json::Value& json);
  
  // Map file format helpers
  bool loadPGM(const std::string& filename, nav_msgs::OccupancyGrid& map);
  bool loadPGMWithParams(const std::string& filename, nav_msgs::OccupancyGrid& map,
                         double occupied_thresh, double free_thresh, bool negate);
  bool savePGM(const std::string& filename, const nav_msgs::OccupancyGrid& map);
  bool loadYAML(const std::string& filename, nav_msgs::OccupancyGrid& map);
  bool saveYAML(const std::string& filename, const nav_msgs::OccupancyGrid& map);

  // FTP工具方法
  bool downloadFileFromFTP(const std::string& remote_filename, const std::string& local_filename);
  bool uploadFileToFTP(const std::string& local_filename, const std::string& remote_filename);
  std::string buildFTPURL(const std::string& remote_path) const;
  bool initCurl();
  void cleanupCurl();

  std::string last_error_;
  FTPConfig ftp_config_;
  bool curl_initialized_;
};

} // end namespace ros_map_edit

#endif // MAP_FILE_MANAGER_H 