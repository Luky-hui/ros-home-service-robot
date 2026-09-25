#ifndef VIRTUAL_WALL_TOOL_H
#define VIRTUAL_WALL_TOOL_H

#include <rviz/tool.h>
#include <rviz/properties/color_property.h>
#include <rviz/properties/float_property.h>
#include <rviz/properties/bool_property.h>
#include <geometry_msgs/Point.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <ros/ros.h>
#include <vector>

namespace rviz
{
class Line;
}

namespace ros_map_edit
{

struct VirtualWall
{
  std::vector<geometry_msgs::Point> points;
  std::string id;
};

class VirtualWallTool : public rviz::Tool
{
Q_OBJECT
public:
  VirtualWallTool();
  virtual ~VirtualWallTool();

  virtual void onInitialize();
  virtual void activate();
  virtual void deactivate();

  virtual int processMouseEvent(rviz::ViewportMouseEvent& event);

  void loadVirtualWalls(const std::string& filename);
  void saveVirtualWalls(const std::string& filename);
  void clearVirtualWalls();
  void loadVirtualWallsForMap(const std::string& map_file_path);
  void setVirtualWalls(const std::vector<VirtualWall>& walls);
  
  std::vector<VirtualWall> getVirtualWalls() const;

  // Auto-loading functionality
  void autoLoadVirtualWalls();

private Q_SLOTS:
  void updateProperties();

private:
  void updateMarkers();
  void addPoint(const geometry_msgs::Point& point);
  void finishCurrentWall();
  void publishMarkers();

  rviz::ColorProperty* color_property_;
  rviz::FloatProperty* line_width_property_;
  rviz::BoolProperty* show_points_property_;
  
  std::vector<VirtualWall> virtual_walls_;
  std::vector<geometry_msgs::Point> current_wall_points_;
  
  ros::Publisher marker_pub_;
  visualization_msgs::MarkerArray marker_array_;
  
  int marker_id_counter_;
  bool drawing_wall_;
};

} // end namespace ros_map_edit

#endif // VIRTUAL_WALL_TOOL_H 