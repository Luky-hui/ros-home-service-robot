#ifndef REGION_TOOL_H
#define REGION_TOOL_H

#include <rviz/tool.h>
#include <rviz/properties/color_property.h>
#include <rviz/properties/float_property.h>
#include <rviz/properties/int_property.h>
#include <rviz/properties/string_property.h>
#include <rviz/properties/bool_property.h>
#include <geometry_msgs/Point.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <ros/ros.h>
#include <vector>

namespace ros_map_edit
{

struct Region
{
  std::string id;
  std::string frame_id;
  int type;
  double param;
  std::vector<geometry_msgs::Point> points;
};

class RegionTool : public rviz::Tool
{
Q_OBJECT
public:
  RegionTool();
  virtual ~RegionTool();

  virtual void onInitialize();
  virtual void activate();
  virtual void deactivate();

  virtual int processMouseEvent(rviz::ViewportMouseEvent& event);

  void loadRegions(const std::string& filename);
  void saveRegions(const std::string& filename);
  void clearRegions();
  void loadRegionsForMap(const std::string& map_file_path);
  void setRegions(const std::vector<Region>& regions);
  
  std::vector<Region> getRegions() const;

  // Auto-loading functionality
  void autoLoadRegions();

private Q_SLOTS:
  void updateProperties();

private:
  void updateMarkers();
  void addPoint(const geometry_msgs::Point& point);
  void finishCurrentRegion();
  void publishMarkers();
  std::string generateRegionId();

  rviz::ColorProperty* color_property_;
  rviz::FloatProperty* alpha_property_;
  rviz::IntProperty* region_type_property_;
  rviz::FloatProperty* region_param_property_;
  rviz::StringProperty* frame_id_property_;
  rviz::BoolProperty* show_points_property_;
  
  std::vector<Region> regions_;
  std::vector<geometry_msgs::Point> current_region_points_;
  
  ros::Publisher marker_pub_;
  visualization_msgs::MarkerArray marker_array_;
  
  int marker_id_counter_;
  int region_id_counter_;
  bool drawing_region_;
};

} // end namespace ros_map_edit

#endif // REGION_TOOL_H 