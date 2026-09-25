#ifndef MAP_ERASER_TOOL_H
#define MAP_ERASER_TOOL_H

#include <rviz/tool.h>
#include <rviz/properties/float_property.h>
#include <rviz/properties/enum_property.h>
#include <rviz/properties/bool_property.h>
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/Point.h>
#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/LinearMath/Transform.h>

namespace ros_map_edit
{

class MapEraserTool : public rviz::Tool
{
Q_OBJECT
public:
  MapEraserTool();
  virtual ~MapEraserTool();

  virtual void onInitialize();
  virtual void activate();
  virtual void deactivate();

  virtual int processMouseEvent(rviz::ViewportMouseEvent& event);

  void loadMap(const std::string& filename);
  void saveMap(const std::string& filename);
  
  nav_msgs::OccupancyGrid getCurrentMap() const;

private Q_SLOTS:
  void updateProperties();

private:
  void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg);
  void eraseAtPoint(const geometry_msgs::Point& point);
  void paintAtPoint(const geometry_msgs::Point& point);
  geometry_msgs::Point screenToMap(int screen_x, int screen_y);
  void publishModifiedMap();

  rviz::FloatProperty* brush_size_property_;
  
  enum BrushMode
  {
    ERASE_TO_FREE,    // 擦除为自由空间 (白色)
    ERASE_TO_OCCUPIED, // 擦除为占用空间 (黑色)
    ERASE_TO_UNKNOWN   // 擦除为未知空间 (灰色)
  };
  
  nav_msgs::OccupancyGrid current_map_;
  ros::Subscriber map_sub_;
  ros::Publisher map_pub_;
  
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  
  bool map_received_;
  bool mouse_pressed_;
  double brush_size_;
  BrushMode brush_mode_;
};

} // end namespace ros_map_edit

#endif // MAP_ERASER_TOOL_H 