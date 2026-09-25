#ifndef MAP_EDIT_TOOL_H
#define MAP_EDIT_TOOL_H

#include <rviz/tool.h>
#include <rviz/properties/enum_property.h>
#include <rviz/properties/bool_property.h>
#include <rviz/properties/float_property.h>

namespace ros_map_edit
{

class MapEditTool : public rviz::Tool
{
Q_OBJECT
public:
  MapEditTool();
  virtual ~MapEditTool();

  virtual void onInitialize();
  virtual void activate();
  virtual void deactivate();

  virtual int processMouseEvent(rviz::ViewportMouseEvent& event);

private Q_SLOTS:
  void updateMode();

private:
  rviz::EnumProperty* mode_property_;
  rviz::BoolProperty* snap_to_grid_property_;
  rviz::FloatProperty* grid_size_property_;
  
  enum EditMode
  {
    VIRTUAL_WALL,
    REGION,
    ERASER,
    NONE
  };
  
  EditMode current_mode_;
  bool active_;
};

} // end namespace ros_map_edit

#endif // MAP_EDIT_TOOL_H 