#include "ros_map_edit/map_edit_tool.h"
#include <rviz/viewport_mouse_event.h>
#include <rviz/visualization_manager.h>
#include <rviz/geometry.h>
#include <rviz/properties/property_tree_model.h>

namespace ros_map_edit
{

MapEditTool::MapEditTool()
  : current_mode_(NONE)
  , active_(false)
{
}

MapEditTool::~MapEditTool()
{
}

void MapEditTool::onInitialize()
{
  mode_property_ = new rviz::EnumProperty("Edit Mode", "None",
                                          "Select the map editing mode",
                                          getPropertyContainer(), SLOT(updateMode()), this);
  mode_property_->addOption("None", NONE);
  mode_property_->addOption("Virtual Wall", VIRTUAL_WALL);
  mode_property_->addOption("Region", REGION);
  mode_property_->addOption("Eraser", ERASER);

  snap_to_grid_property_ = new rviz::BoolProperty("Snap to Grid", false,
                                                  "Snap mouse clicks to grid",
                                                  getPropertyContainer(), SLOT(updateMode()), this);

  grid_size_property_ = new rviz::FloatProperty("Grid Size", 0.1,
                                                "Size of the snap grid in meters",
                                                getPropertyContainer(), SLOT(updateMode()), this);
  grid_size_property_->setMin(0.01);
  grid_size_property_->setMax(1.0);
}

void MapEditTool::activate()
{
  active_ = true;
  setStatus("Map Edit Tool - Select a mode to begin editing");
}

void MapEditTool::deactivate()
{
  active_ = false;
}

int MapEditTool::processMouseEvent(rviz::ViewportMouseEvent& event)
{
  if (!active_)
    return Render;

  switch (current_mode_)
  {
    case VIRTUAL_WALL:
      setStatus("Virtual Wall mode - Click to place wall points, right-click to finish");
      break;
    case REGION:
      setStatus("Region mode - Click to place polygon points, right-click to finish");
      break;
    case ERASER:
      setStatus("Eraser mode - Click and drag to erase/paint map areas");
      break;
    case NONE:
    default:
      setStatus("Select an editing mode from the properties panel");
      break;
  }

  return Render;
}

void MapEditTool::updateMode()
{
  current_mode_ = (EditMode)mode_property_->getOptionInt();
  
  // Update grid settings
  bool snap_enabled = snap_to_grid_property_->getBool();
  grid_size_property_->setHidden(!snap_enabled);
}

} // end namespace ros_map_edit

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(ros_map_edit::MapEditTool, rviz::Tool) 