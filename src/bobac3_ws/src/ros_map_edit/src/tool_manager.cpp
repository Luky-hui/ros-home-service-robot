#include "ros_map_edit/tool_manager.h"

namespace ros_map_edit
{

ToolManager& ToolManager::getInstance()
{
    static ToolManager instance;
    return instance;
}

void ToolManager::registerVirtualWallTool(VirtualWallTool* tool)
{
    virtual_wall_tool_ = tool;
}

void ToolManager::registerRegionTool(RegionTool* tool)
{
    region_tool_ = tool;
}

void ToolManager::registerMapEraserTool(MapEraserTool* tool)
{
    map_eraser_tool_ = tool;
}

VirtualWallTool* ToolManager::getVirtualWallTool() const
{
    return virtual_wall_tool_;
}

RegionTool* ToolManager::getRegionTool() const
{
    return region_tool_;
}

MapEraserTool* ToolManager::getMapEraserTool() const
{
    return map_eraser_tool_;
}

} // end namespace ros_map_edit 