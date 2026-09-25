#ifndef TOOL_MANAGER_H
#define TOOL_MANAGER_H

#include "virtual_wall_tool.h"
#include "region_tool.h"
#include "map_eraser_tool.h"

namespace ros_map_edit
{

class ToolManager
{
public:
    static ToolManager& getInstance();
    
    void registerVirtualWallTool(VirtualWallTool* tool);
    void registerRegionTool(RegionTool* tool);
    void registerMapEraserTool(MapEraserTool* tool);
    
    VirtualWallTool* getVirtualWallTool() const;
    RegionTool* getRegionTool() const;
    MapEraserTool* getMapEraserTool() const;

private:
    ToolManager() : virtual_wall_tool_(nullptr), region_tool_(nullptr), map_eraser_tool_(nullptr) {}
    ~ToolManager() = default;
    ToolManager(const ToolManager&) = delete;
    ToolManager& operator=(const ToolManager&) = delete;
    
    VirtualWallTool* virtual_wall_tool_;
    RegionTool* region_tool_;
    MapEraserTool* map_eraser_tool_;
};

} // end namespace ros_map_edit

#endif // TOOL_MANAGER_H 