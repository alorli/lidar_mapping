///////////////////////////////////////////////////////////////////////////////
// by lichunjing 2021_11_26
///////////////////////////////////////////////////////////////////////////////

#ifndef TO_GRID_MAP_H_
#define TO_GRID_MAP_H_


#include "sensor_msgs/PointCloud2.h"
#include "nav_msgs/OccupancyGrid.h"
#include "sensor_msgs/LaserScan.h"
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include "yaml-cpp/yaml.h"

namespace processing
{

struct OccupancyGridMapParameter
{
    double update_distance;
    double resolution;
    int empty_grid_value;
    int occupancy_grid_value;
};


class ToGridMap
{

public:
    ToGridMap(std::string cfg_file_path);
    ~ToGridMap();

    void PointCloudToOccupancyGridMsg(pcl::PointCloud<pcl::PointXYZI>& pcd_map, 
                                      nav_msgs::OccupancyGrid& occupancy_grid_map_msg);

private:
    YAML::Node cfg_file_;
    OccupancyGridMapParameter occupancy_grid_map_parameter_;
};
}   // namespace processing

#endif
