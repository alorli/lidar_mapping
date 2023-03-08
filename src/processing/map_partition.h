///////////////////////////////////////////////////////////////////////////////
// by lichunjing 2021_11_29
///////////////////////////////////////////////////////////////////////////////

#ifndef MAP_PARTITION_H_
#define MAP_PARTITION_H_

#include "src/common/common.h"
#include "sensor_msgs/PointCloud2.h"
#include "nav_msgs/OccupancyGrid.h"
#include "sensor_msgs/LaserScan.h"
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include "yaml-cpp/yaml.h"

namespace processing
{

// 定义区域的边界
struct Area
{
	double x_min;
	double y_min;
	double z_min;
	double x_max;
	double y_max;
	double z_max;
};

struct MapPartitionParameter
{
	double voxel_filter_size;
    double step_range_x;
    double step_range_y;
};


class MapPartition
{
public:
    MapPartition(std::string cfg_file_path, 
                 std::string project_directory_name);
    ~MapPartition();

    void find_min_max(pcl::PointCloud<pcl::PointXYZI>::Ptr input_cloud_ptr,
                      struct Area &a);
    void RunMapPartition();

private:
    YAML::Node cfg_file_;
    std::vector<std::string> pointcloud_file_list_;
    std::string raw_pointcloud_file_path_;
    std::string output_pointcloud_file_path_;
    std::string all_map_path_;

    MapPartitionParameter map_partition_parameter_;
};
}   // namespace processing

#endif
