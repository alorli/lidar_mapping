///////////////////////////////////////////////////////////////////////////////
// by lichunjing 2021_12_03
///////////////////////////////////////////////////////////////////////////////

#ifndef POINTCLOUD_MAP_AREALIST_H_
#define POINTCLOUD_MAP_AREALIST_H_

#include "src/common/common.h"
#include "sensor_msgs/PointCloud2.h"
#include "nav_msgs/OccupancyGrid.h"
#include "sensor_msgs/LaserScan.h"
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include "yaml-cpp/yaml.h"

namespace processing
{

struct AreaWithPath
{
	std::string path;
	double x_min;
	double y_min;
	double z_min;
	double x_max;
	double y_max;
	double z_max;
};

typedef std::vector<AreaWithPath> AreaList;
typedef std::vector<std::vector<std::string>> Table;


class PointcloudMapArealist
{
public:
    PointcloudMapArealist(std::string cfg_file_path, 
                          std::string project_directory_name);
    ~PointcloudMapArealist();
    void GenerateArealistFile();
    void CalculateAreaList(std::vector<std::string>& pointcoud_map_files, 
                           AreaList& areas);
    int CalculateArea(const std::string& path, struct AreaWithPath& area);
    void FindPointcloudMapfiles(const std::string& path, 
                                std::vector<std::string>& pointcoud_map_files);
    std::string DoubleFormatToString(double v);
    void WriteTableTofile(const std::string& path, 
                          const Table& table);
    void WriteArealist(const std::string& path, 
                       const AreaList& area_list);
private:
    YAML::Node cfg_file_;
    std::string output_map_directory_;
    std::string arealist_filename_;
    // std::vector<std::string> pointcloud_file_list_;
    // std::string raw_pointcloud_file_path_;
    // std::string output_pointcloud_file_path_;

    // MapPartitionParameter map_partition_parameter_;
};
}   // namespace processing

#endif
