///////////////////////////////////////////////////////////////////////////////
// by lichunjing 2021_01_23
///////////////////////////////////////////////////////////////////////////////

#ifndef MAP_VISUALIZATION_NODE_H_
#define MAP_VISUALIZATION_NODE_H_


#include <ros/ros.h>
#include "sensor_msgs/PointCloud2.h"
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>


typedef pcl::PointXYZI PointType;

constexpr double kPublishPcdMapSec = 1;

class MapVisualizationNode
{

public:
    MapVisualizationNode(std::string map_file_path,
                         bool use_voxel_filter,
                         double voxel_leaf_size,
                         ros::NodeHandle node_handle);
    ~MapVisualizationNode();

    void PublishXYZIMap();
    void PublishXYZRGBAMap();

private:
    ros::NodeHandle node_handle_;
    ros::Publisher pcd_map_publisher_;

    pcl::PointCloud<PointType> pcd_map_;
    pcl::PointCloud<pcl::PointXYZRGBA> xyzrgba_map_;
    sensor_msgs::PointCloud2 pcd_map_msg_;

    std::string map_file_path_;
    std::vector<std::string> map_file_lists_;
    bool use_voxel_filter_;
    double voxel_leaf_size_;
    double min_keyframe_distance_;

    std::vector<ros::WallTimer> wall_timers_;
};

#endif
