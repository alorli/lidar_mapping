///////////////////////////////////////////////////////////////////////////////
// by lichunjing 2021_11_20
///////////////////////////////////////////////////////////////////////////////

#ifndef TO_GRID_MAP_NODE_H_
#define TO_GRID_MAP_NODE_H_


#include <ros/ros.h>
#include "sensor_msgs/PointCloud2.h"
#include "nav_msgs/OccupancyGrid.h"
#include "sensor_msgs/LaserScan.h"
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>


typedef pcl::PointXYZI PointType;

struct PointXYZIRT
{
    PCL_ADD_POINT4D                      // 添加pcl里 xyz+padding
    PCL_ADD_INTENSITY;
    uint16_t ring;
    float time;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW      // 确保定义新类型点云内存与SSE对齐
} EIGEN_ALIGN16;                         // 强制SSE填充以正确对齐内存


// 注册点云中点的自定义类型
POINT_CLOUD_REGISTER_POINT_STRUCT (PointXYZIRT,  
    (float, x, x) 
    (float, y, y) 
    (float, z, z) 
    (float, intensity, intensity)
    (uint16_t, ring, ring) 
    (float, time, time)
)


constexpr double kPublishPcdMapSec = 1;

class ToGridMapNode
{

public:
    ToGridMapNode(std::string map_file_path,
                  bool use_voxel_filter,
                  double voxel_leaf_size,
                  ros::NodeHandle node_handle);
    ~ToGridMapNode();

    void PublishXYZIMap();

    void HandlePointCloud2Message(const sensor_msgs::PointCloud2::ConstPtr& msg);
    void TimerPublishMessage(const ros::WallTimerEvent& unused_timer_event);

private:
    ros::NodeHandle node_handle_;
    ros::Publisher pcd_map_publisher_;
    ros::Publisher occupancy_grid_publisher_;
    ros::Publisher occupancy_grid_scan_publisher_;
    ros::Publisher laser_scan_publisher_;
    ros::Subscriber pointcloud_subscriber_;

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

#endif    //TO_GRID_MAP_NODE_H_
