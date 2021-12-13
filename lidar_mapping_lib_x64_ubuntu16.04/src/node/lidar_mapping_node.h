///////////////////////////////////////////////////////////////////////////////
// by lichunjing 2020_12_19
///////////////////////////////////////////////////////////////////////////////

#ifndef LIDAR_MAPPING_NODE_H_
#define LIDAR_MAPPING_NODE_H_


#include <ros/ros.h>
#include "src/map_builder.h"
#include "lidar_mapping/generate_map_srv.h"
#include "sensor_msgs/PointCloud2.h"
#include "sensor_msgs/LaserScan.h"
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/NavSatStatus.h>
#include "tf2_ros/transform_broadcaster.h"

#include "yaml-cpp/yaml.h"

constexpr double kPublishPcdMapSec = 0.1;
constexpr double kPublishTfMsgSec = 0.1;

class LidarMappingNode 
{

public:

    LidarMappingNode(std::string cfg_file_path, 
                     ros::NodeHandle node_handle);
    ~LidarMappingNode();

    
    void HandlePointCloud2Message(const sensor_msgs::PointCloud2::ConstPtr& msg);

    void HandleGnssFixMessage(const sensor_msgs::NavSatFix::ConstPtr& msg);

    void HandleSickMessage(const sensor_msgs::LaserScan::ConstPtr& msg);

    bool GenerateMapCallback(lidar_mapping::generate_map_srv::Request  &req,
                             lidar_mapping::generate_map_srv::Response &res);

    void PublishPcdMap(const ros::WallTimerEvent& unused_timer_event);
    void PublishTfMsg(const ros::WallTimerEvent& unused_timer_event);

private:
    YAML::Node cfg_file_;

    ros::NodeHandle node_handle_;

    ros::Subscriber pointcloud_subscriber_;
    ros::Subscriber gnss_fix_subscriber_;
    ros::Subscriber sick_subscriber_;

    ros::Publisher pcd_map_publisher_;

    ros::ServiceServer generate_map_service_;

    tf2_ros::TransformBroadcaster tf_broadcaster_;

    std::vector<ros::WallTimer> wall_timers_;
    sensor_msgs::PointCloud2 pcd_map_msg_;

    
    bool is_run_gnss_aidded_optimization_;
    bool is_run_closeloop_optimization_;
    bool is_generate_raw_map_;
    bool is_generate_compensation_map_;
    bool is_generate_gnss_optimization_map_;
    bool is_generate_closeloop_optimization_map_;
    bool is_generate_laserscan_map_;



    mapping::MapBuilder map_builder_;
    bool is_generate_map_;
};

#endif


