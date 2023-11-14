///////////////////////////////////////////////////////////////////////////////
// by lichunjing 2020_12_19
///////////////////////////////////////////////////////////////////////////////

#ifndef LIDAR_MAPPING_NODE_H_
#define LIDAR_MAPPING_NODE_H_


#include <ros/ros.h>
#include "src/map_builder.h"
#include "lidar_mapping/generate_map_srv.h"
#include "lidar_mapping/start_builder_srv.h"
#include "lidar_mapping/save_map_srv.h"
#include "sensor_msgs/PointCloud2.h"
#include "sensor_msgs/LaserScan.h"
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/NavSatStatus.h>
#include "tf2_ros/transform_broadcaster.h"
#include <sensor_msgs/Imu.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Int64MultiArray.h>
#include <std_msgs/Float64.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

#include "yaml-cpp/yaml.h"

#include "src/sensor/imu_data.h"
#include "src/sensor/odometry_data.h"

#include "absl/memory/memory.h"
#include <memory>     

#include "glog/logging.h"



constexpr double kPublishPcdMapSec = 0.1;
constexpr double kPublishTfMsgSec = 0.1;
constexpr double kRunGenerateMapSec = 1.0;

struct EncoderData
{
    int64_t encoder_left;
    int64_t encoder_right;
    ros::Time time_stamp;
};

struct OdomData
{
    double odom_x;
    double odom_y;
    double odom_yaw;
};

class LidarMappingNode 
{
public:
    LidarMappingNode(std::string cfg_file_path,
                     std::string project_directory_name,
                     ros::NodeHandle node_handle);
    ~LidarMappingNode();

    
    void HandlePointCloud2Message(const sensor_msgs::PointCloud2::ConstPtr& msg);
    void HandleGnssFixMessage(const sensor_msgs::NavSatFix::ConstPtr& msg);
    void HandleSickMessage(const sensor_msgs::LaserScan::ConstPtr& msg);
    void HandleImuMessage(const sensor_msgs::Imu::ConstPtr& msg);
    void HandleOdomMessage(const nav_msgs::Odometry::ConstPtr& msg);
    void HandleEncoderMessage(const std_msgs::Int64MultiArray::ConstPtr& msg);

    bool GenerateMapCallback(lidar_mapping::generate_map_srv::Request  &req,
                             lidar_mapping::generate_map_srv::Response &res);

    bool StartBuilderCallback(lidar_mapping::start_builder_srv::Request  &req,
                              lidar_mapping::start_builder_srv::Response &res);
    bool SaveMapCallback(lidar_mapping::save_map_srv::Request  &req,
                         lidar_mapping::save_map_srv::Response &res);

    void PublishPcdMap(const ros::WallTimerEvent& unused_timer_event);
    void PublishTfMsg(const ros::WallTimerEvent& unused_timer_event);
    void RunGenerateMap(const ros::WallTimerEvent& unused_timer_event);

    std::unique_ptr<sensor::ImuData> ToImuData(const sensor_msgs::Imu::ConstPtr& msg);
    transform::Rigid3d ToRigid3d(const geometry_msgs::Pose& pose);
    Eigen::Vector3d ToEigen(const geometry_msgs::Vector3& vector3);
    Eigen::Quaterniond ToEigen(const geometry_msgs::Quaternion& quaternion);
    std::unique_ptr<sensor::OdometryData> ToOdometryData(const nav_msgs::Odometry& msg);

private:
    YAML::Node cfg_file_;

    ros::NodeHandle node_handle_;

    ros::Subscriber pointcloud_subscriber_;
    ros::Subscriber gnss_fix_subscriber_;
    ros::Subscriber sick_subscriber_;
    ros::Subscriber imu_subscriber_;
    ros::Subscriber odom_subscriber_;
    ros::Subscriber encoder_subscriber_;


    ros::Publisher pcd_map_publisher_;
    ros::Publisher pose_registration_publisher_;
    bool is_publish_pose_registration_;

    ros::Publisher generate_map_status_publisher_;
    bool is_publish_generate_map_status_;

    ros::Publisher occupancy_grid_map_publisher_;
    bool is_publish_occupancy_grid_map_;
    double occupancy_grid_map_update_distance_;

    ros::ServiceServer generate_map_service_;
    ros::ServiceServer start_builder_service_;
    ros::ServiceServer save_map_service_;

    tf2_ros::TransformBroadcaster tf_broadcaster_;

    std::vector<ros::WallTimer> wall_timers_;
    sensor_msgs::PointCloud2 pcd_map_msg_;

    std::string output_pointcloud_file_path_;

    
    bool is_run_gnss_aidded_optimization_;
    bool is_run_anchor_points_optimization_;
    bool is_run_closeloop_optimization_;
    bool is_generate_raw_map_;
    bool is_generate_compensation_map_;
    bool is_generate_gnss_optimization_map_;
    bool is_generate_closeloop_optimization_map_;
    bool is_generate_laserscan_map_;
    bool is_run_map_partition_;
    bool is_generate_arealist_file_;
    bool is_delete_project_directory_after_save_map_;



    mapping::MapBuilder map_builder_;
    bool is_generate_map_;
    bool is_receive_sensor_data_;
    bool is_run_generate_map_;
    bool is_publish_pcd_map_;
    bool is_publish_generate_map_service_;
    bool is_publish_start_builder_service_;
    bool is_publish_save_map_service_;
    bool is_exit_node_;


    bool is_get_init_encoder_;
    EncoderData encoder_init_;
    EncoderData encoder_previous_;
    EncoderData encoder_current_;
    nav_msgs::Odometry odom_msg_;

    OdomData odom_data_;
};

#endif


