///////////////////////////////////////////////////////////////////////////////
// by lichunjing 2020_12_23
///////////////////////////////////////////////////////////////////////////////
#ifndef MAP_BUILDER_H_
#define MAP_BUILDER_H_

#include "src/registration/ndt_registration.h"
#include "src/processing/motion_compensation.h"
#include "src/processing/pose_interpolation.h"
#include "src/map_generator.h"
#include "src/constraints/gnss_constraints_builder.h"
#include "src/optimization/gnss_optimization.h"
#include "src/constraints/closeloop_constraints_builder.h"
#include "src/optimization/closeloop_optimization.h"

#include "src/sensor/imu_data.h"
#include "src/sensor/odometry_data.h"

#include "src/processing/pose_extrapolator.h"
#include "src/processing/to_grid_map.h"
#include "src/processing/map_partition.h"
#include "src/processing/pointcloud_map_arealist.h"

#include <rosbag/bag.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/NavSatStatus.h>
#include "sensor_msgs/LaserScan.h"
#include "geometry_msgs/TransformStamped.h"
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include "nav_msgs/OccupancyGrid.h"

// #include <pcl/io/io.h>
// #include <pcl/io/pcd_io.h>


#include "yaml-cpp/yaml.h"

// #include<Eigen/Core>
// #include <Eigen/Geometry>
// #include <vector>
#include <string>
// #include <fstream>

#include <proj_api.h>


#include "glog/logging.h"


namespace mapping
{

struct MsgParameter
{
    double pcd_map_voxel_leaf_size;
};


struct MapBuilderParameter
{
    int lidar_n_scans;
    // double min_map_horizontal_radius;
    // double min_keyframe_distance;
    // int num_keyframe_submap;
};

struct MappingParameter
{
    double min_map_horizontal_radius;
    double min_keyframe_distance;
    int num_keyframe_submap;
    double min_intensity;
    double max_intensity;
    double z_offset;
};

struct DualAntennaParameter
{
    bool is_use_dual_antenna;
    double ranges_main_antenna_to_lidar;
    double angle_dual_antenna_to_lidar;
};


class MapBuilder
{
public:
    MapBuilder(std::string cfg_file_path, 
               std::string project_directory_name);
    ~MapBuilder();

    void AddVlpPointCloudData(const sensor_msgs::PointCloud2::ConstPtr& msg);
    void AddGnssFixData(const sensor_msgs::NavSatFix::ConstPtr& msg);
    void AddSickData(const sensor_msgs::LaserScan::ConstPtr& msg);
    void AddSensorData(const sensor::ImuData& imu_data);
    void AddSensorData(const sensor::OdometryData& odometry_data);
    void SaveVlpCompensationPcd(registration::TimedIdPointCloud& timed_id_pointcloud_compensation);
    void SaveVlpRawPcd(registration::TimedIdPointCloud& timed_id_pointcloud);
    void GenerateCompensationMap();
    void GenerateRawMap();
    void GnssAidedOptimization();
    void CloseloopOptimization();
    void GenerateGnssOptimizationMap();
    void GenerateCloseloopOptimizationMap();
    void GenerateLaserScanMap();
    void RunMapPartition();
    void GenerateArealistFile();
    void GetPcdMapMsg(sensor_msgs::PointCloud2& pcd_map);
    bool GetOccupancyGridMsg(nav_msgs::OccupancyGrid& occupancy_grid_map,
                             double update_distance);
    bool GetOccupancyGridMsg(nav_msgs::OccupancyGrid& occupancy_grid_map_msg);
    bool SaveMap(std::string map_name);
    void DeleteProjectDirectory();
    std::vector<geometry_msgs::TransformStamped> GetTfMsg();
    geometry_msgs::PoseWithCovarianceStamped GetPoseRegistration();
    Eigen::Matrix4f PredictPose();

private:
    YAML::Node cfg_file_;
    MsgParameter msg_parameter_;
    MapBuilderParameter map_builder_parameter_;
    MappingParameter vlp_mapping_parameter_;
    MappingParameter sick_mapping_parameter_;
    DualAntennaParameter dual_antenna_parameter_;

    std::string utm_localization_;    // "beijing" 或者 "changsha" added by lichunjing 2021-08-12
    projPJ pj_utm_, pj_latlong_;
    
    registration::NdtRegistration ndt_registration_;
    registration::NdtRegistration ndt_registration_compensation_;
    registration::AlignmentResult alignment_result_previous_;
    registration::TimedIdPointCloud timed_id_pointcloud_previous_;

    processing::MotionCompensation motion_compensation_;
    processing::PoseInterpolation laserscan_pose_interpolation_;
    processing::ToGridMap to_grid_map_;
    processing::MapPartition map_partition_;
    processing::PointcloudMapArealist pointcloud_pap_arealist_;

    constraints::GnssConstraintsBuilder gnss_constraints_builder_;
    constraints::CloseloopConstraintsBuilder closeloop_constraints_builder_;

    optimization::GnssOptimization gnss_optimization_;
    optimization::CloseloopOptimization closeloop_optimization_;

    MapGenerator map_generator_raw_;
    MapGenerator map_generator_compensation_;
    MapGenerator map_generator_gnss_optimization_;
    MapGenerator map_generator_closeloop_optimization_;
    MapGenerator map_generator_laserscan_;

    bool is_save_vlp_raw_pcd_;
    bool is_save_sick_raw_pcd_;
    bool is_save_compensation_bag_;

    rosbag::Bag compensation_bag_;


    long long allframe_id_;
    int pcd_map_previous_size_;

    std::string project_directory_;
    std::string output_map_full_directory_;
    std::string save_map_full_directory_;

    pcl::PointCloud<registration::PointType> pcd_map_filtered_;

    std::string save_velodyne_compensation_pointcloud_path_;
    std::string save_velodyne_raw_pointcloud_path_;
    std::string save_sick_raw_pointcloud_path_;
    std::string save_gnss_path_;
    std::string save_compensation_bag_path_;
    std::ofstream velodyne_compensation_filelist_;
    std::ofstream velodyne_raw_filelist_;
    std::ofstream sick_raw_filelist_;
    std::ofstream gnss_fix_pose_file_;

    std::ofstream g2o_file_raw_;
    std::ofstream g2o_file_compensation_;

    std::unique_ptr<processing::PoseExtrapolator> extrapolator_ptr_;
};

} // namespace io

#endif