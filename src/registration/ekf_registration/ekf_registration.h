///////////////////////////////////////////////////////////////////////////////
// by lichunjing 2022_04_24
///////////////////////////////////////////////////////////////////////////////
#ifndef EKF_REGISTRATION_H_
#define EKF_REGISTRATION_H_

#include "src/common/time.h"
#include "src/common/time_conversion.h"
#include "src/sensor/imu_data.h"
#include "src/registration/common.h"    //added by lichunjing 2022_12_02

#include "sensor_msgs/PointCloud2.h"

#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/voxel_grid.h>

#include "src/registration/ekf_registration/common_types.h"
#include "src/registration/ekf_registration/imu_processor.h"
#include "src/registration/ekf_registration/ekf_processor.h"
#include "src/registration/ekf_registration/ikd_tree.h"

// #include <Eigen/Core>
// #include <Eigen/Eigen>
#include <Eigen/Geometry>
#include <string>
#include "yaml-cpp/yaml.h"


namespace registration
{

class EkfRegistration
{

public:
    EkfRegistration(std::string cfg_file_path);
    ~EkfRegistration();

    void AddSensorData(const sensor_msgs::PointCloud2::ConstPtr& msg);
    void AddSensorData(const sensor::ImuData& imu_data);
    void ProcessPointCloud(pcl::PointCloud<velodyne::Point>& velodyne_pointcloud,
                           TimedIdLidarPointCloud& timed_id_lidar_pointcloud,
                           TimedIdLidarPointCloud& timed_id_lidar_pointcloud_raw_compensationed);
    bool PrepareMeasurements();
    void MoveMap();
    void UpdateMap();

    const AlignmentResult GetAlignmentResult() {return alignment_result_;};
    void GetTimedIdPointCloudRaw(TimedIdPointCloud& timed_id_pointcloud_raw);
    void GetTimedIdPointCloudRawCompensationed(TimedIdPointCloud& timed_id_pointcloud_raw_compensationed);

    pcl::PointCloud<registration::LidarPointType>::ConstPtr GetMapConstPtr() 
    {
        return compensationed_features_pointcloud_downsize_world_.pointcloud_ptr;
    };

private:
    YAML::Node cfg_file_;
    AllParameter all_parameter_;

    EkfState ekf_state_;
    ImuProcessor imu_processor_;
    EkfProcessor ekf_processor_;

    long long allframe_id = 0;
    std::deque<TimedIdLidarPointCloud> timed_id_lidar_pointcloud_buffer_;
    std::deque<TimedIdLidarPointCloud> timed_id_lidar_pointcloud_raw_compensationed_buffer_;
    std::deque<sensor::ImuData> imu_data_buffer_;
    int num_features_points_downsize_ = 0;                         //提取的特征点,进行降采样后的点数
    Measurements measurements_;                                    //这里默认调用TimedIdLidarPointCloud的无参构造函数
    TimedIdLidarPointCloud timed_id_lidar_pointcloud_raw_;         //这里默认调用TimedIdLidarPointCloud的无参构造函数
    TimedIdLidarPointCloud timed_id_lidar_pointcloud_raw_compensationed_;         //这里默认调用TimedIdLidarPointCloud的无参构造函数
    TimedIdLidarPointCloud compensationed_features_pointcloud_;                   //这里默认调用TimedIdLidarPointCloud的无参构造函数
    TimedIdLidarPointCloud compensationed_features_pointcloud_downsize_lidar_;    //这里默认调用TimedIdLidarPointCloud的无参构造函数
    TimedIdLidarPointCloud compensationed_features_pointcloud_downsize_world_;    //这里默认调用TimedIdLidarPointCloud的无参构造函数

    AlignmentResult alignment_result_;   //added by lichunjing 2022_12_02
    
    long long lidar_allframe_id_;
    common::Time last_lidar_time_;
    common::Time last_imu_time_;

    bool is_lidar_prepared_;
    double lidar_average_scan_period_;
    int lidar_scan_count_;
    bool is_start_ = false;  //added by lichunjing 2022-04-26
    bool is_ekf_inited_ = false;
    bool is_first_scan_ = true;
    common::Time first_lidar_time_;
    Eigen::Vector3d lidar_position_;

    // 地图相关操作
    bool is_local_map_initialized_ = false;
    std::vector<ikd_tree::BoxPointType> cube_need_remove_;
    ikd_tree::BoxPointType local_map_points_;
    int kdtree_delete_counter_ = 0;
    pcl::VoxelGrid<LidarPointType> downsize_filter_surf_;

    pcl::PointCloud<LidarPointType>::Ptr features_array_; 
    pcl::PointCloud<LidarPointType>::Ptr normal_vector_; 
    pcl::PointCloud<LidarPointType>::Ptr features_from_map_;

    std::vector<PointVector> nearest_points_vector_; 
};

} // namespace registration

#endif
