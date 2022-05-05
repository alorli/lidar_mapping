///////////////////////////////////////////////////////////////////////////////
// by lichunjing 2022_04_24
///////////////////////////////////////////////////////////////////////////////
#ifndef EKF_REGISTRATION_H_
#define EKF_REGISTRATION_H_

#include "src/common/time.h"
#include "src/common/time_conversion.h"
#include "src/sensor/imu_data.h"

#include "sensor_msgs/PointCloud2.h"

#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include "src/registration/ekf_registration/common_types.h"
#include "src/registration/ekf_registration/imu_processor.h"
#include "src/registration/ekf_registration/ekf_processor.h"

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
                           TimedIdLidarPointCloud& timed_id_lidar_pointcloud);
    bool PrepareMeasurements();

private:
    YAML::Node cfg_file_;
    AllParameter all_parameter_;

    EkfState ekf_state_;
    ImuProcessor imu_processor_;
    EkfProcessor ekf_processor_;

    std::deque<TimedIdLidarPointCloud> timed_id_lidar_pointcloud_buffer_;
    std::deque<sensor::ImuData> imu_data_buffer_;
    Measurements measurements_;
    TimedIdLidarPointCloud compensationed_pointcloud_;
    
    long long lidar_allframe_id_;
    common::Time last_lidar_time_;
    common::Time last_imu_time_;

    bool is_lidar_prepared_;
    double lidar_average_scan_period_;
    int lidar_scan_count_;
    bool is_start_ = false;  //added by lichunjing 2022-04-26
};

} // namespace registration

#endif
