///////////////////////////////////////////////////////////////////////////////
// by lichunjing 2022_04_25
///////////////////////////////////////////////////////////////////////////////

#ifndef IMU_PROCESSOR_H_
#define IMU_PROCESSOR_H_

#include "src/common/time.h"
#include "src/common/time_conversion.h"

#include "src/registration/ekf_registration/common_types.h"
#include "src/registration/ekf_registration/ekf_processor.h"

#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
// #include<Eigen/Core>
#include <Eigen/Geometry>
// #include <vector>
#include <string>
#include "yaml-cpp/yaml.h"

namespace registration
{

class ImuProcessor
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    ImuProcessor(std::string cfg_file_path);
    ~ImuProcessor();
    void Process(EkfProcessor& ekf_processor,
                 Measurements& measurements,
                 TimedIdLidarPointCloud& compensationed_pointcloud,
                 TimedIdLidarPointCloud& compensationed_pointcloud_raw);
    void ImuReset();
    void SetImuParameter(ImuParameter& imu_parameter);
    void SetExtrinsic(ExtrinsicLidarInImu& extrinsic_lidar_in_imu);
    void ImuIntialize(EkfProcessor& ekf_processor,
                      Measurements& measurements,
                      int& num_measurements);
    void LidarMotionCompensation(EkfProcessor& ekf_processor,
                                 Measurements& measurements,
                                 TimedIdLidarPointCloud& compensationed_pointcloud,
                                 TimedIdLidarPointCloud& compensationed_pointcloud_raw
                                 );
    

private:
    YAML::Node cfg_file_;

    ImuParameter imu_parameter_;
    Eigen::Vector3d average_acc_;
    Eigen::Vector3d average_gyro_;
     
    Eigen::Vector3d covariance_acc_;
    Eigen::Vector3d covariance_gyro_;
    Eigen::Vector3d covariance_acc_scale_;
    Eigen::Vector3d covariance_gyro_scale_;
    Eigen::Vector3d covariance_bias_acc_;
    Eigen::Vector3d covariance_bias_gyro_;
    

    Eigen::Matrix<double, 12, 12> Q_;

    Eigen::Vector3d translation_lidar_in_imu_;
    Eigen::Matrix3d rotation_lidar_in_imu_;

    sensor::ImuData last_imu_data_;
    Eigen::Vector3d acc_last_;                // 补偿掉重力加速度的线加速度
    Eigen::Vector3d angle_velocity_last_;     // 角速度
    common::Time last_lidar_end_time_;

    int init_count_ = 1;
    bool is_first_frame_ = true;
    bool is_imu_initialized_ = false;
};

} // namespace registration

#endif
