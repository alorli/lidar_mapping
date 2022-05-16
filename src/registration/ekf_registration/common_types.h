///////////////////////////////////////////////////////////////////////////////
// by lichunjing 2022_04_26
///////////////////////////////////////////////////////////////////////////////
#ifndef COMMON_TYPES_H_
#define COMMON_TYPES_H_

#include "src/common/time.h"
#include "src/common/time_conversion.h"
#include "src/sensor/imu_data.h"

#include "sensor_msgs/PointCloud2.h"

#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <Eigen/Geometry>
#include <string>

namespace velodyne
{
    struct EIGEN_ALIGN16 Point 
    {
        PCL_ADD_POINT4D;
        float intensity;
        float time;
        uint16_t ring;
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    };
}

POINT_CLOUD_REGISTER_POINT_STRUCT(velodyne::Point,
    (float, x, x)
    (float, y, y)
    (float, z, z)
    (float, intensity, intensity)
    (float, time, time)
    (uint16_t, ring, ring)
)



namespace registration
{

struct LidarProcessParameter
{
    double lidar_min_range;
    int ring_filter_interval;
};

struct ImuParameter
{
    double gravity_local;
    int init_count_max;
    double covariance_acc;
    double covariance_gyro;
    double covariance_bias_acc;
    double covariance_bias_gyro;
};

struct ExtrinsicLidarInImu
{
    ExtrinsicLidarInImu()
    {
        translation = std::vector<double>(3, 0.0);
        rotation = std::vector<double>(9, 0.0);
    };

    std::vector<double> translation;
    std::vector<double> rotation;
};

struct EkfParameter
{
    int max_iteration;
    double converge_threshold;
};

struct AllParameter
{
    LidarProcessParameter lidar_process_parameter;
    ImuParameter imu_parameter;
    ExtrinsicLidarInImu extrinsic_lidar_in_imu;
};

typedef pcl::PointXYZINormal LidarPointType;
struct TimedIdLidarPointCloud
{
    common::Time time;
    long long allframe_id;
    pcl::PointCloud<LidarPointType> pointcloud;
};

struct TimedImuPose
{
    // TimedImuPose()
    // {
        // offset_time = 0.0;
        // acc = Eigen::Vector3d::Zero();
        // gyro = Eigen::Vector3d::Zero();
        // velocity = Eigen::Vector3d::Zero();
        // position = Eigen::Vector3d::Zero();
        // rotation = Eigen::Quaterniond::Identity();
    // };

    double offset_time;
    Eigen::Vector3d acc;
    Eigen::Vector3d gyro;
    Eigen::Vector3d velocity;
    Eigen::Vector3d position;
    Eigen::Quaterniond rotation;
};


struct Measurements
{
    Measurements()
    {
        lidar_begin_time = common::FromUniversal(0);
    };
    
    common::Time lidar_begin_time;
    common::Time lidar_end_time;
    TimedIdLidarPointCloud timed_id_lidar_pointcloud;
    std::deque<sensor::ImuData> imu_data_buffer;
};

///////// EKF types ////////////////////////////////////////////////////
// 常量
constexpr double kGravityBase = 9.809;
// constexpr int kStateDimension = 24;
constexpr float kFloatLimit = 1e-5f;
constexpr double kDoubleLimit = 1e-11;

struct EkfState
{
    EkfState()
    {
        rotation = Eigen::Quaterniond::Identity();
        position = Eigen::Vector3d::Zero();
        velocity = Eigen::Vector3d::Zero();
        bias_acc = Eigen::Vector3d::Zero();
        bias_gyro = Eigen::Vector3d::Zero();
        gravity = kGravityBase * Eigen::Vector3d(std::sqrt(1), 0, 0);
        extrinsic_rotation = Eigen::Quaterniond::Identity();
        extrinsic_translation = Eigen::Vector3d::Zero();
    };

    Eigen::Quaterniond rotation;                  //0
    Eigen::Vector3d position;                     //3
    Eigen::Vector3d velocity;                     //6
    Eigen::Vector3d bias_acc;                     //9
    Eigen::Vector3d bias_gyro;                    //12
    Eigen::Vector3d gravity;                      //15
    Eigen::Quaterniond extrinsic_rotation;        //18
    Eigen::Vector3d extrinsic_translation;        //21
};

struct InputState
{
    InputState()
    {
        acc = Eigen::Vector3d::Zero();
        gyro = Eigen::Vector3d::Zero();
    };

    Eigen::Vector3d acc;                     //0
    Eigen::Vector3d gyro;                    //3
};

struct ProcessNoiseState
{
    ProcessNoiseState()
    {
        acc = Eigen::Vector3d::Zero();
        gyro = Eigen::Vector3d::Zero();
        bias_acc = Eigen::Vector3d::Zero();
        bias_gyro = Eigen::Vector3d::Zero();
    };

    Eigen::Vector3d acc;                     //0
    Eigen::Vector3d gyro;                    //3
    Eigen::Vector3d bias_acc;                //6
    Eigen::Vector3d bias_gyro;               //9
};


// 常用内部函数声明
namespace math
{

Eigen::Matrix3d hat(const Eigen::Vector3d& vector3d);
Eigen::Quaterniond exp(Eigen::Vector3d vec, 
		               const double& scale);
Eigen::Matrix3d Exp(const Eigen::Vector3d &angle_velocity, 
                    const double &delta_time);
Eigen::Matrix<double, 3, 3> AMatrix(Eigen::Vector3d& v);
struct GravityManifold;

}


} // namespace registration



#endif
