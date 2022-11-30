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
    double converge_limit;
    bool extrinsic_estimate_enable;
};

struct LocalMapParameter
{
    double cube_length;
    double move_threshold;
    double delta_range;
    double filter_size_surf;
    double filter_size_map;
};

struct AllParameter
{
    LidarProcessParameter lidar_process_parameter;
    ImuParameter imu_parameter;
    ExtrinsicLidarInImu extrinsic_lidar_in_imu;
    LocalMapParameter local_map_parameter;
};

typedef pcl::PointXYZINormal LidarPointType;
typedef std::vector<LidarPointType, Eigen::aligned_allocator<LidarPointType>>  PointVector;


struct TimedIdLidarPointCloud
{
    // 默认构造函数,在没有显示调用构造函数的时候调用,否则会导致编译通过,但存在使用空指针的风险
    TimedIdLidarPointCloud():
        pointcloud_ptr(new pcl::PointCloud<LidarPointType>())
    {
        std::cout << "-------construct TimedIdLidarPointCloud!" << std::endl;
    };


    common::Time time;
    long long allframe_id;
    pcl::PointCloud<LidarPointType>::Ptr pointcloud_ptr;
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
constexpr double kEkfInitTime = 0.1;
constexpr double kLaserPointcovariance = 0.001;
constexpr int KNumPointsEstimatePlane = 5;

/*   原始状态变量定义
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
*/

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

    Eigen::Vector3d position;                        //0
    Eigen::Quaterniond rotation;                     //3
    Eigen::Quaterniond extrinsic_rotation;           //6
    Eigen::Vector3d extrinsic_translation;           //9
    Eigen::Vector3d velocity;                        //12
    Eigen::Vector3d bias_gyro;                       //15
    Eigen::Vector3d bias_acc;                        //18
    Eigen::Vector3d gravity;                         //21
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

Eigen::Vector3d BoxMinusSO3(Eigen::Quaterniond& rotation1,
                            Eigen::Quaterniond& rotation2
                            );

Eigen::Vector2d BoxMinusS2(Eigen::Vector3d& gravity1,
                           Eigen::Vector3d& gravity2
                           );

Eigen::Vector3d BoxPlusS2(Eigen::Vector3d& gravity,
                          Eigen::Vector2d& delta
                          );

void PointcloudLidarToWorld(EkfState& ekf_state, 
                            TimedIdLidarPointCloud& pointcloud_in, 
                            TimedIdLidarPointCloud& pointcloud_out);

bool EstimatePlane(Eigen::Matrix<float, 4, 1>& pca_result, 
                   const PointVector& point, 
                   const float& threshold);

double CalculateLidarPointDistance(const LidarPointType& lidar_point1,
                                   const LidarPointType& lidar_point2);
}


} // namespace registration



#endif
