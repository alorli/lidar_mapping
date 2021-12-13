///////////////////////////////////////////////////////////////////////////////
// by lichunjing 2020_12_23
///////////////////////////////////////////////////////////////////////////////
#ifndef NDT_REGISTRATION_H_
#define NDT_REGISTRATION_H_

#include "src/common/time.h"
#include "src/common/time_conversion.h"

#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/registration/ndt.h>

// #include<Eigen/Core>
#include <Eigen/Geometry>
// #include <vector>
#include <string>
// #include <fstream>
#include "yaml-cpp/yaml.h"

namespace registration
{

typedef pcl::PointXYZI PointType;

struct TimedIdPointCloud
{
    common::Time time;
    long long allframe_id;
    pcl::PointCloud<PointType> pointcloud;
};

struct NdtParameter
{
    double transformation_epsilon;
    double step_size;
    double resolution;
    int max_iterations;
};

struct FilterParameter
{
    double min_horizontal_radius;
    double voxel_leaf_size;
};

struct AlignmentResult
{
    bool is_converged;
    double fitness_score;
    double time_duration_ms;
    int final_num_iteration;
    Eigen::Matrix4f final_transform;
};

struct MappingParameter
{
    double min_keyframe_distance;
    double map_trim_distance;
    
    struct PassThroughFilter
    {
        double x_min, x_max, y_min, y_max, z_min, z_max;
    };
    PassThroughFilter pass_through_filter;
};

struct PoseType
{
  double x;
  double y;
  double z;
  double roll;
  double pitch;
  double yaw;
};

class NdtRegistration
{
public:
    NdtRegistration(std::string cfg_file_path,
                    int registration_flag);
    ~NdtRegistration();
    
    // 如果调用时提供了预测位姿，使用提供的预测位姿进行配准
    void AddSensorData(const TimedIdPointCloud& timed_id_pointcloud, 
                       const Eigen::Matrix4f  predict_matrix); 
    // 如果调用时没有提供预测位姿，内部使用线性外推预测位姿进行配准
    void AddSensorData(const TimedIdPointCloud& timed_id_pointcloud);
    void FilterSensorData(const TimedIdPointCloud& timed__id_pointcloud, 
                          pcl::PointCloud<PointType>::Ptr voxel_filtered_pointcloud_ptr);
    void Alignment(pcl::PointCloud<PointType>::Ptr voxel_filtered_pointcloud_ptr, 
                   const Eigen::Matrix4f  predict_matrix);
    void AddKeyframeToMap();
    void TrimMap();
    Eigen::Matrix4f PredictPose();
    Eigen::Matrix4f PredictPoseUseTf();
    double calcDiffForRadian(const double lhs_rad, const double rhs_rad);
    pcl::PointCloud<PointType>::ConstPtr GetMapConstPtr() {return map_ptr_;};
    const AlignmentResult GetAlignmentResult() {return alignment_result_;};

private:
    YAML::Node cfg_file_;

    FilterParameter filter_parameter_;
    NdtParameter ndt_parameter_;
    MappingParameter mapping_parameter_;
    
    pcl::NormalDistributionsTransform<PointType, PointType> pcl_ndt_;
    pcl::PointCloud<PointType>::Ptr map_ptr_;

    AlignmentResult alignment_result_;
    AlignmentResult alignment_result_previous_;


    PoseType pose_last_keyframe_;
    PoseType pose_last_trim_;
    TimedIdPointCloud filtered_timed_id_pointcloud_;

    bool is_fist_frame_;
    bool is_fist_map_;
    int registration_flag_;   // 0: raw, 1:compensation
};

} // namespace registration

#endif