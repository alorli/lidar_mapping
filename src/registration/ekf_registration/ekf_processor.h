///////////////////////////////////////////////////////////////////////////////
// by lichunjing 2022_04_25
///////////////////////////////////////////////////////////////////////////////
#ifndef EKF_PROCESSOR_H_
#define EKF_PROCESSOR_H_

#include "src/common/time.h"
#include "src/common/time_conversion.h"

#include "src/registration/ekf_registration/common_types.h"
#include "src/registration/ekf_registration/ikd_tree.h"

#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
// #include<Eigen/Core>
#include <Eigen/Geometry>
// #include <vector>
#include <string>
#include "yaml-cpp/yaml.h"

namespace registration
{

struct MeasurementData
{
	bool is_valid;
	bool is_converge;
	Eigen::Matrix<double, Eigen::Dynamic, 1> z;
	Eigen::Matrix<double, Eigen::Dynamic, 1> h;
	Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> h_v;
	Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> h_x;
	Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> R;
};


class EkfProcessor
{
public:
    EkfProcessor(std::string cfg_file_path);
    ~EkfProcessor();

    void SetEkfState(EkfState ekf_state);
    EkfState GetEkfState();
    Eigen::Matrix<double, 23, 23> GetProcessCovariance();
    void SetProcessCovariance(Eigen::Matrix<double, 23, 23> process_covariance);

    void PredictState(double& delta_time, 
                      Eigen::Matrix<double, 12, 12>& Q, 
                      InputState& input_state);
    
    void UpdateState(ikd_tree::KD_TREE& global_ikd_tree,
                     std::vector<PointVector>& nearest_points_vector,
                     TimedIdLidarPointCloud& compensationed_features_pointcloud_downsize_lidar,
                     TimedIdLidarPointCloud& compensationed_features_pointcloud_downsize_world);

    void CalculateMeasurement(ikd_tree::KD_TREE& global_ikd_tree,
                              std::vector<PointVector>& nearest_points_vector,
                              TimedIdLidarPointCloud& compensationed_features_pointcloud_downsize_lidar,
                              TimedIdLidarPointCloud& compensationed_features_pointcloud_downsize_world);
    
    Eigen::Matrix<double, 23, 1> CalculateStateBoxMinus(EkfState& state_propagated);

private:
    YAML::Node cfg_file_;

    EkfState state_;
    Eigen::Matrix<double, 23, 23> process_covariance_;
    Eigen::Matrix<double, 23, 23> L_ = Eigen::Matrix<double, 23, 23>::Identity();

    EkfParameter ekf_parameter_;

    MeasurementData measurement_data_;
    pcl::PointCloud<LidarPointType>::Ptr laser_cloud_origin_; 
    pcl::PointCloud<LidarPointType>::Ptr corresponding_normal_vector_;
    pcl::PointCloud<LidarPointType>::Ptr normal_vector_;
    std::vector<PointVector> nearest_points_vector_;

    bool point_selected_surf_[100000] = {0};
    float residual_last_[100000] = {0.0};

    int effective_features_num_ = 0;
    double converge_limit[23] = {0.001};
};

} // namespace registration

#endif
