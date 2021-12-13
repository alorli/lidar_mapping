///////////////////////////////////////////////////////////////////////////////
// by lichunjing 2021_02_08
///////////////////////////////////////////////////////////////////////////////
#ifndef CLOSELOOP_CONSTRAINTS_BUILDER_H_
#define CLOSELOOP_CONSTRAINTS_BUILDER_H_

#include "src/common/time.h"
#include "src/common/time_conversion.h"
#include "src/map_generator.h"

// #include <pcl/point_types.h>
// #include <pcl_conversions/pcl_conversions.h>
// #include <pcl/registration/ndt.h>

#include <Eigen/Core>
// #include <Eigen/Geometry>
// #include <vector>
#include <string>
// // #include <fstream>
#include "yaml-cpp/yaml.h"

namespace constraints
{


struct CloseloopConstraintsBuilderParameter
{
    int num_threads;
    double min_score;
    int select_interval;
    double min_constraints_time;
    double min_constraints_distance;
    double max_constraints_distance;
    double max_constraints_distance_z;
};

struct CloseloopConstraint
{
    long long allframe_id_from;
    long long allframe_id_to;
    double score;
    Eigen::Matrix4f relative_matrix;
};

class CloseloopConstraintsBuilder
{
public:
    CloseloopConstraintsBuilder(std::string cfg_file_path);
    ~CloseloopConstraintsBuilder();

    void LoadLidarOdomPoses();
    void SelectPairs();

    void ComputeConstraint();

    void ErasePairs();

    void BuildConstraints();

    std::vector<mapping::RegistrationResultsFilelist> GetLidarOdomPoses(){ return registration_results_filelists_;};
    std::vector<CloseloopConstraint> GetCloseloopConstraints(){ return closeloop_constraints_;};
private:
    mapping::RegistrationResultsFilelist SearchRegistrationResults(long long target_allframe_id);
    void ExtractRegistrationResults(mapping::RegistrationResultsFilelist& registration_result_from, 
                                    mapping::RegistrationResultsFilelist& registration_result_to,
                                    int index);
    void FilterPointCloud(pcl::PointCloud<registration::PointType>::Ptr pointcloud_ptr);

private:
    YAML::Node cfg_file_;
    std::string lidar_odom_pose_path_;
    // std::string gnss_pose_path_;

    CloseloopConstraintsBuilderParameter closeloop_constraints_builder_parameter_;

    std::vector<mapping::RegistrationResultsFilelist> registration_results_filelists_;
    std::vector<CloseloopConstraint> closeloop_constraints_;
};

} // namespace constraints

#endif