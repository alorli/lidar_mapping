///////////////////////////////////////////////////////////////////////////////
// by lichunjing 2020_12_23
///////////////////////////////////////////////////////////////////////////////
#ifndef GNSS_CONSTRAINTS_BUILDER_H_
#define GNSS_CONSTRAINTS_BUILDER_H_

#include "src/common/time.h"
#include "src/common/time_conversion.h"
#include "src/map_generator.h"

// #include <pcl/point_types.h>
// #include <pcl_conversions/pcl_conversions.h>
// #include <pcl/registration/ndt.h>

// // #include<Eigen/Core>
// #include <Eigen/Geometry>
#include <vector>
#include <string>
// // #include <fstream>
#include "yaml-cpp/yaml.h"

namespace constraints
{


struct GnssConstraintsBuilderParameter
{
    double min_delta_gnss_time;
    int min_num_satellites_add_constraints;
};

struct GnssPose
{
    long long allframe_id;
    common::Time time;
    int status;
    int service;
    Eigen::Vector3d position;
};

struct GnssPoseConstraint
{
    long long lidar_allframe_id;
    Eigen::Vector3d prior_position;
};

class GnssConstraintsBuilder
{
public:
    GnssConstraintsBuilder(std::string cfg_file_path,
                           std::string project_directory_name);
    ~GnssConstraintsBuilder();

    void LoadLidarOdomPoses();
    void LoadGnssPoses();
    void BuildConstraints();

    std::vector<mapping::RegistrationResultsFilelist> GetLidarOdomPoses(){ return registration_results_filelists_;};
    std::vector<GnssPoseConstraint> GetGnssPoseConstraints(){ return gnss_pose_constraints_;};

private:
    YAML::Node cfg_file_;
    std::string lidar_odom_pose_path_;
    std::string gnss_pose_path_;

    GnssConstraintsBuilderParameter gnss_constraints_builder_parameter_;

    std::vector<mapping::RegistrationResultsFilelist> registration_results_filelists_;
    std::vector<GnssPose> gnss_poses_;
    std::vector<GnssPoseConstraint> gnss_pose_constraints_;
};

} // namespace io

#endif