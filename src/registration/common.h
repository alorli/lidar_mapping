///////////////////////////////////////////////////////////////////////////////
// by lichunjing 2022_12_02
///////////////////////////////////////////////////////////////////////////////
#ifndef REGISTRATION_H_
#define REGISTRATION_H_

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
// #include "yaml-cpp/yaml.h"

namespace registration
{

typedef pcl::PointXYZI PointType;

struct TimedIdPointCloud
{
    common::Time time;
    long long allframe_id;
    pcl::PointCloud<PointType> pointcloud;
};

struct AlignmentResult
{
    bool is_converged;
    double fitness_score;
    double trans_probability;
    double time_duration_ms;
    int final_num_iteration;
    Eigen::Matrix4f final_transform;
};

} // namespace registration

#endif
