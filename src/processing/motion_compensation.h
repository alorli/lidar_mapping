///////////////////////////////////////////////////////////////////////////////
// by lichunjing 2021_01_20
///////////////////////////////////////////////////////////////////////////////
#ifndef MOTION_COMPENSATION_H_
#define MOTION_COMPENSATION_H_


#include "src/registration/ndt_registration.h"
#include "src/processing/pose_extrapolator.h"

namespace processing
{

using registration::PointType;

class MotionCompensation
{
public:
    MotionCompensation(int lidar_n_scans);
    ~MotionCompensation();
    
    void LinearInterpolation(pcl::PointCloud<PointType>& pointcloud,
                             pcl::PointCloud<PointType>& pointcloud_compensation,
                             Eigen::Matrix4f transform_previous,
                             Eigen::Matrix4f transform_current
                            );

    bool CompensationUsingPoseExtrapolater(common::Time point_time_end,
                                           const pcl::PointCloud<PointType>& pointcloud,
                                           pcl::PointCloud<PointType>& pointcloud_compensation,
                                           processing::PoseExtrapolator* extrapolator_ptr
                                          );

    void SetLidarNScans(int lidar_n_scans) { lidar_n_scans_ = lidar_n_scans;};

private:
    int lidar_n_scans_;
    double lidar_frequency_;
};

} // namespace io

#endif