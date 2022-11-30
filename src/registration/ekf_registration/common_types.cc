///////////////////////////////////////////////////////////////////////////////
// by lichunjing 2022_04_26
///////////////////////////////////////////////////////////////////////////////

#include "src/common/time.h"
#include "src/common/time_conversion.h"
#include "src/sensor/imu_data.h"
#include "src/registration/ekf_registration/common_types.h"

#include "sensor_msgs/PointCloud2.h"

#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <Eigen/Geometry>
#include <string>

namespace registration
{
// 内部常用函数
namespace math
{
void PointcloudLidarToWorld(EkfState& ekf_state, 
                            TimedIdLidarPointCloud& pointcloud_in, 
                            TimedIdLidarPointCloud& pointcloud_out)
{
    for(int i = 0; i < pointcloud_in.pointcloud_ptr->points.size(); i++)
    {
        Eigen::Vector3d points_lidar(pointcloud_in.pointcloud_ptr->points.at(i).x, 
                                     pointcloud_in.pointcloud_ptr->points.at(i).y, 
                                     pointcloud_in.pointcloud_ptr->points.at(i).z);

        Eigen::Vector3d points_world = ekf_state.position 
                                     + ekf_state.rotation * (ekf_state.extrinsic_rotation*points_lidar + ekf_state.extrinsic_translation);

        pointcloud_out.pointcloud_ptr->points.at(i).x = points_world(0);
        pointcloud_out.pointcloud_ptr->points.at(i).y = points_world(1);
        pointcloud_out.pointcloud_ptr->points.at(i).z = points_world(2);
    }
}

bool EstimatePlane(Eigen::Matrix<float, 4, 1>& pca_result, 
                   const PointVector& point, 
                   const float& threshold)
{
    Eigen::Matrix<float, KNumPointsEstimatePlane, 3> A;
    Eigen::Matrix<float, KNumPointsEstimatePlane, 1> b;
    A.setZero();
    b.setOnes();
    b *= -1.0f;

    for (int j = 0; j < KNumPointsEstimatePlane; j++)
    {
        A(j,0) = point[j].x;
        A(j,1) = point[j].y;
        A(j,2) = point[j].z;
    }

    Eigen::Matrix<float, 3, 1> normal_vector = A.colPivHouseholderQr().solve(b);

    float normal = normal_vector.norm();
    pca_result(0) = normal_vector(0) / normal;
    pca_result(1) = normal_vector(1) / normal;
    pca_result(2) = normal_vector(2) / normal;
    pca_result(3) = 1.0 / normal;

    for(int j = 0; j < KNumPointsEstimatePlane; j++)
    {
        if(fabs(pca_result(0) * point[j].x + pca_result(1) * point[j].y + pca_result(2) * point[j].z + pca_result(3)) > threshold)
        {
            return false;
        }
    }
    
    return true;    
}


double CalculateLidarPointDistance(const LidarPointType& lidar_point1,
                                   const LidarPointType& lidar_point2)
{
    double distance = (lidar_point1.x - lidar_point2.x) * (lidar_point1.x - lidar_point2.x) 
                    + (lidar_point1.y - lidar_point2.y) * (lidar_point1.y - lidar_point2.y) 
                    + (lidar_point1.z - lidar_point2.z) * (lidar_point1.z - lidar_point2.z);
    return distance;
}


}   //namespace math

}   //namespace registration
