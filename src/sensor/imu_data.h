

#ifndef SENSOR_IMU_DATA_H_
#define SENSOR_IMU_DATA_H_

#include "Eigen/Core"
#include "src/common/time.h"


namespace sensor 
{

struct ImuData 
{
  common::Time time;
  Eigen::Vector3d linear_acceleration;    //线性加速度, m/s2
  Eigen::Vector3d angular_velocity;       //角速度, rad/s
};

}  // namespace sensor

#endif  // SENSOR_IMU_DATA_H_
