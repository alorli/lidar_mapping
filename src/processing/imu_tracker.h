
#ifndef PROCESSING_IMU_TRACKER_H_
#define PROCESSING_IMU_TRACKER_H_

#include "Eigen/Geometry"
#include "src/common/time.h"

namespace processing 
{


class ImuTracker 
{
 public:

  ImuTracker(double imu_gravity_time_constant, common::Time time);


  void Advance(common::Time time);

  void AddImuLinearAccelerationObservation(
      const Eigen::Vector3d& imu_linear_acceleration);

  
  void AddImuAngularVelocityObservation(
      const Eigen::Vector3d& imu_angular_velocity);


  common::Time time() const 
  { 
      return time_; 
  }


  Eigen::Quaterniond orientation() const 
  { 
      return orientation_; 
  }

 private:
  const double imu_gravity_time_constant_;    
  common::Time time_;                         
  common::Time last_linear_acceleration_time_; 
  Eigen::Quaterniond orientation_;              
  Eigen::Vector3d gravity_vector_;          
  Eigen::Vector3d imu_angular_velocity_;  
};

}  // namespace mapping

#endif  // PROCESSING_IMU_TRACKER_H_
