
#include "src/processing/imu_tracker.h"

#include <cmath>
#include <limits>

#include "src/common/math.h"
#include "src/transform/transform.h"
#include "glog/logging.h"


namespace processing
{


ImuTracker::ImuTracker(const double imu_gravity_time_constant,
                       const common::Time time)
    : imu_gravity_time_constant_(imu_gravity_time_constant),
      time_(time),
      last_linear_acceleration_time_(common::Time::min()),   // 1796年
      orientation_(Eigen::Quaterniond::Identity()),
      gravity_vector_(Eigen::Vector3d::UnitZ()),          
      imu_angular_velocity_(Eigen::Vector3d::Zero())
{}



void ImuTracker::Advance(const common::Time time)
{
  const double delta_t = common::ToSeconds(time - time_);

  const Eigen::Quaterniond rotation =
      transform::AngleAxisVectorToRotationQuaternion(
          Eigen::Vector3d(imu_angular_velocity_ * delta_t));

  orientation_ = (orientation_ * rotation).normalized(); 

  gravity_vector_ = rotation.conjugate() * gravity_vector_; 

  time_ = time; 
}


void ImuTracker::AddImuLinearAccelerationObservation(
    const Eigen::Vector3d& imu_linear_acceleration)
{

  const double delta_t = last_linear_acceleration_time_ > common::Time::min()
          ? common::ToSeconds(time_ - last_linear_acceleration_time_) 
          : std::numeric_limits<double>::infinity(); 

  last_linear_acceleration_time_ = time_;

  const double alpha = 1. - std::exp(-delta_t / imu_gravity_time_constant_);


  gravity_vector_ = (1. - alpha) * gravity_vector_ + alpha * imu_linear_acceleration;

  const Eigen::Quaterniond rotation = Eigen::Quaterniond::FromTwoVectors(
      gravity_vector_, orientation_.conjugate() * Eigen::Vector3d::UnitZ());

  orientation_ = (orientation_ * rotation).normalized();

  CHECK_GT((orientation_ * gravity_vector_).z(), 0.);
  CHECK_GT((orientation_ * gravity_vector_).normalized().z(), 0.99);
}

void ImuTracker::AddImuAngularVelocityObservation(
    const Eigen::Vector3d& imu_angular_velocity)
{
  imu_angular_velocity_ = imu_angular_velocity;
}

}  // namespace processing
