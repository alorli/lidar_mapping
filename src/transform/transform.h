
#ifndef TRANSFORM_TRANSFORM_H_
#define TRANSFORM_TRANSFORM_H_

#include <cmath>

#include "Eigen/Core"
#include "Eigen/Geometry"
#include "src/common/math.h"
#include "src/transform/rigid_transform.h"

namespace transform 
{

template <typename FloatType>
FloatType GetAngle(const Rigid3<FloatType>& transform) 
{
  return FloatType(2) * std::atan2(transform.rotation().vec().norm(),
                                   std::abs(transform.rotation().w()));
}

template <typename T>
T GetYaw(const Eigen::Quaternion<T>& rotation) 
{
  const Eigen::Matrix<T, 3, 1> direction = rotation * Eigen::Matrix<T, 3, 1>::UnitX();
  return atan2(direction.y(), direction.x());
}


template <typename T>
T GetYaw(const Rigid3<T>& transform) 
{
  return GetYaw(transform.rotation());
}

template <typename T>
Eigen::Matrix<T, 3, 1> RotationQuaternionToAngleAxisVector(const Eigen::Quaternion<T>& quaternion) 
{
  Eigen::Quaternion<T> normalized_quaternion = quaternion.normalized();

  if (normalized_quaternion.w() < 0.) 
  {
    normalized_quaternion.w() = -1. * normalized_quaternion.w();
    normalized_quaternion.x() = -1. * normalized_quaternion.x();
    normalized_quaternion.y() = -1. * normalized_quaternion.y();
    normalized_quaternion.z() = -1. * normalized_quaternion.z();
  }

  const T angle = 2. * atan2(normalized_quaternion.vec().norm(), normalized_quaternion.w());

  constexpr double kCutoffAngle = 1e-7; 

  const T scale = angle < kCutoffAngle ? T(2.) : angle / sin(angle / 2.);
  
  return Eigen::Matrix<T, 3, 1>(scale * normalized_quaternion.x(),
                                scale * normalized_quaternion.y(),
                                scale * normalized_quaternion.z());
}

template <typename T>
Eigen::Quaternion<T> AngleAxisVectorToRotationQuaternion(
    const Eigen::Matrix<T, 3, 1>& angle_axis) 
{
  T scale = T(0.5);
  T w = T(1.);
  constexpr double kCutoffAngle = 1e-8;  


  if (angle_axis.squaredNorm() > kCutoffAngle) 
  {
    const T norm = angle_axis.norm();
    scale = sin(norm / 2.) / norm;
    w = cos(norm / 2.);
  }

  const Eigen::Matrix<T, 3, 1> quaternion_xyz = scale * angle_axis;
  return Eigen::Quaternion<T>(w, quaternion_xyz.x(), quaternion_xyz.y(),
                              quaternion_xyz.z());
}


template <typename T>
Rigid2<T> Project2D(const Rigid3<T>& transform) 
{
  return Rigid2<T>(transform.translation().template head<2>(),
                   GetYaw(transform));
}


template <typename T>
Rigid3<T> Embed3D(const Rigid2<T>& transform) 
{
  return Rigid3<T>(
      {transform.translation().x(), transform.translation().y(), T(0)},
      Eigen::AngleAxis<T>(transform.rotation().angle(),
                          Eigen::Matrix<T, 3, 1>::UnitZ()));
}


}  // namespace transform

#endif  // TRANSFORM_TRANSFORM_H_
