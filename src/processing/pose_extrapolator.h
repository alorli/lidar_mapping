
#ifndef PROCESSING_POSE_EXTRAPOLATOR_H_
#define PROCESSING_POSE_EXTRAPOLATOR_H_

#include <deque>
#include <memory>

#include "src/common/time.h"
#include "src/processing/imu_tracker.h"
#include "src/sensor/imu_data.h"
#include "src/sensor/odometry_data.h"
#include "src/transform/rigid_transform.h"


namespace processing
{
class PoseExtrapolator
{
 public:
  explicit PoseExtrapolator(common::Duration pose_queue_duration,
                            double imu_gravity_time_constant);

  PoseExtrapolator(const PoseExtrapolator&) = delete;
  PoseExtrapolator& operator=(const PoseExtrapolator&) = delete;

  static std::unique_ptr<PoseExtrapolator> InitializeWithImu(common::Duration pose_queue_duration,
                                                             double imu_gravity_time_constant,
                                                             const sensor::ImuData& imu_data);

  common::Time GetLastPoseTime() const;
  common::Time GetLastExtrapolatedTime() const;

  void AddPose(common::Time time, 
               const transform::Rigid3d& pose);

  void AddImuData(const sensor::ImuData& imu_data);

  void AddOdometryData(const sensor::OdometryData& odometry_data);

  void AddOdometryData_for_ros_node(const sensor::OdometryData& odometry_data);


  transform::Rigid3d ExtrapolatePose(common::Time time);

  transform::Rigid3d ExtrapolatePose_for_ros_node(common::Time time);

  Eigen::Quaterniond EstimateGravityOrientation(common::Time time);

  void SetOdometryMsgFrenquency(double frenquency)
  {
    odometry_msg_frenquency_ = frenquency;
  }

 private:
  void UpdateVelocitiesFromPoses();
  void TrimImuData();
  void TrimOdometryData();
  void AdvanceImuTracker(common::Time time,
                         ImuTracker* imu_tracker) const;

  Eigen::Quaterniond ExtrapolateRotation(common::Time time,
                                         ImuTracker* imu_tracker) const;

  Eigen::Vector3d ExtrapolateTranslation(common::Time time);



  Eigen::Vector3d ExtrapolateTranslation_for_ros_node(common::Time time);

  const common::Duration pose_queue_duration_;

  struct TimedPose
  {
    common::Time time;
    transform::Rigid3d pose;
  };

  std::deque<TimedPose> timed_pose_queue_;

  Eigen::Vector3d linear_velocity_from_poses_ = Eigen::Vector3d::Zero();
  Eigen::Vector3d angular_velocity_from_poses_ = Eigen::Vector3d::Zero();

  const double gravity_time_constant_;

  std::deque<sensor::ImuData> imu_data_;

  std::unique_ptr<ImuTracker> imu_tracker_;            
  std::unique_ptr<ImuTracker> odometry_imu_tracker_;       
  std::unique_ptr<ImuTracker> extrapolation_imu_tracker_;  

  TimedPose cached_extrapolated_pose_;

  std::deque<sensor::OdometryData> odometry_data_;

  Eigen::Vector3d linear_velocity_from_odometry_ = Eigen::Vector3d::Zero();
  Eigen::Vector3d angular_velocity_from_odometry_ = Eigen::Vector3d::Zero();

  Eigen::Vector3d linear_velocity_from_odometry_for_ros_node_ = Eigen::Vector3d::Zero();
  TimedPose exptrapolate_pose_odometry_for_ros_node_;
  double odometry_msg_frenquency_;
};

}  // namespace processing

#endif  // PROCESSING_POSE_EXTRAPOLATOR_H_
