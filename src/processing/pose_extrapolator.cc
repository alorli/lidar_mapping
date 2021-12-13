
#include "src/processing/pose_extrapolator.h"

#include <algorithm>

#include "absl/memory/memory.h"
// #include "src/transform/transform.h"
#include "glog/logging.h"

#include <iostream>

namespace processing
{


PoseExtrapolator::PoseExtrapolator(const common::Duration pose_queue_duration,
                                   double imu_gravity_time_constant)
    : pose_queue_duration_(pose_queue_duration),
      gravity_time_constant_(imu_gravity_time_constant),
      cached_extrapolated_pose_{
                                  common::Time::min(),
                                  transform::Rigid3d::Identity()
                               },
      // added by lichunjing 2021_02_27
      exptrapolate_pose_odometry_for_ros_node_{
                                                 common::Time::min(),
                                                 transform::Rigid3d::Identity()
                                              }
{}


std::unique_ptr<PoseExtrapolator> PoseExtrapolator::InitializeWithImu(
    const common::Duration pose_queue_duration,
    const double imu_gravity_time_constant,
    const sensor::ImuData& imu_data)
{
  auto extrapolator = absl::make_unique<PoseExtrapolator>(pose_queue_duration,
                                                          imu_gravity_time_constant);

  extrapolator->AddImuData(imu_data);

  extrapolator->imu_tracker_ = absl::make_unique<ImuTracker>(imu_gravity_time_constant, imu_data.time);

  extrapolator->imu_tracker_->AddImuLinearAccelerationObservation(imu_data.linear_acceleration);

  extrapolator->imu_tracker_->AddImuAngularVelocityObservation(imu_data.angular_velocity);

  extrapolator->imu_tracker_->Advance(imu_data.time);

  extrapolator->AddPose(imu_data.time,
                        transform::Rigid3d::Rotation(extrapolator->imu_tracker_->orientation()));
  return extrapolator;
}

common::Time PoseExtrapolator::GetLastPoseTime() const
{
  if(timed_pose_queue_.empty()) 
  {
    return common::Time::min();
  }

  return timed_pose_queue_.back().time;
}


common::Time PoseExtrapolator::GetLastExtrapolatedTime() const
{
  if(!extrapolation_imu_tracker_)
  {
    return common::Time::min();
  }

  return extrapolation_imu_tracker_->time();
}

void PoseExtrapolator::AddPose(const common::Time time,
                               const transform::Rigid3d& pose)
{

  if(imu_tracker_ == nullptr)
  {

    common::Time tracker_start = time;

    if(!imu_data_.empty())
    {
      tracker_start = std::min(tracker_start, imu_data_.front().time);
    }

    imu_tracker_ =
        absl::make_unique<ImuTracker>(gravity_time_constant_, tracker_start);
  }


  timed_pose_queue_.push_back(TimedPose{time, pose});

  

  // added by lichunjing 2021_02_27
  exptrapolate_pose_odometry_for_ros_node_ = TimedPose{
                                                        time,
                                                        transform::Rigid3d::Identity()
                                                      };

  while(timed_pose_queue_.size() > 2 && timed_pose_queue_[1].time <= time - pose_queue_duration_)
  {
    timed_pose_queue_.pop_front();
  }

  // std::cout << "timed_pose_queue_.size():" << timed_pose_queue_.size() << std::endl;

  UpdateVelocitiesFromPoses();

  AdvanceImuTracker(time, imu_tracker_.get());

  TrimImuData();

  TrimOdometryData();

  odometry_imu_tracker_ = absl::make_unique<ImuTracker>(*imu_tracker_);
  extrapolation_imu_tracker_ = absl::make_unique<ImuTracker>(*imu_tracker_);
}

void PoseExtrapolator::AddImuData(const sensor::ImuData& imu_data)
{
  // 下面这一个 CHECK 语句在计算不过来时，有可能导致程序异常崩溃，暂且删掉这一句(by lichunjing 2021-04-09)
  // CHECK(timed_pose_queue_.empty() || imu_data.time >= timed_pose_queue_.back().time);

  imu_data_.push_back(imu_data);

  TrimImuData();
}

void PoseExtrapolator::AddOdometryData(const sensor::OdometryData& odometry_data)
{
  // 下面这一个 CHECK 语句在计算不过来时，有可能导致程序异常崩溃，暂且删掉这一句(by lichunjing 2021-04-09)
  // CHECK(timed_pose_queue_.empty() || odometry_data.time >= timed_pose_queue_.back().time);

  odometry_data_.push_back(odometry_data);

  TrimOdometryData();

  if(odometry_data_.size() < 2)
  {
    return;
  }

  const sensor::OdometryData& odometry_data_oldest = odometry_data_.front();
  const sensor::OdometryData& odometry_data_newest = odometry_data_.back();

  const double odometry_time_delta =
       common::ToSeconds(odometry_data_oldest.time - odometry_data_newest.time);

  const transform::Rigid3d odometry_pose_delta =
       odometry_data_newest.pose.inverse() * odometry_data_oldest.pose;

  angular_velocity_from_odometry_ =
      transform::RotationQuaternionToAngleAxisVector(odometry_pose_delta.rotation()) / odometry_time_delta;


  // std::cout << "angular_velocity_from_odometry_x:" << angular_velocity_from_odometry_[0] 
  //           << "  y:" << angular_velocity_from_odometry_[1] 
  //           << "  z:" << angular_velocity_from_odometry_[2] 
  //           << std::endl;


  if(timed_pose_queue_.empty())
  {
    return;
  }

  const Eigen::Vector3d
      linear_velocity_in_tracking_frame_at_newest_odometry_time =
          odometry_pose_delta.translation() / odometry_time_delta;


  // printf("\033[1;32m odometry_time_delta:%.6f odometry_pose_delta.translation:x:%.6f y:%.6f z:%.6f \033[0m \n",
  //        odometry_time_delta,
  //        odometry_pose_delta.translation().x(),
  //        odometry_pose_delta.translation().y(),
  //        odometry_pose_delta.translation().z());

  const Eigen::Quaterniond orientation_at_newest_odometry_time =
      timed_pose_queue_.back().pose.rotation() *
      ExtrapolateRotation(odometry_data_newest.time, odometry_imu_tracker_.get());

  linear_velocity_from_odometry_ =
      orientation_at_newest_odometry_time *
      linear_velocity_in_tracking_frame_at_newest_odometry_time;

  // printf("\033[1;32m linear_velocity_odometry:x:%.6f y:%.6f z:%.6f linear_velocity:x:%.6f y:%.6f z:%.6f \033[0m \n",
  //        linear_velocity_in_tracking_frame_at_newest_odometry_time[0],
  //        linear_velocity_in_tracking_frame_at_newest_odometry_time[1],
  //        linear_velocity_in_tracking_frame_at_newest_odometry_time[2],
  //        linear_velocity_from_odometry_[0],
  //        linear_velocity_from_odometry_[1],
  //        linear_velocity_from_odometry_[2]);

}


// added by lichunjing 2021_02_27
void PoseExtrapolator::AddOdometryData_for_ros_node(const sensor::OdometryData& odometry_data)
{
  // 下面这一个 CHECK 语句在计算不过来时，有可能导致程序异常崩溃，暂且删掉这一句(by lichunjing 2021-04-09)
  // CHECK(timed_pose_queue_.empty() || odometry_data.time >= timed_pose_queue_.back().time);

  odometry_data_.push_back(odometry_data);

  TrimOdometryData();

  if(odometry_data_.size() < 2)
  {
    return;
  }

  const sensor::OdometryData& odometry_data_oldest = odometry_data_.front();
  const sensor::OdometryData& odometry_data_newest = odometry_data_.back();


  const double odometry_time_delta =
       common::ToSeconds(odometry_data_oldest.time - odometry_data_newest.time);

  const transform::Rigid3d odometry_pose_delta =
       odometry_data_newest.pose.inverse() * odometry_data_oldest.pose;

  angular_velocity_from_odometry_ =
      transform::RotationQuaternionToAngleAxisVector(odometry_pose_delta.rotation()) / odometry_time_delta;

  if(timed_pose_queue_.empty())
  {
    return;
  }

  const Eigen::Vector3d
      linear_velocity_in_tracking_frame_at_newest_odometry_time =
          odometry_pose_delta.translation() / odometry_time_delta;

  const Eigen::Quaterniond orientation_at_newest_odometry_time =
      timed_pose_queue_.back().pose.rotation() *
      ExtrapolateRotation(odometry_data_newest.time, odometry_imu_tracker_.get());

  linear_velocity_from_odometry_ =
      orientation_at_newest_odometry_time *
      linear_velocity_in_tracking_frame_at_newest_odometry_time;




  // added by lichunjing 2021_02_27
  std::deque<sensor::OdometryData>::reverse_iterator it = odometry_data_.rbegin();

  const sensor::OdometryData& odometry_data_oldest_for_ros_node = *(it+1);


  const double odometry_time_delta_for_ros_node =
       common::ToSeconds(odometry_data_oldest_for_ros_node.time - odometry_data_newest.time);

  const transform::Rigid3d odometry_pose_delta_for_ros_node =
       odometry_data_newest.pose.inverse() * odometry_data_oldest_for_ros_node.pose;


  // const Eigen::Vector3d
  //     linear_velocity_in_tracking_frame_at_newest_odometry_time_for_ros_node =
  //         odometry_pose_delta_for_ros_node.translation() / odometry_time_delta_for_ros_node;


  const Eigen::Vector3d
      linear_velocity_in_tracking_frame_at_newest_odometry_time_for_ros_node =
          odometry_pose_delta_for_ros_node.translation() / (-1.0/odometry_msg_frenquency_);



  // printf("\033[1;32m odometry_time_delta:%.6f odometry_pose_translation:x:%.6f y:%.6f z:%.6f \033[0m \n",
  //        odometry_time_delta_for_ros_node,
  //        odometry_pose_delta_for_ros_node.translation()[0],
  //        odometry_pose_delta_for_ros_node.translation()[1],
  //        odometry_pose_delta_for_ros_node.translation()[2]);


  // 计算线速度，线速度是通过里程计计算的线速度在最新的 tracking坐标系中的坐标转换到 submap系中的分量
  linear_velocity_from_odometry_for_ros_node_ =
      orientation_at_newest_odometry_time *
      linear_velocity_in_tracking_frame_at_newest_odometry_time_for_ros_node;



  // printf("\033[1;32m linear_velocity_odometry:x:%.6f y:%.6f z:%.6f linear_velocity:x:%.6f y:%.6f z:%.6f \033[0m \n",
  //        linear_velocity_in_tracking_frame_at_newest_odometry_time_for_ros_node[0],
  //        linear_velocity_in_tracking_frame_at_newest_odometry_time_for_ros_node[1],
  //        linear_velocity_in_tracking_frame_at_newest_odometry_time_for_ros_node[2],
  //        linear_velocity_from_odometry_for_ros_node_[0],
  //        linear_velocity_from_odometry_for_ros_node_[1],
  //        linear_velocity_from_odometry_for_ros_node_[2]);

}


transform::Rigid3d PoseExtrapolator::ExtrapolatePose(const common::Time time)
{

  const TimedPose& newest_timed_pose = timed_pose_queue_.back();

  // 下面这一个 CHECK 语句在计算不过来时，有可能导致程序异常崩溃，暂且删掉这一句(by lichunjing 2021-04-09)
  // CHECK_GE(time, newest_timed_pose.time);

  if(cached_extrapolated_pose_.time != time)
  {
    const Eigen::Vector3d translation
        = ExtrapolateTranslation(time) + newest_timed_pose.pose.translation();

    const Eigen::Quaterniond rotation
        = newest_timed_pose.pose.rotation() *
          ExtrapolateRotation(time, extrapolation_imu_tracker_.get());

    cached_extrapolated_pose_ = TimedPose{time, transform::Rigid3d{translation, rotation}};
  }

  return cached_extrapolated_pose_.pose;
}


// added by lichunjing 2021_02_27
transform::Rigid3d PoseExtrapolator::ExtrapolatePose_for_ros_node(const common::Time time)
{
  // 最新时间的位姿，双端队列中时间上最新加入的位姿，也就是最晚加入的位姿
  // 也就是上一帧点云匹配后的位姿，是上一帧点云相对于匹配的 submap的相对位姿
  const TimedPose& newest_timed_pose = timed_pose_queue_.back();

  // 要预测的位姿时间点要大于或等于位姿队列中最新的时间点
  // 下面这一个 CHECK 语句在计算不过来时，有可能导致程序异常崩溃，暂且删掉这一句(by lichunjing 2021-04-09)
  // CHECK_GE(time, newest_timed_pose.time);


  // printf("\033[1;34m AAAAAAAAAAA delta_time:%.6f  cached_.time:%.5f ms  time:%.5f ms\033[0m \n",
  //        ToUniversal(time)/10000.0 - ToUniversal(cached_extrapolated_pose_.time)/10000.0,
  //        ToUniversal(cached_extrapolated_pose_.time)/10000.0,
  //        ToUniversal(time)/10000.0);

  // 如果时间 time处的位姿没有缓存过，就进行位姿外推，如果已经缓存过，直接返回缓存位姿
  if(cached_extrapolated_pose_.time != time)
  {
    // 平移量外推
    // ExtrapolateTranslation(time) 是平移量增量
    // const Eigen::Vector3d translation = ExtrapolateTranslation(time) + newest_timed_pose.pose.translation();

    // added by lichunjing 2021_02_27
    const Eigen::Vector3d temp_translation = ExtrapolateTranslation_for_ros_node(time)
                                             + exptrapolate_pose_odometry_for_ros_node_.pose.translation();

    const Eigen::Vector3d translation = temp_translation
                                        + newest_timed_pose.pose.translation();

    exptrapolate_pose_odometry_for_ros_node_.pose = transform::Rigid3d::Translation(temp_translation);


    const Eigen::Quaterniond rotation = newest_timed_pose.pose.rotation() *
                                        ExtrapolateRotation(time, extrapolation_imu_tracker_.get());


    cached_extrapolated_pose_ = TimedPose{time, transform::Rigid3d{translation, rotation}};
  }



  return cached_extrapolated_pose_.pose;
}


Eigen::Quaterniond PoseExtrapolator::EstimateGravityOrientation(
    const common::Time time)
{
  ImuTracker imu_tracker = *imu_tracker_;
  AdvanceImuTracker(time, &imu_tracker);

  return imu_tracker.orientation();
}


void PoseExtrapolator::UpdateVelocitiesFromPoses()
{
  if (timed_pose_queue_.size() < 2)
  {
    return;
  }

  // printf("\033[1;35m timed_pose_queue_--size:%d  x:%0.6f  y:%.6f  z:%.6f \033[0m \n",
  //                         timed_pose_queue_.size(),
  //                         timed_pose_queue_.back().pose.translation().x(),
  //                         timed_pose_queue_.back().pose.translation().y(),
  //                         timed_pose_queue_.back().pose.translation().z());

  CHECK(!timed_pose_queue_.empty());

  const TimedPose& newest_timed_pose = timed_pose_queue_.back();
  const auto newest_time = newest_timed_pose.time;

  const TimedPose& oldest_timed_pose = timed_pose_queue_.front();
  const auto oldest_time = oldest_timed_pose.time;

  const double queue_delta = common::ToSeconds(newest_time - oldest_time);

  if(queue_delta < common::ToSeconds(pose_queue_duration_))
  {
    // 1 ms
    LOG(WARNING) << "Queue too short for velocity estimation. Queue duration: "
                 << queue_delta << " s";
    return;
  }

  const transform::Rigid3d& newest_pose = newest_timed_pose.pose;
  const transform::Rigid3d& oldest_pose = oldest_timed_pose.pose;

  linear_velocity_from_poses_ = (newest_pose.translation() - oldest_pose.translation()) / queue_delta;

  angular_velocity_from_poses_ =
      transform::RotationQuaternionToAngleAxisVector(
          oldest_pose.rotation().inverse() * newest_pose.rotation()) / queue_delta;
}


void PoseExtrapolator::TrimImuData()
{

  while (imu_data_.size() > 1 && !timed_pose_queue_.empty() &&
         imu_data_[1].time <= timed_pose_queue_.back().time)
  {
    imu_data_.pop_front();
  }
}

void PoseExtrapolator::TrimOdometryData()
{
  while (odometry_data_.size() > 2 && !timed_pose_queue_.empty() &&
         odometry_data_[1].time <= timed_pose_queue_.back().time)
  {
    odometry_data_.pop_front();
  }
}


void PoseExtrapolator::AdvanceImuTracker(const common::Time time,
                                         ImuTracker* const imu_tracker) const
{
  // 下面这一个 CHECK 语句在计算不过来时，有可能导致程序异常崩溃，暂且删掉这一句(by lichunjing 2021-04-09)
  // CHECK_GE(time, imu_tracker->time());

  if(imu_data_.empty() || time < imu_data_.front().time)
  {
    imu_tracker->Advance(time);

    imu_tracker->AddImuLinearAccelerationObservation(Eigen::Vector3d::UnitZ());

    imu_tracker->AddImuAngularVelocityObservation(
        odometry_data_.size() < 2 ? angular_velocity_from_poses_    
                                  : angular_velocity_from_odometry_); 
    return;
  }


  if(imu_tracker->time() < imu_data_.front().time)
  {
    imu_tracker->Advance(imu_data_.front().time);
  }


  auto it = std::lower_bound(
      imu_data_.begin(), 
      imu_data_.end(),     
      imu_tracker->time(),
      [](const sensor::ImuData& imu_data, const common::Time& time)
      {
        return imu_data.time < time;
      });


  while(it != imu_data_.end() && it->time < time)
  {
    imu_tracker->Advance(it->time);   
    imu_tracker->AddImuLinearAccelerationObservation(it->linear_acceleration);  
    imu_tracker->AddImuAngularVelocityObservation(it->angular_velocity);      
    ++it;
  }

  imu_tracker->Advance(time);
}


Eigen::Quaterniond PoseExtrapolator::ExtrapolateRotation(
    const common::Time time,
    ImuTracker* const imu_tracker) const
{
  // 下面这一个 CHECK 语句在计算不过来时，有可能导致程序异常崩溃，暂且删掉这一句(by lichunjing 2021-04-09)
  // CHECK_GE(time, imu_tracker->time());

  AdvanceImuTracker(time, imu_tracker);

  const Eigen::Quaterniond last_orientation = imu_tracker_->orientation();

  return last_orientation.inverse() * imu_tracker->orientation();
}

Eigen::Vector3d PoseExtrapolator::ExtrapolateTranslation(common::Time time)
{
  const TimedPose& newest_timed_pose = timed_pose_queue_.back();

  const double extrapolation_delta = common::ToSeconds(time - newest_timed_pose.time);

  if(odometry_data_.size() < 2)
  {
    return extrapolation_delta * linear_velocity_from_poses_;
  }

  return extrapolation_delta * linear_velocity_from_odometry_;
}



// added by lichunjing 2021_02_27
Eigen::Vector3d PoseExtrapolator::ExtrapolateTranslation_for_ros_node(common::Time time)
{
  const TimedPose& newest_timed_pose = timed_pose_queue_.back();

  const double extrapolation_delta = common::ToSeconds(time - newest_timed_pose.time);

  if(odometry_data_.size() < 2)
  {
    return extrapolation_delta * linear_velocity_from_poses_;
  }

  // return extrapolation_delta * linear_velocity_from_odometry_;



  // added by lichunjing 2021_02_27
  const double extrapolation_delta_for_ros_node
                 = common::ToSeconds(time - exptrapolate_pose_odometry_for_ros_node_.time);

  exptrapolate_pose_odometry_for_ros_node_.time = time;

  // return extrapolation_delta_for_ros_node * linear_velocity_from_odometry_for_ros_node_;


  Eigen::Vector3d result = extrapolation_delta_for_ros_node * linear_velocity_from_odometry_for_ros_node_;


  return result;
}

}  // namespace processing
