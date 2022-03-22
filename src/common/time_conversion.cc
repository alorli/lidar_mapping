
#include "src/common/time_conversion.h"

#include "src/common/time.h"
#include "ros/ros.h"

namespace common
{

ros::Time ToRos(common::Time time)
{
  int64_t uts_timestamp = common::ToUniversal(time);
  int64_t ns_since_unix_epoch = (uts_timestamp - common::kUtsEpochOffsetFromUnixEpochInSeconds *10000000ll) * 100ll;
  ros::Time ros_time;
  ros_time.fromNSec(ns_since_unix_epoch);
  return ros_time;
}

common::Time FromRos(const ::ros::Time& time)
{
  return common::FromUniversal(
      (time.sec + common::kUtsEpochOffsetFromUnixEpochInSeconds) * 10000000ll 
    + (time.nsec + 50) / 100); 
}

common::Time FromRosAddOffset(const ::ros::Time& time, double offset_second)
{
  return common::FromUniversal(
      (time.sec + offset_second + common::kUtsEpochOffsetFromUnixEpochInSeconds) * 10000000ll 
    + (time.nsec + 50) / 100); 
}


}  // namespace common
