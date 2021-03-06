
#ifndef COMMON_TIME_CONVERSION_H
#define COMMON_TIME_CONVERSION_H

#include "src/common/time.h"
#include "ros/ros.h"

namespace common
{

ros::Time ToRos(common::Time time);
common::Time FromRos(const ::ros::Time& time);
common::Time FromRosAddOffset(const ::ros::Time& time, double offset_second);

}  // namespace common

#endif  // COMMON_TIME_CONVERSION_H
