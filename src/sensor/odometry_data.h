
#ifndef SENSOR_ODOMETRY_DATA_H_
#define SENSOR_ODOMETRY_DATA_H_

#include "src/common/time.h"
#include "src/transform/transform.h"

namespace sensor 
{

struct OdometryData 
{
  common::Time time;
  transform::Rigid3d pose;  
};


}  // namespace sensor


#endif  // SENSOR_ODOMETRY_DATA_H_
