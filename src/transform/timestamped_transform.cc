
#include "src/transform/timestamped_transform.h"

#include "src/transform/transform.h"

#include "glog/logging.h"


namespace transform 
{


TimestampedTransform Interpolate(const TimestampedTransform& start,
                                 const TimestampedTransform& end,
                                 const common::Time time) 
{
  CHECK_LE(start.time, time);

  CHECK_GE(end.time, time);

  const double duration = common::ToSeconds(end.time - start.time);
  const double factor = common::ToSeconds(time - start.time) / duration;

  const Eigen::Vector3d origin =
      start.transform.translation() +
      (end.transform.translation() - start.transform.translation()) * factor;

  const Eigen::Quaterniond rotation =
      Eigen::Quaterniond(start.transform.rotation())
          .slerp(factor, Eigen::Quaterniond(end.transform.rotation()));

  return TimestampedTransform{time, transform::Rigid3d(origin, rotation)};
}


}  // namespace transform
