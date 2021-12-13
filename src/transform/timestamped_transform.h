

#ifndef TRANSFORM_TIMESTAMPED_TRANSFORM_H_
#define TRANSFORM_TIMESTAMPED_TRANSFORM_H_

#include "src/common/time.h"
#include "src/transform/rigid_transform.h"

namespace transform 
{

struct TimestampedTransform 
{
  common::Time time;
  transform::Rigid3d transform;
};

TimestampedTransform Interpolate(const TimestampedTransform& start,
                                 const TimestampedTransform& end,
                                 const common::Time time);

}  // namespace transform

#endif  // TRANSFORM_TIMESTAMPED_TRANSFORM_H_
