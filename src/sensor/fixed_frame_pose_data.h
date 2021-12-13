

#ifndef SENSOR_FIXED_FRAME_POSE_DATA_H_
#define SENSOR_FIXED_FRAME_POSE_DATA_H_

#include <memory>

#include "absl/types/optional.h"
#include "src/common/time.h"
#include "src/transform/rigid_transform.h"


namespace sensor 
{

struct FixedFramePoseData 
{
  common::Time time;
  absl::optional<transform::Rigid3d> pose;
  absl::optional<double> translation_weight_scale_xy;
  absl::optional<double> translation_weight_scale_z;
  absl::optional<double> rotation_weight_scale_horizontal_angle;
  absl::optional<double> rotation_weight_scale_heading_angle;
};

}  // namespace sensor

#endif  // SENSOR_FIXED_FRAME_POSE_DATA_H_
