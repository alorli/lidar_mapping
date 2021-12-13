
#include "src/common/common.h"

namespace common
{

double degree_to_radian(double degree)
{
    return (degree / 180.0 * M_PI);
}

double radian_to_degree(double radian)
{
    return (radian / M_PI * 180.0);
}

// Clamps 'value' to be in the range ['min', 'max'].
//将 val截取到区间min至max中.
double Clamp(const double value, const double min, const double max)
{
  if (value > max)
  {
    return max;
  }
  if (value < min)
  {
    return min;
  }
  return value;
}


}  // namespace common
