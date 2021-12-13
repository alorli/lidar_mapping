
#ifndef COMMON_MATH_H_
#define COMMON_MATH_H_

#include <cmath>
#include <vector>

#include "Eigen/Core"
#include "src/common/port.h"


namespace common 
{

template <typename T>
T Clamp(const T value, const T min, const T max) 
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


template <typename T>
constexpr T Power(T base, int exponent) 
{
  return (exponent != 0) ? base * Power(base, exponent - 1) : T(1);
}


template <typename T>
constexpr T Pow2(T a) 
{
  return Power(a, 2);
}


constexpr double DegToRad(double deg) 
{ 
  return M_PI * deg / 180.; 
}


constexpr double RadToDeg(double rad) 
{ 
  return 180. * rad / M_PI; 
}


template <typename T>
T NormalizeAngleDifference(T difference) 
{
  const T kPi = T(M_PI);
  while (difference > kPi) difference -= 2. * kPi;
  while (difference < -kPi) difference += 2. * kPi;
  return difference;
}


template <typename T>
inline void QuaternionProduct(const double* const z, 
                              const T* const w,
                              T* const zw) 
{
  zw[0] = z[0] * w[0] - z[1] * w[1] - z[2] * w[2] - z[3] * w[3];
  zw[1] = z[0] * w[1] + z[1] * w[0] + z[2] * w[3] - z[3] * w[2];
  zw[2] = z[0] * w[2] - z[1] * w[3] + z[2] * w[0] + z[3] * w[1];
  zw[3] = z[0] * w[3] + z[1] * w[2] - z[2] * w[1] + z[3] * w[0];
}

}  // namespace common

#endif  // COMMON_MATH_H_
