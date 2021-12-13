
#ifndef COMMON_TIME_H_
#define COMMON_TIME_H_

#include <chrono>
#include <ostream>
#include <ratio>

namespace common
{
constexpr int64_t kUtsEpochOffsetFromUnixEpochInSeconds = (719162ll * 24ll * 60ll * 60ll);

struct UniversalTimeScaleClock
{
  using rep = int64_t;

  using period = std::ratio<1, 10000000>;

  using duration = std::chrono::duration<rep, period>;

  using time_point = std::chrono::time_point<UniversalTimeScaleClock>;
  static constexpr bool is_steady = true;
};

using Duration = UniversalTimeScaleClock::duration;  //0.1us
using Time = UniversalTimeScaleClock::time_point;    //时间点

Duration FromSeconds(double seconds);
Duration FromMilliseconds(int64_t milliseconds);  //毫秒


double ToSeconds(Duration duration);


Time FromUniversal(int64_t ticks);


int64_t ToUniversal(Time time);


double ToUniversalSeconds(Time time);

std::ostream& operator<<(std::ostream& os, Time time);

}  // namespace common

#endif  // COMMON_TIME_H_
