
#include "src/common/time.h"

#include <string>

namespace common
{

Duration FromSeconds(const double seconds)
{
  return std::chrono::duration_cast<Duration>(std::chrono::duration<double>(seconds));
}

double ToSeconds(const Duration duration)
{
  return std::chrono::duration_cast<std::chrono::duration<double>>(duration).count();
}


Time FromUniversal(const int64_t ticks)
{
  return Time(Duration(ticks));
}


int64_t ToUniversal(const Time time)
{
  return time.time_since_epoch().count();
}

// added by lichunjing 2021-02-08
double ToUniversalSeconds(Time time)
{
  double time_seconds = ToUniversal(time) / 10000000.0;
  return time_seconds;
}

std::ostream& operator<<(std::ostream& os, const Time time)
{
  os << std::to_string(ToUniversal(time));
  return os;
}


common::Duration FromMilliseconds(const int64_t milliseconds)
{
  return std::chrono::duration_cast<Duration>(
      std::chrono::milliseconds(milliseconds));
}

}  // namespace common
