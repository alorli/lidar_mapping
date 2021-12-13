
#include "src/common/time.h"

#include <string>

namespace common
{

//duration_cast是c++11的时间显式转换函数.
Duration FromSeconds(const double seconds)
{
  //将double类型的秒数转化为duration对象
  return std::chrono::duration_cast<Duration>(std::chrono::duration<double>(seconds));
}

double ToSeconds(const Duration duration)
{
  //反转化,count()返回时钟周期数,ticks
  return std::chrono::duration_cast<std::chrono::duration<double>>(duration).count();
}

//先构造一个临时duration对象,再将其转化为time_point对象
//Duration(ticks)调用的是UniversalTimeScaleClock的构造函数
Time FromUniversal(const int64_t ticks)
{
  return Time(Duration(ticks));
}

//count()返回time_point自epoch以来的时钟周期数
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

// 先将Time转化为 int64_t , 再转为字符串形式
std::ostream& operator<<(std::ostream& os, const Time time)
{
  os << std::to_string(ToUniversal(time));
  return os;
}

//mill是ms,micro是us,先将毫秒转为以毫秒计时的duration对象,再转化为以微妙计.
common::Duration FromMilliseconds(const int64_t milliseconds)
{
  return std::chrono::duration_cast<Duration>(
      std::chrono::milliseconds(milliseconds));
}

}  // namespace common
