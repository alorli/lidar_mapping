
#ifndef COMMON_TIME_H_
#define COMMON_TIME_H_

#include <chrono>
#include <ostream>
#include <ratio>

namespace common
{
// 719162 是 0001年1月1日到 1970年1月1日所经历的天数
constexpr int64_t kUtsEpochOffsetFromUnixEpochInSeconds = (719162ll * 24ll * 60ll * 60ll);

// UniversalTimeScaleClock类实现 c++11 的 clock 接口，以 0.1us 为时间精度。
// 时间的起点是 0001年1月1日 00:00:00，和 Unix的时间起点是不同的
struct UniversalTimeScaleClock
{
  using rep = int64_t;

  // 百万分之一秒, 0.1us.
  using period = std::ratio<1, 10000000>;

  using duration = std::chrono::duration<rep, period>;

  // time_point 的模板参数是 UniversalTimeScaleClock, 那为何其可以做模板参数呢：？符合std::关于clock的类型定义和static成员
  using time_point = std::chrono::time_point<UniversalTimeScaleClock>;
  static constexpr bool is_steady = true;
};

// Represents Universal Time Scale durations and timestamps which are 64-bit
// integers representing the 100 nanosecond ticks since the Epoch which is
// January 1, 1 at the start of day in UTC.
// 定义2个别名：
using Duration = UniversalTimeScaleClock::duration;  //0.1us
using Time = UniversalTimeScaleClock::time_point;    //时间点

/*
Time::min()是chrono自带的函数。返回一个低于1970.01.01的数。
编译运行cpp/cppstdlib_2nd/util/chrono1.cpp:
epoch: Thu Jan  1 08:00:00 1970
now:   Tue Jul  4 19:39:29 2017
min:   Tue Sep 21 08:18:27 1677
max:   Sat Apr 12 07:47:16 2262
*/

// Convenience functions to create common::Durations.
//秒数seconds转为c++的duration实例对象
Duration FromSeconds(double seconds);
Duration FromMilliseconds(int64_t milliseconds);  //毫秒

// Returns the given duration in seconds.
//将duration实例对象转为秒数s
double ToSeconds(Duration duration);

// Creates a time from a Universal Time Scale.
//将c++的time_point对象转为UTC时间,单位是0.1us
Time FromUniversal(int64_t ticks);

// Outputs the Universal Time Scale timestamp for a given Time.
int64_t ToUniversal(Time time);

// added by lichunjing 2021-02-08
double ToUniversalSeconds(Time time);

// For logging and unit tests, outputs the timestamp integer.
// 重载了<<操作符,方便将time_point以string输出
std::ostream& operator<<(std::ostream& os, Time time);

}  // namespace common

#endif  // COMMON_TIME_H_
