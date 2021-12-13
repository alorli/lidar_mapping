#ifndef COMMON_PORT_H_
#define COMMON_PORT_H_

#include <boost/iostreams/device/back_inserter.hpp>
#include <boost/iostreams/filter/gzip.hpp>
#include <boost/iostreams/filtering_stream.hpp>
#include <cinttypes>
#include <cmath>
#include <string>


using int8 = int8_t;
using int16 = int16_t;
using int32 = int32_t;
using int64 = int64_t;
using uint8 = uint8_t;
using uint16 = uint16_t;
using uint32 = uint32_t;
using uint64 = uint64_t;

namespace common 
{

inline int RoundToInt(const float x) 
{ 
  return std::lround(x); 
}

inline int RoundToInt(const double x) 
{ 
  return std::lround(x); 
}

inline int64 RoundToInt64(const float x) 
{ 
  return std::lround(x); 
}

inline int64 RoundToInt64(const double x) 
{ 
  return std::lround(x); 
}


}  // namespace common

#endif  // COMMON_PORT_H_
