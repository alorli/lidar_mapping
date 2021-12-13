#ifndef COMMON_H_
#define COMMON_H_

#include <iostream>
#include <string>
#include <string.h>
#include <vector>
#include <dirent.h>


namespace common
{
#ifndef M_PI
# define M_PI 3.141592653589793238462643383279502884196
#endif


double degree_to_radian(double degree);
double radian_to_degree(double radian);
double Clamp(const double value, const double min, const double max);
std::vector<std::string> read_file_list(const char *base_path_ptr);

}  // namespace common


#endif  // COMMON_H_
