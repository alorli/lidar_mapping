#ifndef COMMON_H_
#define COMMON_H_



namespace common
{
#ifndef M_PI
# define M_PI 3.141592653589793238462643383279502884196
#endif


double degree_to_radian(double degree);
double radian_to_degree(double radian);
double Clamp(const double value, const double min, const double max);

}  // namespace common


#endif  // COMMON_H_
