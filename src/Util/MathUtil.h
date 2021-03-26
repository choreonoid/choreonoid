#ifndef CNOID_UTIL_MATH_UTIL_H
#define CNOID_UTIL_MATH_UTIL_H

#include <cmath>

namespace cnoid {

constexpr double PI = 3.141592653589793238462643383279502884;
constexpr double PI_2 = 1.570796326794896619231321691639751442;
constexpr double TO_DEGREE = 180.0 / PI;
constexpr double TO_RADIAN = PI / 180.0;

inline double degree(double rad) { return TO_DEGREE * rad; }
inline double radian(double deg) { return TO_RADIAN * deg; }
inline float degree(float rad) { return (float)TO_DEGREE * rad; }
inline float radian(float deg) { return (float)TO_RADIAN * deg; }
inline double radian(int deg) { return TO_RADIAN * deg; }

template<int precision>
double reducePrecision(double x)
{
#ifndef _MSC_VER
    constexpr double r = std::pow(10.0, precision);
#else
    const double r = std::pow(10.0, precision);
#endif
    return std::nearbyint(r * x) / r;
}

}

#endif
