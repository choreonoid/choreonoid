#ifndef CNOID_UTIL_MATH_UTIL_H
#define CNOID_UTIL_MATH_UTIL_H

#include <cmath>

namespace cnoid {

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
