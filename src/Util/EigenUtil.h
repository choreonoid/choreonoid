/**
   @author Shin'ichiro Nakaoka
*/

#ifndef CNOID_UTIL_EIGEN_UTIL_H
#define CNOID_UTIL_EIGEN_UTIL_H

#include "EigenTypes.h"
#include <memory>
#include "exportdecl.h"

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
inline Vector3 degree(const Vector3& v){ return Vector3(degree(v.x()), degree(v.y()), degree(v.z())); }
inline Vector3 radian(const Vector3& v){ return Vector3(radian(v.x()), radian(v.y()), radian(v.z())); }

CNOID_EXPORT Vector3 rpyFromRot(const Matrix3& R);
CNOID_EXPORT Vector3 rpyFromRot(const Matrix3& R, const Vector3& prevRPY);

CNOID_EXPORT Matrix3 rotFromRpy(double r, double p, double y);

inline Matrix3 rotFromRpy(const Vector3& rpy) {
    return rotFromRpy(rpy[0], rpy[1], rpy[2]);
}

CNOID_EXPORT Vector3 omegaFromRot(const Matrix3& R);

inline Matrix3 hat(const Vector3& x) {
    Matrix3 M;
    M <<  0.0, -x(2),   x(1),
        x(2),   0.0,  -x(0),
        -x(1),  x(0),   0.0;
    return M;
}

CNOID_EXPORT std::string str(const Vector3& v);
CNOID_EXPORT std::string str(const Vector3f& v);
CNOID_EXPORT std::string str(const Vector2& v);
CNOID_EXPORT std::string str(const AngleAxis& a);
CNOID_EXPORT bool toVector3(const std::string& s, Vector3& out_v);
CNOID_EXPORT bool toVector3(const std::string& s, Vector3f& out_v);


CNOID_EXPORT void normalizeRotation(Matrix3& R);
CNOID_EXPORT void normalizeRotation(Position& T);
CNOID_EXPORT void normalizeRotation(Affine3& T);

template<class T, typename... Args>
std::shared_ptr<T> make_shared_aligned(Args&&... args)
{
  return std::allocate_shared<T>(Eigen::aligned_allocator<T>(), std::forward<Args>(args)...);
}

}

#endif
