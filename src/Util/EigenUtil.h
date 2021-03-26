/**
   @author Shin'ichiro Nakaoka
*/

#ifndef CNOID_UTIL_EIGEN_UTIL_H
#define CNOID_UTIL_EIGEN_UTIL_H

#include "EigenTypes.h"
#include "MathUtil.h"
#include <memory>
#include "exportdecl.h"

namespace cnoid {

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
CNOID_EXPORT bool toVector6(const std::string& s, Vector6& out_v);

CNOID_EXPORT void normalizeRotation(Matrix3& R);
CNOID_EXPORT void normalizeRotation(Affine3& T);
CNOID_EXPORT void normalizeRotation(Isometry3& T);

inline Isometry3 convertToIsometryWithOrthonormalization(const Affine3& A)
{
    Isometry3 M;
    auto S = A.linear();
    auto R = M.linear();
    R.col(2) = S.col(2).normalized(); // Z
    R.col(1) = R.col(2).cross(S.col(0).normalized()).normalized(); // Y
    R.col(0) = R.col(1).cross(R.col(2)); // X
    M.translation() = A.translation();
    return M;
}

template<class T, typename... Args>
std::shared_ptr<T> make_shared_aligned(Args&&... args)
{
  return std::allocate_shared<T>(Eigen::aligned_allocator<T>(), std::forward<Args>(args)...);
}

}

#endif
