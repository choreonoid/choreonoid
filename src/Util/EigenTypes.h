/**
   @author Shin'ichiro Nakaoka
*/

#ifndef CNOID_UTIL_EIGEN_TYPES_H
#define CNOID_UTIL_EIGEN_TYPES_H

#include <Eigen/Core>
#include <Eigen/Geometry>

namespace cnoid {

using Eigen::Vector2i;
using Eigen::Matrix2f;
using Eigen::Vector2f;
using Eigen::Matrix2d;
using Eigen::Vector2d;
using Eigen::Array2i;
using Eigen::Array2f;
using Eigen::Array2d;

using Eigen::Vector3i;
using Eigen::Matrix3f;
using Eigen::Vector3f;
using Eigen::Matrix3d;
using Eigen::Vector3d;
using Eigen::Array3i;
using Eigen::Array3f;
using Eigen::Array3d;
    
using Eigen::Matrix4f;
using Eigen::Vector4f;
using Eigen::Matrix4d;
using Eigen::Vector4d;
using Eigen::Array4i;
using Eigen::Array4f;
using Eigen::Array4d;
    
using Eigen::MatrixXf;
using Eigen::VectorXf;
using Eigen::AngleAxisf;
using Eigen::Quaternionf;

using Eigen::MatrixXd;
using Eigen::VectorXd;
using Eigen::AngleAxisd;
using Eigen::Quaterniond;

using Eigen::Affine3f;
using Eigen::Affine3d;
using Eigen::Translation3f;
using Eigen::Translation3d;

typedef Eigen::Matrix2d Matrix2;
typedef Eigen::Vector2d Vector2;
typedef Eigen::Matrix3d Matrix3;
typedef Eigen::Vector3d Vector3;
//typedef Eigen::AlignedVector3<double> Vector3;
typedef Eigen::Matrix4d Matrix4;
typedef Eigen::Vector4d Vector4;
typedef Eigen::VectorXd VectorX;
typedef Eigen::Matrix<double, 6, 1> Vector6;
typedef Eigen::Affine3d Affine3;
typedef Eigen::Translation3d Translation3;
typedef Eigen::AngleAxisd AngleAxis;

//! \deprecated
typedef Eigen::Quaterniond Quat;

typedef Eigen::Quaterniond Quaternion;
    
typedef Eigen::Transform<double, 3, Eigen::AffineCompact> Position;

// The followings should be removed later
using Eigen::Isometry3f;
using Eigen::Isometry3d;
typedef Eigen::Isometry3d Isometry3;
    
class SE3 {
    Vector3 p;
    Quat q;
public:
    SE3() { }
    SE3(const Vector3& translation, const Quat& rotation)
        : p(translation), q(rotation) { }
    SE3(const Vector3& translation, const Matrix3& rotation)
        : p(translation), q(rotation) { }
    SE3(const Position& T)
        : p(T.translation()), q(T.linear()) { }

    void set(const Vector3& translation, const Quat& rotation) {
        this->p = translation;
        this->q = rotation;
    }
    void set(const Vector3& translation, const Matrix3& R) {
        this->p = translation;
        this->q = R;
    }
    void set(const Position& T){
        p = T.translation();
        q = T.linear();
    }

    Vector3& translation() { return p; }
    const Vector3& translation() const { return p; }
    Quat& rotation() { return q; }
    const Quat& rotation() const { return q; }

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW        
};

}

#endif
