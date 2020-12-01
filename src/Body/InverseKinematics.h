/** 
    \author Shin'ichiro Nakaoka
*/

#ifndef CNOID_BODY_INVERSE_KINEMATICS_H
#define CNOID_BODY_INVERSE_KINEMATICS_H

#include <cnoid/EigenTypes>
#include "exportdecl.h"

namespace cnoid {

class CNOID_EXPORT InverseKinematics
{
public:
    virtual ~InverseKinematics();

    virtual bool calcInverseKinematics(const Isometry3& T) = 0;

    virtual bool calcRemainingPartForwardKinematicsForInverseKinematics();

    //! deprecated
    bool calcInverseKinematics(const Vector3& p, const Matrix3& R) {
        Isometry3 T;
        T.linear() = R;
        T.translation() = p;
        return calcInverseKinematics(T);
    }
};

}

#endif
