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

    virtual bool calcInverseKinematics(const Position& T) = 0;

    virtual bool calcRemainingPartForwardKinematicsForInverseKinematics();

    //! deprecated
    bool calcInverseKinematics(const Vector3& p, const Matrix3& R) {
        Position T;
        T.linear() = R;
        T.translation() = p;
        return calcInverseKinematics(T);
    }
};

}

#endif
