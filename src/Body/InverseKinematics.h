/** 
    \author Shin'ichiro Nakaoka
*/

#ifndef CNOID_INVERSE_KINEMATICS_H
#define CNOID_INVERSE_KINEMATICS_H

#include <cnoid/EigenTypes>
#include <memory>
#include "exportdecl.h"

namespace cnoid {

class InverseKinematics
{
public:
    enum AxisSet { NO_AXES = 0, TRANSLATION_3D = 0x1, ROTATION_3D = 0x2, TRANSFORM_6D = 0x3 };
    virtual ~InverseKinematics() {  }
    virtual AxisSet axisType() const { return TRANSFORM_6D; }

    /**
       \todo This should be "bool calcInverseKinematics(const Position& T) = 0
    */
    virtual bool calcInverseKinematics(const Vector3& end_p, const Matrix3& end_R) = 0;
};

typedef std::shared_ptr<InverseKinematics> InverseKinematicsPtr;

}

#endif
