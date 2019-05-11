/** 
    \author Shin'ichiro Nakaoka
*/

#ifndef CNOID_BODY_INVERSE_KINEMATICS_H
#define CNOID_BODY_INVERSE_KINEMATICS_H

#include <cnoid/EigenTypes>
#include <cnoid/ValueTree>

namespace cnoid {

class InverseKinematics
{
public:
    virtual ~InverseKinematics() {  }

    //virtual bool setIkProperty(int ikId, const Mapping* property) = 0;
    //virtual const Mapping* ikProperty() = 0;

    virtual bool calcInverseKinematics(const Position& T) = 0;

    //! deprecated
    enum AxisSet { NO_AXES = 0, TRANSLATION_3D = 0x1, ROTATION_3D = 0x2, TRANSFORM_6D = 0x3 };

    //! deprecated
    virtual AxisSet axisType() const { return TRANSFORM_6D; }

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
