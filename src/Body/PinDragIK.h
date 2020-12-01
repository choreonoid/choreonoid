/*!
  @file
  @author Shin'ichiro Nakaoka
*/

#ifndef CNOID_BODY_PIN_DRAG_IK_H
#define CNOID_BODY_PIN_DRAG_IK_H

#include "InverseKinematics.h"
#include "exportdecl.h"

namespace cnoid {

class Body;
class Link;
class PinDragIKImpl;

class CNOID_EXPORT PinDragIK : public InverseKinematics
{
public:
    PinDragIK(Body* body);
    ~PinDragIK();

    Body* body() const;
  
    void setBaseLink(Link* baseLink);
    void setFreeRootWeight(double translation, double rotation);
    void setTargetLink(Link* targetLink, bool isAttitudeEnabled = false);
    void setJointWeight(int jointId, double weight);

    enum AxisSet { NO_AXES = 0, TRANSLATION_3D = 0x1, ROTATION_3D = 0x2, TRANSFORM_6D = 0x3 };
    void setPin(Link* link, AxisSet axes = TRANSLATION_3D, double weight = 1.0);
    AxisSet pinAxes(Link* link);
    void clearPins();
    int numPinnedLinks();
            
    void setIKErrorThresh(double e);
    bool hasAnalyticalIK();
    AxisSet targetAxes() const;
    void setSRInverseParameters(double k0, double w0);
    void enableJointRangeConstraints(bool on);

    /**
       this must be called before the initial calcInverseKinematics() call
       after settings have been changed.
    */
    bool initialize();

    virtual bool calcInverseKinematics(const Isometry3& T) override;

private:
    PinDragIKImpl* impl;
};

}

#endif
