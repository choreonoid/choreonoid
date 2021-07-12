/**
   @author Shin'ichiro Nakaoka
*/

#ifndef CNOID_BASE_INTERACTIVE_CAMERA_TRANSFORM_H
#define CNOID_BASE_INTERACTIVE_CAMERA_TRANSFORM_H

#include <cnoid/SceneGraph>
#include "exportdecl.h"

namespace cnoid {

class CNOID_EXPORT InteractiveCameraTransform : public SgPosTransform
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    InteractiveCameraTransform();
    InteractiveCameraTransform(const InteractiveCameraTransform& org, CloneMap* cloneMap = nullptr);

    virtual Referenced* doClone(CloneMap* cloneMap) const override;

    void setInteractiveViewpointChangeLocked(bool on) { isInteractiveViewpointChangeLocked_ = on; }
    bool isInteractiveViewpointChangeLocked() { return isInteractiveViewpointChangeLocked_; }

protected:
    InteractiveCameraTransform(int polymorhicId);

private:
    bool isInteractiveViewpointChangeLocked_;
};

typedef ref_ptr<InteractiveCameraTransform> InteractiveCameraTransformPtr;

}

#endif
