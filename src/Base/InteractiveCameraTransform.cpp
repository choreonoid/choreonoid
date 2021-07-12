/**
   @author Shin'ichiro Nakaoka
*/

#include "InteractiveCameraTransform.h"
#include <cnoid/SceneNodeClassRegistry>

using namespace cnoid;


InteractiveCameraTransform::InteractiveCameraTransform(int classId)
    : SgPosTransform(classId)
{
    isInteractiveViewpointChangeLocked_ = false;
}


InteractiveCameraTransform::InteractiveCameraTransform()
    : InteractiveCameraTransform(findClassId<InteractiveCameraTransform>())
{

}


InteractiveCameraTransform::InteractiveCameraTransform(const InteractiveCameraTransform& org, CloneMap* cloneMap)
    : SgPosTransform(org, cloneMap)
{
    isInteractiveViewpointChangeLocked_ = org.isInteractiveViewpointChangeLocked_;
}


Referenced* InteractiveCameraTransform::doClone(CloneMap* cloneMap) const
{
    return new InteractiveCameraTransform(*this, cloneMap);
}


namespace {

struct NodeClassRegistration {
    NodeClassRegistration() {
        SceneNodeClassRegistry::instance().registerClass<InteractiveCameraTransform, SgPosTransform>();
    }
} registration;

}
