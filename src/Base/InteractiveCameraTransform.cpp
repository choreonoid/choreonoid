/**
   @author Shin'ichiro Nakaoka
*/

#include "InteractiveCameraTransform.h"
#include <cnoid/SceneNodeClassRegistry>

using namespace cnoid;


InteractiveCameraTransform::InteractiveCameraTransform(int classId)
    : SgPosTransform(classId)
{
    
}


InteractiveCameraTransform::InteractiveCameraTransform()
    : InteractiveCameraTransform(findClassId<InteractiveCameraTransform>())
{
    
}


InteractiveCameraTransform::InteractiveCameraTransform(const InteractiveCameraTransform& org, CloneMap* cloneMap)
    : SgPosTransform(org, cloneMap)
{

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
