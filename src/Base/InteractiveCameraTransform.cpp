/**
   @author Shin'ichiro Nakaoka
*/

#include "InteractiveCameraTransform.h"

using namespace cnoid;


InteractiveCameraTransform::InteractiveCameraTransform(int polymorhicId)
    : SgPosTransform(polymorhicId)
{
    
}


InteractiveCameraTransform::InteractiveCameraTransform()
    : InteractiveCameraTransform(findPolymorphicId<InteractiveCameraTransform>())
{
    
}


InteractiveCameraTransform::InteractiveCameraTransform(const InteractiveCameraTransform& org, SgCloneMap* cloneMap)
    : SgPosTransform(org, cloneMap)
{

}


SgObject* InteractiveCameraTransform::doClone(SgCloneMap* cloneMap) const
{
    return new InteractiveCameraTransform(*this, cloneMap);
}


namespace {

struct NodeTypeRegistration {
    NodeTypeRegistration() {
        SgNode::registerType<InteractiveCameraTransform, SgPosTransform>();
    }
} registration;

}
