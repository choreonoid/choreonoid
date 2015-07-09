/**
   @author Shin'ichiro Nakaoka
*/

#include "InteractiveCameraTransform.h"

using namespace cnoid;

InteractiveCameraTransform::InteractiveCameraTransform()
{
    
}


InteractiveCameraTransform::InteractiveCameraTransform(const InteractiveCameraTransform& org)
{

}


InteractiveCameraTransform::InteractiveCameraTransform(const InteractiveCameraTransform& org, SgCloneMap& cloneMap)
    : SgPosTransform(org, cloneMap)
{

}


SgObject* InteractiveCameraTransform::clone(SgCloneMap& cloneMap) const
{
    return new InteractiveCameraTransform(*this, cloneMap);
}


void InteractiveCameraTransform::onPositionUpdatedInteractively()
{

}
