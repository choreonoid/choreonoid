#include "CoordinateFrameSet.h"
#include "CoordinateFrameList.h"
#include <cnoid/CloneMap>

using namespace std;
using namespace cnoid;


CoordinateFrameSet::CoordinateFrameSet()
{
    identityFrame = new CoordinateFrame;
}


void CoordinateFrameSet::setName(const std::string& name)
{
    name_ = name;
}


void CoordinateFrameSet::setCoordinateFrameId(CoordinateFrame* frame, const GeneralId& id)
{
    frame->id_ = id;
}


void CoordinateFrameSet::setCoordinateFrameOwner(CoordinateFrame* frame, CoordinateFrameSet* owner)
{
    frame->ownerFrameSet_ = owner;
}
