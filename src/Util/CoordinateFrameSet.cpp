#include "CoordinateFrameSet.h"

using namespace std;
using namespace cnoid;


CoordinateFrameSet::CoordinateFrameSet()
{
    identityFrame = new CoordinateFrame;
}


CoordinateFrameSet::CoordinateFrameSet(const CoordinateFrameSet& org)
    : name_(org.name_)
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
