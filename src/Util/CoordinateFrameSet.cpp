#include "CoordinateFrameSet.h"
#include "CoordinateFrameContainer.h"
#include <cnoid/CloneMap>

using namespace std;
using namespace cnoid;

void CoordinateFrameSet::setCoordinateFrameId(CoordinateFrame* frame, const CoordinateFrameId& id)
{
    frame->id_ = id;
}


void CoordinateFrameSet::setCoordinateFrameOwner(CoordinateFrame* frame, CoordinateFrameSet* owner)
{
    frame->ownerFrameSet_ = owner;
}


CoordinateFrameSetPair::CoordinateFrameSetPair()
{
    for(int i=0; i < 2; ++i){
        frameSets[i] = new CoordinateFrameContainer;
    }
}


CoordinateFrameSetPair::CoordinateFrameSetPair
(CoordinateFrameSet* baseFrames, CoordinateFrameSet* offsetFrames)
{
    frameSets[0] = baseFrames;
    frameSets[1] = offsetFrames;
}


CoordinateFrameSetPair::CoordinateFrameSetPair(const CoordinateFrameSetPair& org)
{
    for(int i=0; i < 2; ++i){
        frameSets[i] = org.frameSets[i]->clone();
    }
}


CoordinateFrameSetPair::CoordinateFrameSetPair(const CoordinateFrameSetPair& org, CloneMap* cloneMap)
{
    for(int i=0; i < 2; ++i){
        frameSets[i] = cloneMap->getClone(org.frameSets[i].get());
    }
}
    
    
Referenced* CoordinateFrameSetPair::doClone(CloneMap* cloneMap) const
{
    if(cloneMap){
        return new CoordinateFrameSetPair(*this, cloneMap);
    } else {
        return new CoordinateFrameSetPair(*this);
    }
}
