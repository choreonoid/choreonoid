#include "CoordinateFrameSet.h"
#include "CoordinateFrameList.h"
#include <cnoid/CloneMap>

using namespace std;
using namespace cnoid;


CoordinateFrameSet::CoordinateFrameSet()
{
    identityFrame = new CoordinateFrame;
}


void CoordinateFrameSet::setCoordinateFrameId(CoordinateFrame* frame, const GeneralId& id)
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
        frameSets[i] = new CoordinateFrameList;
    }
}


CoordinateFrameSetPair::CoordinateFrameSetPair
(CoordinateFrameSet* baseFrameSet, CoordinateFrameSet* localFrameSet)
{
    frameSets[0] = baseFrameSet;
    frameSets[1] = localFrameSet;
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


CoordinateFrameSetPair& CoordinateFrameSetPair::operator=(const CoordinateFrameSetPair& rhs)
{
    for(int i=0; i < 2; ++i){
        frameSets[i] = rhs.frameSets[i];
    }
    return *this;
}

