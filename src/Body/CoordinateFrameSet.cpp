#include "CoordinateFrameSet.h"
#include "CoordinateFrameContainer.h"
#include <cnoid/CloneMap>

using namespace std;
using namespace cnoid;


CoordinateFrame::CoordinateFrame()
{
    T_.setIdentity();
    id_ = -1; // make invalid
}


CoordinateFrame::CoordinateFrame(const Id& id, const Position& T)
    : T_(T),
      id_(id)
{

}


CoordinateFrame::CoordinateFrame(const CoordinateFrame& org)
    : T_(org.T_),
      id_(org.id_)
{

}


Referenced* CoordinateFrame::doClone(CloneMap*) const
{
    return new CoordinateFrame(*this);
}


std::string CoordinateFrame::idLabel() const
{
    int type = stdx::get_variant_index(id_);
    if(type == IntId){
        return std::to_string(stdx::get<int>(id_));
    } else {
        return stdx::get<string>(id_);
    }
}


void CoordinateFrameSet::setCoordinateFrameId(CoordinateFrame* frame, const CoordinateFrame::Id& id)
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
