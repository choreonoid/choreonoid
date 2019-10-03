#include "CoordinateFrameSet.h"
#include <cnoid/CloneMap>

using namespace std;
using namespace cnoid;


CoordinateFrame::CoordinateFrame()
{
    T_.setIdentity();
}


CoordinateFrame::CoordinateFrame(const std::string& name, const Position& T)
    : T_(T),
      name_(name)
{

}


CoordinateFrame::CoordinateFrame(const CoordinateFrame& org)
    : T_(org.T_),
      name_(org.name_)
{

}


Referenced* CoordinateFrame::doClone(CloneMap*) const
{
    return new CoordinateFrame(*this);
}


CoordinateFrameSet::CoordinateFrameSet()
{
    worldFrames_.push_back(new CoordinateFrame("World", Position::Identity()));
    objectFrameOffsets_.push_back(new CoordinateFrame("Mechanical", Position::Identity()));
                          
    currentWorldFrameIndex_ = 0;
    currentObjectFrameOffsetIndex_ = 0;
}


CoordinateFrameSet::CoordinateFrameSet(const CoordinateFrameSet& org)
{
    for(auto& frame : org.worldFrames_){
        worldFrames_.push_back(new CoordinateFrame(*frame));
    }
    for(auto& frame : org.objectFrameOffsets_){
        objectFrameOffsets_.push_back(new CoordinateFrame(*frame));
    }

    currentWorldFrameIndex_ = org.currentWorldFrameIndex_;
    currentObjectFrameOffsetIndex_ = org.currentObjectFrameOffsetIndex_;
}
    

CoordinateFrameSet::CoordinateFrameSet(const CoordinateFrameSet& org, CloneMap* cloneMap)
{
    for(auto& frame : org.worldFrames_){
        worldFrames_.push_back(cloneMap->getClone(frame.get()));
    }
    for(auto& frame : org.objectFrameOffsets_){
        objectFrameOffsets_.push_back(cloneMap->getClone(frame.get()));
    }
        
    currentWorldFrameIndex_ = org.currentWorldFrameIndex_;
    currentObjectFrameOffsetIndex_ = org.currentObjectFrameOffsetIndex_;
}


Referenced* CoordinateFrameSet::doClone(CloneMap* cloneMap) const
{
    return new CoordinateFrameSet(*this, cloneMap);
}


CoordinateFrame* CoordinateFrameSet::worldFrame(int index)
{
    if(index >= worldFrames_.size()){
        worldFrames_.resize(index + 1);
    }
    return worldFrames_[index];
}


CoordinateFrame* CoordinateFrameSet::objectFrameOffset(int index)
{
    if(index >= objectFrameOffsets_.size()){
        objectFrameOffsets_.resize(index + 1);
    }
    return objectFrameOffsets_[index];
}
