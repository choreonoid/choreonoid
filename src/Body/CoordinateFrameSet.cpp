#include "CoordinateFrameSet.h"

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
    baseFrames_.push_back(CoordinateFrame("Base", Position::Identity()));
    toolFrames_.push_back(CoordinateFrame("Mechanical", Position::Identity()));
                          
    currentBaseFrameIndex_ = 0;
    currentToolFrameIndex_ = 0;
}


CoordinateFrameSet::CoordinateFrameSet(const CoordinateFrameSet& org, CloneMap* cloneMap)
{
    if(cloneMap){
        for(auto& frame : org.worldFrames_){
            worldFrames_.push_back(cloneMap->getClone(frame.get()));
        }
        for(auto& frame : org.localFrameOffsets_){
            localFrameOffsets_.push_back(cloneMap->getClone(frame.get()));
        }
    } else {
        for(auto& frame : org.worldFrames_){
            worldFrames_.push_back(frame->clone());
        }
        for(auto& frame : org.localFrameOffsets_){
            localFrameOffsets_.push_back(frame->clone());
        }
    }
        
    currentBaseFrameIndex_ = org.currentBaseFrameIndex_;
    currentToolFrameIndex_ = org.currentToolFrameIndex_;
}


Referenced* CoordinateFrameSet::doClone(CloneMap* cloneMap) const
{
    return new CoordinateFrameSet(*this, cloneMap);
}


CoordinateFrame& CoordinateFrameSet::worldFrame(int index)
{
    if(index >= worldFrames_.size()){
        worldFrames_.resize(index + 1);
    }
    return worldFrames_[index];
}


CoordinateFrame& CoordinateFrameSet::localFrameOffset(int index)
{
    if(index >= localFrameOffsets_.size()){
        localFrameOffsets_.resize(index + 1);
    }
    return localFrameOffsets_[index];
}
