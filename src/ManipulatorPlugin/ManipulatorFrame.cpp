#include "ManipulatorFrame.h"

using namespace std;
using namespace cnoid;


ManipulatorFrame::ManipulatorFrame()
{
    T_.setIdentity();
}


ManipulatorFrame::ManipulatorFrame(const std::string& name, const Position& T)
    : T_(T),
      name_(name)
{

}
    

ManipulatorFrameSet::ManipulatorFrameSet()
{
    baseFrames_.push_back(ManipulatorFrame("Base", Position::Identity()));
    toolFrames_.push_back(ManipulatorFrame("Mechanical", Position::Identity()));
                          
    currentBaseFrameIndex_ = 0;
    currentToolFrameIndex_ = 0;
}


ManipulatorFrameSet::ManipulatorFrameSet(const ManipulatorFrameSet& org)
    : baseFrames_(org.baseFrames_),
      toolFrames_(org.toolFrames_)
{
    currentBaseFrameIndex_ = org.currentBaseFrameIndex_;
    currentToolFrameIndex_ = org.currentToolFrameIndex_;
}


ManipulatorFrame& ManipulatorFrameSet::baseFrame(int index)
{
    if(index >= baseFrames_.size()){
        baseFrames_.resize(index + 1);
    }
    return baseFrames_[index];
}


ManipulatorFrame& ManipulatorFrameSet::toolFrame(int index)
{
    if(index >= toolFrames_.size()){
        toolFrames_.resize(index + 1);
    }
    return toolFrames_[index];
}
