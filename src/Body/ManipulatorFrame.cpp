#include "ManipulatorFrame.h"

using namespace std;
using namespace cnoid;


ManipulatorFrame::ManipulatorFrame()
{
    T.setIdentity();
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
