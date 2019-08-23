#ifndef CNOID_BODY_MANIPULATOR_FRAME_H
#define CNOID_BODY_MANIPULATOR_FRAME_H

#include <cnoid/Referenced>
#include <cnoid/EigenTypes>
#include <vector>
#include "exportdecl.h"

namespace cnoid {

class CNOID_EXPORT ManipulatorFrame
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    ManipulatorFrame();
    
    Position T;
    std::string name;
};


class CNOID_EXPORT ManipulatorFrameSet : public Referenced
{
public:
    ManipulatorFrameSet();
    
    int numBaseFrames() const { return baseFrames_.size(); }
    int numToolFrames() const { return toolFrames_.size(); }

    ManipulatorFrame& baseFrame(int index);
    ManipulatorFrame& toolFrame(int index);

    int currentBaseFrameIndex() const { return currentBaseFrameIndex_; }
    ManipulatorFrame& currentBaseFrame(){ return baseFrames_[currentBaseFrameIndex_]; }
    
    int currentToolFrameIndex() const { return currentToolFrameIndex_; }
    ManipulatorFrame& currentToolFrame(){ return toolFrames_[currentToolFrameIndex_]; }
    
private:
    std::vector<ManipulatorFrame> baseFrames_;
    std::vector<ManipulatorFrame> toolFrames_;
    int currentBaseFrameIndex_;
    int currentToolFrameIndex_;
};

typedef ref_ptr<ManipulatorFrameSet> ManipulatorFrameSetPtr;

}

#endif
