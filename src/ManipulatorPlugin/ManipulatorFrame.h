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
    ManipulatorFrame(const std::string& name, const Position& T);

    void setName(const std::string& name){ name_ = name; }
    const std::string& name() const { return name_; }

    const Position& T() const { return T_; }
    Position& T() { return T_; }

private:
    Position T_;
    std::string name_;
};


class CNOID_EXPORT ManipulatorFrameSet : public Referenced
{
public:
    ManipulatorFrameSet();
    ManipulatorFrameSet(const ManipulatorFrameSet& org);
    
    int numBaseFrames() const { return baseFrames_.size(); }
    int numToolFrames() const { return toolFrames_.size(); }

    ManipulatorFrame& baseFrame(int index);
    ManipulatorFrame& toolFrame(int index);

    int currentBaseFrameIndex() const { return currentBaseFrameIndex_; }
    ManipulatorFrame& currentBaseFrame(){ return baseFrame(currentBaseFrameIndex_); }
    
    int currentToolFrameIndex() const { return currentToolFrameIndex_; }
    ManipulatorFrame& currentToolFrame(){ return toolFrame(currentToolFrameIndex_); }
    
private:
    std::vector<ManipulatorFrame> baseFrames_;
    std::vector<ManipulatorFrame> toolFrames_;
    int currentBaseFrameIndex_;
    int currentToolFrameIndex_;
};

typedef ref_ptr<ManipulatorFrameSet> ManipulatorFrameSetPtr;

}

#endif
