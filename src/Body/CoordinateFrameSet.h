#ifndef CNOID_BODY_COORDINATE_FRAME_SET_H
#define CNOID_BODY_COORDINATE_FRAME_SET_H

#include <cnoid/CloneMappableReferenced>
#include <cnoid/EigenTypes>
#include <vector>
#include "exportdecl.h"

namespace cnoid {

class CNOID_EXPORT CoordinateFrame : public CloneMappableReferenced
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    CoordinateFrame();
    CoordinateFrame(const std::string& name, const Position& T);

    CoordinateFrame* clone() const { return static_cast<CoordinateFrame*>(doClone(nullptr)); }

    void setName(const std::string& name){ name_ = name; }
    const std::string& name() const { return name_; }

    const Position& T() const { return T_; }
    Position& T() { return T_; }

protected:
    CoordinateFrame(const CoordinateFrame& org, CloneMap* cloneMap);
    virtual Referenced* doClone(CloneMap* cloneMap) const override;

private:
    Position T_;
    std::string name_;
};

typedef ref_ptr<CoordinateFrame> CoordinateFramePtr;


class CNOID_EXPORT CoordinateFrameSet : public CloneMappableReferenced
{
public:
    CoordinateFrameSet();

    CoordinateFrameSet* clone() const { return static_cast<CoordinateFrameSet*>(doClone(nullptr)); }
    CoordinateFrameSet* clone(CloneMap* cloneMap) const { return static_cast<CoordinateFrameSet*>(doClone(cloneMap)); }
    
    int numBaseFrames() const { return baseFrames_.size(); }
    int numToolFrames() const { return toolFrames_.size(); }

    CoordinateFrame* worldFrame(int index);
    CoordinateFrame* localFrameOffset(int index);

    int currentWorldFrameIndex() const { return currentWorldFrameIndex_; }
    CoordinateFrame& currentWorldFrame(){ return worldFrame(currentWorldFrameIndex_); }
    
    int currentLocalFrameOffsetIndex() const { return currentLocalFrameOffsetIndex_; }
    CoordinateFrame& currentLocalFrameOffset(){ return toolFrame(currentToolFrameIndex_); }

protected:
    CoordinateFrameSet(const CoordinateFrameSet& org, CloneMap* cloneMap);
    virtual Referenced* doClone(CloneMap* cloneMap) const override;
    
private:
    std::vector<CoordinateFramePtr> worldFrames_;
    std::vector<CoordinateFramePtr> localFrameOffsets_;
    int currentWorldFrameIndex_;
    int currentLocalFrameOffsetIndex_;
};

typedef ref_ptr<CoordinateFrameSet> CoordinateFrameSetPtr;

}

#endif
