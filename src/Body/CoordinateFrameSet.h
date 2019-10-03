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
    CoordinateFrame(const CoordinateFrame& org);

    void setName(const std::string& name){ name_ = name; }
    const std::string& name() const { return name_; }

    const Position& T() const { return T_; }
    Position& T() { return T_; }

protected:
    virtual Referenced* doClone(CloneMap* cloneMap) const override;

private:
    Position T_;
    std::string name_;
    bool isAbsolute_;
};

typedef ref_ptr<CoordinateFrame> CoordinateFramePtr;


class CNOID_EXPORT CoordinateFrameSet : public CloneMappableReferenced
{
public:
    CoordinateFrameSet();
    CoordinateFrameSet(const CoordinateFrameSet& org);

    int numWorldFrames() const { return worldFrames_.size();  }
    int numObjectFrameOffsets() const { return objectFrameOffsets_.size(); }

    CoordinateFrame* worldFrame(int index);
    int currentWorldFrameIndex() const { return currentWorldFrameIndex_; }
    CoordinateFrame* currentWorldFrame(){ return worldFrame(currentWorldFrameIndex_); }

    // The following functions are the same as the worldFrame functions
    CoordinateFrame* baseFrame(int index){ return worldFrame(index); }
    int currentBaseFrameIndex() const { return currentWorldFrameIndex_; }
    CoordinateFrame* currentBaseFrame(){ return worldFrame(currentWorldFrameIndex_); }

    CoordinateFrame* objectFrameOffset(int index);
    int currentObjectFrameOffsetIndex() const { return currentObjectFrameOffsetIndex_; }
    CoordinateFrame* currentObjectFrameOffset(){ return objectFrameOffset(currentObjectFrameOffsetIndex_); } 

protected:
    CoordinateFrameSet(const CoordinateFrameSet& org, CloneMap* cloneMap);
    virtual Referenced* doClone(CloneMap* cloneMap) const override;
    
private:
    std::vector<CoordinateFramePtr> worldFrames_;
    std::vector<CoordinateFramePtr> objectFrameOffsets_;
    int currentWorldFrameIndex_;
    int currentObjectFrameOffsetIndex_;
};

typedef ref_ptr<CoordinateFrameSet> CoordinateFrameSetPtr;

}

#endif
