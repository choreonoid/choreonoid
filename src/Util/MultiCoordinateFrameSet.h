#ifndef CNOID_UTIL_MULTI_COORDINATE_FRAME_SET_H
#define CNOID_UTIL_MULTI_COORDINATE_FRAME_SET_H

#include "CoordinateFrameSet.h"
#include <cnoid/CloneableReferenced>
#include <vector>
#include <initializer_list>
#include "exportdecl.h"

namespace cnoid {

class CNOID_EXPORT MultiCoordinateFrameSet : public CloneableReferenced
{
public:
    MultiCoordinateFrameSet();
    MultiCoordinateFrameSet(int numFrameSets); // initialized with empty frame sets
    MultiCoordinateFrameSet(std::initializer_list<CoordinateFrameSet*> frameSets);
    MultiCoordinateFrameSet(const MultiCoordinateFrameSet& org); // Do deep copy

    MultiCoordinateFrameSet& operator=(const MultiCoordinateFrameSet& rhs);

    int numFrameSets() const { return frameSets_.size(); }
    void setNumFrameSets(int n) { frameSets_.resize(n); }

    void setFrameSet(int index, CoordinateFrameSet* frameSet);

    CoordinateFrameSet* frameSet(int index){ return frameSets_[index]; }

protected:
    MultiCoordinateFrameSet(const MultiCoordinateFrameSet& org, CloneMap* cloneMap);
    virtual Referenced* doClone(CloneMap* cloneMap) const override;

private:
    std::vector<CoordinateFrameSetPtr> frameSets_;
};

typedef ref_ptr<MultiCoordinateFrameSet> MultiCoordinateFrameSetPtr;

}

#endif
