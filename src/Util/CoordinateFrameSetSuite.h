#ifndef CNOID_UTIL_COORDINATE_FRAME_SET_SUIE_H
#define CNOID_UTIL_COORDINATE_FRAME_SET_SUIE_H

#include "CoordinateFrameSet.h"
#include <cnoid/CloneableReferenced>
#include <vector>
#include <initializer_list>
#include "exportdecl.h"

namespace cnoid {

class CNOID_EXPORT CoordinateFrameSetSuite : public CloneableReferenced
{
public:
    CoordinateFrameSetSuite();
    CoordinateFrameSetSuite(int numFrameSets); // initialized with empty frame sets
    CoordinateFrameSetSuite(std::initializer_list<CoordinateFrameSet*> frameSets);
    CoordinateFrameSetSuite(const CoordinateFrameSetSuite& org); // Do deep copy

    CoordinateFrameSetSuite& operator=(const CoordinateFrameSetSuite& rhs);

    int numFrameSets() const { return frameSets_.size(); }
    void setNumFrameSets(int n) { frameSets_.resize(n); }

    void setFrameSet(int index, CoordinateFrameSet* frameSet);

    CoordinateFrameSet* frameSet(int index){ return frameSets_[index]; }

protected:
    CoordinateFrameSetSuite(const CoordinateFrameSetSuite& org, CloneMap* cloneMap);
    virtual Referenced* doClone(CloneMap* cloneMap) const override;

private:
    std::vector<CoordinateFrameSetPtr> frameSets_;
};

typedef ref_ptr<CoordinateFrameSetSuite> CoordinateFrameSetSuitePtr;

}

#endif
