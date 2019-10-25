#ifndef CNOID_UTIL_COORDINATE_FRAME_GROUP_H
#define CNOID_UTIL_COORDINATE_FRAME_GROUP_H

#include "CoordinateFrameSet.h"
#include "exportdecl.h"

namespace cnoid {

class CNOID_EXPORT CoordinateFrameSetGroup : public CoordinateFrameSet
{
public:
    CoordinateFrameSetGroup();
    CoordinateFrameSetGroup(const CoordinateFrameSetGroup& org);

    void clear();
    int numFrameSets() const;
    CoordinateFrameSet* frameSet(int index) const;
    void addFrameSet(CoordinateFrameSet* frameSet);

    virtual int getNumFrames() const override;
    virtual CoordinateFrame* getFrameAt(int index) const override;
    virtual CoordinateFrame* findFrame(const GeneralId& id) const override;
    virtual std::vector<CoordinateFramePtr> getFindableFrameLists() const override;
    virtual bool contains(const CoordinateFrameSet* frameSet) const override;

protected:
    CoordinateFrameSetGroup(const CoordinateFrameSetGroup& org, CloneMap* cloneMap);
    virtual Referenced* doClone(CloneMap* cloneMap) const override;
    
private:
    class Impl;
    Impl* impl;
};

typedef ref_ptr<CoordinateFrameSetGroup> CoordinateFrameSetGroupPtr;

}

#endif
