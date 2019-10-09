#ifndef CNOID_BODY_COORDINATE_FRAME_GROUP_H
#define CNOID_BODY_COORDINATE_FRAME_GROUP_H

#include "CoordinateFrameSet.h"
#include <vector>
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
    virtual CoordinateFrame* getFrame(int index) const override;
    virtual CoordinateFrame* findFrame(
        const CoordinateFrame::Id& id, bool returnIdentityFrameIfNotFound = true) const override;

    void getArrangedFrameLists(
        std::vector<CoordinateFramePtr>& out_numberedFrameList,
        std::vector<CoordinateFramePtr>& out_namedFrameList) const;

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
