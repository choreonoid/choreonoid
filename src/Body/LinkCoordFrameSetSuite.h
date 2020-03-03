#ifndef CNOID_BODY_LINK_COORD_FRAME_SET_SUITE_H
#define CNOID_BODY_LINK_COORD_FRAME_SET_SUITE_H

#include <cnoid/CoordinateFrameSetSuite>
#include "exportdecl.h"

namespace cnoid {

class CNOID_EXPORT LinkCoordFrameSetSuite : public CoordinateFrameSetSuite
{
public:
    LinkCoordFrameSetSuite();
    LinkCoordFrameSetSuite(
        CoordinateFrameSet* worldFrameSet, CoordinateFrameSet* bodyFrameSet, CoordinateFrameSet* linkFrameSet);
    LinkCoordFrameSetSuite(const LinkCoordFrameSetSuite& org);

    LinkCoordFrameSetSuite& operator=(const LinkCoordFrameSetSuite& rhs);

    void resetFrameSets();

    /**
       WorldFrame: Offset from the world origin frame
       BodyFrame: Offset from the base link origin frame
       LinkFrame: Local offset from the target link origin frame
    */
    enum FrameType { WorldFrame = 0, BodyFrame = 1, LinkFrame = 2 };

    CoordinateFrameSet* worldFrameSet() { return frameSet(WorldFrame); }
    CoordinateFrameSet* bodyFrameSet() { return frameSet(BodyFrame); }
    CoordinateFrameSet* linkFrameSet() { return frameSet(LinkFrame); }

    CoordinateFrame* worldFrame(const GeneralId& id) {
        return worldFrameSet()->getFrame(id);
    }
    CoordinateFrame* bodyFrame(const GeneralId& id) {
        return bodyFrameSet()->getFrame(id);
    }
    CoordinateFrame* linkFrame(const GeneralId& id) {
        return linkFrameSet()->getFrame(id);
    }

protected:
    LinkCoordFrameSetSuite(const LinkCoordFrameSetSuite& org, CloneMap* cloneMap);
    virtual Referenced* doClone(CloneMap* cloneMap) const override;
};

typedef ref_ptr<LinkCoordFrameSetSuite> LinkCoordFrameSetSuitePtr;

}

#endif

