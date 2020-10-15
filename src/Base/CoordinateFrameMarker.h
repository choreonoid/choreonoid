#ifndef CNOID_BASE_COORDINATE_FRAME_MARKER_H
#define CNOID_BASE_COORDINATE_FRAME_MARKER_H

#include "PositionDragger.h"
#include "exportdecl.h"

namespace cnoid {

class CoordinateFrame;

class CNOID_EXPORT CoordinateFrameMarker : public PositionDragger
{
public:
    CoordinateFrameMarker();
    CoordinateFrameMarker(CoordinateFrame* frame);
    ~CoordinateFrameMarker();
    CoordinateFrame* frame();
    virtual void onFrameUpdated(int flags);

private:
    class Impl;
    Impl* impl;
};

typedef ref_ptr<CoordinateFrameMarker> CoordinateFrameMarkerPtr;

}

#endif
