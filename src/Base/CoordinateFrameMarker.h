#ifndef CNOID_BASE_COORDINATE_FRAME_MARKER_H
#define CNOID_BASE_COORDINATE_FRAME_MARKER_H

#include "PositionDragger.h"
#include <cnoid/CoordinateFrame>
#include "exportdecl.h"

namespace cnoid {

class CoordinateFrame;

class CNOID_EXPORT CoordinateFrameMarker : public PositionDragger
{
    CoordinateFramePtr frame_;
    ScopedConnection frameConnection;
    
public:
    CoordinateFrameMarker(CoordinateFrame* frame);
    CoordinateFrame* frame() { return frame_; }
    virtual void onFrameUpdated(int flags);
    void onMarkerPositionDragged();
};

typedef ref_ptr<CoordinateFrameMarker> CoordinateFrameMarkerPtr;

}

#endif
