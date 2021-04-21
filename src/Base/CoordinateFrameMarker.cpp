#include "CoordinateFrameMarker.h"
#include <cnoid/CoordinateFrame>

using namespace std;
using namespace cnoid;

namespace cnoid {

class CoordinateFrameMarker::Impl
{
public:
    CoordinateFrameMarker* self;
    CoordinateFramePtr frame;
    ScopedConnection frameConnection;

    void updateMarkerPositionWithCoordinateFramePosition();
};

}


CoordinateFrameMarker::CoordinateFrameMarker()
    : PositionDragger(PositionDragger::AllAxes, PositionDragger::PositiveOnlyHandle)
{
    impl = new Impl;
    impl->self = this;
    
    setDragEnabled(true);
    setOverlayMode(true);
    setPixelSize(96, 3);
    setDisplayMode(PositionDragger::DisplayInEditMode);
}


CoordinateFrameMarker::CoordinateFrameMarker(CoordinateFrame* frame)
    : CoordinateFrameMarker()
{
    impl->frame = frame;
    setPosition(frame->position());

    impl->frameConnection =
        frame->sigUpdated().connect(
            [&](int flags){ onFrameUpdated(flags); });
    
    sigPositionDragged().connect(
        [&](){ impl->updateMarkerPositionWithCoordinateFramePosition(); });
}


CoordinateFrameMarker::~CoordinateFrameMarker()
{
    delete impl;
}


CoordinateFrame* CoordinateFrameMarker::frame()
{
    return impl->frame;
}


void CoordinateFrameMarker::onFrameUpdated(int flags)
{
    if(flags & CoordinateFrame::PositionUpdate){
        setPosition(impl->frame->position());
        notifyUpdate();
    }
}


void CoordinateFrameMarker::Impl::updateMarkerPositionWithCoordinateFramePosition()
{
    frame->setPosition(self->draggingPosition());
    frame->notifyUpdate(CoordinateFrame::PositionUpdate);
}
