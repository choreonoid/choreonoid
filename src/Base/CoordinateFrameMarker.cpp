#include "CoordinateFrameMarker.h"

using namespace std;
using namespace cnoid;


CoordinateFrameMarker::CoordinateFrameMarker(CoordinateFrame* frame)
    : PositionDragger(PositionDragger::AllAxes, PositionDragger::PositiveOnlyHandle),
      frame_(frame)
{
    setDragEnabled(true);
    setOverlayMode(true);
    setConstantPixelSizeMode(true, 92.0);
    setDisplayMode(PositionDragger::DisplayInEditMode);
    setPosition(frame->position());

    frameConnection =
        frame->sigUpdated().connect(
            [&](int flags){ onFrameUpdated(flags); });
    
    sigPositionDragged().connect([&](){ onMarkerPositionDragged(); });
}


void CoordinateFrameMarker::onFrameUpdated(int flags)
{
    if(flags & CoordinateFrame::PositionUpdate){
        setPosition(frame_->position());
        notifyUpdate();
    }
}


void CoordinateFrameMarker::onMarkerPositionDragged()
{
    frame_->setPosition(draggingPosition());
    frame_->notifyUpdate(CoordinateFrame::PositionUpdate);
}
