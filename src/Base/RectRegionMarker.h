/**
   @author Shin'ichiro Nakaoka
*/

#ifndef CNOID_BASE_RECT_REGION_MARKER_H
#define CNOID_BASE_RECT_REGION_MARKER_H

#include "SceneWidgetEventHandler.h"
#include <cnoid/SceneDrawables>
#include <QCursor>
#include "exportdecl.h"

namespace cnoid {

class PolyhedralRegion;
class RectRegionMarkerImpl;

class CNOID_EXPORT RectRegionMarker : public SgViewportOverlay, public SceneWidgetEventHandler
{
public:
    RectRegionMarker();
    ~RectRegionMarker();

    void setRect(int x0, int y0, int x1, int y1);
    
    void setEditModeCursor(QCursor cursor);

    void startEditing(SceneWidget* sceneWidget);
    bool isEditing() const;
    void finishEditing();

    const PolyhedralRegion& region() const;
    SignalProxy<void(const PolyhedralRegion& region)> sigRegionFixed();

    virtual void calcViewVolume(double viewportWidth, double viewportHeight, ViewVolume& io_volume) override;
    virtual void onSceneModeChanged(SceneWidgetEvent* event) override;
    virtual bool onButtonPressEvent(SceneWidgetEvent* event) override;
    virtual bool onButtonReleaseEvent(SceneWidgetEvent* event) override;
    virtual bool onPointerMoveEvent(SceneWidgetEvent* event) override;
    virtual bool onContextMenuRequest(SceneWidgetEvent* event) override;

    SignalProxy<void(SceneWidgetEvent* event)> sigContextMenuRequest();

private:
    RectRegionMarkerImpl* impl;
};

typedef ref_ptr<RectRegionMarker> RectRegionMarkerPtr;

}

#endif
