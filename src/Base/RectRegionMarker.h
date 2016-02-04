/**
   @author Shin'ichiro Nakaoka
*/

#ifndef CNOID_BASE_RECT_REGION_MARKER_H
#define CNOID_BASE_RECT_REGION_MARKER_H

#include "SceneWidgetEditable.h"
#include <cnoid/SceneDrawables>
#include <QCursor>
#include "exportdecl.h"

namespace cnoid {

class PolyhedralRegion;
class RectRegionMarkerImpl;

class CNOID_EXPORT RectRegionMarker : public SgOverlay, public SceneWidgetEditable
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

    virtual void calcViewVolume(double viewportWidth, double viewportHeight, ViewVolume& io_volume);
    virtual void onSceneModeChanged(const SceneWidgetEvent& event);
    virtual bool onButtonPressEvent(const SceneWidgetEvent& event);
    virtual bool onButtonReleaseEvent(const SceneWidgetEvent& event);
    virtual bool onPointerMoveEvent(const SceneWidgetEvent& event);
    virtual void onContextMenuRequest(const SceneWidgetEvent& event, MenuManager& menuManager);

    SignalProxy<void(const SceneWidgetEvent& event, MenuManager& menuManager)> sigContextMenuRequest();

private:
    RectRegionMarkerImpl* impl;
};

typedef ref_ptr<RectRegionMarker> RectRegionMarkerPtr;

}

#endif
