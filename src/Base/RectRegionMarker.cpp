/**
   @file
   @author Shin'ichiro Nakaoka
*/

#include "RectRegionMarker.h"
#include "SceneWidget.h"
#include "MenuManager.h"
#include <cnoid/SceneDrawables>
#include <cnoid/SceneCameras>
#include <cnoid/SceneRenderer>
#include <cnoid/PolyhedralRegion>

using namespace std;
using namespace cnoid;

namespace cnoid {

class RectRegionMarkerImpl
{
public:
    RectRegionMarker* self;
    SceneWidget* sceneWidget;
    SgLineSetPtr lineSet;
    int left, right, top, bottom;
    SgVertexArrayPtr vertices;
    QCursor editModeCursor;
    int x0, y0;
    PolyhedralRegion region;
    Signal<void(const PolyhedralRegion& region)> sigRegionFixed;
    Signal<void(const SceneWidgetEvent& event, MenuManager& menuManager)> sigContextMenuRequest;
    
    RectRegionMarkerImpl(RectRegionMarker* self);
    ~RectRegionMarkerImpl();
    void setRect(int x0, int y0, int x1, int y1);
    void showRectangle(bool on);
    void onSceneModeChanged(const SceneWidgetEvent& event);
    bool onButtonPressEvent(const SceneWidgetEvent& event);
    bool onButtonReleaseEvent(const SceneWidgetEvent& event);
    bool onPointerMoveEvent(const SceneWidgetEvent& event);
};

}


RectRegionMarker::RectRegionMarker()
{
    impl = new RectRegionMarkerImpl(this);
}


RectRegionMarkerImpl::RectRegionMarkerImpl(RectRegionMarker* self)
    : self(self)
{
    sceneWidget = 0;

    lineSet = new SgLineSet;
    vertices = lineSet->getOrCreateVertices();
    vertices->resize(4);
    lineSet->getOrCreateColors()->push_back(Vector3f(1.0f, 0.0f, 0.0f));
    lineSet->reserveNumLines(4);
    SgIndexArray& colorIndices = lineSet->colorIndices();
    colorIndices.reserve(8);
    for(int i=0; i < 4; ++i){
        lineSet->addLine(i, (i + 1) % 4);
        colorIndices.push_back(0);
        colorIndices.push_back(0);
    }

    self->addChild(lineSet);
}


RectRegionMarker::~RectRegionMarker()
{
    delete impl;
}


RectRegionMarkerImpl::~RectRegionMarkerImpl()
{
    self->finishEditing();
    showRectangle(false);
}


void RectRegionMarker::calcViewVolume(double viewportWidth, double viewportHeight, ViewVolume& io_volume)
{
    io_volume.left = 0;
    io_volume.right = viewportWidth;
    io_volume.bottom = 0;
    io_volume.top = viewportHeight;
}


void RectRegionMarker::setRect(int x0, int y0, int x1, int y1)
{
    impl->setRect(x0, y0, x1, y1);
}


void RectRegionMarkerImpl::setRect(int x0, int y0, int x1, int y1)
{
    if(x0 <= x1){
        left = x0;
        right = x1;
    } else {
        left = x1;
        right = x0;
    }
    if(y0 >= y1){
        top = y0;
        bottom = y1;
    } else {
        top = y1;
        bottom = y0;
    }
    
    vertices->at(0) << left,  top,    0.0f;
    vertices->at(1) << left,  bottom, 0.0f;
    vertices->at(2) << right, bottom, 0.0f;
    vertices->at(3) << right, top,    0.0f;
    
    vertices->notifyUpdate();
}


void RectRegionMarker::setEditModeCursor(QCursor cursor)
{
    impl->editModeCursor = cursor;
}


void RectRegionMarker::startEditing(SceneWidget* sceneWidget)
{
    impl->sceneWidget = sceneWidget;
    onSceneModeChanged(sceneWidget->latestEvent());
    sceneWidget->sceneRoot()->addChildOnce(this, true);
    sceneWidget->installEventFilter(this);
}


bool RectRegionMarker::isEditing() const
{
    if(impl->sceneWidget){
        if(impl->sceneWidget->activeEventFilter() == this){
            return true;
        }
    }
    return false;
}


void RectRegionMarker::finishEditing()
{
    if(impl->sceneWidget){
        impl->sceneWidget->removeEventFilter(this);
        impl->sceneWidget->sceneRoot()->removeChild(this);
        impl->sceneWidget = 0;
    }
}


const PolyhedralRegion& RectRegionMarker::region() const
{
    return impl->region;
}


SignalProxy<void(const PolyhedralRegion& region)> RectRegionMarker::sigRegionFixed()
{
    return impl->sigRegionFixed;
}


void RectRegionMarkerImpl::showRectangle(bool on)
{
    if(on){
        self->addChildOnce(lineSet, true);
    } else {
        self->removeChild(lineSet, true);
    }
}


void RectRegionMarker::onSceneModeChanged(const SceneWidgetEvent& event)
{
    impl->onSceneModeChanged(event);
}


void RectRegionMarkerImpl::onSceneModeChanged(const SceneWidgetEvent& event)
{
    if(event.sceneWidget()->isEditMode()){
        event.sceneWidget()->setCursor(editModeCursor);
    } else {
        showRectangle(false);
    }
}


bool RectRegionMarker::onButtonPressEvent(const SceneWidgetEvent& event)
{
    return impl->onButtonPressEvent(event);
}


bool RectRegionMarkerImpl::onButtonPressEvent(const SceneWidgetEvent& event)
{
    x0 = event.x();
    y0 = event.y();
    setRect(x0, y0, x0, y0);
    showRectangle(true);
    return true;
}


bool RectRegionMarker::onButtonReleaseEvent(const SceneWidgetEvent& event)
{
    return impl->onButtonReleaseEvent(event);
}


bool RectRegionMarkerImpl::onButtonReleaseEvent(const SceneWidgetEvent& event)
{
    if(left < right && bottom < top){
        Vector3 points[4];
        event.sceneWidget()->unproject(left, top, 0.0, points[0]);
        event.sceneWidget()->unproject(left, bottom, 0.0, points[1]);
        event.sceneWidget()->unproject(right, bottom, 0.0, points[2]);
        event.sceneWidget()->unproject(right, top, 0.0, points[3]);
        const Vector3 c = event.currentCameraPosition().translation();
        SgCamera* camera = event.sceneWidget()->renderer()->currentCamera();
        region.clear();
        if(dynamic_cast<SgPerspectiveCamera*>(camera)){
            for(int i=0; i < 4; ++i){
                const Vector3 normal = (points[(i + 1) % 4] - c).cross(points[i] - c).normalized();
                region.addBoundingPlane(normal, points[i]);
            }
        } else if(dynamic_cast<SgOrthographicCamera*>(camera)){
            const Vector3 n0 = (points[3] - points[0]).cross(points[1] - points[0]).normalized();
            for(int i=0; i < 4; ++i){
                const Vector3 normal = (points[(i + 1) % 4] - points[i]).cross(n0).normalized();
                region.addBoundingPlane(normal, points[i]);
            }
        }
        sigRegionFixed(region);
    }
    showRectangle(false);
    return true;
}


bool RectRegionMarker::onPointerMoveEvent(const SceneWidgetEvent& event)
{
    return impl->onPointerMoveEvent(event);
}


bool RectRegionMarkerImpl::onPointerMoveEvent(const SceneWidgetEvent& event)
{
    setRect(x0, y0, event.x(), event.y());
    return true;
}


void RectRegionMarker::onContextMenuRequest(const SceneWidgetEvent& event, MenuManager& menuManager)
{
    impl->sigContextMenuRequest(event, menuManager);
}


SignalProxy<void(const SceneWidgetEvent& event, MenuManager& menuManager)> RectRegionMarker::sigContextMenuRequest()
{
    return impl->sigContextMenuRequest;
}
