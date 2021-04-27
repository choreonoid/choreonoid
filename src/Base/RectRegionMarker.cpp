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
    ScopedConnection sceneWidgetConnection;
    SgLineSetPtr lineSet;
    int left, right, top, bottom;
    SgVertexArrayPtr vertices;
    QCursor editModeCursor;
    int x0, y0;
    PolyhedralRegion region;
    bool isDragging;
    Signal<void(const PolyhedralRegion& region)> sigRegionFixed;
    Signal<void(SceneWidgetEvent* event, MenuManager* menuManager)> sigContextMenuRequest;
    
    RectRegionMarkerImpl(RectRegionMarker* self);
    ~RectRegionMarkerImpl();
    void setRect(int x0, int y0, int x1, int y1);
    void showRectangle(bool on);
    void onSceneModeChanged(SceneWidgetEvent* event);
    bool onButtonPressEvent(SceneWidgetEvent* event);
    bool onButtonReleaseEvent(SceneWidgetEvent* event);
    bool onPointerMoveEvent(SceneWidgetEvent* event);
};

}


RectRegionMarker::RectRegionMarker()
{
    setAttribute(Operable);
    impl = new RectRegionMarkerImpl(this);
}


RectRegionMarkerImpl::RectRegionMarkerImpl(RectRegionMarker* self)
    : self(self)
{
    sceneWidget = nullptr;

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

    isDragging = false;
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
    SgTmpUpdate update;
    sceneWidget->sceneRoot()->addChildOnce(this, update);
    sceneWidget->activateCustomMode(this);

    impl->sceneWidgetConnection =
        sceneWidget->sigAboutToBeDestroyed().connect(
            [&](){
                finishEditing();
            });
}


bool RectRegionMarker::isEditing() const
{
    return (impl->sceneWidget && impl->sceneWidget->activeCustomModeHandler() == this);
}


void RectRegionMarker::finishEditing()
{
    if(impl->sceneWidget){
        impl->sceneWidget->deactivateCustomMode(this);
        impl->sceneWidget->sceneRoot()->removeChild(this);
        impl->sceneWidget = nullptr;
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
    SgTmpUpdate update;
    if(on){
        self->addChildOnce(lineSet, update);
    } else {
        self->removeChild(lineSet, update);
    }
}


void RectRegionMarker::onSceneModeChanged(SceneWidgetEvent* event)
{
    impl->onSceneModeChanged(event);
}


void RectRegionMarkerImpl::onSceneModeChanged(SceneWidgetEvent* event)
{
    auto sw = event->sceneWidget();
    if(sw->activeCustomModeHandler() == self && sw->isEditMode()){
        event->sceneWidget()->setCursor(editModeCursor);
    } else {
        showRectangle(false);
    }
}


bool RectRegionMarker::onButtonPressEvent(SceneWidgetEvent* event)
{
    return impl->onButtonPressEvent(event);
}


bool RectRegionMarkerImpl::onButtonPressEvent(SceneWidgetEvent* event)
{
    if(event->button() == Qt::LeftButton){
        x0 = event->x();
        y0 = event->y();
        setRect(x0, y0, x0, y0);
        showRectangle(true);
        isDragging = true;
        return true;
    }
    return false;
}


bool RectRegionMarker::onButtonReleaseEvent(SceneWidgetEvent* event)
{
    return impl->onButtonReleaseEvent(event);
}


bool RectRegionMarkerImpl::onButtonReleaseEvent(SceneWidgetEvent* event)
{
    if(isDragging && (left < right && bottom < top)){
        Vector3 points[4];
        auto sw = event->sceneWidget();
        sw->unproject(left, top, 0.0, points[0]);
        sw->unproject(left, bottom, 0.0, points[1]);
        sw->unproject(right, bottom, 0.0, points[2]);
        sw->unproject(right, top, 0.0, points[3]);
        SgCamera* camera = sw->renderer()->currentCamera();
        region.clear();
        if(dynamic_cast<SgPerspectiveCamera*>(camera)){
            const Vector3 c = event->cameraPosition().translation();
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
        isDragging = false;
    }
    showRectangle(false);
    return true;
}


bool RectRegionMarker::onPointerMoveEvent(SceneWidgetEvent* event)
{
    return impl->onPointerMoveEvent(event);
}


bool RectRegionMarkerImpl::onPointerMoveEvent(SceneWidgetEvent* event)
{
    if(isDragging){
        setRect(x0, y0, event->x(), event->y());
        return true;
    }
    return false;
}


bool RectRegionMarker::onContextMenuRequest(SceneWidgetEvent* event, MenuManager* menuManager)
{
    impl->sigContextMenuRequest(event, menuManager);
    return true;
}


SignalProxy<void(SceneWidgetEvent* event, MenuManager* menuManager)> RectRegionMarker::sigContextMenuRequest()
{
    return impl->sigContextMenuRequest;
}
