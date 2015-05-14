/**
   @file
   @author Shin'ichiro Nakaoka
*/

#include "SceneWidgetRectangle.h"
#include "SceneWidget.h"
#include <cnoid/SceneShape>
#include <cnoid/SceneCamera>
#include <cnoid/SceneRenderer>

using namespace std;
using namespace cnoid;

namespace {

struct RegionImpl
{
    RegionImpl() { }
    RegionImpl(int n) : normals(n), points(n) { }
    vector<Vector3> normals;
    vector<Vector3> points;
};

}

namespace cnoid {

class SceneWidgetRectangleImpl
{
public:
    SceneWidgetRectangle* self;
    SceneWidget* sceneWidget;
    SgLineSetPtr lineSet;
    int left, right, top, bottom;
    SgVertexArrayPtr vertices;
    QCursor editModeCursor;
    int x0, y0;
    SceneWidgetRectangle::Region region;
    Signal<void(const SceneWidgetRectangle::Region& region)> sigRegionFixed;
    Signal<void(const SceneWidgetEvent& event, MenuManager& menuManager)> sigContextMenuRequest;
    
    SceneWidgetRectangleImpl(SceneWidgetRectangle* self);
    ~SceneWidgetRectangleImpl();
    void setRect(int x0, int y0, int x1, int y1);
    void showRectangle(bool on);
    void onSceneModeChanged(const SceneWidgetEvent& event);
    bool onButtonPressEvent(const SceneWidgetEvent& event);
    bool onButtonReleaseEvent(const SceneWidgetEvent& event);
    bool onPointerMoveEvent(const SceneWidgetEvent& event);
};

}


SceneWidgetRectangle::SceneWidgetRectangle()
{
    impl = new SceneWidgetRectangleImpl(this);
}


SceneWidgetRectangleImpl::SceneWidgetRectangleImpl(SceneWidgetRectangle* self)
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


SceneWidgetRectangle::~SceneWidgetRectangle()
{
    delete impl;
}


SceneWidgetRectangleImpl::~SceneWidgetRectangleImpl()
{
    self->finishEditing();
    showRectangle(false);
}


void SceneWidgetRectangle::calcViewVolume(double viewportWidth, double viewportHeight, ViewVolume& io_volume)
{
    io_volume.left = 0;
    io_volume.right = viewportWidth;
    io_volume.bottom = 0;
    io_volume.top = viewportHeight;
}


void SceneWidgetRectangle::setRect(int x0, int y0, int x1, int y1)
{
    impl->setRect(x0, y0, x1, y1);
}


void SceneWidgetRectangleImpl::setRect(int x0, int y0, int x1, int y1)
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


void SceneWidgetRectangle::setEditModeCursor(QCursor cursor)
{
    impl->editModeCursor = cursor;
}


void SceneWidgetRectangle::startEditing(SceneWidget* sceneWidget)
{
    impl->sceneWidget = sceneWidget;
    onSceneModeChanged(sceneWidget->latestEvent());
    sceneWidget->sceneRoot()->addChildOnce(this, true);
    sceneWidget->installEventFilter(this);
}


bool SceneWidgetRectangle::isEditing() const
{
    if(impl->sceneWidget){
        if(impl->sceneWidget->activeEventFilter() == this){
            return true;
        }
    }
    return false;
}


void SceneWidgetRectangle::finishEditing()
{
    if(impl->sceneWidget){
        impl->sceneWidget->removeEventFilter(this);
        impl->sceneWidget->sceneRoot()->removeChild(this);
        impl->sceneWidget = 0;
    }
}


const SceneWidgetRectangle::Region& SceneWidgetRectangle::region() const
{
    return impl->region;
}


SignalProxy<void(const SceneWidgetRectangle::Region& region)> SceneWidgetRectangle::sigRegionFixed()
{
    return impl->sigRegionFixed;
}


void SceneWidgetRectangleImpl::showRectangle(bool on)
{
    if(on){
        self->addChildOnce(lineSet, true);
    } else {
        self->removeChild(lineSet, true);
    }
}


void SceneWidgetRectangle::onSceneModeChanged(const SceneWidgetEvent& event)
{
    impl->onSceneModeChanged(event);
}


void SceneWidgetRectangleImpl::onSceneModeChanged(const SceneWidgetEvent& event)
{
    if(event.sceneWidget()->isEditMode()){
        event.sceneWidget()->setCursor(editModeCursor);
    } else {
        showRectangle(false);
    }
}


bool SceneWidgetRectangle::onButtonPressEvent(const SceneWidgetEvent& event)
{
    return impl->onButtonPressEvent(event);
}


bool SceneWidgetRectangleImpl::onButtonPressEvent(const SceneWidgetEvent& event)
{
    x0 = event.x();
    y0 = event.y();
    setRect(x0, y0, x0, y0);
    showRectangle(true);
    return true;
}


bool SceneWidgetRectangle::onButtonReleaseEvent(const SceneWidgetEvent& event)
{
    return impl->onButtonReleaseEvent(event);
}


bool SceneWidgetRectangleImpl::onButtonReleaseEvent(const SceneWidgetEvent& event)
{
    if(left < right && bottom < top){
        SceneWidgetRectangle::Region& r = region;
        r.setNumSurroundingPlanes(4);
        event.sceneWidget()->unproject(left, top, 0.0, r.point(0));
        event.sceneWidget()->unproject(left, bottom, 0.0, r.point(1));
        event.sceneWidget()->unproject(right, bottom, 0.0, r.point(2));
        event.sceneWidget()->unproject(right, top, 0.0, r.point(3));
        const Vector3 c = event.currentCameraPosition().translation();
        SgCamera* camera = event.sceneWidget()->renderer().currentCamera();
        if(dynamic_cast<SgPerspectiveCamera*>(camera)){
            for(int i=0; i < 4; ++i){
                r.normal(i) = (r.point((i + 1) % 4) - c).cross(r.point(i) - c).normalized();
            }
        } else if(dynamic_cast<SgOrthographicCamera*>(camera)){
            const Vector3 n0 = (r.point(3) - r.point(0)).cross(r.point(1) - r.point(0)).normalized();
            for(int i=0; i< 4; ++i){
                r.normal(i) = (r.point((i + 1) % 4) - r.point(i)).cross(n0).normalized();
            }
        }

        sigRegionFixed(r);
    }
    showRectangle(false);
    return true;
}


bool SceneWidgetRectangle::onPointerMoveEvent(const SceneWidgetEvent& event)
{
    return impl->onPointerMoveEvent(event);
}


bool SceneWidgetRectangleImpl::onPointerMoveEvent(const SceneWidgetEvent& event)
{
    setRect(x0, y0, event.x(), event.y());
    return true;
}


void SceneWidgetRectangle::onContextMenuRequest(const SceneWidgetEvent& event, MenuManager& menuManager)
{
    impl->sigContextMenuRequest(event, menuManager);
}


SignalProxy<void(const SceneWidgetEvent& event, MenuManager& menuManager)> SceneWidgetRectangle::sigContextMenuRequest()
{
    return impl->sigContextMenuRequest;
}


SceneWidgetRectangle::Region::Region()
{
    impl = new RegionImpl;
}


SceneWidgetRectangle::Region::Region(int numSurroundingPlanes)
{
    impl = new RegionImpl(numSurroundingPlanes);
}
    

SceneWidgetRectangle::Region::Region(const SceneWidgetRectangle::Region& org)
{
    RegionImpl* p = new RegionImpl;
    RegionImpl* orgImpl = (RegionImpl*)(org.impl);
    p->normals = orgImpl->normals;
    p->points = orgImpl->points;
    impl = p;
}


SceneWidgetRectangle::Region& SceneWidgetRectangle::Region::operator=(const SceneWidgetRectangle::Region& org)
{
    RegionImpl* p = (RegionImpl*)impl;
    RegionImpl* orgImpl = (RegionImpl*)(org.impl);
    p->normals = orgImpl->normals;
    p->points = orgImpl->points;
    return *this;
}


void SceneWidgetRectangle::Region::setNumSurroundingPlanes(int n)
{
    RegionImpl* p = (RegionImpl*)impl;
    p->normals.resize(n);
    p->points.resize(n);
}


int SceneWidgetRectangle::Region::numSurroundingPlanes() const
{
    RegionImpl* p = (RegionImpl*)impl;
    return p->normals.size();
}

    
void SceneWidgetRectangle::Region::addSurroundingPlane(const Vector3& normal, const Vector3& point)
{
    RegionImpl* p = (RegionImpl*)impl;
    p->normals.push_back(normal);
    p->points.push_back(point);
}


Vector3& SceneWidgetRectangle::Region::normal(int index)
{
    RegionImpl* p = (RegionImpl*)impl;
    return p->normals[index];
}


const Vector3& SceneWidgetRectangle::Region::normal(int index) const
{
    RegionImpl* p = (RegionImpl*)impl;
    return p->normals[index];
}


Vector3& SceneWidgetRectangle::Region::point(int index)
{
    RegionImpl* p = (RegionImpl*)impl;
    return p->points[index];
}


const Vector3& SceneWidgetRectangle::Region::point(int index) const
{
    RegionImpl* p = (RegionImpl*)impl;
    return p->points[index];
}
