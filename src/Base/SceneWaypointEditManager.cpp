#include "SceneWaypointEditManager.h"
#include "SceneWidget.h"
#include <cnoid/SceneDrawables>
#include <cnoid/SceneUtil>
#include <map>
#include "gettext.h"

#include <iostream>

using namespace std;
using namespace cnoid;

namespace {

class SceneWidgetInfo
{
public:
    SceneWidget* widget;
    int prevPolygonDisplayElements;
    QMetaObject::Connection connection;

    ~SceneWidgetInfo(){
        widget->disconnect(connection);
    }
};

}

namespace cnoid {

class SceneWaypointEditManager::Impl
{
public:
    SceneWaypointEditManager* self;
    int modeId;
    SgOverlayPtr pointedVertexOverlay;
    SgPointSetPtr pointedVertexPlot;
    SgVertexArrayPtr pointedVertexArray;
    std::map<SceneWidget*, SceneWidgetInfo> sceneWidgetInfos;
    
    Impl(SceneWaypointEditManager* self);
    void setupSceneWaypointEditMode(SceneWidget* sceneWidget);
    void clearSceneWaypointEditMode(SceneWidget* sceneWidget);
    bool findPointedVertex(
        const SgVertexArray& vertices, const Affine3& T, const Vector3& point, int& out_index);
};

}


SceneWaypointEditManager::SceneWaypointEditManager()
{
    impl = new Impl(this);
}


SceneWaypointEditManager::Impl::Impl(SceneWaypointEditManager* self)
    : self(self)
{
    modeId = 0;

    pointedVertexPlot = new SgPointSet;
    pointedVertexPlot->setPointSize(10.0);
    pointedVertexArray = pointedVertexPlot->getOrCreateVertices();
    auto mat = pointedVertexPlot->getOrCreateMaterial();
    mat->setEmissiveColor(Vector3f(1.0f, 0.0f, 0.0f));
    mat->setDiffuseColor(Vector3f(1.0f, 0.0f, 0.0f));

    pointedVertexOverlay = new SgOverlay;
    pointedVertexOverlay->addChild(pointedVertexPlot);
}


SceneWaypointEditManager::~SceneWaypointEditManager()
{
    delete impl;
}


void SceneWaypointEditManager::setCustomModeId(int id)
{
    impl->modeId = id;
}


void SceneWaypointEditManager::onSceneModeChanged(const SceneWidgetEvent& event)
{
    auto sceneWidget = event.sceneWidget();
    if(sceneWidget->activeCustomMode() == impl->modeId && sceneWidget->isEditMode()){
        impl->setupSceneWaypointEditMode(sceneWidget);
    } else {
        impl->clearSceneWaypointEditMode(sceneWidget);
    }
}


void SceneWaypointEditManager::Impl::setupSceneWaypointEditMode(SceneWidget* sceneWidget)
{
    SceneWidgetInfo* info = nullptr;
    auto p = sceneWidgetInfos.find(sceneWidget);
    if(p != sceneWidgetInfos.end()){
        info = &p->second;
    } else {
        info = &sceneWidgetInfos[sceneWidget];
        info->widget = sceneWidget;
        info->connection =
            QObject::connect(
                sceneWidget, &QWidget::destroyed,
                [this, sceneWidget](){ sceneWidgetInfos.erase(sceneWidget); });
    }
    
    info->prevPolygonDisplayElements = sceneWidget->polygonDisplayElements();

    sceneWidget->setPolygonDisplayElements(SceneWidget::PolygonVertex | SceneWidget::PolygonEdge | SceneWidget::PolygonFace);

    sceneWidget->sceneRoot()->addChildOnce(pointedVertexOverlay, true);
}


void SceneWaypointEditManager::Impl::clearSceneWaypointEditMode(SceneWidget* sceneWidget)
{
    auto p = sceneWidgetInfos.find(sceneWidget);
    if(p != sceneWidgetInfos.end()){
        sceneWidget->setPolygonDisplayElements(p->second.prevPolygonDisplayElements);
        sceneWidget->sceneRoot()->removeChild(pointedVertexOverlay, true);
    }
}


bool SceneWaypointEditManager::onButtonPressEvent(const SceneWidgetEvent& event)
{
    return false;
}


bool SceneWaypointEditManager::onButtonReleaseEvent(const SceneWidgetEvent& event)
{
    return false;
}


bool SceneWaypointEditManager::onDoubleClickEvent(const SceneWidgetEvent& event)
{
    return false;
}


bool SceneWaypointEditManager::onPointerMoveEvent(const SceneWidgetEvent& event)
{
    auto& path = event.nodePath();
    bool pointed = false;
    if(!path.empty()){
        if(auto shape = dynamic_cast<SgShape*>(path.back())){
            auto vertices = *shape->mesh()->vertices();
            Position T = calcTotalTransform(path);
            int  pointedIndex;
            if(impl->findPointedVertex(vertices, T, event.point(), pointedIndex)){
                impl->pointedVertexArray->resize(1);
                Vector3 v = T * vertices[pointedIndex].cast<double>();
                impl->pointedVertexArray->front() = v.cast<float>();
                impl->pointedVertexArray->notifyUpdate();
                pointed = true;
            }
        }
    }
    if(!pointed){
        if(!impl->pointedVertexArray->empty()){
            impl->pointedVertexArray->clear();
            impl->pointedVertexArray->notifyUpdate();
        }
    }
    return false;
}


bool SceneWaypointEditManager::Impl::findPointedVertex
(const SgVertexArray& vertices, const Affine3& T, const Vector3& point, int& out_index)
{
    bool found = false;
    Vector3f localPoint = (T.inverse() * point).cast<float>();
    int minDistanceIndex = -1;
    float minDistance = std::numeric_limits<float>::max();
    const int n = vertices.size();
    for(int i=0; i < n; ++i){
        float distance = (vertices[i] - localPoint).norm();
        if(distance < minDistance){
            minDistance = distance;
            minDistanceIndex = i;
        }
    }
    if(minDistanceIndex >= 0){
        Vector3 v = T * vertices[minDistanceIndex].cast<double>();
        double distance = (v - point).norm();
        //! \todo The distance threshold should be constant in the viewport coordinate
        if(distance < 0.015){
            out_index = minDistanceIndex;
            found = true;
        }
    }
    return found;
}


void SceneWaypointEditManager::onPointerLeaveEvent(const SceneWidgetEvent& event)
{
    if(!impl->pointedVertexArray->empty()){
        impl->pointedVertexArray->clear();
        impl->pointedVertexArray->notifyUpdate();
    }
}


bool SceneWaypointEditManager::onKeyPressEvent(const SceneWidgetEvent& event)
{
    return false;
}


bool SceneWaypointEditManager::onKeyReleaseEvent(const SceneWidgetEvent& event)
{
    return false;
}


void SceneWaypointEditManager::onContextMenuRequest(const SceneWidgetEvent& event, MenuManager& menuManager)
{

}


bool SceneWaypointEditManager::onUndoRequest()
{
    return false;
}


bool SceneWaypointEditManager::onRedoRequest()
{
    return false;
}
