#include "SceneWaypointEditManager.h"
#include "SceneWidget.h"
#include <cnoid/SceneDrawables>
#include <cnoid/SceneUtil>
#include <map>
#include "gettext.h"

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

class VertexInfo
{
public:
    shared_ptr<SgNodePath> path;
    int vertexIndex;
    Vector3f position;

    VertexInfo() : vertexIndex(-1) { }

    bool isValid() const { return bool(path); }

    bool operator==(const VertexInfo& rhs){
        if(path == rhs.path ||
           (path && rhs.path && *path == *rhs.path)){
            if(vertexIndex == rhs.vertexIndex){
                return true;
            }
        }
        return false;
    }
};

}

namespace cnoid {

class SceneWaypointEditManager::Impl
{
public:
    SceneWaypointEditManager* self;
    int modeId;
    std::map<SceneWidget*, SceneWidgetInfo> sceneWidgetInfos;

    SgOverlayPtr vertexOverlay;
    SgPointSetPtr pointedVertexPlot;
    SgVertexArrayPtr pointedVertexArray;
    SgPointSetPtr selectedVertexPlot;
    SgVertexArrayPtr selectedVertexArray;
    SgUpdate update;
    
    VertexInfo pointedVertex;
    std::vector<VertexInfo> selectedVertices;
    
    Impl(SceneWaypointEditManager* self);
    void setupSceneWaypointEditMode(SceneWidget* sceneWidget);
    void clearSceneWaypointEditMode(SceneWidget* sceneWidget);
    bool findPointedVertex(
        const SgVertexArray& vertices, const Affine3& T, const Vector3& point, int& out_index);
    void setPointedVertex(
        const SgNodePath& path, SgVertexArray& vertices, const Affine3& T, int vertexIndex);
    void clearPointedVertex();
    bool onButtonPressEvent(const SceneWidgetEvent& event);
    void updateSelectedVertexArray();
    
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
    pointedVertexPlot->getOrCreateMaterial()->setDiffuseColor(Vector3f(1.0f, 1.0f, 0.0f));

    selectedVertexPlot = new SgPointSet;
    selectedVertexPlot->setPointSize(10.0);
    selectedVertexArray = selectedVertexPlot->getOrCreateVertices();
    selectedVertexPlot->getOrCreateMaterial()->setDiffuseColor(Vector3f(1.0f, 0.0f, 0.0f));
    
    vertexOverlay = new SgOverlay;
    vertexOverlay->addChild(pointedVertexPlot);
    vertexOverlay->addChild(selectedVertexPlot);
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

    sceneWidget->sceneRoot()->addChildOnce(vertexOverlay, true);
}


void SceneWaypointEditManager::Impl::clearSceneWaypointEditMode(SceneWidget* sceneWidget)
{
    auto p = sceneWidgetInfos.find(sceneWidget);
    if(p != sceneWidgetInfos.end()){
        sceneWidget->setPolygonDisplayElements(p->second.prevPolygonDisplayElements);
        sceneWidget->sceneRoot()->removeChild(vertexOverlay, true);
    }
}


bool SceneWaypointEditManager::onPointerMoveEvent(const SceneWidgetEvent& event)
{
    if(!event.sceneWidget()->isEditMode()){
        return false;
    }
    
    auto& path = event.nodePath();
    bool pointed = false;
    if(!path.empty()){
        if(auto shape = dynamic_cast<SgShape*>(path.back())){
            auto vertices = *shape->mesh()->vertices();
            Affine3 T = calcTotalTransform(path);
            int  pointedIndex;
            pointed = impl->findPointedVertex(vertices, T, event.point(), pointedIndex);
            if(pointed){
                impl->setPointedVertex(path, vertices, T, pointedIndex);
            }
        }
    }
    if(!pointed){
        impl->clearPointedVertex();
    }
    return true;
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
        if(distance < 0.01){
            out_index = minDistanceIndex;
            found = true;
        }
    }
    return found;
}


void SceneWaypointEditManager::Impl::setPointedVertex
(const SgNodePath& path, SgVertexArray& vertices, const Affine3& T, int vertexIndex)
{
    Vector3f v = (T * vertices[vertexIndex].cast<double>()).cast<float>();
    pointedVertex.path = make_shared<SgNodePath>(path);
    pointedVertex.vertexIndex = vertexIndex;
    pointedVertex.position = v;
    pointedVertexArray->resize(1);
    pointedVertexArray->front() = v;
    pointedVertexArray->notifyUpdate(update);
}


void SceneWaypointEditManager::Impl::clearPointedVertex()
{
    pointedVertex.path.reset();
    pointedVertex.vertexIndex = -1;

    if(!pointedVertexArray->empty()){
        pointedVertexArray->clear();
        pointedVertexArray->notifyUpdate(update);
    }
}


void SceneWaypointEditManager::onPointerLeaveEvent(const SceneWidgetEvent& event)
{
    if(!impl->pointedVertexArray->empty()){
        impl->pointedVertexArray->clear();
        impl->pointedVertexArray->notifyUpdate(impl->update);
    }
}


bool SceneWaypointEditManager::onButtonPressEvent(const SceneWidgetEvent& event)
{
    return impl->onButtonPressEvent(event);
}


bool SceneWaypointEditManager::Impl::onButtonPressEvent(const SceneWidgetEvent& event)
{
    bool processed = false;
    
    if(event.button() == Qt::LeftButton){
        bool isVertexSelectionUpdated = false;
        if(!pointedVertex.isValid()){
            if(!(event.modifiers() & Qt::ControlModifier)){
                if(!selectedVertices.empty()){
                    selectedVertices.clear();
                    isVertexSelectionUpdated = true;
                }
            }
        } else {
            bool removed = false;
            shared_ptr<SgNodePath> sharedPath;
            if(!(event.modifiers() & Qt::ControlModifier)){
                selectedVertices.clear();
            } else {
                for(auto it = selectedVertices.begin(); it != selectedVertices.end(); ++it){
                    if(*it == pointedVertex){
                        selectedVertices.erase(it);
                        removed = true;
                        break;
                    }
                    if(!sharedPath && (*it->path == *pointedVertex.path)){
                        sharedPath = it->path;
                    }
                }
            }
            if(!removed){
                if(sharedPath){
                    pointedVertex.path = sharedPath;
                }
                selectedVertices.push_back(pointedVertex);
            }
            isVertexSelectionUpdated = true;
        }
        if(isVertexSelectionUpdated){
            updateSelectedVertexArray();
        }
        processed = true;
    }

                    
    return processed;
}


void SceneWaypointEditManager::Impl::updateSelectedVertexArray()
{
    selectedVertexArray->clear();

    for(auto& vertexInfo : selectedVertices){
        selectedVertexArray->push_back(vertexInfo.position);
    }

    selectedVertexArray->notifyUpdate(update);
}


bool SceneWaypointEditManager::onButtonReleaseEvent(const SceneWidgetEvent& event)
{
    return false;
}


bool SceneWaypointEditManager::onDoubleClickEvent(const SceneWidgetEvent& event)
{
    return false;
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
