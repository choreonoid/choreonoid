#include "ScenePointSelectionMode.h"
#include "SceneWidget.h"
#include <cnoid/SceneRenderer>
#include <cnoid/SceneDrawables>
#include <cnoid/SceneEffects>
#include <cnoid/SceneUtil>
#include <map>
#include <unordered_set>
#include "gettext.h"

using namespace std;
using namespace cnoid;

namespace {

class SceneWidgetInfo
{
public:
    SceneWidget* widget;
    int nodeDecorationId;
    QMetaObject::Connection connection;

    SceneWidgetInfo(){
        nodeDecorationId = 1; // temporary
    }
    ~SceneWidgetInfo(){
        widget->disconnect(connection);
    }
};

}

namespace cnoid {

class ScenePointSelectionMode::Impl
{
public:
    ScenePointSelectionMode* self;
    int modeId;
    std::map<SceneWidget*, SceneWidgetInfo> sceneWidgetInfos;
    unordered_set<SgNodePtr> targetNodes;

    SgOverlayPtr pointOverlay;
    SgPointSetPtr highlightedPointPlot;
    SgVertexArrayPtr highlightedPointArray;
    SgPointSetPtr selectedPointPlot;
    SgVertexArrayPtr selectedPointArray;
    SgUpdate update;
    
    PointInfoPtr highlightedPoint;
    std::vector<PointInfoPtr> selectedPoints;
    
    Impl(ScenePointSelectionMode* self);
    void setupScenePointSelectionMode(const SceneWidgetEvent& event);
    void clearScenePointSelectionMode(SceneWidget* sceneWidget);
    bool findPointedVertex(
        const SgVertexArray& vertices, const Affine3& T, const Vector3& point, int& out_index);
    void setHighlightedPoint(
        const SgNodePath& path, SgVertexArray& vertices, const Affine3& T, int vertexIndex);
    void clearHighlightedPoint();
    bool onButtonPressEvent(const SceneWidgetEvent& event);
    void updateSelectedPointArray();
    
};

}


ScenePointSelectionMode::PointInfo::PointInfo()
{
    vertexIndex_ = -1;
}


bool ScenePointSelectionMode::PointInfo::operator==(const PointInfo& rhs) const
{
    if(path_ == rhs.path_ ||
       (path_ && rhs.path_ && *path_ == *rhs.path_)){
        if(vertexIndex_ == rhs.vertexIndex_){
            return true;
        }
    }
    return false;
}


ScenePointSelectionMode::ScenePointSelectionMode()
{
    impl = new Impl(this);
}


ScenePointSelectionMode::Impl::Impl(ScenePointSelectionMode* self)
    : self(self)
{
    modeId = 0;

    highlightedPointPlot = new SgPointSet;
    highlightedPointPlot->setPointSize(10.0);
    highlightedPointArray = highlightedPointPlot->getOrCreateVertices();
    highlightedPointPlot->getOrCreateMaterial()->setDiffuseColor(Vector3f(1.0f, 1.0f, 0.0f));

    selectedPointPlot = new SgPointSet;
    selectedPointPlot->setPointSize(10.0);
    selectedPointArray = selectedPointPlot->getOrCreateVertices();
    selectedPointPlot->getOrCreateMaterial()->setDiffuseColor(Vector3f(1.0f, 0.0f, 0.0f));
    
    pointOverlay = new SgOverlay;
    pointOverlay->addChild(highlightedPointPlot);
    pointOverlay->addChild(selectedPointPlot);
}


ScenePointSelectionMode::~ScenePointSelectionMode()
{
    delete impl;
}


void ScenePointSelectionMode::setCustomModeId(int id)
{
    impl->modeId = id;
}


std::vector<Vector3f> ScenePointSelectionMode::getSelectedPoints() const
{
    std::vector<Vector3f> points;
    points.reserve(impl->selectedPoints.size());
    for(auto& point : impl->selectedPoints){
        points.push_back(point->position());
    }
    return points;
}


void ScenePointSelectionMode::clearSelection()
{
    impl->selectedPoints.clear();
    impl->updateSelectedPointArray();
}


ScenePointSelectionMode::PointInfo* ScenePointSelectionMode::highlightedPoint()
{
    return impl->highlightedPoint;
}


std::vector<SgNode*> ScenePointSelectionMode::getTargetSceneNodes(const SceneWidgetEvent& /* event */)
{
    return std::vector<SgNode*>();
}
    

#if 0
void ScenePointSelectionMode::onSelectionModeActivated(const SceneWidgetEvent& /* event */)
{

}


void ScenePointSelectionMode::onSelectionModeDeactivated(const SceneWidgetEvent& /* event */)
{

}
#endif


void ScenePointSelectionMode::onSceneModeChanged(const SceneWidgetEvent& event)
{
    auto sw = event.sceneWidget();
    int activeMode = sw->activeCustomMode();
    if(activeMode == impl->modeId && sw->isEditMode()){
        impl->setupScenePointSelectionMode(event);
    } else {
        impl->clearScenePointSelectionMode(sw);
    }
}


void ScenePointSelectionMode::Impl::setupScenePointSelectionMode(const SceneWidgetEvent& event)
{
    auto sceneWidget = event.sceneWidget();
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
    
    sceneWidget->systemNodeGroup()->addChildOnce(pointOverlay, true);

    int id = info->nodeDecorationId;
    auto renderer = sceneWidget->renderer();
    renderer->clearNodeDecorations(id);
    targetNodes.clear();
    for(auto& node : self->getTargetSceneNodes(event)){
        targetNodes.insert(node);
        SgPolygonDrawStylePtr style = new SgPolygonDrawStyle;
        style->setPolygonElements(
            SgPolygonDrawStyle::Face | SgPolygonDrawStyle::Edge | SgPolygonDrawStyle::Vertex);
        renderer->addNodeDecoration(
            node,
            [style](SgNode* node){
                style->setSingleChild(node);
                return style;
            },
            id);
    }
    
}


void ScenePointSelectionMode::Impl::clearScenePointSelectionMode(SceneWidget* sceneWidget)
{
    auto p = sceneWidgetInfos.find(sceneWidget);
    if(p != sceneWidgetInfos.end()){
        SceneWidgetInfo& info = p->second;;
        sceneWidget->systemNodeGroup()->removeChild(pointOverlay, true);
        sceneWidget->renderer()->clearNodeDecorations(info.nodeDecorationId);
    }
    targetNodes.clear();
}


bool ScenePointSelectionMode::onPointerMoveEvent(const SceneWidgetEvent& event)
{
    if(!event.sceneWidget()->isEditMode()){
        return false;
    }

    bool isTargetNode = false;
    auto& path = event.nodePath();
    for(auto iter = path.rbegin(); iter != path.rend(); ++iter){
        auto& node = *iter;
        if(impl->targetNodes.find(node) != impl->targetNodes.end()){
            isTargetNode = true;
            break;
        }
    }
            
    bool pointed = false;
    if(isTargetNode){
        if(auto shape = dynamic_cast<SgShape*>(path.back())){
            auto vertices = *shape->mesh()->vertices();
            Affine3 T = calcTotalTransform(path);
            int  pointedIndex;
            pointed = impl->findPointedVertex(vertices, T, event.point(), pointedIndex);
            if(pointed){
                impl->setHighlightedPoint(path, vertices, T, pointedIndex);
            }
        }
    }
    if(!pointed){
        impl->clearHighlightedPoint();
    }
    return true;
}


bool ScenePointSelectionMode::Impl::findPointedVertex
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


void ScenePointSelectionMode::Impl::setHighlightedPoint
(const SgNodePath& path, SgVertexArray& vertices, const Affine3& T, int vertexIndex)
{
    highlightedPoint = new ScenePointSelectionMode::PointInfo;
    Vector3f v = (T * vertices[vertexIndex].cast<double>()).cast<float>();
    highlightedPoint->path_ = make_shared<SgNodePath>(path);
    highlightedPoint->vertexIndex_ = vertexIndex;
    highlightedPoint->position_ = v;
    highlightedPointArray->resize(1);
    highlightedPointArray->front() = v;
    highlightedPointArray->notifyUpdate(update);
}


void ScenePointSelectionMode::Impl::clearHighlightedPoint()
{
    highlightedPoint.reset();

    if(!highlightedPointArray->empty()){
        highlightedPointArray->clear();
        highlightedPointArray->notifyUpdate(update);
    }
}


void ScenePointSelectionMode::onPointerLeaveEvent(const SceneWidgetEvent& event)
{
    if(!impl->highlightedPointArray->empty()){
        impl->highlightedPointArray->clear();
        impl->highlightedPointArray->notifyUpdate(impl->update);
    }
}


bool ScenePointSelectionMode::onButtonPressEvent(const SceneWidgetEvent& event)
{
    return impl->onButtonPressEvent(event);
}


bool ScenePointSelectionMode::Impl::onButtonPressEvent(const SceneWidgetEvent& event)
{
    bool processed = false;
    
    if(event.button() == Qt::LeftButton){
        bool isPointSelectionUpdated = false;
        if(!highlightedPoint){
            if(!(event.modifiers() & Qt::ControlModifier)){
                if(!selectedPoints.empty()){
                    selectedPoints.clear();
                    isPointSelectionUpdated = true;
                }
            }
        } else {
            bool removed = false;
            shared_ptr<SgNodePath> sharedPath;
            if(!(event.modifiers() & Qt::ControlModifier)){
                selectedPoints.clear();
            } else {
                for(auto it = selectedPoints.begin(); it != selectedPoints.end(); ++it){
                    PointInfo* point = *it;
                    if(point == highlightedPoint){
                        selectedPoints.erase(it);
                        removed = true;
                        break;
                    }
                    if(!sharedPath && (point->path() == highlightedPoint->path())){
                        sharedPath = point->path_;
                    }
                }
            }
            if(!removed){
                if(sharedPath){
                    highlightedPoint->path_ = sharedPath;
                }
                selectedPoints.push_back(highlightedPoint);
            }
            isPointSelectionUpdated = true;
        }
        if(isPointSelectionUpdated){
            updateSelectedPointArray();
        }
        processed = true;

    }
    
    return processed;
}


void ScenePointSelectionMode::Impl::updateSelectedPointArray()
{
    selectedPointArray->clear();

    for(auto& point : selectedPoints){
        selectedPointArray->push_back(point->position());
    }

    selectedPointArray->notifyUpdate(update);
}


bool ScenePointSelectionMode::onButtonReleaseEvent(const SceneWidgetEvent& event)
{
    return false;
}


bool ScenePointSelectionMode::onDoubleClickEvent(const SceneWidgetEvent& event)
{
    return false;
}


bool ScenePointSelectionMode::onKeyPressEvent(const SceneWidgetEvent& event)
{
    return false;
}


bool ScenePointSelectionMode::onKeyReleaseEvent(const SceneWidgetEvent& event)
{
    return false;
}


bool ScenePointSelectionMode::onContextMenuRequest(const SceneWidgetEvent& event, MenuManager& menu)
{
    return false;
}


bool ScenePointSelectionMode::onUndoRequest()
{
    return false;
}


bool ScenePointSelectionMode::onRedoRequest()
{
    return false;
}
