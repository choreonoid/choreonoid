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

constexpr bool MAKE_NORMALS_FIXED_PIXEL_SIZE = true;

typedef ScenePointSelectionMode::PointInfo PointInfo;
typedef ScenePointSelectionMode::PointInfoPtr PointInfoPtr;

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

class FixedPixelSizeNormal : public SgPosTransform
{
public:
    SgLineSetPtr sharedLine;
    
    FixedPixelSizeNormal(SgLineSetPtr& sharedLine, SgMaterial* material);
    void setNormal(const Vector3f& position, const Vector3f& normal);

};

class ScenePointPlot : public SgGroup
{
    SgPointSetPtr pointSet;
    SgVertexArrayPtr points;
    SgLineSetPtr lineSet;
    SgVertexArrayPtr normalVertices;
    SgGroupPtr fpsNormalGroup;
    SgMaterialPtr material_;

public:
    ScenePointPlot();
    SgMaterial* material() { return material_; }
    void clearPoints(bool doNotify);
    void updatePoints(const std::vector<PointInfoPtr>& infos);
    void resetPoint(PointInfo* info);

private:
    void addPoint(PointInfo* info, bool doNotify);
};

typedef ref_ptr<ScenePointPlot> ScenePointPlotPtr;

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
    ScenePointPlotPtr highlightedPointPlot;
    SgVertexArrayPtr highlightedPointArray;
    SgLineSetPtr highlightedPointNormalPlot;
    ScenePointPlotPtr selectedPointPlot;
    SgVertexArrayPtr selectedPointArray;
    
    PointInfoPtr highlightedPoint;
    std::vector<PointInfoPtr> selectedPoints;

    Signal<void(const std::vector<PointInfoPtr>& points)> sigPointSelectionAdded;
    
    Impl(ScenePointSelectionMode* self);
    void setupScenePointSelectionMode(SceneWidgetEvent* event);
    void clearScenePointSelectionMode(SceneWidget* sceneWidget);
    bool checkIfPointingTargetNode(SceneWidgetEvent* event);
    bool findPointedTriangleVertex(
        SgMesh* mesh, const Affine3& T, SceneWidgetEvent* event, int& out_index);
    void setHighlightedPoint(
        const SgNodePath& path, SgMesh* mesh, const Affine3& T, int vertexIndex);
    void clearHighlightedPoint();
    bool onButtonPressEvent(SceneWidgetEvent* event);
};

}


ScenePointSelectionMode::PointInfo::PointInfo()
{
    vertexIndex_ = -1;
    triangleVertexIndex_ = -1;
    hasNormal_ = false;
}


bool ScenePointSelectionMode::PointInfo::hasSameVertexWith(const PointInfo& point) const
{
    if(path_ == point.path_ ||
       (path_ && point.path_ && *path_ == *point.path_)){
        if(position_ == point.position_){
            return true;
        }
    }
    return false;
}


namespace {

FixedPixelSizeNormal::FixedPixelSizeNormal(SgLineSetPtr& sharedLine, SgMaterial* material)
    : sharedLine(sharedLine)
{
    if(!sharedLine){
        sharedLine = new SgLineSet;
        sharedLine->setMaterial(material);
        auto& vertices = *sharedLine->getOrCreateVertices(2);
        vertices[0] = Vector3f::Zero();
        vertices[1] = Vector3f::UnitZ();
        sharedLine->addLine(0, 1);
    }

    auto fpsg = new SgFixedPixelSizeGroup;
    fpsg->setPixelSizeRatio(32.0f);
    fpsg->addChild(sharedLine);
    addChild(fpsg);
}


void FixedPixelSizeNormal::setNormal(const Vector3f& position, const Vector3f& normal)
{
    setTranslation(position);
    setRotation(Quaternion::FromTwoVectors(Vector3::UnitZ(), normal.cast<double>()));
}


ScenePointPlot::ScenePointPlot()
{
    material_ = new SgMaterial;
    
    pointSet = new SgPointSet;
    pointSet->setPointSize(10.0);
    pointSet->setMaterial(material_);
    points = pointSet->getOrCreateVertices();
    addChild(pointSet);

    if(MAKE_NORMALS_FIXED_PIXEL_SIZE){
        fpsNormalGroup = new SgGroup;
        addChild(fpsNormalGroup);
    } else {
        lineSet = new SgLineSet;
        lineSet->setMaterial(material_);
        normalVertices = lineSet->getOrCreateVertices();
        addChild(lineSet);
    }
}


void ScenePointPlot::clearPoints(bool doNotify)
{
    SgTmpUpdate update;
    
    if(!points->empty()){
        points->clear();
        if(doNotify){
            points->notifyUpdate(update);
        }
    }
    if(MAKE_NORMALS_FIXED_PIXEL_SIZE){
        fpsNormalGroup->clearChildren();
        if(doNotify){
            fpsNormalGroup->notifyUpdate(update);
        }
    } else {
        if(!normalVertices->empty()){
            normalVertices->clear();
            lineSet->clearLines();
            if(doNotify){
                normalVertices->notifyUpdate(update);
            }
        }
    }
}


void ScenePointPlot::updatePoints(const std::vector<PointInfoPtr>& infos)
{
    clearPoints(false);

    for(auto& info : infos){
        addPoint(info, false);
    }

    SgTmpUpdate update;
    points->notifyUpdate(update);

    if(MAKE_NORMALS_FIXED_PIXEL_SIZE){
        fpsNormalGroup->notifyUpdate(update);
    } else {
        normalVertices->notifyUpdate(update);
    }
}


void ScenePointPlot::resetPoint(PointInfo* info)
{
    clearPoints(false);
    addPoint(info, true);
}


void ScenePointPlot::addPoint(PointInfo* info, bool doNotify)
{
    points->push_back(info->position());

    SgTmpUpdate update;
    
    if(doNotify){
        points->notifyUpdate(update);
    }
    
    if(info->hasNormal()){
        if(MAKE_NORMALS_FIXED_PIXEL_SIZE){
            auto fpsNormal = new FixedPixelSizeNormal(lineSet, material_);
            fpsNormal->setNormal(info->position(), info->normal());
            fpsNormalGroup->addChild(fpsNormal);
            if(doNotify){
                fpsNormalGroup->notifyUpdate(update);
            }
        } else {
            auto i = normalVertices->size();
            normalVertices->push_back(info->position());
            normalVertices->push_back(info->position() + info->normal() * 0.015);
            lineSet->addLine(i, i + 1);
            if(doNotify){
                normalVertices->notifyUpdate(update);
            }
        }
    }
}

}


ScenePointSelectionMode::ScenePointSelectionMode()
{
    impl = new Impl(this);
}


ScenePointSelectionMode::Impl::Impl(ScenePointSelectionMode* self)
    : self(self)
{
    modeId = 0;

    highlightedPointPlot = new ScenePointPlot;
    highlightedPointPlot->material()->setDiffuseColor(Vector3f(1.0f, 1.0f, 0.0f));
    
    selectedPointPlot = new ScenePointPlot;
    selectedPointPlot->material()->setDiffuseColor(Vector3f(1.0f, 0.0f, 0.0f));
    
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


const std::vector<PointInfoPtr>& ScenePointSelectionMode::selectedPoints() const
{
    return impl->selectedPoints;
}


void ScenePointSelectionMode::clearSelection()
{
    impl->selectedPoints.clear();
    impl->selectedPointPlot->clearPoints(true);
}


SignalProxy<void(const std::vector<PointInfoPtr>& points)> ScenePointSelectionMode::sigPointSelectionAdded()
{
    return impl->sigPointSelectionAdded;
}


ScenePointSelectionMode::PointInfo* ScenePointSelectionMode::highlightedPoint()
{
    return impl->highlightedPoint;
}


std::vector<SgNode*> ScenePointSelectionMode::getTargetSceneNodes(SceneWidgetEvent* /* event */)
{
    return std::vector<SgNode*>();
}
    

#if 0
void ScenePointSelectionMode::onSelectionModeActivated(SceneWidgetEvent* /* event */)
{

}


void ScenePointSelectionMode::onSelectionModeDeactivated(SceneWidgetEvent* /* event */)
{

}
#endif


void ScenePointSelectionMode::onSceneModeChanged(SceneWidgetEvent* event)
{
    auto sw = event->sceneWidget();
    int activeMode = sw->activeCustomMode();
    if(activeMode == impl->modeId && sw->isEditMode()){
        impl->setupScenePointSelectionMode(event);
    } else {
        impl->clearScenePointSelectionMode(sw);
    }
}


void ScenePointSelectionMode::Impl::setupScenePointSelectionMode(SceneWidgetEvent* event)
{
    auto sceneWidget = event->sceneWidget();
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

    SgTmpUpdate update;
    sceneWidget->systemNodeGroup()->addChildOnce(pointOverlay, update);

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
        SgTmpUpdate update;
        sceneWidget->systemNodeGroup()->removeChild(pointOverlay, update);
        sceneWidget->renderer()->clearNodeDecorations(info.nodeDecorationId);
    }
    targetNodes.clear();
}


bool ScenePointSelectionMode::Impl::checkIfPointingTargetNode(SceneWidgetEvent* event)
{
    bool isTargetNode = false;
    auto& path = event->nodePath();
    for(auto iter = path.rbegin(); iter != path.rend(); ++iter){
        auto& node = *iter;
        if(targetNodes.find(node) != targetNodes.end()){
            isTargetNode = true;
            break;
        }
    }
    return isTargetNode;
}


bool ScenePointSelectionMode::onPointerMoveEvent(SceneWidgetEvent* event)
{
    if(!event->sceneWidget()->isEditMode()){
        return false;
    }

    bool isTargetNode = impl->checkIfPointingTargetNode(event);

    bool pointed = false;
    if(isTargetNode){
        auto& path = event->nodePath();
        if(auto shape = dynamic_cast<SgShape*>(path.back())){
            auto mesh = shape->mesh();
            Affine3 T = calcTotalTransform(path);
            int  pointedIndex;
            if(impl->findPointedTriangleVertex(mesh, T, event, pointedIndex)){
                impl->setHighlightedPoint(path, mesh, T, pointedIndex);
                pointed = true;
            }
        }
    }
    if(!pointed){
        impl->clearHighlightedPoint();
    }

    return isTargetNode;
}


static bool checkRayTraiangleIntersection
(const Vector3& rayOrigin, const Vector3& rayDirection,
 const Vector3& vertex0, const Vector3& vertex1, const Vector3& vertex2,
 bool doCulling, double margin,
 double& out_t, double& out_u, double& out_v)
{
    constexpr double epsilon = std::numeric_limits<double>::epsilon();
    constexpr double GU_CULLING_EPSILON_RAY_TRIANGLE = epsilon * epsilon;
    
    const Vector3 edge1 = vertex1 - vertex0;
    const Vector3 edge2 = vertex2 - vertex0;
    const Vector3 pvec = rayDirection.cross(edge2);
    const double det = edge1.dot(pvec);

    if(doCulling){
        if(det < GU_CULLING_EPSILON_RAY_TRIANGLE){
            return false;
        }
        const Vector3 tvec = rayOrigin - vertex0;
        const double u = tvec.dot(pvec);
        const double marginCoeff = margin * det;
        const double uvlimit = -marginCoeff;
        const double uvlimit2 = det + marginCoeff;
        if(u < uvlimit || u > uvlimit2){
            return false;
        }
        const Vector3 qvec = tvec.cross(edge1);
        const double v = rayDirection.dot(qvec);
        if(v < uvlimit || (u + v) > uvlimit2){
            return false;
        }
        const double t = edge2.dot(qvec);
        const double inv_det = 1.0f / det;
        out_t = t * inv_det;
        out_u = u * inv_det;
        out_v = v * inv_det;

    } else {
        if(std::abs(det) < GU_CULLING_EPSILON_RAY_TRIANGLE){
            return false;
        }
        const double inv_det = 1.0f / det;
        const Vector3 tvec = rayOrigin - vertex0;
        const double u = tvec.dot(pvec) * inv_det;
        if(u < -margin || u > 1.0f + margin){
            return false;
        }
        const Vector3 qvec = tvec.cross(edge1);
        const double v = rayDirection.dot(qvec) * inv_det;
        if(v < -margin || ( u + v) > 1.0f + margin){
            return false;
        }
        const double t = edge2.dot(qvec) * inv_det;
        out_t = t;
        out_u = u;
        out_v = v;
    }
    
    return true;
}


bool ScenePointSelectionMode::Impl::findPointedTriangleVertex
(SgMesh* mesh, const Affine3& T, SceneWidgetEvent* event, int& out_index)
{
    bool found = false;
    const Affine3 T_inv = T.inverse();
    const Vector3 point = event->point();
    const Vector3f localPoint = (T_inv * point).cast<float>();
    float minDistance = std::numeric_limits<float>::max();
    Vector3f minDistanceVertex;
    vector<int> minDistanceIndices;

    auto& vertices = *mesh->vertices();
    auto& vertexIndices = mesh->triangleVertices();
    const int n = vertexIndices.size();
    for(int i=0; i < n; ++i){
        auto& vertex = vertices[vertexIndices[i]]; 
        float distance = (vertex - localPoint).norm();
        if(distance < minDistance){
            minDistance = distance;
            minDistanceVertex = vertex;
            minDistanceIndices.clear();
            minDistanceIndices.push_back(i);
        } else if(distance == minDistance){
            if(vertex == minDistanceVertex){
                minDistanceIndices.push_back(i);
            }
        }

    }
    if(!minDistanceIndices.empty()){
        Vector3 v = T * minDistanceVertex.cast<double>();
        double distance = (v - point).norm();
        //! \todo The distance threshold should be constant in the viewport coordinate
        if(distance < 0.01){
            Vector3 origin0, direction0;
            if(event->getRay(origin0, direction0)){
                const Vector3 origin = T_inv * origin0;
                const Vector3 direction = T_inv.linear() * direction0;
                out_index = minDistanceIndices.front();
                double minRayDistance = std::numeric_limits<double>::max();
                for(auto& index : minDistanceIndices){
                    int triangleIndex = index / 3;
                    auto triangle = mesh->triangle(triangleIndex);
                    double t, u, v;
                    bool intersected =
                        checkRayTraiangleIntersection(
                            origin, direction,
                            vertices[triangle[0]].cast<Vector3::Scalar>(),
                            vertices[triangle[1]].cast<Vector3::Scalar>(),
                            vertices[triangle[2]].cast<Vector3::Scalar>(),
                            false, 0.0, t, u, v);
                    if(intersected){
                        if(t < minRayDistance){
                            minRayDistance = t;
                            out_index = index;
                        }
                    }
                }
            }
            found = true;            
        }
    }
    return found;
}


void ScenePointSelectionMode::Impl::setHighlightedPoint
(const SgNodePath& path, SgMesh* mesh, const Affine3& T, int triangleVertexIndex)
{
    highlightedPoint = new ScenePointSelectionMode::PointInfo;
    int vertexIndex = mesh->triangleVertices()[triangleVertexIndex];
    auto vertex = mesh->vertices()->at(vertexIndex);
    Vector3f v = (T * vertex.cast<double>()).cast<float>();
    highlightedPoint->path_ = make_shared<SgNodePath>(path);
    highlightedPoint->vertexIndex_ = vertexIndex;
    highlightedPoint->triangleVertexIndex_ = triangleVertexIndex;
    highlightedPoint->position_ = v;

    if(mesh->hasNormals()){
        auto& normals = *mesh->normals();
        int normalIndex = -1;
        if(mesh->hasNormalIndices()){
            normalIndex = mesh->normalIndices()[triangleVertexIndex];
        } else if(vertexIndex < normals.size()){
            normalIndex = vertexIndex;
        }
        if(normalIndex >= 0){
            highlightedPoint->normal_ = (T.linear() * normals[normalIndex].cast<double>()).cast<float>();
            highlightedPoint->hasNormal_ = true;
        }
    }

    highlightedPointPlot->resetPoint(highlightedPoint);
}


void ScenePointSelectionMode::Impl::clearHighlightedPoint()
{
    highlightedPoint.reset();
    highlightedPointPlot->clearPoints(true);
}


void ScenePointSelectionMode::onPointerLeaveEvent(SceneWidgetEvent* event)
{
    impl->clearHighlightedPoint();
}


bool ScenePointSelectionMode::onButtonPressEvent(SceneWidgetEvent* event)
{
    return impl->onButtonPressEvent(event);
}


bool ScenePointSelectionMode::Impl::onButtonPressEvent(SceneWidgetEvent* event)
{
    bool isTargetNode = checkIfPointingTargetNode(event);

    if(event->button() == Qt::LeftButton){
        bool isPointSelectionUpdated = false;
        vector<PointInfoPtr> additionalSelectedPoints;
        if(isTargetNode && highlightedPoint){
            bool removed = false;
            shared_ptr<SgNodePath> sharedPath;
            if(!(event->modifiers() & Qt::ControlModifier)){
                selectedPoints.clear();
            } else {
                for(auto it = selectedPoints.begin(); it != selectedPoints.end(); ++it){
                    PointInfo* point = *it;
                    if(point->hasSameVertexWith(*highlightedPoint)){
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
                additionalSelectedPoints.push_back(highlightedPoint);
            }
            isPointSelectionUpdated = true;

        } else { // !isTargetNode || !highlightedPoint
            if(!(event->modifiers() & Qt::ControlModifier)){
                if(!selectedPoints.empty()){
                    selectedPoints.clear();
                    isPointSelectionUpdated = true;
                }
            }
        }
        if(isPointSelectionUpdated){
            selectedPointPlot->updatePoints(selectedPoints);
            if(!additionalSelectedPoints.empty()){
                sigPointSelectionAdded(additionalSelectedPoints);
            }
        }
    } else if(isTargetNode && event->button() == Qt::RightButton){
        event->sceneWidget()->showContextMenuAtPointerPosition();
    }
    
    return isTargetNode;
}


bool ScenePointSelectionMode::onButtonReleaseEvent(SceneWidgetEvent*)
{
    return false;
}


bool ScenePointSelectionMode::onDoubleClickEvent(SceneWidgetEvent*)
{
    return false;
}


bool ScenePointSelectionMode::onKeyPressEvent(SceneWidgetEvent*)
{
    return false;
}


bool ScenePointSelectionMode::onKeyReleaseEvent(SceneWidgetEvent*)
{
    return false;
}


bool ScenePointSelectionMode::onContextMenuRequest(SceneWidgetEvent* event, MenuManager*)
{
    return impl->checkIfPointingTargetNode(event);
}
