#include "ScenePointSelectionMode.h"
#include "SceneWidget.h"
#include <cnoid/SceneRenderer>
#include <cnoid/SceneDrawables>
#include <cnoid/SceneEffects>
#include <cnoid/SceneUtil>
#include <cnoid/SceneNodeClassRegistry>
#include <map>
#include <unordered_set>
#include "gettext.h"

using namespace std;
using namespace cnoid;

namespace {

typedef ScenePointSelectionMode::PointInfo PointInfo;
typedef ScenePointSelectionMode::PointInfoPtr PointInfoPtr;

class SceneWidgetInfo
{
public:
    SceneWidget* widget;
    bool isDuringPointSelection;
    int nodeDecorationId;
    QMetaObject::Connection connection;

    SceneWidgetInfo(){
        isDuringPointSelection = false;
        nodeDecorationId = 1; // temporary
    }
    ~SceneWidgetInfo(){
        widget->disconnect(connection);
    }
};

class FixedPixelSizeNormal : public SgPosTransform
{
public:
    FixedPixelSizeNormal(SgLineSet* normalLine);
};

class NormalSet : public SgGroup
{
public:
    SgLineSetPtr normalLine;

    NormalSet(SgMaterial* material);
    void addNormal(const Vector3& position, const Vector3& normal);    
};

typedef ref_ptr<NormalSet> NormalSetPtr;

class ViewportCrossMarkerSet : public SgViewportOverlay
{
public:
    SgLineSetPtr crossLine;

    struct CrossPosition {
        Vector3 position;
        SgPosTransformPtr viewportPosition;
        CrossPosition(const Vector3& position, SgPosTransform* viewportPosition)
            : position(position), viewportPosition(viewportPosition) { }
    };
    vector<CrossPosition> crossPositions;
    
    ViewportCrossMarkerSet(SgMaterial* material);
    bool clear();
    void addCross(const Vector3& position);
    void render(SceneRenderer* renderer);
    virtual void calcViewVolume(double viewportWidth, double viewportHeight, ViewVolume& io_volume) override;
};

typedef ref_ptr<ViewportCrossMarkerSet> ViewportCrossMarkerSetPtr;

struct ViewportCrossMarkerSetRegistration {
    ViewportCrossMarkerSetRegistration(){
        SceneNodeClassRegistry::instance().registerClass<ViewportCrossMarkerSet, SgViewportOverlay>();
        SceneRenderer::addExtension(
            [](SceneRenderer* renderer){
                renderer->renderingFunctions()->setFunction<ViewportCrossMarkerSet>(
                    [renderer](SgNode* node){
                        static_cast<ViewportCrossMarkerSet*>(node)->render(renderer);
                    });
            });
    }
};

class ScenePointPlot : public SgGroup
{
    SgPointSetPtr pointSet;
    ViewportCrossMarkerSetPtr crossMarkerSet;
    NormalSetPtr normalSet;
    SgMaterialPtr material_;
    SgUpdate update;
    bool isPointSetEnabled;
    bool isCrossMarkerSetEnabled;
    bool isNormalSetEnabled;

public:
    ScenePointPlot();
    SgMaterial* material() { return material_; }
    void updateVisualization(int subMode, bool isNormalDetectionEnabled);
    void setPointSetEnabled(bool on);
    void setCrossMarkerSetEnabled(bool on);
    void setNormalSetEnabled(bool on);
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
    int subMode;
    bool isNormalDetectionEnabled;
    bool isControlModifierEnabled;
    std::map<SceneWidget*, SceneWidgetInfo> sceneWidgetInfos;
    unordered_set<SgNodePtr> targetNodes;

    SgOverlayPtr pointOverlay;
    ScenePointPlotPtr highlightedPointPlot;
    ScenePointPlotPtr selectedPointPlot;
    PointInfoPtr highlightedPoint;
    std::vector<PointInfoPtr> selectedPoints;

    Signal<void(const std::vector<PointInfoPtr>& additionalPoints)> sigPointSelectionAdded;
    
    Impl(ScenePointSelectionMode* self);
    void setHighlightedPointColor(const Vector3f& color);
    void setSelectedPointColor(const Vector3f& color);
    void setupScenePointSelectionMode(SceneWidget* sceneWidget);
    void clearScenePointSelectionMode(SceneWidget* sceneWidget);
    bool checkIfPointingTargetNode(SceneWidgetEvent* event);
    bool findPointedTriangleVertex(SgMesh* mesh, const Affine3& T, SceneWidgetEvent* event, int& out_index);
    void setHighlightedPoint(const SgNodePath& path, const Vector3& point);
    void setHighlightedPoint(const SgNodePath& path, SgMesh* mesh, const Affine3& T, int vertexIndex);
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


FixedPixelSizeNormal::FixedPixelSizeNormal(SgLineSet* normalLine)
{
    auto fpsg = new SgFixedPixelSizeGroup;
    fpsg->setPixelSizeRatio(32.0f);
    fpsg->addChild(normalLine);
    addChild(fpsg);
}


NormalSet::NormalSet(SgMaterial* material)
{
    normalLine = new SgLineSet;
    normalLine->setMaterial(material);
    auto& vertices = *normalLine->getOrCreateVertices(2);
    vertices[0] = Vector3f::Zero();
    vertices[1] = Vector3f::UnitZ();
    normalLine->addLine(0, 1);
}


void NormalSet::addNormal(const Vector3& position, const Vector3& normal)
{
    auto normalNode = new FixedPixelSizeNormal(normalLine);
    normalNode->setTranslation(position);
    normalNode->setRotation(Quaternion::FromTwoVectors(Vector3::UnitZ(), normal));
    addChild(normalNode);
}


ViewportCrossMarkerSet::ViewportCrossMarkerSet(SgMaterial* material)
    : SgViewportOverlay(findClassId<ViewportCrossMarkerSet>())
{
    float length = 24.0f;
    crossLine = new SgLineSet;
    crossLine->setMaterial(material);
    auto& vertices = *crossLine->getOrCreateVertices(4);
    vertices[0] =  length * Vector3f::UnitX();
    vertices[1] = -length * Vector3f::UnitX();
    vertices[2] =  length * Vector3f::UnitY();
    vertices[3] = -length * Vector3f::UnitY();
    crossLine->addLine(0, 1);
    crossLine->addLine(2, 3);
}


bool ViewportCrossMarkerSet::clear()
{
    if(!crossPositions.empty()){
        clearChildren();
        crossPositions.clear();
        return true;
    }
    return false;
}


void ViewportCrossMarkerSet::addCross(const Vector3& position)
{
    auto viewportPosition = new SgPosTransform;
    viewportPosition->addChild(crossLine);
    addChild(viewportPosition);
    crossPositions.emplace_back(position, viewportPosition);
}


void ViewportCrossMarkerSet::render(SceneRenderer* renderer)
{
    for(auto& p : crossPositions){
        Vector3 pv = renderer->project(p.position);
        p.viewportPosition->setTranslation(pv);
    }
    renderer->renderingFunctions()->dispatchAs<SgViewportOverlay>(this);    
}


void ViewportCrossMarkerSet::calcViewVolume(double viewportWidth, double viewportHeight, ViewVolume& io_volume)
{
    io_volume.left = 0;
    io_volume.right = viewportWidth;
    io_volume.bottom = 0;
    io_volume.top = viewportHeight;
    io_volume.zNear = 1.0;
    io_volume.zFar = -1.0;
}


ScenePointPlot::ScenePointPlot()
{
    material_ = new SgMaterial;
    isPointSetEnabled = false;
    isCrossMarkerSetEnabled = false;
    isNormalSetEnabled = false;
}


void ScenePointPlot::updateVisualization(int subMode, bool isNormalDetectionEnabled)
{
    if(subMode == ScenePointSelectionMode::SurfacePointMode){
        setPointSetEnabled(isNormalDetectionEnabled);
        setCrossMarkerSetEnabled(!isNormalDetectionEnabled);
        setNormalSetEnabled(isNormalDetectionEnabled);
    } else { // MeshVertexMode
        setPointSetEnabled(true);
        setCrossMarkerSetEnabled(false);
        setNormalSetEnabled(isNormalDetectionEnabled);
    }
}


void ScenePointPlot::setPointSetEnabled(bool on)
{
    if(on != isPointSetEnabled){
        if(on){
            if(!pointSet){
                pointSet = new SgPointSet;
                pointSet->setPointSize(10.0);
                pointSet->setMaterial(material_);
                pointSet->getOrCreateVertices();
            }
            addChildOnce(pointSet);
        } else {
            if(pointSet){
                removeChild(pointSet);
            }
        }
        isPointSetEnabled = on;
    }
}


void ScenePointPlot::setCrossMarkerSetEnabled(bool on)
{
    if(on != isCrossMarkerSetEnabled){
        if(on){
            if(!crossMarkerSet){
                static ViewportCrossMarkerSetRegistration registration;
                crossMarkerSet = new ViewportCrossMarkerSet(material_);
            }
            addChildOnce(crossMarkerSet);
        } else {
            if(crossMarkerSet){
                removeChild(crossMarkerSet);
            }
        }
        isCrossMarkerSetEnabled = on;
    }
}


void ScenePointPlot::setNormalSetEnabled(bool on)
{
    if(on != isNormalSetEnabled){
        if(on){
            if(!normalSet){
                normalSet = new NormalSet(material_);
            }
            addChildOnce(normalSet);
        } else {
            if(normalSet){
                removeChild(normalSet);
            }
        }
        isNormalSetEnabled = on;
    }
}


void ScenePointPlot::clearPoints(bool doNotify)
{
    bool isUpdated = false;
    SgTmpUpdate update;

    if(isCrossMarkerSetEnabled){
        if(crossMarkerSet->clear()){
            isUpdated = true;
        }
    }
    
    if(isNormalSetEnabled){
        if(!normalSet->empty()){
            normalSet->clearChildren();
            isUpdated = true;
        }
    }
    
    if(isPointSetEnabled){
        auto points = pointSet->vertices();
        if(!points->empty()){
            points->clear();
            if(doNotify){
                points->notifyUpdate(update);
                doNotify = false;
            }
        }
    }
    
    if(doNotify && isUpdated){
        notifyUpdate(update);
    }
}


void ScenePointPlot::updatePoints(const std::vector<PointInfoPtr>& infos)
{
    clearPoints(false);

    for(auto& info : infos){
        addPoint(info, false);
    }

    SgTmpUpdate update;
    if(isPointSetEnabled){
        pointSet->vertices()->notifyUpdate(update);
    } else if(isCrossMarkerSetEnabled || isNormalSetEnabled){
        notifyUpdate(update);
    }
}


void ScenePointPlot::resetPoint(PointInfo* info)
{
    clearPoints(false);
    addPoint(info, true);
}


void ScenePointPlot::addPoint(PointInfo* info, bool doNotify)
{
    bool isUpdated = false;

    if(isCrossMarkerSetEnabled){
        crossMarkerSet->addCross(info->position());
        isUpdated = true;
    }
    if(isNormalSetEnabled && info->hasNormal()){
        normalSet->addNormal(info->position(), info->normal());
        isUpdated = true;
    }
    if(isPointSetEnabled){
        auto points = pointSet->vertices();
        points->push_back(info->position().cast<SgVertexArray::Scalar>());
        if(doNotify){
            points->notifyUpdate(update.withAction(SgUpdate::GeometryModified));
            doNotify = false;
        }
    }
    if(doNotify && isUpdated){
        notifyUpdate(update.withAction(SgUpdate::Added));
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
    subMode = SurfacePointMode;
    isNormalDetectionEnabled = false;
    isControlModifierEnabled = true;

    highlightedPointPlot = new ScenePointPlot;
    setHighlightedPointColor(Vector3f(1.0f, 1.0f, 0.0f)); // Yellow
    
    selectedPointPlot = new ScenePointPlot;
    setSelectedPointColor(Vector3f(1.0f, 0.0f, 0.0f)); // Red
    
    pointOverlay = new SgOverlay;
    pointOverlay->setAttribute(SgObject::MetaScene);
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


int ScenePointSelectionMode::customModeId() const
{
    return impl->modeId;
}


int ScenePointSelectionMode::subMode() const
{
    return impl->subMode;
}


void ScenePointSelectionMode::setSubMode(int mode)
{
    if(mode != impl->subMode){
        impl->subMode = mode;
        for(auto& kw : impl->sceneWidgetInfos){
            auto& info = kw.second;
            if(info.isDuringPointSelection){
                impl->setupScenePointSelectionMode(info.widget);
                info.widget->renderScene();
            }
        }
    }
}


void ScenePointSelectionMode::setNormalDetectionEnabled(bool on)
{
    if(on != impl->isNormalDetectionEnabled){
        impl->isNormalDetectionEnabled = on;
    }
}


void ScenePointSelectionMode::setControlModifierEnabled(bool on)
{
    impl->isControlModifierEnabled = on;
}


void ScenePointSelectionMode::updateTargetSceneNodes()
{
    for(auto& kw : impl->sceneWidgetInfos){
        auto& info = kw.second;
        if(info.isDuringPointSelection){
            impl->setupScenePointSelectionMode(info.widget);
            info.widget->renderScene();
        }
    }
}


const std::vector<PointInfoPtr>& ScenePointSelectionMode::selectedPoints() const
{
    return impl->selectedPoints;
}


void ScenePointSelectionMode::clearSelection()
{
    impl->selectedPoints.clear();
    impl->selectedPointPlot->clearPoints(true);
    impl->clearHighlightedPoint();
}


SignalProxy<void(const std::vector<PointInfoPtr>& additionalPoints)> ScenePointSelectionMode::sigPointSelectionAdded()
{
    return impl->sigPointSelectionAdded;
}


void ScenePointSelectionMode::onPointSelectionAdded(const std::vector<PointInfoPtr>& /* additionalPoints */)
{

}


ScenePointSelectionMode::PointInfo* ScenePointSelectionMode::highlightedPoint()
{
    return impl->highlightedPoint;
}


void ScenePointSelectionMode::setHighlightedPointColor(const Vector3f& color)
{
    impl->setHighlightedPointColor(color);
}


void ScenePointSelectionMode::Impl::setHighlightedPointColor(const Vector3f& color)
{
    highlightedPointPlot->material()->setDiffuseColor(color);
}


void ScenePointSelectionMode::setSelectedPointColor(const Vector3f& color)
{
    impl->setSelectedPointColor(color);
}


void ScenePointSelectionMode::Impl::setSelectedPointColor(const Vector3f& color)
{
    selectedPointPlot->material()->setDiffuseColor(color);
}


std::vector<SgNode*> ScenePointSelectionMode::getTargetSceneNodes(SceneWidget* /* sceneWidget */)
{
    return std::vector<SgNode*>();
}
    

void ScenePointSelectionMode::onSceneModeChanged(SceneWidgetEvent* event)
{
    auto sw = event->sceneWidget();
    int activeMode = sw->activeCustomMode();
    if(activeMode == impl->modeId && sw->isEditMode()){
        impl->setupScenePointSelectionMode(sw);
    } else {
        impl->clearScenePointSelectionMode(sw);
    }
}


#if 0
void ScenePointSelectionMode::onSelectionModeActivated(SceneWidgetEvent* /* event */)
{

}


void ScenePointSelectionMode::onSelectionModeDeactivated(SceneWidgetEvent* /* event */)
{

}
#endif


void ScenePointSelectionMode::Impl::setupScenePointSelectionMode(SceneWidget* sceneWidget)
{
    highlightedPointPlot->updateVisualization(subMode, isNormalDetectionEnabled);
    selectedPointPlot->updateVisualization(subMode, isNormalDetectionEnabled);
    
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

    info->isDuringPointSelection = true;

    int id = info->nodeDecorationId;
    auto renderer = sceneWidget->renderer();
    renderer->clearNodeDecorations(id);
    targetNodes.clear();
    for(auto& node : self->getTargetSceneNodes(sceneWidget)){
        targetNodes.insert(node);
        if(subMode == MeshVertexMode){
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
}


void ScenePointSelectionMode::Impl::clearScenePointSelectionMode(SceneWidget* sceneWidget)
{
    auto p = sceneWidgetInfos.find(sceneWidget);
    if(p != sceneWidgetInfos.end()){
        SceneWidgetInfo& info = p->second;;
        SgTmpUpdate update;
        sceneWidget->systemNodeGroup()->removeChild(pointOverlay, update);
        sceneWidget->renderer()->clearNodeDecorations(info.nodeDecorationId);
        info.isDuringPointSelection = false;
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
        if(impl->subMode == SurfacePointMode){
            impl->setHighlightedPoint(path, event->point());
            pointed = true;
        } else {
            if(auto shape = dynamic_cast<SgShape*>(path.back().get())){
                auto mesh = shape->mesh();
                Affine3 T = calcTotalTransform(path);
                int  pointedIndex;
                if(impl->findPointedTriangleVertex(mesh, T, event, pointedIndex)){
                    impl->setHighlightedPoint(path, mesh, T, pointedIndex);
                    pointed = true;
                }
            }
        }
    }
    if(!pointed){
        impl->clearHighlightedPoint();
    }

    return isTargetNode;
}


static bool checkRayTriangleIntersection
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
                        checkRayTriangleIntersection(
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


void ScenePointSelectionMode::Impl::setHighlightedPoint(const SgNodePath& path, const Vector3& point)
{
    highlightedPoint = new ScenePointSelectionMode::PointInfo;
    highlightedPoint->path_ = make_shared<SgNodePath>(path);
    highlightedPoint->vertexIndex_ = -1;
    highlightedPoint->triangleVertexIndex_ = -1;
    highlightedPoint->position_ = point;
    highlightedPoint->hasNormal_ = false;
    highlightedPointPlot->resetPoint(highlightedPoint);
}


void ScenePointSelectionMode::Impl::setHighlightedPoint
(const SgNodePath& path, SgMesh* mesh, const Affine3& T, int triangleVertexIndex)
{
    highlightedPoint = new ScenePointSelectionMode::PointInfo;
    int vertexIndex = mesh->triangleVertices()[triangleVertexIndex];
    auto vertex = mesh->vertices()->at(vertexIndex);
    Vector3 v = T * vertex.cast<double>();
    highlightedPoint->path_ = make_shared<SgNodePath>(path);
    highlightedPoint->vertexIndex_ = vertexIndex;
    highlightedPoint->triangleVertexIndex_ = triangleVertexIndex;
    highlightedPoint->position_ = v;

    if(!isNormalDetectionEnabled){
        highlightedPoint->hasNormal_ = false;

    } else if(mesh->hasNormals()){
        auto& normals = *mesh->normals();
        int normalIndex = -1;
        if(mesh->hasNormalIndices()){
            normalIndex = mesh->normalIndices()[triangleVertexIndex];
        } else if(vertexIndex < normals.size()){
            normalIndex = vertexIndex;
        }
        if(normalIndex >= 0){
            highlightedPoint->normal_ = T.linear() * normals[normalIndex].cast<double>();
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
            if(isControlModifierEnabled && !(event->modifiers() & Qt::ControlModifier)){
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
            if(isControlModifierEnabled && !(event->modifiers() & Qt::ControlModifier)){
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


bool ScenePointSelectionMode::onContextMenuRequest(SceneWidgetEvent* event)
{
    return impl->checkIfPointingTargetNode(event);
}
