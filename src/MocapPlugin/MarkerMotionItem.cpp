#include "MarkerMotionItem.h"
#include "MocapMappingItem.h"
#include <cnoid/MessageView>
#include <cnoid/TimeBar>
#include <cnoid/ItemManager>
#include <cnoid/Archive>
#include <cnoid/PutPropertyFunction>
#include <cnoid/EigenUtil>
#include <cnoid/EigenArchive>
#include <cnoid/SceneShape>
#include <cnoid/SceneCameras>
#include <cnoid/SceneWidget>
#include <cnoid/SceneWidgetEventHandler>
#include <cnoid/SceneRenderer>
#include <cnoid/SceneProjector>
#include <cnoid/MeshGenerator>
#include <fmt/format.h>
#include "gettext.h"

using namespace std;
using namespace cnoid;
using fmt::format;

namespace cnoid {

class SceneMarkerMotion : public SgPosTransform, public SceneWidgetEventHandler
{
public:
    MarkerMotionItem* item;
    MarkerMotionPtr motion;

    class Marker : public SgPosTransform
    {
    public:
        int index;
        SgMaterialPtr material;
    };
    typedef ref_ptr<Marker> MarkerPtr;

    SgGroupPtr markers;
    SgLineSetPtr edgeLineSet;

    enum { NO_DRAG, MARKER_DRAG, GLOBAL_OFFSET_DRAG } dragMode;
    MarkerPtr targetMarker;
    Vector3 orgPointerPos;
    Vector3 orgMarkerPosition;
    Vector3 orgGlobalOffset;
    ScenePlaneProjector projector;

    SceneMarkerMotion(MarkerMotionItem* item);

    void resetMarkers();
    void updateMarkerPositions();
    Marker* findPointedMarker(SceneWidgetEvent* event);
    bool setTargetMarker(Marker* marker, SceneWidgetEvent* event);

    virtual bool onButtonPressEvent(SceneWidgetEvent* event) override;
    virtual bool onButtonReleaseEvent(SceneWidgetEvent* event) override;
    virtual bool onPointerMoveEvent(SceneWidgetEvent* event) override;
    virtual void onPointerLeaveEvent(SceneWidgetEvent* event) override;
};

}


void MarkerMotionItem::initialize(ExtensionManager* ext)
{
    static bool initialized = false;
    if(!initialized){

        ItemManager& im = ext->itemManager();

        im.registerClass<MarkerMotionItem>(N_("MarkerMotionItem"));

        im.addLoaderAndSaver<MarkerMotionItem>(
            _("Marker Motion"), "MARKER-MOTION-YAML", "yaml",
            [](MarkerMotionItem* item, const std::string& filename, std::ostream& os, Item* parentItem){
                return item->loadStdYamlFormat(filename, os);
            },
            [](MarkerMotionItem* item, const std::string& filename, std::ostream& os, Item* parentItem){
                return item->saveAsStdYamlFormat(filename, os);
            });

        im.addLoader<MarkerMotionItem>(
            _("VPM format mocap file"), "MOCAP-VPM", "vpm",
            [](MarkerMotionItem* item, const std::string& filename, std::ostream& os, Item* parentItem){
                return item->loadVPM(filename, os); },
            ItemManager::PRIORITY_CONVERSION);

        im.addLoader<MarkerMotionItem>(
            _("TRC format mocap file"), "MOCAP-TRC", "trc",
            [](MarkerMotionItem* item, const std::string& filename, std::ostream& os, Item* parentItem){
                return item->loadTRC(filename, os); },
            ItemManager::PRIORITY_CONVERSION);

        initialized = true;
    }
}


MarkerMotionItem::MarkerMotionItem()
    : MultiVector3SeqItem(make_shared<MarkerMotion>()),
      motion_(static_pointer_cast<MarkerMotion>(seq()))
{
    setAttribute(Reloadable);    

    initInstance();
    
    isEdgeRenderingEnabled_ = true;
    color_ << 1.0, 1.0, 1.0;
    edgeWidth_ = 1.0;
}


MarkerMotionItem::MarkerMotionItem(const MarkerMotionItem& org)
    : MultiVector3SeqItem(org, make_shared<MarkerMotion>(*org.motion_)),
      motion_(static_pointer_cast<MarkerMotion>(seq())),
      isEdgeRenderingEnabled_(org.isEdgeRenderingEnabled_),
      color_(org.color_),
      edgeWidth_(org.edgeWidth_),
      currentMarkerPositions(org.currentMarkerPositions)
{
    initInstance();
}


void MarkerMotionItem::initInstance()
{
    currentFrame_ = -1;
    needToUpdateCurrentMarkerPositions = false;

    sigCheckToggled().connect([&](bool){ onItemCheckToggled(); });
}


MarkerMotionItem::~MarkerMotionItem()
{

}


void MarkerMotionItem::onConnectedToRoot()
{
    onItemCheckToggled();
}


void MarkerMotionItem::onDisconnectedFromRoot()
{
    timeBarConnection.disconnect();
}


void MarkerMotionItem::onItemCheckToggled()
{
    if(isChecked()){
        timeBarConnection =
            TimeBar::instance()->sigTimeChanged().connect(
                [&](double time){ return onTimeChanged(time); });
    } else {
        timeBarConnection.disconnect();
    }
}


bool MarkerMotionItem::onTimeChanged(double time)
{
    int numFrames = motion_->numFrames();
    int newFrame = motion_->frameOfTime(time);
    bool isValid = true;
    if(newFrame < 0){
        newFrame = 0;
        isValid = false;
    } else if(newFrame >= numFrames){
        newFrame = numFrames - 1;
        isValid = false;
    }
    if(newFrame != currentFrame_ || needToUpdateCurrentMarkerPositions){
        currentFrame_ = newFrame;
        MarkerMotion::Frame f = motion_->frame(currentFrame_);
        currentMarkerPositions.resize(f.size());
        for(int i=0; i < f.size(); ++i){
            currentMarkerPositions[i] = motion_->applyOffset(f[i]);
        }
        needToUpdateCurrentMarkerPositions = false;
        sigCurrentMarkerPositionsChanged_();
    }
    return isValid;
}


void MarkerMotionItem::requestMarkerPositionChange(int markerIndex, const Vector3& newPosition)
{
    sigMarkerPositionChangeRequest_(markerIndex, newPosition);
}


void MarkerMotionItem::notifyCurrentMarkerPositionsChanged()
{
    sigCurrentMarkerPositionsChanged_();
    needToUpdateCurrentMarkerPositions = true;
}


void MarkerMotionItem::onTreePathChanged()
{
    auto mocapMappingItem = findOwnerItem<MocapMappingItem>();
    if(mocapMappingItem){
        mocapMapping_ = mocapMappingItem->mocapMapping();
    } else {
        mocapMapping_ = nullptr;
    }
}


void MarkerMotionItem::notifyUpdate()
{
    needToUpdateCurrentMarkerPositions = true;
    onTimeChanged(TimeBar::instance()->time());
    Item::notifyUpdate();
}



bool MarkerMotionItem::loadStdYamlFormat(const std::string& filename, std::ostream& os)
{
    bool loaded = motion()->loadStdYAMLformat(filename);
    if(!motion()->seqMessage().empty()){
        os << motion()->seqMessage();
    }
    return loaded;
}


bool MarkerMotionItem::saveAsStdYamlFormat(const std::string& filename, std::ostream& os)
{
    bool saved = motion()->saveAsStdYAMLformat(filename);
    if(!motion()->seqMessage().empty()){
        os << motion()->seqMessage();
    }
    return saved;
}


bool MarkerMotionItem::loadVPM(const std::string& filename, std::ostream& os)
{
    bool loaded = motion()->loadVPM(filename);
    if(!motion()->seqMessage().empty()){
        os << motion()->seqMessage();
    }
    return loaded;
}


bool MarkerMotionItem::loadTRC(const std::string& filename, std::ostream& os)
{
    bool loaded = motion()->loadTRC(filename);
    if(!motion()->seqMessage().empty()){
        os << motion()->seqMessage();
    }
    return loaded;
}


Item* MarkerMotionItem::doCloneItem(CloneMap* /* cloneMap */) const
{
    return new MarkerMotionItem(*this);
}


bool MarkerMotionItem::onPositionOffsetChanged(const std::string& value)
{
    Vector3 p;
    if(toVector3(value, p)){
        motion_->setPositionOffset(p);
        return true;
    }
    return false;
}


void MarkerMotionItem::doPutProperties(PutPropertyFunction& putProperty)
{
    MultiVector3SeqItem::doPutProperties(putProperty);

    putProperty(_("Position offset"), str(motion_->positionOffset()),
                [&](const string& value){ return onPositionOffsetChanged(value); });
    putProperty(_("Edge rendering"), isEdgeRenderingEnabled_, changeProperty(isEdgeRenderingEnabled_));
    putProperty(_("Color"), str(color_), [&](const string& value){ return toVector3(value, color_); });
    putProperty(_("Edge width"), edgeWidth_, changeProperty(edgeWidth_));
}


bool MarkerMotionItem::store(Archive& archive)
{
    if(MultiVector3SeqItem::store(archive)){
        write(archive, "translation", motion_->positionOffset());
        write(archive, "color", color_);
        archive.write("show_edges", isEdgeRenderingEnabled_);
        archive.write("edge_width", edgeWidth_);
        return true;
    }
    return false;
}


bool MarkerMotionItem::restore(const Archive& archive)
{
    if(MultiVector3SeqItem::restore(archive)){
        read(archive, "color", color_);
        archive.read("show_edges", isEdgeRenderingEnabled_);
        archive.read("edge_width", edgeWidth_);
        Vector3 v;
        if(read(archive, { "translation", "positionOffset" }, v)){
            motion_->setPositionOffset(v);
        }
        return true;
    }
    return false;
}


SgNode* MarkerMotionItem::getScene()
{
    if(!scene_){
        if(currentMarkerPositions.size() < motion_->numMarkers()){
            needToUpdateCurrentMarkerPositions = true;
            onTimeChanged(TimeBar::instance()->time());
        }
        scene_ = new SceneMarkerMotion(this);
        scene_->sigGraphConnection().connect([&](bool on){ onSceneConnection(on); });
    }
    return scene_;
}


void MarkerMotionItem::onSceneConnection(bool on)
{
    if(!on){
        sceneConnections.disconnect();
    } else {
        sceneConnections.add(
            sigCurrentMarkerPositionsChanged().connect(
                [&](){ scene_->updateMarkerPositions(); }));
        sceneConnections.add(
            sigUpdated().connect(
                [&](){ scene_->resetMarkers(); }));

        onTimeChanged(TimeBar::instance()->time());
    }
}


SceneMarkerMotion::SceneMarkerMotion(MarkerMotionItem* item)
    : item(item),
      motion(item->motion())
{
    dragMode = NO_DRAG;
    markers = new SgGroup;
    addChild(markers);
    edgeLineSet = new SgLineSet;
    addChild(edgeLineSet);
    resetMarkers();
}


void SceneMarkerMotion::resetMarkers()
{
    MeshGenerator meshGenerator;
    SgMeshPtr sphere = meshGenerator.generateSphere(0.025);

    markers->clearChildren();

    for(int i=0; i < motion->numMarkers(); ++i){
        Marker* marker = new Marker;
        marker->index = i;
        SgShape* shape = new SgShape;
        shape->setMesh(sphere);
        SgMaterial* material = new SgMaterial;
        material->setDiffuseColor(item->color());
        shape->setMaterial(material);
        marker->material = material;
        marker->addChild(shape);
        markers->addChild(marker);
    }

    updateMarkerPositions();
}


void SceneMarkerMotion::updateMarkerPositions()
{
    // update marker positions
    const int numMarkers = motion->numMarkers();
    SgVertexArray& vertices = *edgeLineSet->getOrCreateVertices();
    vertices.resize(numMarkers);
    for(int i=0; i < numMarkers; ++i){
        Marker* marker = static_cast<Marker*>(markers->child(i));
        const Vector3& p = item->currentMarkerPosition(i);
        marker->setTranslation(p);
        vertices[i] = p.cast<Vector3f::Scalar>();
    }

    // update edge positions
    MocapMapping* mocapMapping = item->mocapMapping();
    if(mocapMapping && item->isEdgeRenderingEnabled()){
        edgeLineSet->clearLines();
        for(int i=0; i < mocapMapping->numMarkerEdges(); ++i){
            const MocapMapping::Edge& e = mocapMapping->markerEdge(i);
            int marker0 = motion->markerIndex(e.label[0]);
            int marker1 = motion->markerIndex(e.label[1]);
            if(marker0 >= 0 && marker1 >= 0){
                edgeLineSet->addLine(marker0, marker1);
            }
        }
    }

    edgeLineSet->getOrCreateMaterial()->setDiffuseColor(item->color());
    edgeLineSet->setLineWidth(item->edgeWidth());
        
    edgeLineSet->notifyUpdate();
}


SceneMarkerMotion::Marker* SceneMarkerMotion::findPointedMarker(SceneWidgetEvent* event)
{
    Marker* marker = nullptr;
    auto& path = event->nodePath();
    int markerIndex = -1;
    const int n = path.size();
    for(int i=0; i < n; ++i){
        marker = dynamic_cast<Marker*>(path[i].get());
        if(marker){
            break;
        }
    }
    return marker;
}


bool SceneMarkerMotion::setTargetMarker(Marker* marker, SceneWidgetEvent* event)
{
    if(marker == targetMarker){
        return false;
    }

    if(targetMarker){
        targetMarker->material->setDiffuseColor(item->color());
    }
    if(marker){
        marker->material->setDiffuseColor(Vector3f(1.0, 0.0, 0.0)); // set red
        event->updateIndicator(
            format("Marker: {0} of {1}", motion->markerLabel(marker->index), item->name()));
    } else {
        event->updateIndicator("");
    }

    targetMarker = marker;
    notifyUpdate();

    return true;
}


bool SceneMarkerMotion::onButtonPressEvent(SceneWidgetEvent* event)
{
    dragMode = NO_DRAG;
    Marker* marker = findPointedMarker(event);
    setTargetMarker(marker, event);

    if(marker){
        if(event->button() == Qt::LeftButton){
            dragMode = MARKER_DRAG;
            orgMarkerPosition = item->currentMarkerPosition(marker->index);

        } else if(event->button() == Qt::MiddleButton){
            dragMode = GLOBAL_OFFSET_DRAG;
            orgGlobalOffset = motion->positionOffset();
        }
        if(dragMode != NO_DRAG){
            orgPointerPos << event->point().x(), event->point().y(), event->point().z();
            auto T = event->sceneWidget()->renderer()->currentCameraPosition();
            const Vector3 direction = -SgCamera::direction(T);
            projector.setPlane(direction, event->point());
        }
    }
    return (marker);
}


bool SceneMarkerMotion::onButtonReleaseEvent(SceneWidgetEvent* event)
{
    dragMode = NO_DRAG;
    bool handled = targetMarker;
    setTargetMarker(findPointedMarker(event), event);
    return handled;
}


bool SceneMarkerMotion::onPointerMoveEvent(SceneWidgetEvent* event)
{
    bool handled = false;

    if(dragMode != NO_DRAG){
        Vector3 p;
        if(projector.project(event, p)){
            Vector3 dp = p - orgPointerPos;

            if(dragMode == MARKER_DRAG){
                item->requestMarkerPositionChange(targetMarker->index, orgMarkerPosition + dp);

            } else if(dragMode == GLOBAL_OFFSET_DRAG){
                motion->setPositionOffset(orgGlobalOffset + dp);
                item->notifyUpdate();
            }
        }
        handled = true;

    } else {
        Marker* marker = findPointedMarker(event);
        setTargetMarker(marker, event);
        handled = marker;
    }

    return handled;
}


void SceneMarkerMotion::onPointerLeaveEvent(SceneWidgetEvent* event)
{
    setTargetMarker(0, event);
}
