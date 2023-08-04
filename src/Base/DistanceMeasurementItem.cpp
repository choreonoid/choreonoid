#include "DistanceMeasurementItem.h"
#include "ItemManager.h"
#include "MenuManager.h"
#include "GeometryMeasurementTracker.h"
#include "ItemTreeView.h"
#include "DistanceMeasurementDialog.h"
#include "DisplayValueFormat.h"
#include "PutPropertyFunction.h"
#include "Archive.h"
#include "LazyCaller.h"
#include <cnoid/SceneDrawables>
#include <cnoid/SceneNodeClassRegistry>
#include <cnoid/SceneRenderer>
#include <cnoid/CollisionDetector>
#include <cnoid/ThreadPool>
#include <cnoid/MathUtil>
#include <cnoid/EigenUtil>
#include <cnoid/EigenArchive>
#include <cnoid/stdx/optional>
#include <fmt/format.h>
#include <vector>
#include "gettext.h"

using namespace std;
using namespace cnoid;
using fmt::format;

namespace {

class TargetInfo : public Referenced
{
public:
    Item* item;
    ScopedConnection itemConnection;
    GeometryMeasurementTrackerPtr tracker;
    ScopedConnection trackerConnection;
    vector<stdx::optional<CollisionDetector::GeometryHandle>> geometryHandles;
    bool isShortestDistanceFixedPoint;
    Vector3 measurementPoint;

    GeometryMeasurementTracker* getOrCreateTracker(){
        if(!tracker){
            tracker = GeometryMeasurementTracker::createTracker(item);
        }
        return tracker;
    }
};

typedef ref_ptr<TargetInfo> TargetInfoPtr;

class ViewportText : public SgViewportOverlay
{
public:
    ViewportText();
    void setMeasurementData(const Vector3& p1, const Vector3& p2, double distance);
    void setColor(const Vector3f& color);
    void render(SceneRenderer* renderer);
    virtual void calcViewVolume(double viewportWidth, double viewportHeight, ViewVolume& io_volume) override;

    SgPosTransformPtr distanceTextTransform;
    SgTextPtr distanceText;
    DisplayValueFormat* displayValueFormat;
    string distanceDisplayFormat;
    double textHeight;
    Vector3 p1;
    Vector3 p2;
};

typedef ref_ptr<ViewportText> ViewportTextPtr;

struct ViewportTextRegistration {
    ViewportTextRegistration(){
        SceneNodeClassRegistry::instance().registerClass<ViewportText, SgViewportOverlay>();
        SceneRenderer::addExtension(
            [](SceneRenderer* renderer){
                renderer->renderingFunctions()->setFunction<ViewportText>(
                    [renderer](SgNode* node){
                        static_cast<ViewportText*>(node)->render(renderer);
                    });
            });
    }
};

}

namespace cnoid {

class DistanceMeasurementItem::Impl
{
public:
    DistanceMeasurementItem* self;
    
    TargetInfoPtr targetInfos[2];
    bool isMeasurementActive;
    bool isShortestDistanceMode;
    bool isShortestDistanceFixOneSideMode;
    int shortestDistanceFixedSide;
    Signal<void()> sigMeasurementConfigurationChanged;

    CollisionDetectorPtr collisionDetector;
    CollisionDetectorDistanceAPI* collisionDetectorDistanceAPI;
    typedef CollisionDetector::GeometryHandle GeometryHandle;
    vector<std::pair<GeometryHandle, GeometryHandle>> handlePairs;
    unique_ptr<ThreadPool> threadPool;
    LazyCaller calcDistanceLater;
    double distance;
    Signal<void(bool isValid)> sigDistanceUpdated;
    
    Vector3f markerColor;
    int distanceLineWidth;
    bool isDistanceLineOverlayEnabled;
    SgGroupPtr distanceMarker;
    SgOverlayPtr distanceLineOverlay;
    SgLineSetPtr distanceLine;
    ViewportTextPtr viewportDistanceText;
    SgUpdate sgUpdate;
    bool isDistanceMarkerConnected;

    Impl(DistanceMeasurementItem* self);
    Impl(DistanceMeasurementItem* self, const Impl& org);
    TargetInfo* setTargetItem(int which, Item* item, GeometryMeasurementTracker* tracker);
    void onTargetItemDisconnectedFromRoot(int which);
    void setShortestDistanceMode(bool on);
    void setShortestDistanceFixOneSideMode(bool on);
    void setShortestDistanceFixedSide(int which);
    bool startMeasurement();
    void stopMeasurement();
    void onGeometryChanged(int which);
    void initializeCollisionDetector();
    void updateCollisionDetectionPositions(int which);
    void calcShortestDistance();
    void calcDistance();
    void updateDistance(double distance, const Vector3& p0, const Vector3& p1);
    void createDistanceMarker();
    void updateDistanceMarker();
    void setDistanceLineOverlayEnabled(bool on);
    void setDistanceMarkerColor(const Vector3f& color);
    void setDistanceLineWidth(int width);
    void doPutProperties(PutPropertyFunction& putProperty);
    bool store(Archive& archive);
    bool restore(const Archive& archive);
    void restoreItems(const Archive& archive);
};

}


void DistanceMeasurementItem::initializeClass(ExtensionManager* ext)
{
    auto& im = ext->itemManager();
    im.registerClass<DistanceMeasurementItem>(N_("DistanceMeasurementItem"));

    ItemTreeView::customizeContextMenu<DistanceMeasurementItem>(
        [](DistanceMeasurementItem* item, MenuManager& menuManager, ItemFunctionDispatcher menuFunction){
            menuManager.addItem(_("Configuration"))->sigTriggered().connect(
                [item](){ DistanceMeasurementDialog::instance()->show(item); });
            menuManager.addSeparator();
            menuFunction.dispatchAs<Item>(item);
        });
}


DistanceMeasurementItem::DistanceMeasurementItem()
{
    impl = new Impl(this);
}


DistanceMeasurementItem::DistanceMeasurementItem(const DistanceMeasurementItem& org)
    : Item(org)
{
    impl = new Impl(this, *org.impl);
}


DistanceMeasurementItem::~DistanceMeasurementItem()
{
    delete impl;
}


Item* DistanceMeasurementItem::doCloneItem(CloneMap* /* cloneMap */) const
{
    return new DistanceMeasurementItem(*this);
}


DistanceMeasurementItem::Impl::Impl(DistanceMeasurementItem* self)
    : self(self),
      calcDistanceLater([this](){ calcDistance(); }, LazyCaller::LowPriority)
{
    isMeasurementActive = false;
    isShortestDistanceMode = false;
    isShortestDistanceFixOneSideMode = false;
    shortestDistanceFixedSide = 0;
    distance = 0.0;
    markerColor << 1.0f, 0.0f, 0.0f; // Red;
    distanceLineWidth = 2;
    isDistanceLineOverlayEnabled = true;
    isDistanceMarkerConnected = false;

    self->sigCheckToggled(LogicalSumOfAllChecks).connect(
        [this](bool on){
            if(on && !isMeasurementActive){
                startMeasurement();
            }
        });
}


DistanceMeasurementItem::Impl::Impl(DistanceMeasurementItem* self, const Impl& org)
    : Impl(self)
{
    isShortestDistanceMode = org.isShortestDistanceMode;
    isShortestDistanceFixOneSideMode = org.isShortestDistanceFixOneSideMode;
    shortestDistanceFixedSide = org.shortestDistanceFixedSide;
    markerColor = org.markerColor;
    distanceLineWidth = org.distanceLineWidth;
    isDistanceLineOverlayEnabled = org.isDistanceLineOverlayEnabled;

    for(int i=0; i < 2; ++i){
        if(auto info = org.targetInfos[i]){
            setTargetItem(i, info->item, nullptr);
        }
    }
}


void DistanceMeasurementItem::onDisconnectedFromRoot()
{
    impl->stopMeasurement();
}


bool DistanceMeasurementItem::isMeasurementActive() const
{
    return impl->isMeasurementActive;
}


void DistanceMeasurementItem::setTargetItem(int which, Item* item, GeometryMeasurementTracker* tracker)
{
    impl->setTargetItem(which, item, tracker);
}


TargetInfo* DistanceMeasurementItem::Impl::setTargetItem(int which, Item* item, GeometryMeasurementTracker* tracker)
{
    auto& info = targetInfos[which];

    if(!info || info->item != item || info->tracker != tracker){
        auto newInfo = new TargetInfo;
        newInfo->item = item;
        newInfo->itemConnection =
            item->sigDisconnectedFromRoot().connect(
                [this, which](){ onTargetItemDisconnectedFromRoot(which); });
        newInfo->tracker = tracker ? tracker : GeometryMeasurementTracker::createTracker(item);
        info = newInfo;
    }

    return info;
}


void DistanceMeasurementItem::Impl::onTargetItemDisconnectedFromRoot(int which)
{
    stopMeasurement();
    targetInfos[which].reset();
    self->notifyUpdate();
}


Item* DistanceMeasurementItem::targetItem(int which)
{
    auto& info = impl->targetInfos[which];
    return info ? info->item : nullptr;
}


GeometryMeasurementTracker* DistanceMeasurementItem::targetTracker(int which)
{
    auto& info = impl->targetInfos[which];
    return info ? info->tracker : nullptr;
}


bool DistanceMeasurementItem::hasValidTargetItems() const
{
    for(int i=0; i < 2; ++i){
        auto& info = impl->targetInfos[i];
        if(!info || !info->item || !info->tracker){
            return false;
        }
    }
    return true;
}


void DistanceMeasurementItem::setShortestDistanceMode(bool on)
{
    impl->setShortestDistanceMode(on);
}


void DistanceMeasurementItem::Impl::setShortestDistanceMode(bool on)
{
    if(on != isShortestDistanceMode){
        isShortestDistanceMode = on;
        initializeCollisionDetector();
    }
}


void DistanceMeasurementItem::setShortestDistanceFixOneSideMode(bool on)
{
    impl->setShortestDistanceFixOneSideMode(on);
}


void DistanceMeasurementItem::Impl::setShortestDistanceFixOneSideMode(bool on)
{
    if(on != isShortestDistanceFixOneSideMode){
        isShortestDistanceFixOneSideMode = on;
        initializeCollisionDetector();
    }
}


void DistanceMeasurementItem::setShortestDistanceFixedSide(int which)
{
    impl->setShortestDistanceFixedSide(which);
}


void DistanceMeasurementItem::Impl::setShortestDistanceFixedSide(int which)
{
    if(which != shortestDistanceFixedSide){
        shortestDistanceFixedSide = which;
        initializeCollisionDetector();
    }
}


bool DistanceMeasurementItem::isShortestDistanceMode() const
{
    return impl->isShortestDistanceMode;
}


bool DistanceMeasurementItem::isShortestDistanceFixOneSideMode() const
{
    return impl->isShortestDistanceFixOneSideMode;
}


int DistanceMeasurementItem::shortestDistanceFixedSide() const
{
    return impl->shortestDistanceFixedSide;
}


SignalProxy<void()> DistanceMeasurementItem::sigMeasurementConfigurationChanged()
{
    return impl->sigMeasurementConfigurationChanged;
}


void DistanceMeasurementItem::notifyMeasurmentConfigurationChange()
{
    impl->sigMeasurementConfigurationChanged();
    notifyUpdate();
}


bool DistanceMeasurementItem::startMeasurement()
{
    return impl->startMeasurement();
}


bool DistanceMeasurementItem::Impl::startMeasurement()
{
    if(isMeasurementActive){
        stopMeasurement();
    }

    int numValidItems = 0;
    for(int i=0; i < 2; ++i){
        auto& info = targetInfos[i];
        if(info && info->item && info->tracker){
            ++numValidItems;
        }
    }

    if(numValidItems < 2){
        updateDistanceMarker();
    } else {
        isMeasurementActive = true;

        if(isShortestDistanceMode){
            initializeCollisionDetector();
        }
        for(int i=0; i < 2; ++i){
            auto info = targetInfos[i];
            info->trackerConnection =
                info->tracker->sigGeometryChanged().connect(
                    [this, i](){ onGeometryChanged(i); });
        }
        
        calcDistance();
    }

    return isMeasurementActive;
}


void DistanceMeasurementItem::stopMeasurement()
{
    impl->stopMeasurement();
}


void DistanceMeasurementItem::Impl::stopMeasurement()
{
    if(isMeasurementActive){
        for(int i=0; i < 2; ++i){
            auto info = targetInfos[i];
            info->trackerConnection.disconnect();
            info->geometryHandles.clear();
        }
        if(collisionDetector){
            collisionDetector->clearGeometries();
        }
        isMeasurementActive = false;
        updateDistanceMarker();
    }
}


const Vector3& DistanceMeasurementItem::measurementPoint(int which) const
{
    if(auto info = impl->targetInfos[which]){
        return info->measurementPoint;
    }
    static Vector3 zero = Vector3::Zero();
    return zero;
}


double DistanceMeasurementItem::distance() const
{
    return impl->distance;
}


SignalProxy<void(bool isValid)> DistanceMeasurementItem::sigDistanceUpdated()
{
    return impl->sigDistanceUpdated;
}


void DistanceMeasurementItem::Impl::onGeometryChanged(int which)
{
    if(isShortestDistanceMode){
        updateCollisionDetectionPositions(which);
    }
    calcDistanceLater();
}


void DistanceMeasurementItem::Impl::initializeCollisionDetector()
{
    if(!isMeasurementActive){
        return;
    }
    
    if(!collisionDetector){
        int collisionDetectorIndex = CollisionDetector::factoryIndex("AISTCollisionDetector");
        collisionDetector = CollisionDetector::create(collisionDetectorIndex);
        collisionDetectorDistanceAPI = dynamic_cast<CollisionDetectorDistanceAPI*>(collisionDetector.get());
        if(!collisionDetectorDistanceAPI){
            collisionDetector.reset();
        }
    }

    collisionDetector->clearGeometries();
    
    for(int i=0; i < 2; ++i){
        auto& info = targetInfos[i];
        info->geometryHandles.clear();
        auto tracker = info->tracker;
        if(isShortestDistanceFixOneSideMode && shortestDistanceFixedSide == i){
            auto shape = new SgShape;
            auto mesh = shape->getOrCreateMesh();
            mesh->getOrCreateVertices()->push_back(Vector3f::Zero());
            mesh->addTriangle(0, 0, 0);
            auto handle = collisionDetector->addGeometry(shape);
            info->geometryHandles.push_back(handle);
            info->isShortestDistanceFixedPoint = true;
        } else {
            int n = tracker->getNumShapes();
            for(int i=0; i < n; ++i){
                auto handle = collisionDetector->addGeometry(tracker->getShape(i));
                info->geometryHandles.push_back(handle);
            }
            info->isShortestDistanceFixedPoint = false;
        }
        updateCollisionDetectionPositions(i);
    }

    handlePairs.clear();
    for(auto& handle1 : targetInfos[0]->geometryHandles){
        if(handle1){
            for(auto& handle2 : targetInfos[1]->geometryHandles){
                if(handle2){
                    handlePairs.emplace_back(*handle1, *handle2);
                }
            }
        }
    }
    int numThreads = std::min(static_cast<unsigned int>(handlePairs.size()), thread::hardware_concurrency());
    threadPool = make_unique<ThreadPool>(numThreads);
    
    collisionDetector->makeReady();
}


void DistanceMeasurementItem::Impl::updateCollisionDetectionPositions(int which)
{
    auto info = targetInfos[which];
    auto tracker = info->tracker;
    if(info->isShortestDistanceFixedPoint){
        Isometry3 T = Isometry3::Identity();
        T.translation() = tracker->getMeasurementPoint();
        collisionDetector->updatePosition(*info->geometryHandles.front(), T);
    } else {
        int n = tracker->getNumShapes();
        for(int i=0; i < n; ++i){
            if(auto handle = info->geometryHandles[i]){
                collisionDetector->updatePosition(*handle, tracker->getShapePosition(i));
            }
        }
    }
}


/**
   \todo Make the main process to find the shortest distance a background process
   to improve the response of dragging a target object.
*/
void DistanceMeasurementItem::Impl::calcShortestDistance()
{
    double shortestDistance = std::numeric_limits<double>::max();
    Vector3 p1s, p2s;
    std::mutex distanceMutex;
    bool detected = false;
    
    for(auto& handlePair : handlePairs){
        threadPool->start([&](){
            Vector3 p1, p2;
            auto distance = collisionDetectorDistanceAPI->detectDistance(
                handlePair.first, handlePair.second, p1, p2);
            {
                std::lock_guard<std::mutex> guard(distanceMutex);
                if(distance < shortestDistance){
                    shortestDistance = distance;
                    p1s = p1;
                    p2s = p2;
                    detected = true;
                }
            }
        });
    }
    threadPool->wait();

    if(detected){
        updateDistance(shortestDistance, p1s, p2s);
    }
    
    sigDistanceUpdated(detected);
    self->notifyUpdate();
}


void DistanceMeasurementItem::Impl::calcDistance()
{
    if(!isMeasurementActive){
        return;
    }

    if(isShortestDistanceMode){
        calcShortestDistance();

    } else {
        Vector3 p[2];
        for(int i=0; i < 2; ++i){
            auto& info = targetInfos[i];
            info->measurementPoint = info->tracker->getMeasurementPoint();
            p[i] = info->measurementPoint;
        }
        double d = Vector3(p[1] - p[0]).norm();
        updateDistance(d, p[0], p[1]);
        sigDistanceUpdated(true);
        self->notifyUpdate();
    }
}


void DistanceMeasurementItem::Impl::updateDistance(double distance, const Vector3& p0, const Vector3& p1)
{
    this->distance = distance;
    
    targetInfos[0]->measurementPoint = p0;
    targetInfos[1]->measurementPoint = p1;
    
    updateDistanceMarker();
}


SgNode* DistanceMeasurementItem::getScene()
{
    if(!impl->distanceMarker){
        impl->createDistanceMarker();
    }
    return impl->distanceMarker;
}


void DistanceMeasurementItem::Impl::createDistanceMarker()
{
    static ViewportTextRegistration registration;
    
    distanceLine = new SgLineSet;
    distanceLine->getOrCreateVertices(2);
    distanceLine->addLine(0, 1);
    distanceLine->setLineWidth(distanceLineWidth);

    viewportDistanceText = new ViewportText;

    distanceMarker = new SgGroup;
    distanceMarker->setAttribute(SgObject::MetaScene);
    distanceLineOverlay = new SgOverlay;
    distanceLineOverlay->addChild(distanceLine);

    setDistanceMarkerColor(markerColor);

    isDistanceMarkerConnected = false;
    updateDistanceMarker();
}


void DistanceMeasurementItem::Impl::updateDistanceMarker()
{
    if(!distanceMarker){
        return;
    }
    
    if(isMeasurementActive){
        sgUpdate.setAction(SgUpdate::GeometryModified);
        if(!isDistanceMarkerConnected){
            distanceMarker->addChild(viewportDistanceText);
            if(isDistanceLineOverlayEnabled){
                distanceMarker->addChild(distanceLineOverlay);
            } else {
                distanceMarker->addChild(distanceLine);
            }
            sgUpdate.addAction(SgUpdate::Added);
            isDistanceMarkerConnected = true;
        }
        auto& vertices = *distanceLine->vertices();
        const auto& p0 = targetInfos[0]->measurementPoint;
        const auto& p1 = targetInfos[1]->measurementPoint;
        vertices[0] = p0.cast<Vector3f::Scalar>();
        vertices[1] = p1.cast<Vector3f::Scalar>();
        viewportDistanceText->setMeasurementData(p0, p1, distance);
        vertices.notifyUpdate(sgUpdate);
        
    } else {
        if(isDistanceMarkerConnected){
            distanceMarker->clearChildren();
            isDistanceMarkerConnected = false;
            distanceMarker->notifyUpdate(sgUpdate.withAction(SgUpdate::Removed));
        }
    }
}


void DistanceMeasurementItem::setDistanceLineOverlayEnabled(bool on)
{
    impl->setDistanceLineOverlayEnabled(on);
}


void DistanceMeasurementItem::Impl::setDistanceLineOverlayEnabled(bool on)
{
    if(on != isDistanceLineOverlayEnabled){
        isDistanceLineOverlayEnabled = on;
        if(distanceMarker && isMeasurementActive){
            if(on){
                distanceMarker->removeChild(distanceLine);
                distanceMarker->addChildOnce(distanceLineOverlay, sgUpdate);
            } else {
                distanceMarker->removeChild(distanceLineOverlay);
                distanceMarker->addChildOnce(distanceLine, sgUpdate);
            }
        }
    }
}


bool DistanceMeasurementItem::isDistanceLineOverlayEnabled() const
{
    return impl->isDistanceLineOverlayEnabled;
}


void DistanceMeasurementItem::setDistanceMarkerColor(const Vector3f& color)
{
    impl->setDistanceMarkerColor(color);
}


void DistanceMeasurementItem::Impl::setDistanceMarkerColor(const Vector3f& color)
{
    markerColor = color;

    if(distanceMarker){
        viewportDistanceText->setColor(color);
        auto material = distanceLine->getOrCreateMaterial();
        material->setDiffuseColor(color);
        material->notifyUpdate(sgUpdate.withAction(SgUpdate::AppearanceModified));
    }
}


const Vector3f& DistanceMeasurementItem::distanceMarkerColor() const
{
    return impl->markerColor;
}


void DistanceMeasurementItem::setDistanceLineWidth(int width)
{
    impl->setDistanceLineWidth(width);
}


void DistanceMeasurementItem::Impl::setDistanceLineWidth(int width)
{
    if(width != distanceLineWidth){
        distanceLineWidth = width;
        if(distanceLine){
            distanceLine->setLineWidth(width);
            distanceLine->notifyUpdate(sgUpdate.withAction(SgUpdate::Modified));
        }
    }
}


int DistanceMeasurementItem::distanceLineWidth() const
{
    return impl->distanceLineWidth;
}


void DistanceMeasurementItem::doPutProperties(PutPropertyFunction& putProperty)
{
    impl->doPutProperties(putProperty);
}


void DistanceMeasurementItem::Impl::doPutProperties(PutPropertyFunction& putProperty)
{
    string itemName;
    string subEntryName;
    for(int i=0; i < 2; ++i){
        itemName.clear();
        subEntryName.clear();
        if(auto info = targetInfos[i]){
            if(auto item = info->item){
                itemName = item->name();
                if(auto tracker = info->tracker){
                    int index = tracker->getCurrentSubEntryIndex();
                    subEntryName = tracker->getSubEntryName(index);
                }
            }
        }
        if(itemName.empty()){
            itemName = _("None");
        }
        putProperty(format(_("Item{0}"), i + 1), itemName);

        if(subEntryName.empty()){
            subEntryName = _("None");
        }
        putProperty(format(_("Item{0} sub entry"), i + 1), subEntryName);
    }
    
    putProperty(_("Active"), isMeasurementActive,
                [this](bool on){
                    if(on){
                        startMeasurement();
                    } else {
                        stopMeasurement();
                    }
                    sigMeasurementConfigurationChanged();
                    return true;
                });
                        
    putProperty(_("Distance"), DisplayValueFormat::instance()->toDisplayLength(distance));

    putProperty
        (_("Shortest distance mode"), isShortestDistanceMode,
         [this](bool on){
             setShortestDistanceMode(on);
             if(on){
                 calcDistance();
             }
             sigMeasurementConfigurationChanged();
             return true;
         });

    putProperty
        (_("Fix one side for shortest distance"), isShortestDistanceFixOneSideMode,
         [this](bool on){
             setShortestDistanceFixOneSideMode(on);
             if(isShortestDistanceMode){
                 calcDistance();
             }
             sigMeasurementConfigurationChanged();
             return true;
         });

    putProperty.min(0).max(1)
        (_("Fixed side"), shortestDistanceFixedSide,
         [this](int which){
             setShortestDistanceFixedSide(which);
             if(isShortestDistanceMode){
                 calcDistance();
             }
             sigMeasurementConfigurationChanged();
             return true;
         });

    putProperty
        (_("Overlay"), isDistanceLineOverlayEnabled,
           [this](bool on){
               setDistanceLineOverlayEnabled(on);
               sigMeasurementConfigurationChanged();
               return true;
           });

    putProperty
        (_("Color"), str(markerColor),
         [this](const string& value){
             Vector3f c;
             if(toVector3(value, c)){
                 setDistanceMarkerColor(c);
                 sigMeasurementConfigurationChanged();
                 return true;
             }
             return false;
         });
    
    putProperty.max(9)
        (_("Line width"), distanceLineWidth,
           [this](int width){
               setDistanceLineWidth(width);
               sigMeasurementConfigurationChanged();
               return true;
           });
}


bool DistanceMeasurementItem::store(Archive& archive)
{
    return impl->store(archive);
}


bool DistanceMeasurementItem::Impl::store(Archive& archive)
{
    for(int i=0; i < 2; ++i){
        if(auto info = targetInfos[i]){
            if(auto item = info->item){
                string itemKey = format("item{0}", i + 1);
                archive.writeItemId(itemKey, item);
                if(auto tracker = info->tracker){
                    int subEntryIndex = tracker->getCurrentSubEntryIndex();
                    if(subEntryIndex > 0){
                        auto subEntry = tracker->getSubEntryName(subEntryIndex);
                        if(!subEntry.empty()){
                            archive.write(itemKey + "_sub_entry", subEntry, DOUBLE_QUOTED);
                        } else {
                            archive.write(itemKey + "_sub_entry_index", subEntryIndex);
                        }
                    }
                }
            }
        }
    }
    archive.write("is_active", isMeasurementActive);
    archive.write("is_shortest_distance_mode", isShortestDistanceMode);
    if(isShortestDistanceMode && isShortestDistanceFixOneSideMode){
        archive.write("fix_side", shortestDistanceFixedSide);
    }
    write(archive, "color", markerColor);
    archive.write("width", distanceLineWidth);
    archive.write("overlay", isDistanceLineOverlayEnabled);
    
    return true;
}


bool DistanceMeasurementItem::restore(const Archive& archive)
{
    return impl->restore(archive);
}


bool DistanceMeasurementItem::Impl::restore(const Archive& archive)
{
    archive.read("is_shortest_distance_mode", isShortestDistanceMode);
    if(isShortestDistanceMode){
        isShortestDistanceFixOneSideMode = archive.read("fix_side", shortestDistanceFixedSide);
    }
    read(archive, "color", markerColor);
    archive.read("width", distanceLineWidth);
    archive.read("overlay", isDistanceLineOverlayEnabled);

    archive.addPostProcess([this, &archive]{ restoreItems(archive); });
    
    return true;
}


void DistanceMeasurementItem::Impl::restoreItems(const Archive& archive)
{
    bool restored = false;
    
    for(int i=0; i < 2; ++i){
        string itemKey = format("item{0}", i + 1);
        if(auto item = archive.findItem<Item>(itemKey)){
            auto info = setTargetItem(i, item, nullptr);
            int subEntryIndex = 0;
            if(!archive.read(itemKey + "_sub_entry_index", subEntryIndex)){
                string subEntryName;
                if(archive.read(itemKey + "_sub_entry", subEntryName)){
                    subEntryIndex = info->tracker->findSubEntryIndex(subEntryName);
                }
            }
            if(subEntryIndex > 0){
                info->tracker->setCurrentSubEntry(subEntryIndex);
            }
            restored = true;
        }
    }

    if(restored && (self->isChecked(LogicalSumOfAllChecks) || archive.get("is_active", false))){
        startMeasurement();
    } else if(restored){
        self->notifyUpdate();
    }
}


ViewportText::ViewportText()
    : SgViewportOverlay(findClassId<ViewportText>())
{
    displayValueFormat = DisplayValueFormat::instance();

    if(displayValueFormat->isMillimeter()){
        distanceDisplayFormat = format("{{0:.{0}f}}mm", displayValueFormat->lengthDecimals());
    } else {
        distanceDisplayFormat = format("{{0:.{0}f}}m", displayValueFormat->lengthDecimals());
    }
    
    textHeight = 20.0;
    distanceText = new SgText;
    distanceText->setTextHeight(textHeight);
    distanceTextTransform = new SgPosTransform;
    distanceTextTransform->addChild(distanceText);
    addChild(distanceTextTransform);
}


void ViewportText::setMeasurementData(const Vector3& p1, const Vector3& p2, double distance)
{
    this->p1 = p1;
    this->p2 = p2;
    double d = displayValueFormat->toDisplayLength(distance);
    distanceText->setText(format(distanceDisplayFormat, d).c_str());
    distanceText->notifyUpdate();
}


void ViewportText::setColor(const Vector3f& color)
{
    distanceText->setColor(color);
}


void ViewportText::render(SceneRenderer* renderer)
{
    Vector3 q1 = renderer->project(p1);
    Vector3 q2 = renderer->project(p2);
    Vector3 p = (q1 + q2) / 2.0;
    double x = q2.x() - q1.x();
    double y = q2.y() - q1.y();
    double theta = degree(atan2(y, x));
    if((theta > 0 && theta < 90.0) || theta < -90.0){
        p.y() -= textHeight;
    }
    distanceTextTransform->setTranslation(p);
    renderer->renderingFunctions()->dispatchAs<SgViewportOverlay>(this);    
}


void ViewportText::calcViewVolume(double viewportWidth, double viewportHeight, ViewVolume& io_volume)
{
    io_volume.left = 0;
    io_volume.right = viewportWidth;
    io_volume.bottom = 0;
    io_volume.top = viewportHeight;
    io_volume.zNear = 1.0;
    io_volume.zFar = -1.0;
}
