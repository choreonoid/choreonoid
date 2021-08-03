/*!
  @file
  @author Shin'ichiro Nakaoka
*/

#include "SensorVisualizerItem.h"
#include "BodyItem.h"
#include <cnoid/ItemManager>
#include <cnoid/ImageableItem>
#include <cnoid/PointSetItem>
#include <cnoid/RenderableItem>
#include <cnoid/BasicSensors>
#include <cnoid/RangeCamera>
#include <cnoid/RangeSensor>
#include <cnoid/SceneGraph>
#include <cnoid/SceneDrawables>
#include <cnoid/MeshGenerator>
#include <cnoid/ConnectionSet>
#include <cnoid/PutPropertyFunction>
#include <cnoid/Archive>
#include <cnoid/EigenArchive>
#include <cnoid/EigenUtil>
#include "gettext.h"

using namespace std;
using namespace cnoid;

namespace {

class ArrowMarker : public SgPosTransform
{
public:
    SgSwitchableGroupPtr sgroup;
    SgPosTransformPtr cylinderPosition;
    SgScaleTransformPtr cylinderScale;
    SgPosTransformPtr conePosition;
    SgMaterialPtr material;
    bool isVisible;
    
    static SgMeshPtr cylinderMesh;
    static SgMeshPtr coneMesh;
    
    ArrowMarker(SgMaterial* material)
        : material(material)
    {
        sgroup = new SgSwitchableGroup;
        
        if(!cylinderMesh){
            MeshGenerator meshGenerator;
            cylinderMesh = meshGenerator.generateCylinder(0.01, 1.0);
            coneMesh = meshGenerator.generateCone(0.03,  0.04);
        }

        auto cylinder = new SgShape;
        cylinder->setMesh(cylinderMesh);
        cylinder->setMaterial(material);
        cylinderScale = new SgScaleTransform;
        cylinderScale->addChild(cylinder);
        cylinderPosition = new SgPosTransform;
        cylinderPosition->addChild(cylinderScale);
        sgroup->addChild(cylinderPosition);

        auto cone = new SgShape;
        cone->setMesh(coneMesh);
        cone->setMaterial(material);
        conePosition = new SgPosTransform;
        conePosition->addChild(cone);
        sgroup->addChild(conePosition);

        addChild(sgroup);
        isVisible = true;

        setVector(Vector3::Zero(), 1.0, nullptr);
    }

    void setVector(const Vector3& v, double threshold, SgUpdateRef update)
    {
        double len = v.norm();

        if(len < threshold){
            if(isVisible){
                sgroup->setTurnedOn(false, update);
                isVisible = false;
            }
        } else {
            if(!isVisible){
                sgroup->setTurnedOn(true);
                isVisible = true;
            }
            cylinderScale->setScale(Vector3(1.0, len, 1.0));
            cylinderPosition->setTranslation(Vector3(0.0, len / 2.0, 0.0));
            conePosition->setTranslation(Vector3(0.0, len, 0.0));

            Vector3 axis = (Vector3::UnitY().cross(v)).normalized();
            double angle = acos(Vector3::UnitY().dot(v) / len);
            setRotation(AngleAxis(angle, axis));

            if(update){
                notifyUpdate(*update);
            }
        }
    }
};

SgMeshPtr ArrowMarker::cylinderMesh;
SgMeshPtr ArrowMarker::coneMesh;


typedef ref_ptr<ArrowMarker> ArrowMarkerPtr;

class SensorVisualizerItemBase
{
public:
    Item* item;
    BodyItem* bodyItem;
    ScopedConnection sigCheckToggledConnection;
    SgUpdate update;
    SensorVisualizerItemBase(Item* item);
    void setBodyItem(BodyItem* bodyItem);
    void updateVisualization();

    virtual void enableVisualization(bool on) = 0;
    virtual void doUpdateVisualization() = 0;
};


class Vector3SensorVisualizerItem : public Item, public SensorVisualizerItemBase, public RenderableItem
{
public:
    Vector3SensorVisualizerItem();

    virtual SgNode* getScene() override;
    virtual void enableVisualization(bool on) override;
    virtual void doUpdateVisualization() override;
    virtual void updateSensors() = 0;
    void updateSensorMarkerPositions(bool doNotify);
    void updateSensorMarkerVector(int index);
    virtual Vector3 getSensorMarkerVector(Device* sensor) = 0;
    virtual void doPutProperties(PutPropertyFunction& putProperty) override;
    virtual bool store(Archive& archive) override;
    virtual bool restore(const Archive& archive) override;

    SgGroupPtr scene;
    SgMaterialPtr material;
    Vector3f color;
    DeviceList<> sensors;
    vector<ArrowMarkerPtr> markers;
    Vector3 offset;
    double threshold;
    double visualRatio;
    ScopedConnectionSet connections;
};
    

class ForceSensorVisualizerItem : public Vector3SensorVisualizerItem
{
public:
    ForceSensorVisualizerItem();
    virtual void updateSensors() override;
    virtual Vector3 getSensorMarkerVector(Device* sensor) override;
};


class AccelerationSensorVisualizerItem : public Vector3SensorVisualizerItem
{
public:
    AccelerationSensorVisualizerItem();
    virtual void updateSensors() override;
    virtual Vector3 getSensorMarkerVector(Device* sensor) override;
};


class RateGyroSensorVisualizerItem : public Vector3SensorVisualizerItem
{
public:
    RateGyroSensorVisualizerItem();
    virtual void updateSensors() override;
    virtual Vector3 getSensorMarkerVector(Device* sensor) override;
};


class CameraImageVisualizerItem : public Item, public SensorVisualizerItemBase, public ImageableItem
{
public:
    CameraImageVisualizerItem();
    virtual const Image* getImage() override;
    virtual SignalProxy<void()> sigImageUpdated() override;
    void setBodyItem(BodyItem* bodyItem, Camera* camera);
    virtual void enableVisualization(bool on) override;
    virtual void doUpdateVisualization() override;

    CameraPtr camera;
    ScopedConnectionSet connections;
    std::shared_ptr<const Image> image;
    Signal<void()> sigImageUpdated_;
};


class PointCloudVisualizerItem : public PointSetItem, public SensorVisualizerItemBase
{
public:
    PointCloudVisualizerItem();
    void setBodyItem(BodyItem* bodyItem, RangeCamera* rangeCamera);
    virtual void enableVisualization(bool on) override;
    virtual void doUpdateVisualization() override;
    void updateSensorPosition(bool doNitfy);
    void updateRangeCameraState();

    RangeCameraPtr rangeCamera;
    ScopedConnectionSet connections;
};


class RangeSensorVisualizerItem : public PointSetItem, public SensorVisualizerItemBase
{
public:
    RangeSensorVisualizerItem();
    void setBodyItem(BodyItem* bodyItem, RangeSensor* rangeSensor);
    virtual void enableVisualization(bool on) override;
    virtual void doUpdateVisualization() override;
    void updateSensorPosition(bool doNitfy);
    void updateRangeSensorState();

    RangeSensorPtr rangeSensor;
    ScopedConnectionSet connections;
};

}

namespace cnoid {

class SensorVisualizerItem::Impl
{
public:
    SensorVisualizerItem* self;
    BodyItem* bodyItem;
    vector<Item*> subItems;
    vector<ItemPtr> restoredSubItems;
    SgUpdate update;

    Impl(SensorVisualizerItem* self);

    template<class ItemType, class SensorType>
    void addSensorVisualizerItem(Body* body, int& itemIndex);

    template<class ItemType, class SensorType, bool isCamera>
    void addVisionSensorVisualizerItem(Body* body, int& itemIndex);
    
    void onTreePathChanged();
};

}


void SensorVisualizerItem::initializeClass(ExtensionManager* ext)
{
    ItemManager& im = ext->itemManager();

    im.registerClass<SensorVisualizerItem>(N_("SensorVisualizerItem"));
    im.registerClass<ForceSensorVisualizerItem>(N_("ForceSensorVisualizerItem"));
    im.registerClass<AccelerationSensorVisualizerItem>(N_("AccelerationSensorVisualizerItem"));
    im.registerClass<RateGyroSensorVisualizerItem>(N_("RateGyroSensorVisualizerItem"));
    im.registerClass<PointCloudVisualizerItem, PointSetItem>(N_("PointCloudVisualizerItem"));
    im.registerClass<RangeSensorVisualizerItem, PointSetItem>(N_("RangeSensorVisualizerItem"));
    im.registerClass<CameraImageVisualizerItem>(N_("CameraImageVisualizerItem"));

    im.addCreationPanel<SensorVisualizerItem>();

    // For reading old project files
    im.addAlias<SensorVisualizerItem>("SensorVisualizer", "Body");
    im.addAlias<ForceSensorVisualizerItem>("ForceSensorVisualizer", "Body");
    im.addAlias<PointCloudVisualizerItem>("PointCloudVisualizer", "Body");
    im.addAlias<RangeSensorVisualizerItem>("RangeSensorVisualizer", "Body");
    im.addAlias<CameraImageVisualizerItem>("CameraImageVisualizer", "Body");
}


SensorVisualizerItem::SensorVisualizerItem()
{
    impl = new Impl(this);
}


SensorVisualizerItem::Impl::Impl(SensorVisualizerItem* self)
    : self(self)
{

}


SensorVisualizerItem::SensorVisualizerItem(const SensorVisualizerItem& org)
    : Item(org)
{
    impl = new Impl(this);
}


SensorVisualizerItem::~SensorVisualizerItem()
{
    delete impl;
}


Item* SensorVisualizerItem::doDuplicate() const
{
    return new SensorVisualizerItem(*this);
}


void SensorVisualizerItem::onTreePathChanged()
{
    if(parentItem()){
        impl->onTreePathChanged();
    }
}


template<class ItemType, class SensorType>
void SensorVisualizerItem::Impl::addSensorVisualizerItem(Body* body, int& itemIndex)
{
    auto sensors = body->devices<SensorType>();
    if(!sensors.empty()){
        ItemType* item = nullptr;
        if(itemIndex < static_cast<int>(restoredSubItems.size())){
            item = dynamic_cast<ItemType*>(restoredSubItems[itemIndex].get());
            if(item){
                ++itemIndex;
            }
        }
        if(!item){
            item = new ItemType;;
        }
        item->setBodyItem(bodyItem);
        self->addSubItem(item);
        subItems.push_back(item);
    }
}


template<class ItemType, class SensorType, bool isCamera>
void SensorVisualizerItem::Impl::addVisionSensorVisualizerItem(Body* body, int& itemIndex)
{
    auto sensors = body->devices<SensorType>();
    for(size_t i=0; i < sensors.size(); ++i){
        if(isCamera){
            if(reinterpret_cast<Camera*>(sensors[i].get())->imageType() == Camera::NO_IMAGE){
                continue;
            }
        }
        ItemType* item = nullptr;
        if(itemIndex < static_cast<int>(restoredSubItems.size())){
            item = dynamic_cast<ItemType*>(restoredSubItems[itemIndex].get());
            if(item){
                ++itemIndex;
            }
        }
        if(!item){
            item = new ItemType;;
        }
        item->setBodyItem(bodyItem, sensors[i]);
        self->addSubItem(item);
        subItems.push_back(item);
    }
}


void SensorVisualizerItem::Impl::onTreePathChanged()
{
    BodyItem* newBodyItem = self->findOwnerItem<BodyItem>();

    if(newBodyItem != bodyItem){
        
        bodyItem = newBodyItem;
        for(size_t i=0; i < subItems.size(); ++i){
            subItems[i]->removeFromParentItem();
        }
        subItems.clear();

        if(bodyItem){
            auto body = bodyItem->body();
            int itemIndex = 0;
            addSensorVisualizerItem<ForceSensorVisualizerItem, ForceSensor>(body, itemIndex);
            addSensorVisualizerItem<AccelerationSensorVisualizerItem, AccelerationSensor>(body, itemIndex);
            addSensorVisualizerItem<RateGyroSensorVisualizerItem, RateGyroSensor>(body, itemIndex);
            addVisionSensorVisualizerItem<CameraImageVisualizerItem, Camera, true>(body, itemIndex);
            addVisionSensorVisualizerItem<PointCloudVisualizerItem, RangeCamera, false>(body, itemIndex);
            addVisionSensorVisualizerItem<RangeSensorVisualizerItem, RangeSensor, false>(body, itemIndex);
        }

        restoredSubItems.clear();
    }
}

void SensorVisualizerItem::onDisconnectedFromRoot()
{
    for(size_t i=0; i < impl->subItems.size(); i++){
        impl->subItems[i]->removeFromParentItem();
    }
    impl->subItems.clear();
}


bool SensorVisualizerItem::store(Archive& archive)
{
    ListingPtr subItems = new Listing;

    for(size_t i=0; i < impl->subItems.size(); i++){
        Item* item = impl->subItems[i];
        string pluginName, className;
        ItemManager::getClassIdentifier(item, pluginName, className);

        ArchivePtr subArchive = new Archive;
        subArchive->write("class", className);
        subArchive->write("name", item->name());
        if(item->isSelected()){
            subArchive->write("is_selected", true);
        }
        if(item->isChecked()){
            subArchive->write("is_checked", true);
        }
        item->store(*subArchive);

        subItems->append(subArchive);
    }

    archive.insert("sub_items", subItems);

    return true;
}


bool SensorVisualizerItem::restore(const Archive& archive)
{
    impl->restoredSubItems.clear();

    ListingPtr subItems = archive.findListing({ "sub_items", "subItems" });
    if(subItems->isValid()){
        for(int i=0; i < subItems->size(); ++i){
            auto subArchive = archive.subArchive(subItems->at(i)->toMapping());
            string className, itemName;
            subArchive->read("class", className);
            subArchive->read("name", itemName);
            if(ItemPtr item = ItemManager::createItem("Body", className)){
                item->setName(itemName);
                item->restore(*subArchive);
                if(subArchive->get("is_selected", false)){
                    item->setSelected(true);
                }
                if(subArchive->get("is_checked", false)){
                    item->setChecked(true);
                }
                impl->restoredSubItems.push_back(item);
            }
        }
    }
    return true;
}


SensorVisualizerItemBase::SensorVisualizerItemBase(Item* item)
    : item(item),
      bodyItem(nullptr)
{
    sigCheckToggledConnection.reset(
        item->sigCheckToggled(Item::LogicalSumOfAllChecks).connect(
            [&](bool on){ enableVisualization(on); }));
}


void SensorVisualizerItemBase::setBodyItem(BodyItem* bodyItem)
{
    this->bodyItem = bodyItem;
    enableVisualization(item->isChecked(Item::LogicalSumOfAllChecks));
}


void SensorVisualizerItemBase::updateVisualization()
{
    if(item->isChecked(Item::LogicalSumOfAllChecks)){
        doUpdateVisualization();
    }
}
    

Vector3SensorVisualizerItem::Vector3SensorVisualizerItem()
    : SensorVisualizerItemBase(this)
{

}


SgNode* Vector3SensorVisualizerItem::getScene()
{
    if(!scene){
        scene = new SgGroup;
    }
    return scene;
}


void Vector3SensorVisualizerItem::enableVisualization(bool on)
{
    getScene();
    
    connections.disconnect();
    scene->clearChildren();
    sensors.clear();
    markers.clear();

    if(bodyItem && on){
        if(!material){
            material = new SgMaterial;
            material->setDiffuseColor(Vector3f::Zero());
            material->setEmissiveColor(color);
            material->setAmbientIntensity(0.0f);
            material->setTransparency(0.5f);
        }

        updateSensors();

        for(size_t i=0; i < sensors.size(); ++i){
            auto marker = new ArrowMarker(material);
            markers.push_back(marker);
            scene->addChild(marker);
            connections.add(
                sensors[i]->sigStateChanged().connect(
                    [this, i](){ updateSensorMarkerVector(i); }));
        }
        if(!sensors.empty()){
            connections.add(
                bodyItem->sigKinematicStateChanged().connect(
                    [&](){ updateSensorMarkerPositions(true); }));
        }
        doUpdateVisualization();
    }
}


void Vector3SensorVisualizerItem::doUpdateVisualization()
{
    updateSensorMarkerPositions(false);
    for(size_t i=0; i < sensors.size(); ++i){
        updateSensorMarkerVector(i);
    }
}

    
void Vector3SensorVisualizerItem::updateSensorMarkerPositions(bool doNotify)
{
    for(size_t i=0; i < sensors.size(); ++i){
        auto sensor = sensors[i];
        Vector3 p = sensor->link()->T() * sensor->localTranslation();
        markers[i]->setTranslation(p);
        if(doNotify){
            markers[i]->notifyUpdate(update.withAction(SgUpdate::Modified));
        }
    }
}


void Vector3SensorVisualizerItem::updateSensorMarkerVector(int index)
{
    if(index < static_cast<int>(sensors.size())){
        auto sensor = sensors[index];
        Vector3 v_local = getSensorMarkerVector(sensor) + offset;
        Vector3 v_global = sensor->link()->R() * sensor->R_local() * v_local;
        markers[index]->setVector(visualRatio * v_global, threshold, update);
    }
}


void Vector3SensorVisualizerItem::doPutProperties(PutPropertyFunction& putProperty)
{
    putProperty.decimals(4)(
        _("Visual ratio"), visualRatio,
        [&](double ratio){
            if(ratio > 0.0){
                visualRatio = ratio;
                updateVisualization();
                return true;
            }
            return false;
        });
    
    putProperty.decimals(3)(_("Visual threshold"), threshold, changeProperty(threshold));
    
    putProperty.decimals(2)(
        _("Offset"), str(offset), [&](const string& v){ return toVector3(v, offset); });
}


bool Vector3SensorVisualizerItem::store(Archive& archive)
{
    archive.write("ratio", visualRatio);
    archive.write("threshold", threshold);
    write(archive, "offset", offset);
    return true;
}


bool Vector3SensorVisualizerItem::restore(const Archive& archive)
{
    archive.read({ "ratio", "visualRatio" }, visualRatio);
    archive.read("threshold", threshold);
    read(archive, "offset", offset);
    return true;
}


ForceSensorVisualizerItem::ForceSensorVisualizerItem()
{
    setName("ForceSensor");
    color << 1.0f, 0.8f, 0.0f;
    offset.setZero();    
    threshold = 0.1;
    visualRatio = 0.01;
}


void ForceSensorVisualizerItem::updateSensors()
{
    sensors = bodyItem->body()->devices<ForceSensor>();
}


Vector3 ForceSensorVisualizerItem::getSensorMarkerVector(Device* sensor)
{
    return static_cast<ForceSensor*>(sensor)->f();
}


AccelerationSensorVisualizerItem::AccelerationSensorVisualizerItem()
{
    setName("AccelerationSensor");
    color << 1.0f, 0.2f, 0.2f;
    offset << 0.0, 0.0, -9.8;
    threshold = 0.05;
    visualRatio = 0.1;
}


void AccelerationSensorVisualizerItem::updateSensors()
{
    sensors = bodyItem->body()->devices<AccelerationSensor>();
}


Vector3 AccelerationSensorVisualizerItem::getSensorMarkerVector(Device* sensor)
{
    return static_cast<AccelerationSensor*>(sensor)->dv();
}


RateGyroSensorVisualizerItem::RateGyroSensorVisualizerItem()
{
    setName("RateGyro");
    color << 0.2f, 0.2f, 1.0f;
    offset.setZero();    
    threshold = 0.01;
    visualRatio = 0.4;
}


void RateGyroSensorVisualizerItem::updateSensors()
{
    sensors = bodyItem->body()->devices<RateGyroSensor>();
}


Vector3 RateGyroSensorVisualizerItem::getSensorMarkerVector(Device* sensor)
{
    return static_cast<RateGyroSensor*>(sensor)->w();
}


CameraImageVisualizerItem::CameraImageVisualizerItem()
    : SensorVisualizerItemBase(this)
{

}


const Image* CameraImageVisualizerItem::getImage()
{
    return image.get();
}


SignalProxy<void()> CameraImageVisualizerItem::sigImageUpdated()
{
    return sigImageUpdated_;
}


void CameraImageVisualizerItem::setBodyItem(BodyItem* bodyItem, Camera* camera)
{
    if(name().empty()){
        string name = camera->name();
        if(dynamic_cast<RangeCamera*>(camera)){
            name += "-Image";
        }
        setName(name);
    }

    this->camera = camera;

    SensorVisualizerItemBase::setBodyItem(bodyItem);
}


void CameraImageVisualizerItem::enableVisualization(bool on)
{
    connections.disconnect();

    if(camera && on){
        connections.add(
            camera->sigStateChanged().connect(
                [&](){ doUpdateVisualization(); }));

        doUpdateVisualization();
    }
}


void CameraImageVisualizerItem::doUpdateVisualization()
{
    if(camera){
        image = camera->sharedImage();
    } else {
        image.reset();
    }
    sigImageUpdated_();
}


PointCloudVisualizerItem::PointCloudVisualizerItem()
    : SensorVisualizerItemBase(this)
{

}


void PointCloudVisualizerItem::setBodyItem(BodyItem* bodyItem, RangeCamera* rangeCamera)
{
    if(name().empty()){
        setName(rangeCamera->name());
    }
    this->rangeCamera = rangeCamera;
    
    SensorVisualizerItemBase::setBodyItem(bodyItem);
}


void PointCloudVisualizerItem::enableVisualization(bool on)
{
    connections.disconnect();

    if(bodyItem && rangeCamera && on){
        connections.add(
            bodyItem->sigKinematicStateChanged().connect(
                [&](){ updateSensorPosition(true); }));
        connections.add(
            rangeCamera->sigStateChanged().connect(
                [&](){ updateRangeCameraState(); }));

        doUpdateVisualization();
    }
}


void PointCloudVisualizerItem::doUpdateVisualization()
{
    if(rangeCamera){
        updateSensorPosition(false);
        updateRangeCameraState();
    }
}


void PointCloudVisualizerItem::updateSensorPosition(bool doNotify)
{
    const Isometry3 T =  (rangeCamera->link()->T() * rangeCamera->T_local());
    setOffsetPosition(T);
    notifyOffsetPositionChange(doNotify);
}


void PointCloudVisualizerItem::updateRangeCameraState()
{
    auto pointSet_ = pointSet();
    
    const vector<Vector3f>& src = rangeCamera->constPoints();
    SgVertexArray& points = *pointSet_->getOrCreateVertices();
    const int numPoints = src.size();
    points.resize(numPoints);
    for(int i=0; i < numPoints; ++i){
        points[i] = src[i];
    }

    SgColorArray& colors = *pointSet_->getOrCreateColors();
    const Image& image = rangeCamera->constImage();
    if(image.empty() || image.numComponents() != 3){
        colors.clear();
    } else {
        const unsigned char* pixels = image.pixels();
        const int numPixels = image.width() * image.height();
        const int n = std::min(numPixels, numPoints);
        colors.resize(n);
        for(int i=0; i < n; ++i){
            Vector3f& c = colors[i];
            c[0] = *pixels++ / 255.0f;
            c[1] = *pixels++ / 255.0f;
            c[2] = *pixels++ / 255.0f;
        }
    }
    pointSet_->notifyUpdate(update.withAction(SgUpdate::Modified));
}


RangeSensorVisualizerItem::RangeSensorVisualizerItem()
    : SensorVisualizerItemBase(this)
{

}


void RangeSensorVisualizerItem::setBodyItem(BodyItem* bodyItem, RangeSensor* rangeSensor)
{
    if(name().empty()){
        setName(rangeSensor->name());
    }
    this->rangeSensor = rangeSensor;

    SensorVisualizerItemBase::setBodyItem(bodyItem);
}


void RangeSensorVisualizerItem::enableVisualization(bool on)
{
    connections.disconnect();

    if(bodyItem && rangeSensor && on){
        connections.add(
            bodyItem->sigKinematicStateChanged().connect(
                [&](){ updateSensorPosition(true); }));
        connections.add(
            rangeSensor->sigStateChanged().connect(
                [&](){ updateRangeSensorState(); }));

        doUpdateVisualization();
    }
}


void RangeSensorVisualizerItem::doUpdateVisualization()
{
    if(rangeSensor){
        updateSensorPosition(false);
        updateRangeSensorState();
    }
}


void RangeSensorVisualizerItem::updateSensorPosition(bool doNotify)
{
    const Isometry3 T = (rangeSensor->link()->T() * rangeSensor->T_local());
    setOffsetPosition(T);
    notifyOffsetPositionChange(doNotify);
}


void RangeSensorVisualizerItem::updateRangeSensorState()
{
    auto pointSet_ = pointSet();

    const RangeSensor::RangeData& src = rangeSensor->constRangeData();
    const int numPoints = src.size();
    SgVertexArray& points = *pointSet_->getOrCreateVertices();
    points.clear();

    if(!src.empty()){
        points.reserve(numPoints);
        const int numPitchSamples = rangeSensor->numPitchSamples();
        const double pitchStep = rangeSensor->pitchStep();
        const int numYawSamples = rangeSensor->numYawSamples();
        const double yawStep = rangeSensor->yawStep();
        
        for(int pitch=0; pitch < numPitchSamples; ++pitch){
            const double pitchAngle = pitch * pitchStep - rangeSensor->pitchRange() / 2.0;
            const double cosPitchAngle = cos(pitchAngle);
            const int srctop = pitch * numYawSamples;
            
            for(int yaw=0; yaw < numYawSamples; ++yaw){
                const double distance = src[srctop + yaw];
                if(distance <= rangeSensor->maxDistance()){
                    double yawAngle = yaw * yawStep - rangeSensor->yawRange() / 2.0;
                    float x = distance *  cosPitchAngle * sin(-yawAngle);
                    float y  = distance * sin(pitchAngle);
                    float z  = -distance * cosPitchAngle * cos(yawAngle);
                    points.push_back(Vector3f(x, y, z));
                }
            }
        }
    }
    pointSet_->notifyUpdate(update.withAction(SgUpdate::Modified));
}
