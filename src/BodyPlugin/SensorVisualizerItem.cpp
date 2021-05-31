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
#include "gettext.h"

using namespace std;
using namespace cnoid;

namespace {

class Arrow : public SgPosTransform
{
public:
    SgPosTransformPtr cylinderPosition;
    SgScaleTransformPtr cylinderScale;
    SgShapePtr cylinder;
    SgPosTransformPtr conePosition;
    SgShapePtr cone;
    
    Arrow(SgShape* cylinder, SgShape* cone)
        : cylinder(cylinder),
          cone(cone)
    {
        cylinderScale = new SgScaleTransform();
        cylinderScale->addChild(cylinder);
        cylinderPosition = new SgPosTransform();
        cylinderPosition->addChild(cylinderScale);
        addChild(cylinderPosition);

        conePosition = new SgPosTransform();
        conePosition->addChild(cone);
        addChild(conePosition);

        setVector(Vector3(0.0, 0.0, 0.0), nullptr);
    }

    void setVector(const Vector3& v, SgUpdateRef update) {
        double len = v.norm();
        cylinderScale->setScale(Vector3(1.0, len, 1.0));
        cylinderPosition->setTranslation(Vector3(0.0, len / 2.0, 0.0));
        conePosition->setTranslation(Vector3(0.0, len, 0.0));

        if(len > 0.0){
            Vector3 axis = (Vector3::UnitY().cross(v)).normalized();
            double angle = acos(Vector3::UnitY().dot(v) / len);
            setRotation(AngleAxis(angle, axis));
        }

        if(update){
            notifyUpdate(*update);
        }
    }
};

typedef ref_ptr<Arrow> ArrowPtr;

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
    

class ForceSensorVisualizerItem : public Item, public SensorVisualizerItemBase, public RenderableItem
{
public:
    ForceSensorVisualizerItem();
    virtual SgNode* getScene() override;
    virtual void enableVisualization(bool on) override;
    virtual void doUpdateVisualization() override;
    void updateSensorPositions(bool doNotify);
    void updateForceSensorState(int index);
    virtual void doPutProperties(PutPropertyFunction& putProperty) override;
    virtual bool store(Archive& archive) override;
    virtual bool restore(const Archive& archive) override;

    SgGroupPtr scene;
    SgShapePtr cylinder;
    SgShapePtr cone;
    DeviceList<ForceSensor> forceSensors;
    vector<ArrowPtr> forceSensorArrows;
    double visualRatio;
    ScopedConnectionSet connections;
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

class SensorVisualizerItemImpl
{
public:
    SensorVisualizerItem* self;
    BodyItem* bodyItem;
    vector<Item*> subItems;
    vector<ItemPtr> restoredSubItems;
    SgUpdate update;

    SensorVisualizerItemImpl(SensorVisualizerItem* self);
    void onTreePathChanged();
};

}


void SensorVisualizerItem::initializeClass(ExtensionManager* ext)
{
    ItemManager& im = ext->itemManager();

    im.registerClass<SensorVisualizerItem>(N_("SensorVisualizerItem"));
    im.registerClass<ForceSensorVisualizerItem>(N_("ForceSensorVisualizerItem"));
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
    impl = new SensorVisualizerItemImpl(this);
}


SensorVisualizerItemImpl::SensorVisualizerItemImpl(SensorVisualizerItem* self)
    : self(self)
{

}


SensorVisualizerItem::SensorVisualizerItem(const SensorVisualizerItem& org)
    : Item(org)
{
    impl = new SensorVisualizerItemImpl(this);
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


void SensorVisualizerItemImpl::onTreePathChanged()
{
    BodyItem* newBodyItem = self->findOwnerItem<BodyItem>();
    if(newBodyItem != bodyItem){
        bodyItem = newBodyItem;
        for(size_t i=0; i < subItems.size(); i++){
            subItems[i]->removeFromParentItem();
        }
        subItems.clear();

        int n = restoredSubItems.size();
        int j = 0;

        if(bodyItem){
            Body* body = bodyItem->body();

            DeviceList<ForceSensor> forceSensors = body->devices<ForceSensor>();
            if(!forceSensors.empty()){
                ForceSensorVisualizerItem* forceSensorVisualizerItem= 0;
                if(j<n){
                    forceSensorVisualizerItem = dynamic_cast<ForceSensorVisualizerItem*>(restoredSubItems[j++].get());
                } else {
                    forceSensorVisualizerItem = new ForceSensorVisualizerItem();
                }
                if(forceSensorVisualizerItem){
                    forceSensorVisualizerItem->setBodyItem(bodyItem);
                    self->addSubItem(forceSensorVisualizerItem);
                    subItems.push_back(forceSensorVisualizerItem);
                }
            }

            DeviceList<RangeCamera> rangeCameras = body->devices<RangeCamera>();
            for(size_t i=0; i < rangeCameras.size(); ++i){
                PointCloudVisualizerItem* pointCloudVisualizerItem =
                        j<n ? dynamic_cast<PointCloudVisualizerItem*>(restoredSubItems[j++].get()) : new PointCloudVisualizerItem();
                if(pointCloudVisualizerItem){
                    pointCloudVisualizerItem->setBodyItem(bodyItem, rangeCameras[i]);
                    self->addSubItem(pointCloudVisualizerItem);
                    subItems.push_back(pointCloudVisualizerItem);
                }
            }

            DeviceList<RangeSensor> rangeSensors = body->devices<RangeSensor>();
            for(size_t i=0; i < rangeSensors.size(); ++i){
                RangeSensorVisualizerItem* rangeSensorVisualizerItem =
                        j<n ? dynamic_cast<RangeSensorVisualizerItem*>(restoredSubItems[j++].get()) : new RangeSensorVisualizerItem();
                if(rangeSensorVisualizerItem){
                    rangeSensorVisualizerItem->setBodyItem(bodyItem, rangeSensors[i]);
                    self->addSubItem(rangeSensorVisualizerItem);
                    subItems.push_back(rangeSensorVisualizerItem);
                }
            }

            DeviceList<Camera> cameras = body->devices<Camera>();
            for(size_t i=0; i < cameras.size(); ++i){
                if(cameras[i]->imageType()!=Camera::NO_IMAGE){
                    CameraImageVisualizerItem* cameraImageVisualizerItem =
                            j<n ? dynamic_cast<CameraImageVisualizerItem*>(restoredSubItems[j++].get()) : new CameraImageVisualizerItem();
                    if(cameraImageVisualizerItem){
                        cameraImageVisualizerItem->setBodyItem(bodyItem, cameras[i]);
                        self->addSubItem(cameraImageVisualizerItem);
                        subItems.push_back(cameraImageVisualizerItem);
                    }
                }
            }
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
    ListingPtr subItems = new Listing();

    for(size_t i=0; i < impl->subItems.size(); i++){
        Item* item = impl->subItems[i];
        string pluginName, className;
        ItemManager::getClassIdentifier(item, pluginName, className);

        ArchivePtr subArchive = new Archive();
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

    ListingPtr subItems = archive.findListing("sub_items");
    if(!subItems->isValid()){
        subItems = archive.findListing("subItems"); // Old
    }
    if(subItems->isValid()){
        for(int i=0; i < subItems->size(); i++){
            Archive* subArchive = dynamic_cast<Archive*>(subItems->at(i)->toMapping());
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


namespace {

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
    

ForceSensorVisualizerItem::ForceSensorVisualizerItem()
    : SensorVisualizerItemBase(this)
{
    setName("ForceSensor");
    
    scene = new SgGroup;

    SgMaterial* material = new SgMaterial;
    Vector3f color(1.0f, 0.2f, 0.2f);
    material->setDiffuseColor(Vector3f::Zero());
    material->setEmissiveColor(color);
    material->setAmbientIntensity(0.0f);
    material->setTransparency(0.6f);

    MeshGenerator meshGenerator;
    cone = new SgShape;
    cone->setMesh(meshGenerator.generateCone(0.03,  0.04));
    cone->setMaterial(material);

    cylinder = new SgShape;
    cylinder->setMesh(meshGenerator.generateCylinder(0.01, 1.0));
    cylinder->setMaterial(material);

    visualRatio = 0.002;
}


SgNode* ForceSensorVisualizerItem::getScene()
{
    return scene;
}


void ForceSensorVisualizerItem::enableVisualization(bool on)
{
    connections.disconnect();
    scene->clearChildren();
    forceSensors.clear();

    if(bodyItem && on){
        connections.add(
            bodyItem->sigKinematicStateChanged().connect(
                [&](){ updateSensorPositions(true); }));

        Body* body = bodyItem->body();
        forceSensors = body->devices<ForceSensor>();
        forceSensorArrows.clear();
        for(size_t i=0; i < forceSensors.size(); ++i){
            ArrowPtr arrow = new Arrow(cylinder, cone);
            forceSensorArrows.push_back(arrow);
            scene->addChild(arrow);
            connections.add(
                forceSensors[i]->sigStateChanged().connect(
                    [this, i](){ updateForceSensorState(i); }));
        }

        doUpdateVisualization();
    }
}


void ForceSensorVisualizerItem::doUpdateVisualization()
{
    updateSensorPositions(false);
    for(size_t i=0; i < forceSensors.size(); ++i){
        updateForceSensorState(i);
    }
}

    
void ForceSensorVisualizerItem::updateSensorPositions(bool doNotify)
{
    for(size_t i=0; i < forceSensors.size(); ++i){
        ForceSensor* sensor = forceSensors[i];
        Vector3 p = sensor->link()->T() * sensor->localTranslation();
        forceSensorArrows[i]->setTranslation(p);
        if(doNotify){
            forceSensorArrows[i]->notifyUpdate(update.withAction(SgUpdate::Modified));
        }
    }
}


void ForceSensorVisualizerItem::updateForceSensorState(int index)
{
    if(index < static_cast<int>(forceSensors.size())){
        ForceSensor* sensor = forceSensors[index];
        Vector3 v = sensor->link()->T() * sensor->T_local() * sensor->f();
        forceSensorArrows[index]->setVector(v * visualRatio, update);
    }
}


void ForceSensorVisualizerItem::doPutProperties(PutPropertyFunction& putProperty)
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
}


bool ForceSensorVisualizerItem::store(Archive& archive)
{
    archive.write("visualRatio", visualRatio);
    return true;
}


bool ForceSensorVisualizerItem::restore(const Archive& archive)
{
    archive.read("visualRatio", visualRatio);
    return true;
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
        if(dynamic_cast<RangeCamera*>(camera))
            name += "_Image";
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

}
