/*!
  @file
  @author Shin'ichiro Nakaoka
*/

#include "SensorVisualizerItem.h"
#include "BodyItem.h"
#include <cnoid/ItemTreeView>
#include <cnoid/BasicSensors>
#include <cnoid/ItemManager>
#include <cnoid/Archive>
#include <cnoid/SceneDrawables>
#include <cnoid/MeshGenerator>
#include <cnoid/ConnectionSet>
#include "gettext.h"

using namespace std;
using namespace cnoid;

namespace {

class Arrow : public SgPosTransform
{
public:
    SgUpdate update;
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

        setVector(Vector3(0.0, 0.0, 0.0));
    }

    void setVector(const Vector3& v) {
        double len = v.norm();
        cylinderScale->setScale(Vector3(1.0, len, 1.0));
        cylinderPosition->setTranslation(Vector3(0.0, len / 2.0, 0.0));
        conePosition->setTranslation(Vector3(0.0, len, 0.0));

        if(len > 0.0){
            Vector3 axis = (Vector3::UnitY().cross(v)).normalized();
            double angle = acos(Vector3::UnitY().dot(v) / len);
            setRotation(AngleAxis(angle, axis));
        }
            
        notifyUpdate(update);
    }
};

typedef ref_ptr<Arrow> ArrowPtr;

}

namespace cnoid {

class SensorVisualizerItemImpl
{
public:
    SensorVisualizerItem* self;
    BodyItem* bodyItem;
    vector<Item*> subItems;
    vector<ItemPtr> restoredSubItems;
    double forceSensor_visualRatio;

    SensorVisualizerItemImpl(SensorVisualizerItem* self);
    void onPositionChanged();
};

class ForceSensorVisualizerItemImpl
{
public:
    ForceSensorVisualizerItem* self;
    SgGroupPtr scene;
    SgShapePtr cylinder;
    SgShapePtr cone;
    DeviceList<ForceSensor> forceSensors;
    vector<ArrowPtr> forceSensorArrows;
    double visualRatio;
    ScopedConnectionSet connections;

    ForceSensorVisualizerItemImpl(ForceSensorVisualizerItem* self);
    void setBodyItem(BodyItem* bodyItem);
    void onSensorPositionsChanged();
    void updateSensorState();
    void updateForceSensorState(int index);
    void onForceSensorStateChanged(int index);
};

class PointCloudVisualizerItemImpl
{
public:
    PointCloudVisualizerItem* self;
    RangeCamera* rangeCamera;
    ScopedConnectionSet connections;
    SgPointSet* pointSet;

    PointCloudVisualizerItemImpl(PointCloudVisualizerItem* self);
    void setBodyItem(BodyItem* bodyItem, RangeCamera* rangeCamera);
    void onSensorPositionsChanged();
    void updateRangeCameraState();
};

class RangeSensorVisualizerItemImpl
{
public:
    RangeSensorVisualizerItem* self;
    RangeSensor* rangeSensor;
    ScopedConnectionSet connections;
    SgPointSet* pointSet;

    RangeSensorVisualizerItemImpl(RangeSensorVisualizerItem* self);
    void setBodyItem(BodyItem* bodyItem, RangeSensor* rangeSensor);
    void onSensorPositionsChanged();
    void updateRangeSensorState();
};

class CameraImageVisualizerItemImpl
{
public:
    CameraImageVisualizerItem* self;
    Camera* camera;
    ScopedConnectionSet connections;
    std::shared_ptr<const Image> image;

    CameraImageVisualizerItemImpl(CameraImageVisualizerItem* self);
    void setBodyItem(BodyItem* bodyItem, Camera* camera);
    void updateCameraImage();
};

}


void SensorVisualizerItem::initializeClass(ExtensionManager* ext)
{
    ItemManager& im = ext->itemManager();
    im.registerClass<SensorVisualizerItem>(N_("SensorVisualizer"));
    im.addCreationPanel<SensorVisualizerItem>();

    ForceSensorVisualizerItem::initializeClass(ext);
    PointCloudVisualizerItem::initializeClass(ext);
    RangeSensorVisualizerItem::initializeClass(ext);
    CameraImageVisualizerItem::initializeClass(ext);
}


SensorVisualizerItem::SensorVisualizerItem()
{
    impl = new SensorVisualizerItemImpl(this);
}

SensorVisualizerItemImpl::SensorVisualizerItemImpl(SensorVisualizerItem* self)
    : self(self)
{
    forceSensor_visualRatio = 0.002;
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


void SensorVisualizerItem::onPositionChanged()
{
    if(parentItem())
        impl->onPositionChanged();
}


void SensorVisualizerItemImpl::onPositionChanged()
{
    BodyItem* newBodyItem = self->findOwnerItem<BodyItem>();
    if(newBodyItem != bodyItem){
        bodyItem = newBodyItem;
        for(size_t i=0; i < subItems.size(); i++){
            subItems[i]->detachFromParentItem();
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
                }else{
                    forceSensorVisualizerItem = new ForceSensorVisualizerItem();
                    forceSensorVisualizerItem->setVisualRatio(forceSensor_visualRatio);
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
        impl->subItems[i]->detachFromParentItem();
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
        item->store(*subArchive);

        subItems->append(subArchive);
    }

    archive.insert("subItems", subItems);

    return true;
}


bool SensorVisualizerItem::restore(const Archive& archive)
{
    impl->restoredSubItems.clear();

    ListingPtr subItems = archive.findListing("subItems");
    if(subItems->isValid()){
        for(int i=0; i < subItems->size(); i++){
            Archive* subArchive = dynamic_cast<Archive*>(subItems->at(i)->toMapping());
            string className, itemName;
            subArchive->read("class", className);
            subArchive->read("name", itemName);

            ItemPtr item;
            item = ItemManager::create("Body", className);
            item->setName(itemName);

            item->restore(*subArchive);
            impl->restoredSubItems.push_back(item);
        }
    }
    return true;
}


void ForceSensorVisualizerItem::initializeClass(ExtensionManager* ext)
{
    ItemManager& im = ext->itemManager();
    im.registerClass<ForceSensorVisualizerItem>(N_("ForceSensorVisualizer"));
}


ForceSensorVisualizerItem::ForceSensorVisualizerItem()
{
    impl = new ForceSensorVisualizerItemImpl(this);
}


ForceSensorVisualizerItemImpl::ForceSensorVisualizerItemImpl(ForceSensorVisualizerItem* self)
    : self(self)
{
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


ForceSensorVisualizerItem::~ForceSensorVisualizerItem()
{
    delete impl;
}


void ForceSensorVisualizerItem::setBodyItem(BodyItem* bodyItem)
{
    if(name().empty())
        setName("ForceSensor");

    impl->setBodyItem(bodyItem);
}


void ForceSensorVisualizerItemImpl::setBodyItem(BodyItem* bodyItem)
{
    connections.disconnect();
    connections.add(
        bodyItem->sigKinematicStateChanged().connect(
            [&](){ onSensorPositionsChanged(); }));

    Body* body = bodyItem->body();
    forceSensors = body->devices<ForceSensor>();
    scene->clearChildren();
    forceSensorArrows.clear();
    for(size_t i=0; i < forceSensors.size(); ++i){
        ArrowPtr arrow = new Arrow(cylinder, cone);
        forceSensorArrows.push_back(arrow);
        scene->addChild(arrow);
        connections.add(
            forceSensors[i]->sigStateChanged().connect(
                [this, i](){ updateForceSensorState(i); }));
    }

    onSensorPositionsChanged();
    updateSensorState();
}


SgNode* ForceSensorVisualizerItem::getScene()
{
    return impl->scene;
}


void ForceSensorVisualizerItemImpl::onSensorPositionsChanged()
{
    if(!ItemTreeView::instance()->isItemChecked(self, ItemTreeView::ID_ANY)){
        return;
    }
    for(size_t i=0; i < forceSensors.size(); ++i){
        ForceSensor* sensor = forceSensors[i];
        Vector3 p = sensor->link()->T() * sensor->localTranslation();
        forceSensorArrows[i]->setTranslation(p);
    }
}


void ForceSensorVisualizerItemImpl::updateSensorState()
{
    if(!ItemTreeView::instance()->isItemChecked(self, ItemTreeView::ID_ANY)){
        return;
    }
    for(size_t i=0; i < forceSensors.size(); ++i){
        updateForceSensorState(i);
    }
}

    
void ForceSensorVisualizerItemImpl::updateForceSensorState(int index)
{
    if(index < static_cast<int>(forceSensors.size())){
        ForceSensor* sensor = forceSensors[index];
        Vector3 v = sensor->link()->T() * sensor->T_local() * sensor->f();
        forceSensorArrows[index]->setVector(v * visualRatio);
    }
}


void ForceSensorVisualizerItem::doPutProperties(PutPropertyFunction& putProperty)
{
    putProperty.decimals(4)(
        _("Visual ratio"), impl->visualRatio,
        [&](double ratio){
            if(ratio > 0.0){
                impl->visualRatio = ratio;
                impl->updateSensorState();
                return true;
            }
            return false;
        });
}


bool ForceSensorVisualizerItem::store(Archive& archive)
{
    archive.write("visualRatio", impl->visualRatio);
    return true;
}


bool ForceSensorVisualizerItem::restore(const Archive& archive)
{
    archive.read("visualRatio", impl->visualRatio);
    return true;
}


void ForceSensorVisualizerItem::setVisualRatio(double visualRatio_)
{
    impl->visualRatio = visualRatio_;
}


void PointCloudVisualizerItem::initializeClass(ExtensionManager* ext)
{
    ItemManager& im = ext->itemManager();
    im.registerClass<PointCloudVisualizerItem>(N_("PointCloudVisualizer"));
}


PointCloudVisualizerItem::PointCloudVisualizerItem()
{
    impl = new PointCloudVisualizerItemImpl(this);
}


PointCloudVisualizerItemImpl::PointCloudVisualizerItemImpl(PointCloudVisualizerItem* self)
    : self(self)
{
    pointSet = self->pointSet();
}


PointCloudVisualizerItem::~PointCloudVisualizerItem()
{
    delete impl;
}


void PointCloudVisualizerItem::setBodyItem(BodyItem* bodyItem, RangeCamera* rangeCamera)
{
    if(name().empty())
        setName(rangeCamera->name());

    impl->setBodyItem(bodyItem, rangeCamera);
}


void PointCloudVisualizerItemImpl::setBodyItem(BodyItem* bodyItem, RangeCamera* rangeCamera_)
{
    connections.disconnect();
    connections.add(
        bodyItem->sigKinematicStateChanged().connect(
            [&](){ onSensorPositionsChanged(); }));

    rangeCamera = rangeCamera_;

    connections.add(
        rangeCamera->sigStateChanged().connect(
            [&](){ updateRangeCameraState(); }));

    onSensorPositionsChanged();
    updateRangeCameraState();
}


void PointCloudVisualizerItemImpl::onSensorPositionsChanged()
{
    if(!ItemTreeView::instance()->isItemChecked(self, ItemTreeView::ID_ANY))
        return;

    const Affine3 T =  (rangeCamera->link()->T() * rangeCamera->T_local());
    self->setOffsetTransform(T);
}


void PointCloudVisualizerItemImpl::updateRangeCameraState()
{
    if(!ItemTreeView::instance()->isItemChecked(self, ItemTreeView::ID_ANY))
        return;

    const vector<Vector3f>& src = rangeCamera->constPoints();
    SgVertexArray& points = *pointSet->getOrCreateVertices();
    const int numPoints = src.size();
    points.resize(numPoints);
    for(int i=0; i < numPoints; ++i){
        points[i] = src[i];
    }

    SgColorArray& colors = *pointSet->getOrCreateColors();
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
            c[0] = *pixels++ / 255.0;
            c[1] = *pixels++ / 255.0;;
            c[2] = *pixels++ / 255.0;;
        }
    }

    pointSet->notifyUpdate();
}


void RangeSensorVisualizerItem::initializeClass(ExtensionManager* ext)
{
    ItemManager& im = ext->itemManager();
    im.registerClass<RangeSensorVisualizerItem>(N_("RangeSensorVisualizer"));
}


RangeSensorVisualizerItem::RangeSensorVisualizerItem()
{
    impl = new RangeSensorVisualizerItemImpl(this);
}


RangeSensorVisualizerItemImpl::RangeSensorVisualizerItemImpl(RangeSensorVisualizerItem* self)
    : self(self)
{
    pointSet = self->pointSet();
}


RangeSensorVisualizerItem::~RangeSensorVisualizerItem()
{
    delete impl;
}


void RangeSensorVisualizerItem::setBodyItem(BodyItem* bodyItem, RangeSensor* rangeSensor)
{
    if(name().empty())
        setName(rangeSensor->name());

    impl->setBodyItem(bodyItem, rangeSensor);
}


void RangeSensorVisualizerItemImpl::setBodyItem(BodyItem* bodyItem, RangeSensor* rangeSensor_)
{
    connections.disconnect();
    connections.add(
        bodyItem->sigKinematicStateChanged().connect(
            [&](){ onSensorPositionsChanged(); }));

    rangeSensor = rangeSensor_;

    connections.add(
        rangeSensor->sigStateChanged().connect(
            [&](){ updateRangeSensorState(); }));

    onSensorPositionsChanged();
    updateRangeSensorState();
}


void RangeSensorVisualizerItemImpl::onSensorPositionsChanged()
{
    if(!ItemTreeView::instance()->isItemChecked(self, ItemTreeView::ID_ANY))
        return;

    const Affine3 T = (rangeSensor->link()->T() * rangeSensor->T_local());
    self->setOffsetTransform(T);
}


void RangeSensorVisualizerItemImpl::updateRangeSensorState()
{
    if(!ItemTreeView::instance()->isItemChecked(self, ItemTreeView::ID_ANY)){
        return;
    }

    const RangeSensor::RangeData& src = rangeSensor->constRangeData();
    SgVertexArray& points = *pointSet->getOrCreateVertices();
    const int numPoints = src.size();
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

    pointSet->notifyUpdate();
}

void CameraImageVisualizerItem::initializeClass(ExtensionManager* ext)
{
    ItemManager& im = ext->itemManager();
    im.registerClass<CameraImageVisualizerItem>(N_("CameraImageVisualizer"));
}


CameraImageVisualizerItem::CameraImageVisualizerItem()
{
    impl = new CameraImageVisualizerItemImpl(this);
}


CameraImageVisualizerItemImpl::CameraImageVisualizerItemImpl(CameraImageVisualizerItem* self)
    : self(self)
{

}


CameraImageVisualizerItem::~CameraImageVisualizerItem()
{
    delete impl;
}


void CameraImageVisualizerItem::setBodyItem(BodyItem* bodyItem, Camera* camera)
{
    if(name().empty()){
        string name = camera->name();
        if(dynamic_cast<RangeCamera*>(camera))
            name += "_Image";
        setName(name);
    }

    impl->setBodyItem(bodyItem, camera);
}


void CameraImageVisualizerItemImpl::setBodyItem(BodyItem* bodyItem, Camera* camera_)
{
    connections.disconnect();
    camera = camera_;

    connections.add(
        camera->sigStateChanged().connect(
            [&](){ updateCameraImage(); }));

    updateCameraImage();
}


void CameraImageVisualizerItemImpl::updateCameraImage()
{
    image = camera->sharedImage();
    self->notifyImageUpdate();
}


const Image* CameraImageVisualizerItem::getImage()
{
    return impl->image.get();
}
