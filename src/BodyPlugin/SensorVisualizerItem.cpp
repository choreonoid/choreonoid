/*!
  @file
  @author Shin'ichiro Nakaoka
*/

#include "SensorVisualizerItem.h"
#include "BodyItem.h"
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
    }

    void setVector(const Vector3& v) {
        double len = v.norm();
        cylinderScale->setScale(Vector3(1.0, len, 1.0));
        cylinderPosition->setTranslation(Vector3(0.0, len / 2.0, 0.0));
        conePosition->setTranslation(Vector3(0.0, len, 0.0));

        Vector3 axis = (Vector3::UnitY().cross(v)).normalized();
        double angle = acos(Vector3::UnitY().dot(v) / len);
        setRotation(AngleAxis(angle, axis));

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

}


void SensorVisualizerItem::initializeClass(ExtensionManager* ext)
{
    ItemManager& im = ext->itemManager();
    im.registerClass<SensorVisualizerItem>(N_("SensorVisualizer"));
    im.addCreationPanel<SensorVisualizerItem>();

    ForceSensorVisualizerItem::initializeClass(ext);
    PointCloudVisualizerItem::initializeClass(ext);
    RangeSensorVisualizerItem::initializeClass(ext);
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
    impl->onPositionChanged();
}


void SensorVisualizerItemImpl::onPositionChanged()
{
    BodyItem* newBodyItem = self->findOwnerItem<BodyItem>();
    if(newBodyItem != bodyItem){
        bodyItem = newBodyItem;
        for(int i=0; i<subItems.size(); i++){
            subItems[i]->detachFromParentItem();
        }
        subItems.clear();

        if(bodyItem){
            Body* body = bodyItem->body();

            DeviceList<ForceSensor> forceSensors = body->devices<ForceSensor>();
            if(!forceSensors.empty()){
                ForceSensorVisualizerItemPtr forceSensorVisualizerItem = new ForceSensorVisualizerItem();
                forceSensorVisualizerItem->setVisualRatio(forceSensor_visualRatio);
                forceSensorVisualizerItem->setBodyItem(bodyItem);
                self->addSubItem(forceSensorVisualizerItem);
                subItems.push_back(forceSensorVisualizerItem);
            }

            DeviceList<RangeCamera> rangeCameras = body->devices<RangeCamera>();
            for(size_t i=0; i < rangeCameras.size(); ++i){
                PointCloudVisualizerItemPtr pointCloudVisualizerItem = new PointCloudVisualizerItem();
                pointCloudVisualizerItem->setBodyItem(bodyItem, rangeCameras[i]);
                self->addSubItem(pointCloudVisualizerItem);
                subItems.push_back(pointCloudVisualizerItem);
            }

            DeviceList<RangeSensor> rangeSensors = body->devices<RangeSensor>();
            for(size_t i=0; i < rangeSensors.size(); ++i){
                RangeSensorVisualizerItemPtr rangeSensorVisualizerItem = new RangeSensorVisualizerItem();
                rangeSensorVisualizerItem->setBodyItem(bodyItem, rangeSensors[i]);
                self->addSubItem(rangeSensorVisualizerItem);
                subItems.push_back(rangeSensorVisualizerItem);
            }
        }
    }
}


bool SensorVisualizerItem::store(Archive& archive)
{
    for(int i=0; i<impl->subItems.size(); i++){
        impl->subItems[i]->store(archive);
    }

    return true;
}


bool SensorVisualizerItem::restore(const Archive& archive)
{
    archive.read("forceSensor_visualRatio", impl->forceSensor_visualRatio);

    return true;
}


void ForceSensorVisualizerItem::initializeClass(ExtensionManager* ext)
{
    ItemManager& im = ext->itemManager();
    im.registerClass<ForceSensorVisualizerItem>(N_("ForceSensorVisualizer"));
}


ForceSensorVisualizerItem::ForceSensorVisualizerItem()
{
    setName("ForceSensor");
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
    for(size_t i=0; i < forceSensors.size(); ++i){
        ForceSensor* sensor = forceSensors[i];
        Vector3 p = sensor->link()->T() * sensor->localTranslation();
        forceSensorArrows[i]->setTranslation(p);
    }
}


void ForceSensorVisualizerItemImpl::updateSensorState()
{
    for(size_t i=0; i < forceSensors.size(); ++i){
        updateForceSensorState(i);
    }
}

    
void ForceSensorVisualizerItemImpl::updateForceSensorState(int index)
{
    if(index < forceSensors.size()){
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
    archive.write("forceSensor_visualRatio", impl->visualRatio);
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
    setName("PointCloud");
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
    const Affine3 T =  (rangeCamera->link()->T() * rangeCamera->T_local());
    self->setOffsetTransform(T);
}


void PointCloudVisualizerItemImpl::updateRangeCameraState()
{
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
    setName("RangeSensor");
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
    const Affine3 T = (rangeSensor->link()->T() * rangeSensor->T_local());
    self->setOffsetTransform(T);
}


void RangeSensorVisualizerItemImpl::updateRangeSensorState()
{
    const RangeSensor::RangeData& src = rangeSensor->constRangeData();
    SgVertexArray& points = *pointSet->getOrCreateVertices();
    const int numPoints = src.size();
    points.clear();
    if(!src.empty()){
        points.reserve(numPoints);
        const double pitchStep = rangeSensor->pitchStep();
        const double yawStep = rangeSensor->yawStep();
        for(int pitch=0; pitch < rangeSensor->pitchResolution(); ++pitch){
            const double pitchAngle = pitch * pitchStep - rangeSensor->pitchRange() / 2.0;
            const int srctop = pitch * rangeSensor->yawResolution();
            for(int yaw=0; yaw < rangeSensor->yawResolution(); ++yaw){
                const double distance = src[srctop + yaw];
                if(distance <= rangeSensor->maxDistance()){
                    double yawAngle = yaw * yawStep - rangeSensor->yawRange() / 2.0;
                    float x = distance * sin(-yawAngle);
                    float y  = distance * sin(pitchAngle);
                    float z  = -distance * cos(pitchAngle) * cos(yawAngle);
                    points.push_back(Vector3f(x, y, z));
                }
            }
        }
    }

    pointSet->notifyUpdate();
}
