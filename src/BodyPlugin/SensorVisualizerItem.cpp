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
#include <iostream>
#include "gettext.h"

using namespace std;
using namespace std::placeholders;
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
    SgGroupPtr scene;
    SgShapePtr cylinder;
    SgShapePtr cone;
    DeviceList<ForceSensor> forceSensors;
    vector<ArrowPtr> forceSensorArrows;
    double visualRatio;
    ScopedConnectionSet connections;

    SensorVisualizerItemImpl(SensorVisualizerItem* self);
    void onPositionChanged();
    void onSensorPositionsChanged();
    void updateSensorState();
    void updateForceSensorState(int index);
    void onForceSensorStateChanged(int index);
    bool onLengthRatioPropertyChanged(double ratio);
};

}


void SensorVisualizerItem::initializeClass(ExtensionManager* ext)
{
    ItemManager& im = ext->itemManager();
    im.registerClass<SensorVisualizerItem>(N_("SensorVisualizer"));
    im.addCreationPanel<SensorVisualizerItem>();
}


SensorVisualizerItem::SensorVisualizerItem()
{
    impl = new SensorVisualizerItemImpl(this);
}


SensorVisualizerItemImpl::SensorVisualizerItemImpl(SensorVisualizerItem* self)
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


SgNode* SensorVisualizerItem::getScene()
{
    return impl->scene;
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
        connections.disconnect();
        forceSensors.clear();
        if(bodyItem){
            Body* body = bodyItem->body();

            connections.add(
                bodyItem->sigKinematicStateChanged().connect(
                    std::bind(&SensorVisualizerItemImpl::onSensorPositionsChanged, this)));

            scene->clearChildren();
            forceSensorArrows.clear();
            forceSensors << body->devices();
            for(size_t i=0; i < forceSensors.size(); ++i){
                ArrowPtr arrow = new Arrow(cylinder, cone);
                forceSensorArrows.push_back(arrow);
                scene->addChild(arrow);
                connections.add(
                    forceSensors[i]->sigStateChanged().connect(
                        std::bind(&SensorVisualizerItemImpl::updateForceSensorState, this, i)));
                updateForceSensorState(i);
            }
        }
    }
}


void SensorVisualizerItemImpl::onSensorPositionsChanged()
{
    for(size_t i=0; i < forceSensors.size(); ++i){
        ForceSensor* sensor = forceSensors[i];
        Vector3 p = sensor->link()->T() * sensor->localTranslation();
        forceSensorArrows[i]->setTranslation(p);
    }
}


void SensorVisualizerItemImpl::updateSensorState()
{
    for(size_t i=0; i < forceSensors.size(); ++i){
        updateForceSensorState(i);
    }
}

    
void SensorVisualizerItemImpl::updateForceSensorState(int index)
{
    if(index < forceSensors.size()){
        ForceSensor* sensor = forceSensors[index];
	Vector3 v = sensor->link()->T() * sensor->T_local() * sensor->f();
        forceSensorArrows[index]->setVector(v * visualRatio);
    }
}


void SensorVisualizerItem::doPutProperties(PutPropertyFunction& putProperty)
{
    putProperty.decimals(4);
    putProperty(_("Visual ratio"), impl->visualRatio,
                std::bind(&SensorVisualizerItemImpl::onLengthRatioPropertyChanged, impl, _1));
}


bool SensorVisualizerItemImpl::onLengthRatioPropertyChanged(double ratio)
{
    if(ratio > 0.0){
        visualRatio = ratio;
        updateSensorState();
        return true;
    }
    return false;
}


bool SensorVisualizerItem::store(Archive& archive)
{
    archive.write("visualRatio", impl->visualRatio);
    return true;
}


bool SensorVisualizerItem::restore(const Archive& archive)
{
    archive.read("visualRatio", impl->visualRatio);
    return true;
}
