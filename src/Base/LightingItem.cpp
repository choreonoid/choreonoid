#include "LightingItem.h"
#include "ItemManager.h"
#include "PutPropertyFunction.h"
#include "Archive.h"
#include <cnoid/SceneGraph>
#include <cnoid/SceneLights>
#include <cnoid/MeshGenerator>
#include <cnoid/EigenUtil>
#include <cnoid/EigenArchive>
#include "gettext.h"

using namespace std;
using namespace cnoid;

namespace {

SgMaterialPtr lightMaterial;
SgShapePtr lightSphere;
SgShapePtr lightCapsule;
SgGroupPtr directionalLightShape;
SgGroupPtr pointLightShape;
SgGroupPtr spotLightShape;

SgMaterial* getOrCreateLightMaterial();
SgShape* getOrCreateLightSphere();
SgShape* getOrCreateLightCapsule();
SgGroup* getOrCreateDirectionalLightShape();
SgGroup* getOrCreatePointLightShape();
SgGroup* getOrCreateSpotLightShape();

}

namespace cnoid {

class LightingItem::Impl
{
public:
    LightingItem* self;
    Selection lightType;
    
    SgPosTransformPtr lightPosTransform;
    SgLightPtr light;
    SgGroupPtr lightShape;
    SgUpdate sgUpdate;
    bool isMarkerEnabled;

    Vector3f color;
    float intensity;
    float ambientIntensity;
    // For the spot light and the directional light
    Vector3 direction;
    // For the point light
    float constantAttenuation;
    float linearAttenuation;
    float quadraticAttenuation;
    // For the spot light
    float beamWidth;
    float cutOffAngle;
    float cutOffExponent;
    bool on;

    Impl(LightingItem* self);
    Impl(LightingItem* self, const Impl& org);
    void setLightType(LightType type);
    void updateLight();
    void doPutProperties(PutPropertyFunction& putProperty);
    bool store(Archive& archive);
    bool restore(const Archive& archive);
};

}


void LightingItem::initializeClass(ExtensionManager* ext)
{
    static bool initialized = false;
    if(!initialized){
        ext->itemManager()
            .registerClass<LightingItem>(N_("LightingItem"))
            .addCreationPanel<LightingItem>();
        initialized = true;
    }
}


LightingItem::LightingItem()
{
    setName("Lighting");
    impl = new Impl(this);
}


LightingItem::LightingItem(const LightingItem& org)
    : Item(org)
{
    impl = new Impl(this, *org.impl);
}


LightingItem::~LightingItem()
{
    delete impl;
}


Item* LightingItem::doCloneItem(CloneMap* /* cloneMap */) const
{
    return new LightingItem(*this);
}


LightingItem::Impl::Impl(LightingItem* self)
    : self(self),
      lightType(NumLightTypes, CNOID_GETTEXT_DOMAIN_NAME)
{
    lightType.setSymbol(DirectionalLight, N_("Directional light"));
    lightType.setSymbol(PointLight, N_("Point light"));
    lightType.setSymbol(SpotLight, N_("Spot light"));

    lightPosTransform = new SgPosTransform;
    lightPosTransform->setTranslation(Vector3(0.0, 0.0, 3.0));
    isMarkerEnabled = false;
    
    // Get the default values
    static SgSpotLightPtr refLight = new SgSpotLight;
    color = refLight->color();
    intensity = refLight->intensity();
    direction = refLight->direction();
    ambientIntensity = refLight->ambientIntensity();
    constantAttenuation = refLight->constantAttenuation();
    linearAttenuation = refLight->linearAttenuation();
    quadraticAttenuation = refLight->quadraticAttenuation();
    beamWidth = refLight->beamWidth();
    cutOffAngle = refLight->cutOffAngle();
    cutOffExponent = refLight->cutOffExponent();
    on = true;

    setLightType(SpotLight);
}


LightingItem::Impl::Impl(LightingItem* self, const Impl& org)
    : self(self),
      lightType(org.lightType)
{
    lightPosTransform = new SgPosTransform(*org.lightPosTransform);
    isMarkerEnabled = org.isMarkerEnabled;
    
    color = org.color;
    intensity = org.intensity;
    ambientIntensity = org.ambientIntensity;
    constantAttenuation = org.constantAttenuation;
    linearAttenuation = org.linearAttenuation;
    quadraticAttenuation = org.quadraticAttenuation;
    direction = org.direction;
    beamWidth = org.beamWidth;
    cutOffAngle = org.cutOffAngle;
    cutOffExponent = org.cutOffExponent;
    on = org.on;

    setLightType(static_cast<LightType>(lightType.which()));
}


void LightingItem::setLightType(LightType type)
{
    impl->setLightType(type);
}


void LightingItem::Impl::setLightType(LightType type)
{
    lightType.select(type);

    if(!lightPosTransform->empty()){
        updateLight();
    }
}


void LightingItem::Impl::updateLight()
{
    switch(lightType.which()){
    case DirectionalLight: {
        auto directionalLight = new SgDirectionalLight;
        directionalLight->setDirection(direction);
        light = directionalLight;
        lightShape = getOrCreateDirectionalLightShape();
        break;
    }
    case PointLight: {
        auto pointLight = new SgPointLight;
        pointLight->setConstantAttenuation(constantAttenuation);
        pointLight->setLinearAttenuation(linearAttenuation);
        pointLight->setQuadraticAttenuation(quadraticAttenuation);
        light = pointLight;
        lightShape = getOrCreatePointLightShape();
        break;
    }
    case SpotLight: {
        auto spotLight = new SgSpotLight;
        spotLight->setConstantAttenuation(constantAttenuation);
        spotLight->setLinearAttenuation(linearAttenuation);
        spotLight->setQuadraticAttenuation(quadraticAttenuation);
        spotLight->setDirection(direction);
        spotLight->setBeamWidth(beamWidth);
        spotLight->setCutOffAngle(cutOffAngle);
        spotLight->setCutOffExponent(cutOffExponent);
        light = spotLight;
        lightShape = getOrCreateSpotLightShape();
        break;
    }
    default:
        break;
    }

    light->setColor(color);
    light->setIntensity(intensity);
    light->setAmbientIntensity(ambientIntensity);
    light->on(on);

    lightPosTransform->clearChildren();
    lightPosTransform->addChild(light);
    if(isMarkerEnabled){
        lightPosTransform->addChild(lightShape);
    }
    lightPosTransform->notifyUpdate(sgUpdate.withAction(SgUpdate::Added | SgUpdate::Removed));
}


void LightingItem::setTranslation(const Vector3& translation)
{
    impl->lightPosTransform->setTranslation(translation);
    impl->lightPosTransform->notifyUpdate();
}
    

void LightingItem::setDirection(const Vector3& direction)
{
    if(direction != impl->direction){
        impl->direction = direction;
        if(impl->light){
            if(auto directionalLight = dynamic_pointer_cast<SgDirectionalLight>(impl->light)){
                directionalLight->setDirection(direction);
                directionalLight->notifyUpdate(
                    impl->sgUpdate.withAction(SgUpdate::AppearanceModified));
            } else if(auto spotLight = dynamic_pointer_cast<SgSpotLight>(impl->light)){
                spotLight->setDirection(direction);
                spotLight->notifyUpdate(
                    impl->sgUpdate.withAction(SgUpdate::AppearanceModified));
            }
        }
    }
}


void LightingItem::setLightEnabled(bool on)
{
    if(on != impl->on){
        impl->on = on;
        if(impl->light){
            impl->light->on(on);
            impl->light->notifyUpdate(impl->sgUpdate.withAction(SgUpdate::AppearanceModified));
        }
    }
}


void LightingItem::setIntensity(float intensity)
{
    if(intensity != impl->intensity){
        impl->intensity = intensity;
        if(impl->light){
            impl->light->setIntensity(intensity);
            impl->light->notifyUpdate(impl->sgUpdate.withAction(SgUpdate::AppearanceModified));
        }
    }
}


void LightingItem::setAmbientIntensity(float intensity)
{
    if(intensity != impl->ambientIntensity){
        impl->ambientIntensity = intensity;
        if(impl->light){
            impl->light->setAmbientIntensity(intensity);
            impl->light->notifyUpdate(impl->sgUpdate.withAction(SgUpdate::AppearanceModified));
        }
    }
}


void LightingItem::setColor(const Vector3f& color)
{
    if(color != impl->color){
        impl->color = color;
        if(impl->light){
            impl->light->setColor(color);
            impl->light->notifyUpdate(impl->sgUpdate.withAction(SgUpdate::AppearanceModified));
        }
    }
}

    
void LightingItem::setConstantAttenuation(float a0)
{
    if(a0 != impl->constantAttenuation){
        impl->constantAttenuation = a0;
        if(auto pointLight = dynamic_pointer_cast<SgPointLight>(impl->light)){
            pointLight->setConstantAttenuation(a0);
            pointLight->notifyUpdate(impl->sgUpdate.withAction(SgUpdate::AppearanceModified));
        }
    }
}
    

void LightingItem::setLinearAttenuation(float a1)
{
    if(a1 != impl->linearAttenuation){
        impl->linearAttenuation = a1;
        if(auto pointLight = dynamic_pointer_cast<SgPointLight>(impl->light)){
            pointLight->setLinearAttenuation(a1);
            pointLight->notifyUpdate(impl->sgUpdate.withAction(SgUpdate::AppearanceModified));
        }
    }
}
    

void LightingItem::setQuadraticAttenuation(float a2)
{
    if(a2 != impl->quadraticAttenuation){
        impl->quadraticAttenuation = a2;
        if(auto pointLight = dynamic_pointer_cast<SgPointLight>(impl->light)){
            pointLight->setQuadraticAttenuation(a2);
            pointLight->notifyUpdate(impl->sgUpdate.withAction(SgUpdate::AppearanceModified));
        }
    }
}
    

void LightingItem::setBeamWidth(float w)
{
    if(w != impl->beamWidth){
        impl->beamWidth = w;
        if(auto spotLight = dynamic_pointer_cast<SgSpotLight>(impl->light)){
            spotLight->setBeamWidth(w);
            spotLight->notifyUpdate(impl->sgUpdate.withAction(SgUpdate::AppearanceModified));
        }
    }
}
    

void LightingItem::setCutOffAngle(float a)
{
    if(a != impl->cutOffAngle){
        impl->cutOffAngle = a;
        if(auto spotLight = dynamic_pointer_cast<SgSpotLight>(impl->light)){
            spotLight->setCutOffAngle(a);
            spotLight->notifyUpdate(impl->sgUpdate.withAction(SgUpdate::AppearanceModified));
        }
    }
}


void LightingItem::setCutOffExponent(float e)
{
    if(e != impl->cutOffExponent){
        impl->cutOffExponent = e;
        if(auto spotLight = dynamic_pointer_cast<SgSpotLight>(impl->light)){
            spotLight->setCutOffExponent(e);
            spotLight->notifyUpdate(impl->sgUpdate.withAction(SgUpdate::AppearanceModified));
        }
    }
}


void LightingItem::setLightMarkerEnabled(bool on)
{
    if(on != impl->isMarkerEnabled){
        impl->isMarkerEnabled = on;
        if(!impl->lightPosTransform->empty()){
            if(on){
                impl->lightPosTransform->addChildOnce(
                    impl->lightShape, impl->sgUpdate.withAction(SgUpdate::Added));
            } else {
                impl->lightPosTransform->removeChild(
                    impl->lightShape, impl->sgUpdate.withAction(SgUpdate::Removed));
            }
        }
    }
}


SgNode* LightingItem::getScene()
{
    if(impl->lightPosTransform->empty()){
        impl->updateLight();
    }
    return impl->lightPosTransform;
}


void LightingItem::doPutProperties(PutPropertyFunction& putProperty)
{
    return impl->doPutProperties(putProperty);
}


void LightingItem::Impl::doPutProperties(PutPropertyFunction& putProperty)
{
    putProperty(_("Light type"), lightType,
                [&](int index){ setLightType((LightType)index); return true; });

    putProperty(_("Translation"), str(Vector3(lightPosTransform->translation())),
                [&](const string& value){
                    Vector3 p;
                    if(toVector3(value, p)){
                        self->setTranslation(p);
                        return true;
                    }
                    return false;
                });
    
    if(lightType.is(DirectionalLight) || lightType.is(SpotLight)){
        putProperty(_("Direction"), str(direction),
                    [&](const string& value){
                        Vector3 d;
                        if(toVector3(value, d)){
                            self->setDirection(d);
                            return true;
                        }
                        return false;
                    });
    }
        
    putProperty(_("ON"), on,
                [&](bool value){ self->setLightEnabled(value); return true;});

    putProperty(_("Intensity"), intensity,
                [&](float value){ self->setIntensity(value); return true; });
    putProperty(_("Ambient intensity"), ambientIntensity,
                [&](float value){ self->setAmbientIntensity(value); return true; });

    putProperty(_("Color"), str(color),
                [&](const string& value){
                    Vector3f c;
                    if(toVector3(value, c)){
                        self->setColor(c);
                        return true;
                    }
                    return false;
                });

    if(lightType.is(PointLight) || lightType.is(SpotLight)){
        putProperty(_("Constant attenuation"), constantAttenuation,
                    [&](float value){ self->setConstantAttenuation(value); return true;});
        putProperty(_("Linear attenuation"), linearAttenuation,
                    [&](float value){ self->setLinearAttenuation(value); return true; });
        putProperty(_("Quadratic attenuation"), quadraticAttenuation,
                    [&](float value){ self->setQuadraticAttenuation(value); return true; });
    }
    
    if(lightType.is(SpotLight)){
        putProperty(_("Beam width"), degree(beamWidth),
                    [&](float angle){ self->setBeamWidth(radian(angle)); return true; });
        putProperty(_("Cut-off angle"), degree(cutOffAngle),
                    [&](float angle){ self->setCutOffAngle(radian(angle)); return true; });
        putProperty(_("Cut-off exponent"), cutOffExponent,
                    [&](float value){ self->setCutOffExponent(value); return true; });
    }

    putProperty(_("Show marker"), isMarkerEnabled,
                [&](bool on){ self->setLightMarkerEnabled(on); return true; });
}


bool LightingItem::store(Archive& archive)
{
    return impl->store(archive);
}


bool LightingItem::Impl::store(Archive& archive)
{
    archive.write("light_type", lightType.selectedSymbol(), DOUBLE_QUOTED);
    write(archive, "translation", lightPosTransform->translation());
    write(archive, "direction", direction);
    archive.write("on", on);
    write(archive, "color", color);
    archive.write("intensity", intensity);
    archive.write("ambient_intensity", ambientIntensity);
    archive.write("constant_attenuation", constantAttenuation);
    archive.write("linear_attenuation", linearAttenuation);
    archive.write("quadratic_attenuation", quadraticAttenuation);
    archive.write("beam_width", degree(beamWidth));
    archive.write("cutoff_angle", degree(cutOffAngle));
    archive.write("cutoff_exponent", cutOffExponent);
    archive.write("show_marker", isMarkerEnabled);
    return true;
}


bool LightingItem::restore(const Archive& archive)
{
    return impl->restore(archive);
}


bool LightingItem::Impl::restore(const Archive& archive)
{
    string symbol;
    if(archive.read("light_type", symbol)){
        lightType.select(symbol);
    }
    Vector3 translation;
    if(read(archive, "translation", translation)){
        lightPosTransform->setTranslation(translation);
    }
    read(archive, "direction", direction);
    archive.read("on", on);
    read(archive, "color", color);
    archive.read("intensity", intensity);
    archive.read("ambient_intensity", ambientIntensity);
    archive.read("constant_attenuation", constantAttenuation);
    archive.read("linear_attenuation", linearAttenuation);
    archive.read("quadratic_attenuation", quadraticAttenuation);
    double angle;
    if(archive.read("beam_width", angle)){
        beamWidth = radian(angle);
    }
    if(archive.read("cutoff_angle", angle)){
        cutOffAngle = radian(angle);
    }
    archive.read("cutoff_exponent", cutOffExponent);
    archive.read("show_marker", isMarkerEnabled);

    setLightType((LightType)lightType.selectedIndex());
                       
    return true;
}


namespace {

SgMaterial* getOrCreateLightMaterial()
{
    if(!lightMaterial){
        lightMaterial = new SgMaterial;
        lightMaterial->setDiffuseColor(Vector3f(1.0f, 1.0f, 0.0f));
        lightMaterial->setAmbientIntensity(0.2f);
    }
    return lightMaterial;
}


SgShape* getOrCreateLightSphere()
{
    if(!lightSphere){
        lightSphere = new SgShape;
        lightSphere->setMesh(
            MeshGenerator::mainThreadInstance()->generateSphere(0.05, false));
        lightSphere->setMaterial(getOrCreateLightMaterial());
    }
    return lightSphere;
}


SgShape* getOrCreateLightCapsule()
{
    if(lightCapsule){
        lightCapsule = new SgShape;
        lightCapsule->setMesh(
            MeshGenerator::mainThreadInstance()->generateCapsule(0.005, 0.03));
        lightCapsule->setMaterial(getOrCreateLightMaterial());
    }
    return lightCapsule;
}


SgGroup* getOrCreateDirectionalLightShape()
{
    if(!directionalLightShape){
        directionalLightShape = new SgGroup;
        directionalLightShape->addChild(getOrCreateLightSphere());
        
        static const std::vector<Vector3> d_pos = {
            Vector3(0.0, 0.0, -0.07), Vector3(0, 0.04, -0.06),
            Vector3(0.0, -0.04, -0.06), Vector3(0.04, 0, -0.06),
            Vector3(-0.04, 0, -0.06)
        };

        auto capsule = getOrCreateLightCapsule();
        for(size_t i=0; i < d_pos.size(); ++i){
            auto cT = new SgPosTransform;
            cT->setRotation(AngleAxis(radian(90), Vector3::UnitX()));
            cT->setTranslation(d_pos[i]);
            cT->addChild(capsule);
            directionalLightShape->addChild(cT);
        }
    }
    return directionalLightShape;
}

    
SgGroup* getOrCreatePointLightShape()
{
    if(!pointLightShape){
        pointLightShape = new SgGroup;
        pointLightShape->addChild(getOrCreateLightSphere());

        static const std::vector<Vector3> p_pos = {
            Vector3(0.0, 0.0, 0.07), Vector3(0.0, 0.0, -0.07),
            Vector3(0.07, 0.0, 0.0), Vector3(-0.07, 0.0, 0.0),
            Vector3(0.0, 0.07, 0.0), Vector3(0.0, -0.07, 0.0)
        };
        static const std::vector<AngleAxis> p_att = {
            AngleAxis(radian(90.0), Vector3::UnitX()), AngleAxis(radian(90.0), Vector3::UnitX()),
            AngleAxis(radian(90.0), Vector3::UnitZ()), AngleAxis(radian(90.0), Vector3::UnitZ()),
            AngleAxis(radian(0.0),  Vector3::UnitZ()), AngleAxis(radian(0.0),  Vector3::UnitZ())
        };
        
        auto capsule = getOrCreateLightCapsule();
        for(size_t i=0; i < p_pos.size(); ++i){
            auto cT = new SgPosTransform;
            cT->setRotation(p_att[i]);
            cT->setTranslation(p_pos[i]);
            cT->addChild(capsule);
            pointLightShape->addChild(cT);
        }
    }
    return pointLightShape;
}


SgGroup* getOrCreateSpotLightShape()
{
    if(!spotLightShape){
        auto meshGenerator = MeshGenerator::mainThreadInstance();
        auto material = getOrCreateLightMaterial();
        spotLightShape = new SgGroup;
        auto box = new SgShape;
        box->setMesh(meshGenerator->generateBox(Vector3(0.07, 0.07, 0.07)));
        box->setMaterial(material);
        auto cone = new SgShape;
        cone->setMesh(meshGenerator->generateCone(0.07, 0.07));
        cone->setMaterial(material);
        auto coneT = new SgPosTransform;
        coneT->setRotation(AngleAxis(radian(90), Vector3::UnitX()));
        coneT->addChild(cone);
        spotLightShape->addChild(box);
        spotLightShape->addChild(coneT);

        static const std::vector<Vector3> s_pos = {
            Vector3(0, 0, -0.055), Vector3(0.0, 0.05, -0.055)
        };
        static const std::vector<AngleAxis> s_att = {
            AngleAxis(radian(90.0), Vector3::UnitX()), AngleAxis(radian(125), Vector3::UnitX())
        };
        auto capsule = getOrCreateLightCapsule();

        auto cT0 = new SgPosTransform;
        cT0->setRotation(s_att[0]);
        cT0->setTranslation(s_pos[0]);
        cT0->addChild(capsule);
        spotLightShape->addChild(cT0);

        for(int i=0; i < 4; ++i){
            auto cT1 = new SgPosTransform;
            cT1->setRotation(s_att[1]);
            cT1->setTranslation(s_pos[1]);
            cT1->addChild(capsule);
            auto cT2 = new SgPosTransform;
            cT2->setRotation(AngleAxis(radian(90.0 * i), Vector3::UnitZ()));
            cT2->addChild(cT1);
            spotLightShape->addChild(cT2);
        }
    }
    return spotLightShape;
}

}
