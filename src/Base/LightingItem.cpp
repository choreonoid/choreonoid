/*!
  @file
  @author Shizuko Hattori
*/

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

enum LightType { DIRECTIONAL=0, POINT, SPOT, N_LIGHT_TYPES };

}

namespace cnoid {

class LightingItem::Impl
{
public:
    LightingItem* self;
    SgPosTransformPtr scene;
    Selection lightType;
    SgLight* light;

    Vector3f color;
    float intensity;
    float ambientIntensity;
    bool on;
    // SPOT & DIRECTIONAL
    Vector3 direction;
    //POINT
    float constantAttenuation;
    float linearAttenuation;
    float quadraticAttenuation;
    //SPOT
    float beamWidth;
    float cutOffAngle;
    float cutOffExponent;

    bool showMarker;
    SgGroup* lightShape;
    SgGroupPtr dlightShape;
    SgGroupPtr plightShape;
    SgGroupPtr slightShape;

    Impl(LightingItem* self);
    Impl(LightingItem* self, const Impl& org);
    bool updateLightType(LightType type);
    void directionToMatrix3(Vector3& direction, Matrix3& R);
    bool onTranslationPropertyChanged(const std::string& value);
    bool onDirectionPropertyChanged(const std::string& value);
    bool onColorPropertyChanged(const std::string& value);
    bool onIntensityPropertyChanged(float value);
    bool onAmbientIntensityPropertyChanged(float value);
    bool onSwitchPropertyChanged(bool value);
    bool onConstantAttenuationPropertyChanged(float value);
    bool onLinearAttenuationPropertyChanged(float value);
    bool onQuadraticAttenuationPropertyChanged(float value);
    bool onBeamWidthPropertyChanged(float value);
    bool onCutOffAnglePropertyChanged(float angle);
    bool onCutOffExponentPropertyChanged(float value);
    bool onShowMarkerPropertyChanged(bool on);
    void genarateLightShape();
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

Item* LightingItem::doDuplicate() const
{
    return new LightingItem(*this);
}

SgNode* LightingItem::getScene()
{
    return impl->scene;
}


LightingItem::Impl::Impl(LightingItem* self)
    : self(self),
      lightType(N_LIGHT_TYPES, CNOID_GETTEXT_DOMAIN_NAME)
{
    lightType.setSymbol(DIRECTIONAL, N_("Directional light"));
    lightType.setSymbol(POINT, N_("Point light"));
    lightType.setSymbol(SPOT, N_("Spot light"));

    on = true;

    // get default value
    SgSpotLight light;
    color = light.color();
    intensity = light.intensity();
    ambientIntensity = light.ambientIntensity();
    constantAttenuation = light.constantAttenuation();
    linearAttenuation = light.linearAttenuation();
    quadraticAttenuation = light.quadraticAttenuation();
    direction = Vector3(0, 0, -1);
    beamWidth = light.beamWidth();
    cutOffAngle = light.cutOffAngle();
    cutOffExponent = light.cutOffExponent();

    showMarker = true;
    scene = new SgPosTransform;
    scene->setTranslation(Vector3(0,0,3));
    genarateLightShape();
    updateLightType(SPOT);
}


void LightingItem::Impl::genarateLightShape()
{
    SgMaterial* material = new SgMaterial;
    material->setDiffuseColor(Vector3f(1.0f, 1.0f, 0.0f));
    material->setAmbientIntensity(0.2f);
    dlightShape = new SgGroup;
    MeshGenerator meshGenerator;
    SgShape* sphere = new SgShape;
    sphere->setMesh(meshGenerator.generateSphere(0.05, false));
    sphere->setMaterial(material);
    dlightShape->addChild(sphere);
    SgShape* capsule = new SgShape;
    capsule->setMesh(meshGenerator.generateCapsule(0.005, 0.03));
    capsule->setMaterial(material);

    static const std::vector<Vector3> d_pos = {
            Vector3(0, 0, -0.07), Vector3(0, 0.04, -0.06),
            Vector3(0, -0.04, -0.06), Vector3(0.04, 0, -0.06),
            Vector3(-0.04, 0, -0.06)
    };
    for(size_t i=0; i < d_pos.size(); i++){
        SgPosTransform* cT = new SgPosTransform;
        cT->setRotation(AngleAxis(radian(90), Vector3(1, 0, 0)));
        cT->setTranslation(d_pos[i]);
        cT->addChild(capsule);
        dlightShape->addChild(cT);
    }

    plightShape = new SgGroup;
    plightShape->addChild(sphere);

    static const std::vector<Vector3> p_pos = {
            Vector3(0, 0, 0.07), Vector3(0, 0, -0.07),
            Vector3(0.07, 0, 0), Vector3(-0.07, 0, 0),
            Vector3(0, 0.07, 0), Vector3(0, -0.07, 0)
    };
    static const std::vector<AngleAxis> p_att = {
            AngleAxis(radian(90), Vector3(1, 0, 0)), AngleAxis(radian(90), Vector3(1, 0, 0)),
            AngleAxis(radian(90), Vector3(0, 0, 1)), AngleAxis(radian(90), Vector3(0, 0, 1)),
            AngleAxis(radian(0), Vector3(0, 0, 1)), AngleAxis(radian(0), Vector3(0, 0, 1))
    };
    for(size_t i=0; i < p_pos.size(); i++){
        SgPosTransform* cT = new SgPosTransform;
        cT->setRotation(p_att[i]);
        cT->setTranslation(p_pos[i]);
        cT->addChild(capsule);
        plightShape->addChild(cT);
    }

    slightShape = new SgGroup;
    SgShape* box = new SgShape;
    box->setMesh(meshGenerator.generateBox(Vector3(0.07, 0.07, 0.07)));
    box->setMaterial(material);
    SgShape* cone = new SgShape;
    cone->setMesh(meshGenerator.generateCone(0.07, 0.07));
    cone->setMaterial(material);
    SgPosTransform* coneT = new SgPosTransform;
    coneT->setRotation(AngleAxis(radian(90), Vector3(1, 0, 0)));
    coneT->addChild(cone);
    slightShape->addChild(box);
    slightShape->addChild(coneT);

    static const std::vector<Vector3> s_pos = {
            Vector3(0, 0, -0.055), Vector3(0, 0.05, -0.055)
    };
    static const std::vector<AngleAxis> s_att = {
            AngleAxis(radian(90), Vector3(1, 0, 0)), AngleAxis(radian(125), Vector3(1, 0, 0))
    };
    SgPosTransform* cT0 = new SgPosTransform;
    cT0->setRotation(s_att[0]);
    cT0->setTranslation(s_pos[0]);
    cT0->addChild(capsule);
    slightShape->addChild(cT0);
    for(int i=0; i<4; i++){
        SgPosTransform* cT1 = new SgPosTransform;
        cT1->setRotation(s_att[1]);
        cT1->setTranslation(s_pos[1]);
        cT1->addChild(capsule);
        SgPosTransform* cT2 = new SgPosTransform;
        cT2->setRotation(AngleAxis(radian(90*i), Vector3(0, 0, 1)));
        cT2->addChild(cT1);
        slightShape->addChild(cT2);
    }
}


LightingItem::Impl::Impl(LightingItem* self, const Impl& org)
    : self(self),
      lightType(org.lightType),
      on(org.on)
{
    light = org.light;
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
    dlightShape = org.dlightShape;
    plightShape = org.plightShape;
    slightShape = org.slightShape;
    showMarker = org.showMarker;
    lightShape = org.lightShape;
    scene = new SgPosTransform(*org.scene);
}


bool LightingItem::Impl::updateLightType(LightType type)
{
    lightType.select(type);
    scene->clearChildren();
    switch(lightType.which()){
    case DIRECTIONAL :
        light = new SgDirectionalLight;
        ((SgDirectionalLight*)light)->setDirection(Vector3(0, 0, -1));
        lightShape = dlightShape.get();
        break;
    case POINT :
        light = new SgPointLight;
        ((SgPointLight*)light)->setConstantAttenuation(constantAttenuation);
        ((SgPointLight*)light)->setLinearAttenuation(linearAttenuation);
        ((SgPointLight*)light)->setQuadraticAttenuation(quadraticAttenuation);
        lightShape = plightShape.get();
        break;
    case SPOT :
        light = new SgSpotLight;
        ((SgPointLight*)light)->setConstantAttenuation(constantAttenuation);
        ((SgPointLight*)light)->setLinearAttenuation(linearAttenuation);
        ((SgPointLight*)light)->setQuadraticAttenuation(quadraticAttenuation);
        ((SgSpotLight*)light)->setDirection(Vector3(0, 0, -1));
        ((SgSpotLight*)light)->setBeamWidth(beamWidth);
        ((SgSpotLight*)light)->setCutOffAngle(cutOffAngle);
        ((SgSpotLight*)light)->setCutOffExponent(cutOffExponent);
        lightShape = slightShape.get();
        break;
    }

    light->setColor(color);
    light->setIntensity(intensity);
    light->setAmbientIntensity(ambientIntensity);
    light->on(on);
    if(showMarker){
        scene->addChild(lightShape);
    }
    SgTmpUpdate update;
    scene->addChild(light, update);

    return true;
}


bool LightingItem::Impl::onTranslationPropertyChanged(const string& value)
{
    Vector3 p;
    if(toVector3(value, p)){
        scene->setTranslation(p);
        scene->notifyUpdate();
        return true;
    }
    return false;
}


void LightingItem::Impl::directionToMatrix3(Vector3& direction, Matrix3& R)
{
    Vector3 nx = Vector3::UnitZ().cross(-direction);
    if(nx.norm()==0){
        R = Matrix3::Identity();
        if(-direction!=Vector3::UnitZ()){
            R.col(1) = -Vector3::UnitY();
            R.col(2) = -Vector3::UnitZ();
        }
    }else{
        nx.normalize();
        Vector3 ny = -direction.cross(nx).normalized();
        R.col(0) = nx;
        R.col(1) = ny;
        R.col(2) = -direction;
    }
}


bool LightingItem::Impl::onDirectionPropertyChanged(const std::string& value)
{
    Vector3 direction_;
    if(toVector3(value, direction_)){
        direction_.normalize();
        direction = direction_;
        Matrix3 R;
        directionToMatrix3(direction, R);

        scene->setRotation(R);
        scene->notifyUpdate();
        return true;
    }
    return false;
}


bool LightingItem::Impl::onColorPropertyChanged(const std::string& value)
{
    Vector3f color_;
    if(toVector3(value, color_)){
        color = color_;
        light->setColor(color);
        light->notifyUpdate();
        return true;
    }
    return false;
}


bool LightingItem::Impl::onIntensityPropertyChanged(float value)
{
    intensity = value;
    light->setIntensity(intensity);
    light->notifyUpdate();
    return true;
}


bool LightingItem::Impl::onAmbientIntensityPropertyChanged(float value)
{
    ambientIntensity = value;
    light->setAmbientIntensity(ambientIntensity);
    light->notifyUpdate();
    return true;
}


bool LightingItem::Impl::onSwitchPropertyChanged(bool value)
{
    on = value;
    light->on(on);
    light->notifyUpdate();
    return true;
}


bool LightingItem::Impl::onConstantAttenuationPropertyChanged(float value)
{
    constantAttenuation = value;
    SgPointLight* pointLight = dynamic_cast<SgPointLight*>(light);
    if(pointLight){
        pointLight->setConstantAttenuation(constantAttenuation);
        pointLight->notifyUpdate();
        return true;
    }
    return false;
}


bool LightingItem::Impl::onLinearAttenuationPropertyChanged(float value)
{
    linearAttenuation = value;
    SgPointLight* pointLight = dynamic_cast<SgPointLight*>(light);
    if(pointLight){
        pointLight->setLinearAttenuation(linearAttenuation);
        pointLight->notifyUpdate();
        return true;
    }
    return false;
}


bool LightingItem::Impl::onQuadraticAttenuationPropertyChanged(float value)
{
    quadraticAttenuation = value;
    SgPointLight* pointLight = dynamic_cast<SgPointLight*>(light);
    if(pointLight){
        pointLight->setQuadraticAttenuation(quadraticAttenuation);
        pointLight->notifyUpdate();
        return true;
    }
    return false;
}


bool LightingItem::Impl::onBeamWidthPropertyChanged(float value)
{
    beamWidth = value;
    SgSpotLight* spotLight = dynamic_cast<SgSpotLight*>(light);
    if(spotLight){
        spotLight->setBeamWidth(beamWidth);
        spotLight->notifyUpdate();
        return true;
    }
    return false;
}


bool LightingItem::Impl::onCutOffAnglePropertyChanged(float angle)
{
    cutOffAngle = angle;
    SgSpotLight* spotLight = dynamic_cast<SgSpotLight*>(light);
    if(spotLight){
        spotLight->setCutOffAngle(cutOffAngle);
        spotLight->notifyUpdate();
        return true;
    }
    return false;
}


bool LightingItem::Impl::onCutOffExponentPropertyChanged(float value)
{
    cutOffExponent = value;
    SgSpotLight* spotLight = dynamic_cast<SgSpotLight*>(light);
    if(spotLight){
        spotLight->setCutOffExponent(cutOffExponent);
        spotLight->notifyUpdate();
        return true;
    }
    return false;
}


bool LightingItem::Impl::onShowMarkerPropertyChanged(bool on)
{
    showMarker = on;
    SgTmpUpdate update;
    if(showMarker){
        scene->addChildOnce(lightShape, update);
    }else{
        scene->removeChild(lightShape, update);
    }
    return true;
}


void LightingItem::doPutProperties(PutPropertyFunction& putProperty)
{
    return impl->doPutProperties(putProperty);
}


void LightingItem::Impl::doPutProperties(PutPropertyFunction& putProperty)
{
    putProperty(_("Light type"), lightType,
                [&](int index){ return updateLightType((LightType)index); });
    putProperty(_("Translation"), str(Vector3(scene->translation())),
                [&](const string& value){ return onTranslationPropertyChanged(value); });
    
    LightType type = (LightType)lightType.selectedIndex();
    if(type == DIRECTIONAL || type == SPOT){
        putProperty(_("Direction"), str(direction),
                    [&](const string& value){ return onDirectionPropertyChanged(value);});
    }

    putProperty(_("ON"), on,
                [&](bool value){ return onSwitchPropertyChanged(value); });
    putProperty(_("Color"), str(color),
                [&](const string& value){ return onColorPropertyChanged(value); });
    putProperty(_("Intensity"), intensity,
                [&](float value){ return onIntensityPropertyChanged(value); });
    putProperty(_("Ambient intensity"), ambientIntensity,
                [&](float value){ return onAmbientIntensityPropertyChanged(value); });

    if(type == POINT || type == SPOT){
        putProperty(_("Constant attenuation"), constantAttenuation,
                    [&](float value){ return onConstantAttenuationPropertyChanged(value); });
        putProperty(_("Linear attenuation"), linearAttenuation,
                    [&](float value){ return onLinearAttenuationPropertyChanged(value); });
        putProperty(_("Quadratic attenuation"), quadraticAttenuation,
                    [&](float value){ return onQuadraticAttenuationPropertyChanged(value); });
    }
    if(type == SPOT){
        putProperty(_("Beam width"), degree(beamWidth),
                    [&](float angle){ return onBeamWidthPropertyChanged(radian(angle)); });
        putProperty(_("Cut-off angle"), degree(cutOffAngle),
                    [&](float angle){ return onCutOffAnglePropertyChanged(radian(angle)); });
        putProperty(_("Cut-off Exponent"), cutOffExponent,
                    [&](float value){ return onCutOffExponentPropertyChanged(value); });
    }

    putProperty(_("Show Marker"), showMarker,
            [&](bool on){ return onShowMarkerPropertyChanged(on); });
}


bool LightingItem::store(Archive& archive)
{
    return impl->store(archive);
}


bool LightingItem::Impl::store(Archive& archive)
{
    archive.write("light_type", lightType.selectedSymbol(), DOUBLE_QUOTED);
    write(archive, "translation", scene->translation());
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
    archive.write("show_marker", showMarker);
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
        scene->setTranslation(translation);
    }
    if(read(archive, "direction", direction)){
        Matrix3 R;
        directionToMatrix3(direction, R);
        scene->setRotation(R);
    }
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
    archive.read("show_marker", showMarker);

    updateLightType((LightType)lightType.selectedIndex());
                       
    return true;
}
