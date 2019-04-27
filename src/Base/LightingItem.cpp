/*!
  @file
  @author Shizuko Hattori
*/

#include "LightingItem.h"
#include <cnoid/ItemManager>
#include <cnoid/SceneGraph>
#include <cnoid/SceneLights>
#include <cnoid/MeshGenerator>
#include <cnoid/EigenUtil>
#include <cnoid/Archive>
#include <cnoid/EigenArchive>
#include "gettext.h"

using namespace std;
using namespace cnoid;

namespace {

enum LightType { DIRECTIONAL=0, POINT, SPOT, N_LIGHT_TYPES };

}

namespace cnoid {

class LightingItemImpl
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

    LightingItemImpl(LightingItem* self);
    LightingItemImpl(LightingItem* self, const LightingItemImpl& org);
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
    bool onCutOffAnglePropertyChanged(float value);
    bool onCutOffExponentPropertyChanged(float value);
    bool onShowMarkerPropertyChanged(bool on);
    void genarateLightShape();
};
}


void LightingItem::initializeClass(ExtensionManager* ext)
{
    static bool initialized = false;
    if(!initialized){
        ext->itemManager().registerClass<LightingItem>(N_("LightingItem"));
        ext->itemManager().addCreationPanel<LightingItem>();
        initialized = true;
    }
}


LightingItem::LightingItem()
{
    impl = new LightingItemImpl(this);
}


LightingItem::LightingItem(const LightingItem& org)
    : Item(org)
{
    impl = new LightingItemImpl(this, *org.impl);
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


LightingItemImpl::LightingItemImpl(LightingItem* self)
    : self(self),
      lightType(N_LIGHT_TYPES, CNOID_GETTEXT_DOMAIN_NAME)
{
    lightType.setSymbol(DIRECTIONAL, _("Directional light"));
    lightType.setSymbol(POINT, _("Point light"));
    lightType.setSymbol(SPOT, _("Spot light"));

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


void LightingItemImpl::genarateLightShape()
{
    SgMaterial* material = new SgMaterial;
    material->setDiffuseColor(Vector3f(1.0f, 1.0f, 0.0f));
    material->setAmbientIntensity(0.2f);
    material->setShininess(0.2f);
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
    box->setMesh(meshGenerator.generateBox(Vector3(0.07, 0.07, 0.07), false));
    box->setMaterial(material);
    SgShape* cone = new SgShape;
    cone->setMesh(meshGenerator.generateCone(0.07, 0.07, true, true, false));
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


LightingItemImpl::LightingItemImpl(LightingItem* self, const LightingItemImpl& org)
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


bool LightingItemImpl::updateLightType(LightType type)
{
    lightType.select(type);
    scene->clearChildren(false);
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
        scene->addChild(lightShape, false);
    }
    scene->addChild(light, true);

    return true;
}


bool LightingItemImpl::onTranslationPropertyChanged(const string& value)
{
    Vector3 p;
    if(toVector3(value, p)){
        scene->setTranslation(p);
        scene->notifyUpdate();
        return true;
    }
    return false;
}


void LightingItemImpl::directionToMatrix3(Vector3& direction, Matrix3& R)
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


bool LightingItemImpl::onDirectionPropertyChanged(const std::string& value)
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


bool LightingItemImpl::onColorPropertyChanged(const std::string& value)
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


bool LightingItemImpl::onIntensityPropertyChanged(float value)
{
    intensity = value;
    light->setIntensity(intensity);
    light->notifyUpdate();
    return true;
}


bool LightingItemImpl::onAmbientIntensityPropertyChanged(float value)
{
    ambientIntensity = value;
    light->setAmbientIntensity(ambientIntensity);
    light->notifyUpdate();
    return true;
}


bool LightingItemImpl::onSwitchPropertyChanged(bool value)
{
    on = value;
    light->on(on);
    light->notifyUpdate();
    return true;
}


bool LightingItemImpl::onConstantAttenuationPropertyChanged(float value)
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


bool LightingItemImpl::onLinearAttenuationPropertyChanged(float value)
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


bool LightingItemImpl::onQuadraticAttenuationPropertyChanged(float value)
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


bool LightingItemImpl::onBeamWidthPropertyChanged(float value)
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


bool LightingItemImpl::onCutOffAnglePropertyChanged(float value)
{
    cutOffAngle = value;
    SgSpotLight* spotLight = dynamic_cast<SgSpotLight*>(light);
    if(spotLight){
        spotLight->setCutOffAngle(cutOffAngle);
        spotLight->notifyUpdate();
        return true;
    }
    return false;
}


bool LightingItemImpl::onCutOffExponentPropertyChanged(float value)
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


bool LightingItemImpl::onShowMarkerPropertyChanged(bool on)
{
    showMarker = on;
    if(showMarker){
        scene->addChildOnce(lightShape, true);
    }else{
        scene->removeChild(lightShape, true);
    }
    return true;
}


void LightingItem::doPutProperties(PutPropertyFunction& putProperty)
{
    putProperty(_("Light Type"), impl->lightType,
            [&](int index){ return impl->updateLightType((LightType)index); });
    putProperty(_("Translation"), str(Vector3(impl->scene->translation())),
            [&](const string& value){ return impl->onTranslationPropertyChanged(value); });

    LightType type = (LightType)impl->lightType.selectedIndex();
    if(type == DIRECTIONAL || type == SPOT){
        putProperty(_("Direction"), str(impl->direction),
                [&](const string& value){ return impl->onDirectionPropertyChanged(value);});
    }

    putProperty(_("ON"), impl->on,
            [&](bool value){ return impl->onSwitchPropertyChanged(value); });
    putProperty(_("Color"), str(impl->color),
            [&](const string& value){ return impl->onColorPropertyChanged(value); });
    putProperty(_("Intensity"), impl->intensity,
            [&](float value){ return impl->onIntensityPropertyChanged(value); });
    putProperty(_("AmbientIntensity"), impl->ambientIntensity,
            [&](float value){ return impl->onAmbientIntensityPropertyChanged(value); });

    if(type == POINT || type == SPOT){
        putProperty(_("ConstantAttenuation"), impl->constantAttenuation,
                [&](float value){ return impl->onConstantAttenuationPropertyChanged(value); });
        putProperty(_("LinearAttenuation"), impl->linearAttenuation,
                [&](float value){ return impl->onLinearAttenuationPropertyChanged(value); });
        putProperty(_("QuadraticAttenuation"), impl->quadraticAttenuation,
                [&](float value){ return impl->onQuadraticAttenuationPropertyChanged(value); });
    }
    if(type == SPOT){
        putProperty(_("BeamWidth"), impl->beamWidth,
                [&](float value){ return impl->onBeamWidthPropertyChanged(value); });
        putProperty(_("CutOffAngle"), impl->cutOffAngle,
                [&](float value){ return impl->onCutOffAnglePropertyChanged(value); });
        putProperty(_("CutOffExponent"), impl->cutOffExponent,
                [&](float value){ return impl->onCutOffExponentPropertyChanged(value); });
    }

    putProperty(_("Show Marker"), impl->showMarker,
            [&](bool on){ return impl->onShowMarkerPropertyChanged(on); });
}


bool LightingItem::store(Archive& archive)
{
    archive.write("lightType", impl->lightType.selectedSymbol(), DOUBLE_QUOTED);
    write(archive, "translation", impl->scene->translation());
    write(archive, "direction", impl->direction);
    archive.write("on", impl->on);
    write(archive, "color", impl->color);
    archive.write("intensity", impl->intensity);
    archive.write("ambientIntensity", impl->ambientIntensity);
    archive.write("constantAttenuation", impl->constantAttenuation);
    archive.write("linearAttenuation", impl->linearAttenuation);
    archive.write("quadraticAttenuation", impl->quadraticAttenuation);
    archive.write("beamWidth", impl->beamWidth);
    archive.write("cutOffAngle", impl->cutOffAngle);
    archive.write("cutOffExponent", impl->cutOffExponent);
    archive.write("showMarker", impl->showMarker);

    return true;
}


bool LightingItem::restore(const Archive& archive)
{
    string symbol;
    if(archive.read("lightType", symbol)){
        impl->lightType.select(symbol);
    }
    Vector3 translation;
    if(read(archive, "translation", translation)){
        impl->scene->setTranslation(translation);
    }
    if(read(archive, "direction", impl->direction)){
        Matrix3 R;
        impl->directionToMatrix3(impl->direction, R);
        impl->scene->setRotation(R);
    }
    archive.read("on", impl->on);
    read(archive, "color", impl->color);
    archive.read("intensity", impl->intensity);
    archive.read("ambientIntensity", impl->ambientIntensity);
    archive.read("constantAttenuation", impl->constantAttenuation);
    archive.read("linearAttenuation", impl->linearAttenuation);
    archive.read("quadraticAttenuation", impl->quadraticAttenuation);
    archive.read("beamWidth", impl->beamWidth);
    archive.read("cutOffAngle", impl->cutOffAngle);
    archive.read("cutOffExponent", impl->cutOffExponent);
    archive.read("showMarker", impl->showMarker);

    impl->updateLightType((LightType)impl->lightType.selectedIndex());
    return true;
}
