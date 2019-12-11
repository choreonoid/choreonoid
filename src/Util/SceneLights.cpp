/*!
  @file
  @author Shin'ichiro Nakaoka
*/

#include "SceneLights.h"
#include "SceneNodeClassRegistry.h"

using namespace std;
using namespace cnoid;


SgLight::SgLight(int classId)
    : SgPreprocessed(classId)
{
    on_ = true;
    color_.setOnes();
    intensity_ = 1.0f;
    ambientIntensity_ = 0.0f;
}


SgLight::SgLight(const SgLight& org)
    : SgPreprocessed(org)
{
    on_ = org.on_;
    color_ = org.color_;
    intensity_ = org.intensity_;
    ambientIntensity_ = org.ambientIntensity_;
}


Referenced* SgLight::doClone(CloneMap*) const
{
    return new SgLight(*this);
}


SgDirectionalLight::SgDirectionalLight(int classId)
    : SgLight(classId)
{
    direction_ << 0.0, 0.0, -1.0;
}


SgDirectionalLight::SgDirectionalLight()
    : SgDirectionalLight(findClassId<SgDirectionalLight>())
{

}


SgDirectionalLight::SgDirectionalLight(const SgDirectionalLight& org)
    : SgLight(org)
{
    direction_ = org.direction_;
}


Referenced* SgDirectionalLight::doClone(CloneMap*) const
{
    return new SgDirectionalLight(*this);
}


SgPointLight::SgPointLight(int classId)
    : SgLight(classId)
{
    constantAttenuation_ = 1.0f;
    linearAttenuation_ = 0.0f;
    quadraticAttenuation_ = 0.0f;
}


SgPointLight::SgPointLight()
    : SgPointLight(findClassId<SgPointLight>())
{

}


SgPointLight::SgPointLight(const SgPointLight& org)
    : SgLight(org)
{
    constantAttenuation_ = org.constantAttenuation_;
    linearAttenuation_ = org.linearAttenuation_;
    quadraticAttenuation_ = org.quadraticAttenuation_;
}


Referenced* SgPointLight::doClone(CloneMap*) const
{
    return new SgPointLight(*this);
}


SgSpotLight::SgSpotLight(int classId)
    : SgPointLight(classId)
{
    direction_ << 0.0, 0.0, -1.0;
    beamWidth_ = 1.570796f;
    cutOffAngle_ = 0.785398f;
    cutOffExponent_ = 1.0f;
}


SgSpotLight::SgSpotLight()
    : SgSpotLight(findClassId<SgSpotLight>())
{

}


SgSpotLight::SgSpotLight(const SgSpotLight& org)
    : SgPointLight(org)
{
    direction_ = org.direction_;
    beamWidth_ = org.beamWidth_;
    cutOffAngle_ = org.cutOffAngle_;
    cutOffExponent_ = org.cutOffExponent_;
}


Referenced* SgSpotLight::doClone(CloneMap*) const
{
    return new SgSpotLight(*this);
}


namespace {

struct NodeTypeRegistration {
    NodeTypeRegistration() {
        SceneNodeClassRegistry::instance()
            .registerClass<SgLight, SgPreprocessed>()
            .registerClass<SgDirectionalLight, SgLight>()
            .registerClass<SgPointLight, SgLight>()
            .registerClass<SgSpotLight, SgPointLight>();
    }
} registration;

}
