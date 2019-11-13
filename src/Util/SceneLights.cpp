/*!
  @file
  @author Shin'ichiro Nakaoka
*/

#include "SceneLights.h"

using namespace std;
using namespace cnoid;


SgLight::SgLight(int polymorhicId)
    : SgPreprocessed(polymorhicId)
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


SgDirectionalLight::SgDirectionalLight(int polymorhicId)
    : SgLight(polymorhicId)
{
    direction_ << 0.0, 0.0, -1.0;
}


SgDirectionalLight::SgDirectionalLight()
    : SgDirectionalLight(findPolymorphicId<SgDirectionalLight>())
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


SgPointLight::SgPointLight(int polymorhicId)
    : SgLight(polymorhicId)
{
    constantAttenuation_ = 1.0f;
    linearAttenuation_ = 0.0f;
    quadraticAttenuation_ = 0.0f;
}


SgPointLight::SgPointLight()
    : SgPointLight(findPolymorphicId<SgPointLight>())
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


SgSpotLight::SgSpotLight(int polymorhicId)
    : SgPointLight(polymorhicId)
{
    direction_ << 0.0, 0.0, -1.0;
    beamWidth_ = 1.570796f;
    cutOffAngle_ = 0.785398f;
    cutOffExponent_ = 1.0f;
}


SgSpotLight::SgSpotLight()
    : SgSpotLight(findPolymorphicId<SgSpotLight>())
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
        SgNode::registerType<SgLight, SgPreprocessed>();
        SgNode::registerType<SgDirectionalLight, SgLight>();
        SgNode::registerType<SgPointLight, SgLight>();
        SgNode::registerType<SgSpotLight, SgPointLight>();
    }
} registration;

}
