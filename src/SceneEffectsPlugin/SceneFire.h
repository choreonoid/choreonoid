/**
   @author Shin'ichiro Nakaoka
*/

#ifndef CNOID_PHENOMENON_PLUGIN_SCENE_FIRE_H
#define CNOID_PHENOMENON_PLUGIN_SCENE_FIRE_H

#include "SceneParticles.h"

namespace cnoid {

class SceneFire : public SceneParticles
{
public:
    SceneFire();
    SceneFire(const SceneFire& org);
    virtual SgObject* clone(SgCloneMap& cloneMap) const override;

    float lifeTime() const { return lifeTime_; }
    void setLifeTime(float t) { lifeTime_ = t; }
    void setAcceleration(const Vector3f& a) { acceleration_ = a; }
    const Vector3f& acceleration() const { return acceleration_; }
    void setInitialSpeedAverage(float average){
        initialSpeedAverage_ = average;
    }
    void setInitialSpeedVariation(float variation) {
        initialSpeedVariation_ = variation;
    }
    void setInitialVelocityAngleRange(float angle){
        initialVelocityAngleRange_ = angle;
    }
    float initialSpeedAverage() const { return initialSpeedAverage_; }
    float initialSpeedVariation() const { return initialSpeedVariation_; }
    float initialVelocityAngleRange() const { return initialVelocityAngleRange_; }

private:
    float lifeTime_;
    Vector3f acceleration_;
    float initialSpeedAverage_;
    float initialSpeedVariation_;
    float initialVelocityAngleRange_;
};

}

#endif
