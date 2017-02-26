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
    
private:
    float angle_;
    float lifeTime_;
    Vector3f acceleration_;
};

}

#endif
