/**
   @author Shin'ichiro Nakaoka
*/

#ifndef CNOID_PHENOMENON_PLUGIN_SCENE_FOUNTAIN_H
#define CNOID_PHENOMENON_PLUGIN_SCENE_FOUNTAIN_H

#include "SceneParticles.h"

namespace cnoid {

class SceneFountain : public SceneParticles
{
public:
    SceneFountain();
    SceneFountain(const SceneFountain& org);
    virtual SgObject* clone(SgCloneMap& cloneMap) const override;

    float lifeTime() const { return lifeTime_; }
    void setLifeTime(float t) { lifeTime_ = t; }
    void setGravity(const Vector3f& g) { gravity_ = g; }
    
private:
    float angle_;
    float lifeTime_;
    Vector3f gravity_;
};

}

#endif
