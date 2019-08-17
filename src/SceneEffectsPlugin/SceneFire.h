/**
   @author Shin'ichiro Nakaoka
*/

#ifndef CNOID_SCENE_EFFECTS_PLUGIN_SCENE_FIRE_H
#define CNOID_SCENE_EFFECTS_PLUGIN_SCENE_FIRE_H

#include "SceneParticles.h"
#include "ParticleSystem.h"

namespace cnoid {

class SceneFire : public SceneParticles
{
public:
    SceneFire();
    SceneFire(const SceneFire& org);
    virtual SgObject* doClone(SgCloneMap* cloneMap) const override;
    virtual ParticleSystem* getParticleSystem() override;
    ParticleSystem& particleSystem() { return particleSystem_; }

private:
    ParticleSystem particleSystem_;
};

}

#endif
