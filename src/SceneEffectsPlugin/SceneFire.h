/**
   @author Shin'ichiro Nakaoka
*/

#ifndef CNOID_SCENE_EFFECTS_PLUGIN_SCENE_FIRE_H
#define CNOID_SCENE_EFFECTS_PLUGIN_SCENE_FIRE_H

#include "SceneParticles.h"
#include "FireParticleSystem.h"

namespace cnoid {

class SceneFire : public SceneParticles
{
public:
    SceneFire();
    SceneFire(const SceneFire& org);
    virtual SgObject* clone(SgCloneMap& cloneMap) const override;

    virtual ParticleSystem* getParticleSystem() override;

    FireParticleSystem& particleSystem() { return particleSystem_; }

private:
    FireParticleSystem particleSystem_;
};

}

#endif
