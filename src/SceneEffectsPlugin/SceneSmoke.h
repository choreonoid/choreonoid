/**
   @author Shin'ichiro Nakaoka
*/

#ifndef CNOID_SCENE_EFFECTS_PLUGIN_SCENE_SMOKE_H
#define CNOID_SCENE_EFFECTS_PLUGIN_SCENE_SMOKE_H

#include "SceneParticles.h"
#include "ParticleSystem.h"

namespace cnoid {

class SceneSmoke : public SceneParticles
{
public:
    SceneSmoke();
    SceneSmoke(const SceneSmoke& org);

    virtual ParticleSystem* getParticleSystem();

    ParticleSystem& particleSystem() { return particleSystem_; }

protected:
    virtual Referenced* doClone(CloneMap* cloneMap) const override;
    
private:
    ParticleSystem particleSystem_;
};

}

#endif
