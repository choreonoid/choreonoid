/**
   @author Shin'ichiro Nakaoka
*/

#ifndef CNOID_SCENE_EFFECTS_PLUGIN_SCENE_FOUNTAIN_H
#define CNOID_SCENE_EFFECTS_PLUGIN_SCENE_FOUNTAIN_H

#include "SceneParticles.h"
#include "ParticleSystem.h"

namespace cnoid {

class SceneFountain : public SceneParticles
{
public:
    SceneFountain();
    SceneFountain(const SceneFountain& org);

    virtual ParticleSystem* getParticleSystem();

    ParticleSystem& particleSystem() { return particleSystem_; }

protected:
    virtual Referenced* doClone(CloneMap* cloneMap) const override;
    
private:
    ParticleSystem particleSystem_;
};

}

#endif
