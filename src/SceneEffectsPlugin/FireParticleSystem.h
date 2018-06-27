/**
   @author Shin'ichiro Nakaoka
*/

#ifndef CNOID_SCENE_EFFECTS_PLUGIN_FIRE_PARTICLE_SYSTEMS_H
#define CNOID_SCENE_EFFECTS_PLUGIN_FIRE_PARTICLE_SYSTEMS_H

#include "ParticleSystem.h"

namespace cnoid {

class FireParticleSystem : public ParticleSystem
{
public:
    FireParticleSystem();
    FireParticleSystem(const FireParticleSystem& org);

    void readParameters(YAMLSceneReader& reader, const Mapping& node);

    float initialSpeedAverage() const { return initialSpeedAverage_; }
    void setInitialSpeedAverage(float v){ initialSpeedAverage_ = v; }

    float initialSpeedVariation() const { return initialSpeedVariation_; }
    void setInitialSpeedVariation(float v){ initialSpeedVariation_ = v; }

private:
    float initialSpeedAverage_;
    float initialSpeedVariation_;
};

}

#endif
