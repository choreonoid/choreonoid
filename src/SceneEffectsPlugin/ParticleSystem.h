/**
   @author Shin'ichiro Nakaoka
*/

#ifndef CNOID_SCENE_EFFECTS_PLUGIN_PARTICLE_SYSTEMS_H
#define CNOID_SCENE_EFFECTS_PLUGIN_PARTICLE_SYSTEMS_H

#include <cnoid/EigenTypes>

namespace cnoid {

class Mapping;

class ParticleSystem
{
public:
    ParticleSystem();
    ParticleSystem(const ParticleSystem& org);
    virtual ~ParticleSystem();

    bool on() const { return on_; }
    void on(bool on) { on_ = on; }

    float offsetTime() const { return offsetTime_; }
    void setOffsetTime(float t) { offsetTime_ = t; }

    float lifeTime() const { return lifeTime_; }
    void setLifeTime(float t) { lifeTime_ = t; }
    
    float particleSize() const { return particleSize_; }
    void setParticleSize(float s) { particleSize_ = s; }

    int numParticles() const { return numParticles_; }
    void setNumParticles(int n) { numParticles_ = n; }

    float initialSpeedAverage() const { return initialSpeedAverage_; }
    void setInitialSpeedAverage(float v){ initialSpeedAverage_ = v; }

    float initialSpeedVariation() const { return initialSpeedVariation_; }
    void setInitialSpeedVariation(float v){ initialSpeedVariation_ = v; }

    float emissionRange() const { return emissionRange_; }
    void setEmissionRange(float r) { emissionRange_ = r; }

    const Vector3f& acceleration() const { return acceleration_; }
    void setAcceleration(const Vector3f& a){ acceleration_ = a; }

    void readParameters(const Mapping* info);
    void writeParameters(Mapping* info) const;

private:
    bool on_;
    float offsetTime_;
    float lifeTime_;
    int numParticles_;
    float particleSize_;
    float initialSpeedAverage_;
    float initialSpeedVariation_;
    float emissionRange_;
    Vector3f acceleration_;
};

}

#endif
