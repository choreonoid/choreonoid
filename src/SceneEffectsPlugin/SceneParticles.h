/**
   @author Shin'ichiro Nakaoka
*/

#ifndef CNOID_SCENE_EFFECTS_PLUGIN_SCENE_PARTICLES_H
#define CNOID_SCENE_EFFECTS_PLUGIN_SCENE_PARTICLES_H

#include "ParticleSystem.h"
#include <cnoid/SceneGraph>

namespace cnoid {

class SceneParticles : public SgNode
{
protected:
    SceneParticles(int classId);
    SceneParticles(const SceneParticles& org);

public:
    float time() const { return time_; }
    void setTime(float t) { time_ = t; }

    virtual ParticleSystem* getParticleSystem() = 0;
    
    const std::string& texture() const { return texture_; }
    void setTexture(const std::string& file) { texture_ = file; }

private:
    float time_;
    std::string texture_;
};

}

#endif
