/**
   @author Shin'ichiro Nakaoka
*/

#ifndef CNOID_PHENOMENON_PLUGIN_SCENE_PARTICLES_H
#define CNOID_PHENOMENON_PLUGIN_SCENE_PARTICLES_H

#include <cnoid/SceneGraph>

namespace cnoid {

class SceneParticles : public SgNode
{
protected:
    SceneParticles(int polymorphicId);
    SceneParticles(const SceneParticles& org);

public:
    float time() const { return time_; }
    float particleSize() const { return particleSize_; }
    const std::string& texture() const { return texture_; }

    void setTime(float t) { time_ = t; }
    void setParticleSize(float s) { particleSize_ = s; }
    void setTexture(const std::string& file) { texture_ = file; }

private:
    float time_;
    float particleSize_;
    std::string texture_;
};

}

#endif
