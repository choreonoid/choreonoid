/**
   @author Shin'ichiro Nakaoka
*/

#ifndef CNOID_PHENOMENON_PLUGIN_SCENE_FOUNTAIN_H
#define CNOID_PHENOMENON_PLUGIN_SCENE_FOUNTAIN_H

#include <cnoid/SceneGraph>

namespace cnoid {

class SceneFountain : public SgNode
{
public:
    static void initializeClass();
    
    SceneFountain();

    float time() const { return time_; }
    float lifeTime() const { return lifeTime_; }

    void setTime(double t) { time_ = t; }
    void setLifeTime(double t) { lifeTime_ = t; }
    void setGravity(const Vector3f& g) { gravity_ = g; }
    void setTexture(const std::string& file) { texture_ = file; }
private:
    float angle_;
    float time_;
    float lifeTime_;
    Vector3f gravity_;
    std::string texture_;
};

}

#endif
