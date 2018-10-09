/**
   @file
   @author Shin'ichiro Nakaoka
*/

#include "ParticleSystem.h"
#include <cnoid/YAMLSceneReader>
#include <cnoid/EigenArchive>

using namespace std;
using namespace cnoid;


ParticleSystem::ParticleSystem()
{
    on_ = true;
    offsetTime_ = 0.0f;
    lifeTime_ = 3.0f;
    particleSize_ = 0.1f;
    numParticles_ = 500;
    acceleration_.setZero();
    emissionRange_ = PI / 3.0f;
}


ParticleSystem::ParticleSystem(const ParticleSystem& org)
{
    on_ = org.on_;
    offsetTime_ = org.offsetTime_;
    lifeTime_ = org.lifeTime_;
    particleSize_ = org.particleSize_;
    numParticles_ = org.numParticles_;
    acceleration_ = org.acceleration_;
    emissionRange_ = org.emissionRange_;
}


ParticleSystem::~ParticleSystem()
{

}


void ParticleSystem::readParameters(const YAMLSceneReader& reader, const Mapping& node)
{
    node.read("offsetTime", offsetTime_);
    node.read("lifeTime", lifeTime_);
    node.read("particleSize", particleSize_);
    node.read("numParticles", numParticles_);
    read(node, "acceleration", acceleration_);
    reader.readAngle(node, "emissionRange", emissionRange_);
}
