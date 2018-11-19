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
    initialSpeedAverage_ = 0.1f;
    initialSpeedVariation_ = 0.1f;
    emissionRange_ = PI / 3.0f;
    acceleration_.setZero();
}


ParticleSystem::ParticleSystem(const ParticleSystem& org)
{
    on_ = org.on_;
    offsetTime_ = org.offsetTime_;
    lifeTime_ = org.lifeTime_;
    particleSize_ = org.particleSize_;
    numParticles_ = org.numParticles_;
    initialSpeedAverage_ = org.initialSpeedAverage_;
    initialSpeedVariation_ = org.initialSpeedVariation_;
    emissionRange_ = org.emissionRange_;
    acceleration_ = org.acceleration_;
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
    node.read("initialSpeedAverage", initialSpeedAverage_);
    node.read("initialSpeedVariation", initialSpeedVariation_);
    reader.readAngle(node, "emissionRange", emissionRange_);
    read(node, "acceleration", acceleration_);
}
