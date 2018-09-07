/**
   @file
   @author Shin'ichiro Nakaoka
*/

#include "FireParticleSystem.h"
#include <cnoid/YAMLSceneReader>

using namespace std;
using namespace cnoid;


FireParticleSystem::FireParticleSystem()
{
    setLifeTime(2.5f);
    setParticleSize(0.15f);
    setNumParticles(500);
    setAcceleration(Vector3f(0.0f, 0.0f, 0.1f));
    setEmissionRange(radian(120.0f));

    initialSpeedAverage_ = 0.15f;
    initialSpeedVariation_ = 0.1f;
}


FireParticleSystem::FireParticleSystem(const FireParticleSystem& org)
    : ParticleSystem(org)
{
    initialSpeedAverage_ = org.initialSpeedAverage_;
    initialSpeedVariation_ = org.initialSpeedVariation_;
}


void FireParticleSystem::readParameters(YAMLSceneReader& reader, const Mapping& node)
{
    ParticleSystem::readParameters(reader, node);
    node.read("initialSpeedAverage", initialSpeedAverage_);
    node.read("initialSpeedVariation", initialSpeedVariation_);
}
