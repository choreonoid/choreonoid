/**
   @file
   @author Shin'ichiro Nakaoka
*/

#include "ParticleSystem.h"
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


void ParticleSystem::readParameters(const Mapping* info)
{
    info->read({ "offset_time", "offsetTime" }, offsetTime_);
    info->read({ "life_time", "lifeTime" }, lifeTime_);
    info->read({ "particle_size", "particleSize" }, particleSize_);
    info->read({ "num_particles", "numParticles" }, numParticles_);
    info->read({ "initial_speed_average", "initialSpeedAverage" }, initialSpeedAverage_);
    info->read({ "initial_speed_variation", "initialSpeedVariation" }, initialSpeedVariation_);
    info->readAngle({ "emission_range", "emissionRange" }, emissionRange_);
    read(info, "acceleration", acceleration_);
}


void ParticleSystem::writeParameters(Mapping* info) const
{
    info->write("offset_time", offsetTime_);
    info->write("life_time", lifeTime_);
    info->write("particle_size", particleSize_);
    info->write("num_particles", numParticles_);
    info->write("initial_speed_average", initialSpeedAverage_);
    info->write("initial_speed_variation", initialSpeedVariation_);
    info->write("emission_range", degree(emissionRange_));
    write(info, "acceleration", acceleration_);
}
   
