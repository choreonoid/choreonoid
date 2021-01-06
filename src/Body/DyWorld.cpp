/**
   \author Shin'ichiro Nakaoka
*/

#include "DyWorld.h"

using namespace std;
using namespace cnoid;

static const double DEFAULT_GRAVITY_ACCELERATION = 9.80665;


DyWorldBase::DyWorldBase()
{
    currentTime_ = 0.0;
    timeStep_ = 0.005;

    g << 0.0, 0.0, -DEFAULT_GRAVITY_ACCELERATION;

    isEulerMethod =false;
    hasHighGainDynamics_ = false;
    sensorsAreEnabled = false;
    isOldAccelSensorCalcMode = false;
    numRegisteredLinkPairs = 0;
}


DyWorldBase::~DyWorldBase()
{

}


DyBody* DyWorldBase::body(const std::string& name) const
{
    auto p = nameToBodyMap.find(name);
    if(p != nameToBodyMap.end()){
        return p->second;
    }
    return nullptr;
}


void DyWorldBase::setTimeStep(double ts)
{
    timeStep_ = ts;
}


void DyWorldBase::setCurrentTime(double time)
{
    currentTime_ = time;
}


void DyWorldBase::setGravityAcceleration(const Vector3& g)
{
    this->g = g;
}


void DyWorldBase::enableSensors(bool on)
{
    sensorsAreEnabled = on;
}


void DyWorldBase::setOldAccelSensorCalcMode(bool on)
{
    isOldAccelSensorCalcMode = on;
}


void DyWorldBase::initialize()
{
    for(auto& subBody : subBodies_){
        auto forwardDynamics = subBody->forwardDynamics();
        if(isEulerMethod){
            forwardDynamics->setEulerMethod();
        } else {
            forwardDynamics->setRungeKuttaMethod();
        }
        forwardDynamics->setGravityAcceleration(g);
        forwardDynamics->setTimeStep(timeStep_);
        forwardDynamics->enableSensors(sensorsAreEnabled);
        forwardDynamics->setOldAccelSensorCalcMode(isOldAccelSensorCalcMode);
        forwardDynamics->initialize();
    }
}


void DyWorldBase::setVirtualJointForces()
{
    for(auto& body : bodiesWithVirtualJointForces_){
        body->setVirtualJointForces(timeStep_);
    }
}


void DyWorldBase::calcNextState()
{
    for(auto& subBody : subBodies_){
        subBody->forwardDynamics()->calcNextState();
    }
    currentTime_ += timeStep_;
}


void DyWorldBase::refreshState()
{
    for(auto& subBody : subBodies_){
        subBody->forwardDynamics()->refreshState();
    }
}


int DyWorldBase::addBody(DyBody* body)
{
    int index = bodies_.size();
    
    bodies_.push_back(body);
    if(body->hasVirtualJointForces()){
        bodiesWithVirtualJointForces_.push_back(body);
    }

    body->initializeSubBodies();

    for(auto& subBody : body->subBodies()){
        subBodies_.push_back(subBody);
        if(subBody->forwardDynamicsCBM()){
            hasHighGainDynamics_ = true;
        }
    }

    if(!body->name().empty()){
        nameToBodyMap[body->name()] = body;
    }

    return index;
}


void DyWorldBase::clearBodies()
{
    bodies_.clear();
    bodiesWithVirtualJointForces_.clear();
    subBodies_.clear();
    nameToBodyMap.clear();
    hasHighGainDynamics_ = false;
}


void DyWorldBase::clearCollisionPairs()
{
    linkPairKeyToIndexMap.clear();
    numRegisteredLinkPairs = 0;
}


void DyWorldBase::setEulerMethod()
{
    isEulerMethod = true;
}


void DyWorldBase::setRungeKuttaMethod()
{
    isEulerMethod = false;
}


std::pair<int,bool> DyWorldBase::getIndexOfLinkPairs(DyLink* link1, DyLink* link2)
{
    int index = -1;
    bool isRegistered = false;

    if(link1 != link2){

        LinkPairKey linkPair;
        if(link1 < link2){
            linkPair.link1 = link1;
            linkPair.link2 = link2;
        } else {
            linkPair.link1 = link2;
            linkPair.link2 = link1;
        }

        auto p = linkPairKeyToIndexMap.find(linkPair);
        if(p != linkPairKeyToIndexMap.end()){
            index = p->second;
            isRegistered = true;
        } else {
            index = numRegisteredLinkPairs++;
            linkPairKeyToIndexMap[linkPair] = index;
        }
    }

    return std::make_pair(index, isRegistered);
}


bool DyWorldBase::LinkPairKey::operator<(const LinkPairKey& pair2) const
{
    if(link1 < pair2.link1){
        return true;
    } else if(link1 == pair2.link1){
        return (link2 < pair2.link2);
    } else {
        return false;
    }
}
