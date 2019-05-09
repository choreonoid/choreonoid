/**
   \author Shin'ichiro Nakaoka
*/

#include "DyWorld.h"
#include "DyBody.h"
#include "ForwardDynamicsABM.h"
#include "ForwardDynamicsCBM.h"
#include <cnoid/EigenUtil>
#include <string>
#include <iostream>

using namespace std;
using namespace cnoid;

static const double DEFAULT_GRAVITY_ACCELERATION = 9.80665;

static const bool debugMode = false;


WorldBase::WorldBase()
{
    currentTime_ = 0.0;
    timeStep_ = 0.005;

    g << 0.0, 0.0, -DEFAULT_GRAVITY_ACCELERATION;

    isEulerMethod =false;
    sensorsAreEnabled = false;
    isOldAccelSensorCalcMode = false;
    numRegisteredLinkPairs = 0;
}


WorldBase::~WorldBase()
{

}


int WorldBase::bodyIndex(const std::string& name) const
{
    NameToIndexMap::const_iterator p = nameToBodyIndexMap.find(name);
    return (p != nameToBodyIndexMap.end()) ? p->second : -1;
}


DyBody* WorldBase::body(int index) const
{
    if(index < 0 || (int)bodyInfoArray.size() <= index){
        return 0;
    }
    return bodyInfoArray[index].body; 
}


DyBody* WorldBase::body(const std::string& name) const
{
    int idx = bodyIndex(name);
    if(idx < 0 || (int)bodyInfoArray.size() <= idx){
        return 0;
    }
    return bodyInfoArray[idx].body;
}


void WorldBase::setTimeStep(double ts)
{
    timeStep_ = ts;
}


void WorldBase::setCurrentTime(double time)
{
    currentTime_ = time;
}


void WorldBase::setGravityAcceleration(const Vector3& g)
{
    this->g = g;
}


void WorldBase::enableSensors(bool on)
{
    sensorsAreEnabled = on;
}


void WorldBase::setOldAccelSensorCalcMode(bool on)
{
    isOldAccelSensorCalcMode = on;
}


void WorldBase::initialize()
{
    const int n = bodyInfoArray.size();

    for(int i=0; i < n; ++i){

        BodyInfo& info = bodyInfoArray[i];

        if(!info.forwardDynamics){
            info.forwardDynamics = make_shared_aligned<ForwardDynamicsABM>(info.body);
        }
        
        if(isEulerMethod){
            info.forwardDynamics->setEulerMethod();
        } else {
            info.forwardDynamics->setRungeKuttaMethod();
        }
        info.forwardDynamics->setGravityAcceleration(g);
        info.forwardDynamics->setTimeStep(timeStep_);
        info.forwardDynamics->enableSensors(sensorsAreEnabled);
        info.forwardDynamics->setOldAccelSensorCalcMode(isOldAccelSensorCalcMode);
        info.forwardDynamics->initialize();
    }
}


void WorldBase::setVirtualJointForces()
{
    for(size_t i=0; i < bodyInfoArray.size(); ++i){
        BodyInfo& info = bodyInfoArray[i];
        if(info.hasVirtualJointForces){
            info.body->setVirtualJointForces(timeStep_);
        }
    }
}


void WorldBase::calcNextState()
{
    if(debugMode){
        cout << "World current time = " << currentTime_ << endl;
    }
    const int n = bodyInfoArray.size();

    for(int i=0; i < n; ++i){
        BodyInfo& info = bodyInfoArray[i];
        info.forwardDynamics->calcNextState();
    }
    currentTime_ += timeStep_;
}


int WorldBase::addBody(DyBody* body)
{
    if(!body->name().empty()){
        nameToBodyIndexMap[body->name()] = bodyInfoArray.size();
    }
    BodyInfo info;
    info.body = body;
    info.hasVirtualJointForces = body->hasVirtualJointForces();
    bodyInfoArray.push_back(info);

    return bodyInfoArray.size() - 1;
}


int WorldBase::addBody(DyBody* body, std::shared_ptr<ForwardDynamics> forwardDynamics)
{
    int index = addBody(body);
    bodyInfoArray[index].forwardDynamics = forwardDynamics;
    return index;
}


void WorldBase::clearBodies()
{
    nameToBodyIndexMap.clear();
    bodyInfoArray.clear();
}


void WorldBase::clearCollisionPairs()
{
    linkPairKeyToIndexMap.clear();
    numRegisteredLinkPairs = 0;
}


void WorldBase::setEulerMethod()
{
    isEulerMethod = true;
}


void WorldBase::setRungeKuttaMethod()
{
    isEulerMethod = false;
}


std::pair<int,bool> WorldBase::getIndexOfLinkPairs(DyLink* link1, DyLink* link2)
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

        LinkPairKeyToIndexMap::iterator p = linkPairKeyToIndexMap.find(linkPair);

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


bool WorldBase::LinkPairKey::operator<(const LinkPairKey& pair2) const
{
    if(link1 < pair2.link1){
        return true;
    } else if(link1 == pair2.link1){
        return (link2 < pair2.link2);
    } else {
        return false;
    }
}
