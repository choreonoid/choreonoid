#include "BodyState.h"
#include "Body.h"
#include "Link.h"

using namespace std;
using namespace cnoid;


void BodyStateBlock::storeStateOfBody(const Body* body, bool storeDeviceState)
{
    int numLinks = std::min(body->numLinks(), numLinkPositions());
    if(numLinks > 0){
        for(int i=0; i < numLinks; ++i){
            linkPosition(i).set(body->link(i)->T());
        }
    }
    
    int numJoints = std::min(body->numAllJoints(), numJointDisplacements());
    if(numJoints > 0){
        auto displacements = jointDisplacements();
        for(int i=0; i < numJoints; ++i){
            displacements[i] = body->joint(i)->q();
        }
    }

    if(storeDeviceState){
        int numDevices = std::min(body->numDevices(), numDeviceStates());
        for(int i=0; i < numDevices; ++i){
            setDeviceState(i, body->device(i)->cloneState());
        }
    }
}


bool BodyStateBlock::restoreStateToBody(Body* body, bool restoreDeviceState) const
{
    int numJoints = std::min(body->numAllJoints(), numJointDisplacements());
    auto displacements = jointDisplacements();
    for(int i=0; i < numJoints; ++i){
        body->joint(i)->q() = displacements[i];
    }

    int numLinks = std::min(body->numLinks(), numLinkPositions());
    if(numLinks > 0){
        body->rootLink()->setPosition(linkPosition(0).T());
        body->calcForwardKinematics();
        for(int i=1; i < numLinks; ++i){
            body->link(i)->setPosition(linkPosition(i).T());
        }
    }

    if(restoreDeviceState){
        int numDevices = std::min(body->numDevices(), numDeviceStates());
        for(int i=0; i < numDevices; ++i){
            if(auto state = deviceState(i)){
                auto device = body->device(i);
                device->copyStateFrom(*state);
                device->notifyStateChange();
            }
        }
    }

    return (numLinks > 0) || (numJoints > 0);
}


BodyState::BodyState(const Body* body)
{
    storeStateOfBody(body, true);
}


BodyState::BodyState(const Body& body)
{
    storeStateOfBody(&body, true);
}


void BodyState::storeStateOfBody(const Body* body, bool storeDeviceState)
{
    allocate(body->numLinks(), body->numAllJoints(), body->numDevices());
    firstBlock().storeStateOfBody(body, storeDeviceState);
}


bool BodyState::restoreStateToBody(Body* body, bool restoreDeviceState) const
{
    return firstBlock().restoreStateToBody(body, restoreDeviceState);
}


void BodyState::storeMultiplexStateOfBody(const Body* body)
{
    const int numLinks = body->numLinks();
    const int numJoints = body->numAllJoints();
    const int numDevices = body->numDevices();
    allocate(numLinks, numJoints, numDevices);
    firstBlock().storeStateOfBody(body, true);

    auto multiplexBody = body->nextMultiplexBody();
    while(multiplexBody){
        auto block = extend(numLinks, numJoints, numDevices);
        block.storeStateOfBody(multiplexBody, true);
        multiplexBody = multiplexBody->nextMultiplexBody();
    }
}


bool BodyState::restoreMultiplexStateToBody(Body* body) const
{
    bool restored = false;
    
    auto block = firstBlock();
    restored = block.restoreStateToBody(body, true);
    
    block = nextBlockOf(block);
    while(block){
        body = body->getOrCreateNextMultiplexBody();
        if(block.restoreStateToBody(body, true)){
            restored = true;
        }
        block = nextBlockOf(block);
    }
    body->clearMultiplexBodies();

    return restored;
}


BodyState& cnoid::operator<<(BodyState& state, const Body& body)
{
    state.storeStateOfBody(&body);
    return state;
}


BodyState& cnoid::operator>>(BodyState& state, Body& body)
{
    state.restoreStateToBody(&body);
    return state;
}


const BodyState& cnoid::operator>>(const BodyState& state, Body& body)
{
    state.restoreStateToBody(&body);
    return state;
}


Body& cnoid::operator<<(Body& body, const BodyState& state)
{
    state.restoreStateToBody(&body);
    return body;
}


Body& cnoid::operator>>(Body& body, BodyState& state)
{
    state.storeStateOfBody(&body);
    return body;
}


const Body& cnoid::operator>>(const Body& body, BodyState& state)
{
    state.storeStateOfBody(&body);
    return body;
}
