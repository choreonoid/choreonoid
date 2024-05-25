#include "BodyState.h"
#include "Body.h"
#include "Link.h"

using namespace std;
using namespace cnoid;


void BodyStateBlock::storeStateOfBody(const Body* body)
{
    int numLinks = std::min(body->numLinks(), numLinkPositions());
    if(numLinks > 0){
        auto rootLink = body->rootLink();
        if(auto parentLink = body->parentBodyLink()){
            Isometry3 T_parent_inv = parentLink->T().inverse(Eigen::Isometry);
            for(int i=0; i < numLinks; ++i){
                linkPosition(i).set(T_parent_inv * body->link(i)->T());
            }
        } else {
            for(int i=0; i < numLinks; ++i){
                linkPosition(i).set(body->link(i)->T());
            }
        }
    }
    
    int numJoints = std::min(body->numJoints(), numJointDisplacements());
    if(numJoints > 0){
        auto displacements = jointDisplacements();
        for(int i=0; i < numJoints; ++i){
            displacements[i] = body->joint(i)->q();
        }
    }

    int numDevices = std::min(body->numDevices(), numDeviceStates());
    for(int i=0; i < numDevices; ++i){
        setDeviceState(i, body->device(i)->cloneState());
    }
}


bool BodyStateBlock::restoreStateToBody(Body* body) const
{
    int numJoints = std::min(body->numJoints(), numJointDisplacements());
    auto displacements = jointDisplacements();
    for(int i=0; i < numJoints; ++i){
        body->joint(i)->q() = displacements[i];
    }

    int numLinks = std::min(body->numLinks(), numLinkPositions());
    if(numLinks > 0){
        if(auto parentLink = body->parentBodyLink()){
            const Isometry3& T_parent = parentLink->T();
            body->rootLink()->setPosition(T_parent * linkPosition(0).T());
            body->calcForwardKinematics();
            for(int i=1; i < numLinks; ++i){
                body->link(i)->setPosition(T_parent * linkPosition(i).T());
            }
        } else {
            body->rootLink()->setPosition(linkPosition(0).T());
            body->calcForwardKinematics();
            for(int i=1; i < numLinks; ++i){
                body->link(i)->setPosition(linkPosition(i).T());
            }
        }
    }

    int numDevices = std::min(body->numDevices(), numDeviceStates());
    for(int i=0; i < numDevices; ++i){
        if(auto state = deviceState(i)){
            auto device = body->device(i);
            device->copyStateFrom(*state);
            device->notifyStateChange();
        }
    }

    return (numLinks > 0) || (numJoints > 0);
}


BodyState::BodyState(const Body* body)
{
    storeStateOfBody(body);
}


BodyState::BodyState(const Body& body)
{
    storeStateOfBody(&body);
}


void BodyState::storeStateOfBody(const Body* body)
{
    allocate(body->numLinks(), body->numJoints(), body->numDevices());
    firstBlock().storeStateOfBody(body);
}


bool BodyState::restoreStateToBody(Body* body) const
{
    return firstBlock().restoreStateToBody(body);
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
