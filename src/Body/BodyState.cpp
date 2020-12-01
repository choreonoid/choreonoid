/**
   \file
   \author Shin'ichiro Nakaoka
*/

#include "BodyState.h"
#include "Body.h"
#include "Link.h"

using namespace std;
using namespace cnoid;


BodyState::BodyState()
{

}


BodyState::BodyState(const Body& body)
{
    storePositions(body);
}


BodyState::BodyState(const BodyState& state)
    : DataMap<double>(state)
{

}
    

void BodyState::setRootLinkPosition(const Isometry3& T)
{
    Data& p = data(LINK_POSITIONS);
    p.resize(7);
    Eigen::Map<Vector3> pmap(&p[0]);
    pmap = T.translation();
    Eigen::Map<Quaternion> qmap(&p[3]);
    qmap = T.linear();
}


void BodyState::setRootLinkPosition(const SE3& position)
{
    Data& p = data(LINK_POSITIONS);
    p.resize(7);
    Eigen::Map<Vector3> pmap(&p[0]);
    pmap = position.translation();
    Eigen::Map<Quaternion> qmap(&p[3]);
    qmap = position.rotation();
}


void BodyState::storePositions(const Body& body)
{
    Data& q = data(JOINT_POSITIONS);
    const int n = body.numAllJoints();
    q.resize(n);
    for(int i=0; i < n; ++i){
        q[i] = body.joint(i)->q();
    }

    auto rootLink = body.rootLink();
    if(auto parentLink = body.parentBodyLink()){
        setRootLinkPosition(parentLink->T().inverse(Eigen::Isometry) * rootLink->T());
    } else {
        setRootLinkPosition(rootLink->T());
    }
}


void BodyState::setZMP(const Vector3& zmp)
{
    Data& zmpData = data(ZMP);
    zmpData.resize(3);
    Eigen::Map<Vector3> zmpMap(&zmpData[0]);
    zmpMap = zmp;
}


bool BodyState::getRootLinkPosition(SE3& out_position) const
{
    const Data& p = data(LINK_POSITIONS);
    if(p.size() >= 7){
        out_position.translation() = Eigen::Map<const Vector3>(&p[0]);
        out_position.rotation() = Eigen::Map<const Quaternion>(&p[3]);
        return true;
    }
    return false;
}


bool BodyState::getRootLinkPosition(Isometry3& T) const
{
    const Data& p = data(LINK_POSITIONS);
    if(p.size() >= 7){
        T.translation() = Eigen::Map<const Vector3>(&p[0]);
        T.linear() = Eigen::Map<const Quaternion>(&p[3]).toRotationMatrix();
        return true;
    }
    return false;
}


bool BodyState::restorePositions(Body& io_body) const
{
    bool isComplete = true;
    
    const Data& q = data(JOINT_POSITIONS);

    size_t n = io_body.numAllJoints();
    if(q.size() < n){
        n = q.size();
        isComplete = false;
    } 
    for(size_t i=0; i < n; ++i){
        io_body.joint(i)->q() = q[i];
    }

    const Data& p = data(LINK_POSITIONS);
    if(p.size() < 7){
        isComplete = false;
    } else {
        Isometry3 T0;
        if(auto parentLink = io_body.parentBodyLink()){
            T0 = parentLink->T();
        } else {
            T0.setIdentity();
        }
        Isometry3 T;
        T.translation() = Eigen::Map<const Vector3>(&p[0]);
        T.linear() = Eigen::Map<const Quaternion>(&p[3]).toRotationMatrix();
        io_body.rootLink()->setPosition(T0 * T);

        io_body.calcForwardKinematics();

        if(p.size() > 7){
            const int numRemainingLinks = (p.size() - 7) / 7;
            for(int i = 1; i < numRemainingLinks + 1; ++i){
                Link* link = io_body.link(i);
                T.translation() = Eigen::Map<const Vector3>(&p[i*7]);
                T.linear() = Eigen::Map<const Quaternion>(&p[i*7 + 3]).toRotationMatrix();
                link->setPosition(T0 * T);
            }
        }
    }

    return isComplete;
}


bool BodyState::getZMP(Vector3& out_zmp) const
{
    const Data& zmp = data(ZMP);
    if(zmp.size() == 3){
        out_zmp = Eigen::Map<const Vector3>(&zmp[0]);
        return true;
    }
    return false;
}
        

std::map<std::string, int>& BodyState::nameToIdMap()
{
    static std::map<std::string, int> nameToIdMap_;
    return nameToIdMap_;
}


std::map<int, std::string>& BodyState::idToNameMap()
{
    static std::map<int, std::string> idToNameMap_;
    return idToNameMap_;
}


int BodyState::nextDynamicId()
{
    static int dynamicIdCounter = MIN_DYNAMIC_ID;
    return dynamicIdCounter++;
}


namespace cnoid {

BodyState& operator<<(BodyState& state, const Body& body)
{
    state.storePositions(body);
    return state;
}

const BodyState& operator>>(const BodyState& state, Body& body)
{
    state.restorePositions(body);
    return state;
}

Body& operator<<(Body& body, const BodyState& state)
{
    state.restorePositions(body);
    return body;
}

const Body& operator>>(const Body& body, BodyState& state)
{
    state.storePositions(body);
    return body;
}

}
