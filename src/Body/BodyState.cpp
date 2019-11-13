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
    

void BodyState::setRootLinkPosition(const Position& T)
{
    Data& p = data(LINK_POSITIONS);
    p.resize(7);
    Eigen::Map<Vector3> pmap(&p[0]);
    pmap = T.translation();
    Eigen::Map<Quat> qmap(&p[3]);
    qmap = T.linear();
}


void BodyState::setRootLinkPosition(const SE3& position)
{
    Data& p = data(LINK_POSITIONS);
    p.resize(7);
    Eigen::Map<Vector3> pmap(&p[0]);
    pmap = position.translation();
    Eigen::Map<Quat> qmap(&p[3]);
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

    const Link* rootLink = body.rootLink();
    setRootLinkPosition(rootLink->T());
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
        out_position.rotation() = Eigen::Map<const Quat>(&p[3]);
        return true;
    }
    return false;
}


bool BodyState::getRootLinkPosition(Position& T) const
{
    const Data& p = data(LINK_POSITIONS);
    if(p.size() >= 7){
        T.translation() = Eigen::Map<const Vector3>(&p[0]);
        T.linear() = Eigen::Map<const Quat>(&p[3]).toRotationMatrix();
        return true;
    }
    return false;
}


#ifdef CNOID_BACKWARD_COMPATIBILITY

void BodyState::setRootLinkPosition(const Vector3& translation, const Matrix3& rotation)
{
    Data& p = data(LINK_POSITIONS);
    p.resize(7);
    Eigen::Map<Vector3> pmap(&p[0]);
    pmap = translation;
    Eigen::Map<Quat> qmap(&p[3]);
    qmap = rotation;
}


bool BodyState::getRootLinkPosition(Vector3& translation, Matrix3& rotation) const
{
    const Data& p = data(LINK_POSITIONS);
    if(p.size() >= 7){
        translation = Eigen::Map<const Vector3>(&p[0]);
        rotation = Eigen::Map<const Quat>(&p[3]);
        return true;
    }
    return false;
}

#endif



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

    if(io_body.parentBody()){
        io_body.calcForwardKinematics();
        isComplete = true;

    } else {
        //! \todo Use relative coordinates for a child body
        const Data& p = data(LINK_POSITIONS);
        if(p.size() < 7){
            isComplete = false;
        } else {
            Link* rootLink = io_body.rootLink();
            rootLink->p() = Eigen::Map<const Vector3>(&p[0]);
            rootLink->R() = Eigen::Map<const Quat>(&p[3]).toRotationMatrix();
        }

        io_body.calcForwardKinematics();

        if(p.size() > 7){
            const int numNonRootLinks = (p.size() - 7) / 7;
            for(int i=1; i < numNonRootLinks + 1; ++i){
                Link* link = io_body.link(i);
                link->p() = Eigen::Map<const Vector3>(&p[i*7]);
                link->R() = Eigen::Map<const Quat>(&p[i*7 + 3]).toRotationMatrix();
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
