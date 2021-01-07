/**
   \file
   \author Shin'ichiro Nakaoka
*/

#include "DyBody.h"
#include "ForwardDynamicsABM.h"
#include "ForwardDynamicsCBM.h"
#include <cnoid/Exception>

using namespace std;
using namespace cnoid;


DySubBody::DySubBody(DyLink* rootLink)
{
    std::multimap<Link*, ForceSensor*> emptyMap;
    initialize(rootLink, emptyMap);
}
    

DySubBody::DySubBody(DyLink* rootLink, std::multimap<Link*, ForceSensor*>& forceSensorMap)
{
    initialize(rootLink, forceSensorMap);
}


void DySubBody::initialize(DyLink* rootLink, std::multimap<Link*, ForceSensor*>& forceSensorMap)
{
    rootLink_ = rootLink;
    body_ = rootLink->body();
    forwardDynamicsCBM_ = nullptr;
    isStatic_ = true;
    
    bool hasHighGainJoints = false;

    extractLinksInSubBody(rootLink, forceSensorMap, hasHighGainJoints);

    if(hasHighGainJoints){
        forwardDynamicsCBM_ = new ForwardDynamicsCBM(this);
        forwardDynamics_.reset(forwardDynamicsCBM_);
    } else {
        forwardDynamics_.reset(new ForwardDynamicsABM(this));
    }
}


void DySubBody::extractLinksInSubBody
(DyLink* link, std::multimap<Link*, ForceSensor*>& forceSensorMap, bool& hasHighGainJoints)
{
    link->subBody_ = this;
    links_.push_back(link);

    if(!link->isFixedJoint()){
        isStatic_ = false;
    }

    while(true){
        auto p = forceSensorMap.find(link);
        if(p == forceSensorMap.end()){
            break;
        }
        forceSensors_.push_back(p->second);
        forceSensorMap.erase(p);
    }

    int mode = link->actuationMode();
    if(mode == Link::JointDisplacement ||
       (mode == Link::JointVelocity && link->jointType() != Link::PseudoContinuousTrackJoint) ||
       mode == Link::LinkPosition){
        hasHighGainJoints = true;
    }
    
    for(auto child = link->child(); child; child = child->sibling()){
        if(!child->isFreeJoint()){
            extractLinksInSubBody(child, forceSensorMap, hasHighGainJoints);
        }
    }
}


void DySubBody::clearExternalForces()
{
    for(auto& link : links_){
        link->F_ext().setZero();
    }
}


void DySubBody::calcSpatialForwardKinematics()
{
    const int n = links_.size();
    for(int i=0; i < n; ++i){
        DyLink* link = static_cast<DyLink*>(links_[i]);
        const DyLink* parent = link->parent();
        if(parent){
            switch(link->jointType()){
            case Link::ROTATIONAL_JOINT:
                link->R().noalias() = parent->R() * AngleAxisd(link->q(), link->a());
                link->p().noalias() = parent->R() * link->b() + parent->p();
                link->sw().noalias() = parent->R() * link->a();
                link->sv().noalias() = link->p().cross(link->sw());
                link->w().noalias() = link->dq() * link->sw() + parent->w();
                break;
            case Link::SLIDE_JOINT:
                link->p().noalias() = parent->R() * (link->b() + link->q() * link->d()) + parent->p();
                link->R() = parent->R();
                link->sw().setZero();
                link->sv().noalias() = parent->R() * link->d();
                link->w() = parent->w();
                break;
            case Link::FIXED_JOINT:
            default:
                link->p().noalias() = parent->R() * link->b() + parent->p();
                link->R() = parent->R();
                link->w() = parent->w();
                link->vo() = parent->vo();
                link->sw().setZero();
                link->sv().setZero();
                link->cv().setZero();
                link->cw().setZero();
                goto COMMON_CALCS_FOR_ALL_JOINT_TYPES;
            }
            // Common for ROTATE and SLIDE
            link->vo().noalias() = link->dq() * link->sv() + parent->vo();
            const Vector3 dsv = parent->w().cross(link->sv()) + parent->vo().cross(link->sw());
            const Vector3 dsw = parent->w().cross(link->sw());
            link->cv() = link->dq() * dsv;
            link->cw() = link->dq() * dsw;
        }
COMMON_CALCS_FOR_ALL_JOINT_TYPES:
        link->v().noalias() = link->vo() + link->w().cross(link->p());
        link->wc().noalias() = link->R() * link->c() + link->p();
    }
}
    

DyLink::DyLink()
{
    subBody_ = nullptr;
}


DyLink::DyLink(const Link& link)
    : Link(link)
{
    subBody_ = nullptr;
}


DyLink::DyLink(const DyLink& org)
    : Link(org)
{
    subBody_ = nullptr;
}


Referenced* DyLink::doClone(CloneMap*) const
{
    return new DyLink(*this);
}


Link* DyBody::createLink(const Link* org) const
{
    return org ? new DyLink(*org) : new DyLink;
}


void DyLink::initializeState()
{
    Link::initializeState();

    vo_.setZero();
    dvo_.setZero();
    uu_ = 0.0;
    dd_ = 0.0;
}


void DyLink::prependChild(Link* link)
{
    if(DyLink* dyLink = dynamic_cast<DyLink*>(link)){
        Link::prependChild(dyLink);
    } else {
        throw type_mismatch_error();
    }
}


void DyLink::appendChild(Link* link)
{
    if(DyLink* dyLink = dynamic_cast<DyLink*>(link)){
        Link::appendChild(dyLink);
    } else {
        throw type_mismatch_error();
    }
}


DyBody::DyBody()
    : Body(new DyLink)
{

}


Referenced* DyBody::doClone(CloneMap* cloneMap) const
{
    auto body = new DyBody;
    body->copyFrom(this, cloneMap);
    return body;
}


void DyBody::initializeSubBodies()
{
    subBodies_.clear();

    typedef std::multimap<Link*, ForceSensor*> ForceSensorMap;
    ForceSensorMap forceSensorMap;
    for(auto& sensor : devices<ForceSensor>()){
        forceSensorMap.insert(ForceSensorMap::value_type(sensor->link(), sensor));
    }

    subBodies_.push_back(new DySubBody(rootLink(), forceSensorMap));

    int n = numLinks();
    for(int i=1; i < n; ++i){
        auto lnk = link(i);
        if(lnk->isFreeJoint()){
            subBodies_.push_back(new DySubBody(lnk, forceSensorMap));
        }
    }
}


void DyBody::calcSpatialForwardKinematics()
{
    for(auto& subBody : subBodies_){
        subBody->calcSpatialForwardKinematics();
    }
}
