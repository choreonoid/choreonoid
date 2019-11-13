/**
   \file
   \author Shin'ichiro Nakaoka
*/

#include "DyBody.h"
#include <cnoid/Exception>

using namespace std;
using namespace cnoid;


DyLink::DyLink()
{

}


DyLink::DyLink(const Link& link)
    : Link(link)
{

}


Referenced* DyLink::doClone(CloneMap*) const
{
    return new DyLink(*this);
}


void DyLink::initializeState()
{
    Link::initializeState();

    vo_.setZero();
    dvo_.setZero();
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


Link* DyBody::createLink(const Link* org) const
{
    return org ? new DyLink(*org) : new DyLink();
}


void DyBody::calcSpatialForwardKinematics()
{
    const LinkTraverse& traverse = linkTraverse();
    const int n = traverse.numLinks();
    for(int i=0; i < n; ++i){
        DyLink* link = static_cast<DyLink*>(traverse[i]);
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
