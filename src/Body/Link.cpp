/**
   \file
   \author Shin'ichiro Nakaoka
*/

#include "Link.h"
#include <cnoid/SceneGraph>

using namespace std;
using namespace cnoid;


Link::Link()
{
    index_ = -1;
    jointId_ = -1;
    parent_ = 0;
    sibling_ = 0;
    child_ = 0;
    T_.setIdentity();
    Tb_.setIdentity();
    Rs_.setIdentity();
    a_ = Vector3::UnitZ();
    jointType_ = FIXED_JOINT;
    q_ = 0.0;
    dq_ = 0.0;
    ddq_ = 0.0;
    u_ = 0.0;
    v_.setZero();
    w_.setZero();
    dv_.setZero();
    dw_.setZero();
    c_.setZero();
    wc_.setZero();
    m_ = 0.0;
    I_.setZero();
    Jm2_ = 0.0;
    F_ext_.setZero();
    q_upper_ = std::numeric_limits<double>::max();
    q_lower_ = -std::numeric_limits<double>::max();
    dq_upper_ = std::numeric_limits<double>::max();
    dq_lower_ = -std::numeric_limits<double>::max();
}


Link::Link(const Link& org)
    : name_(org.name_)
{
    index_ = -1; // should be set by a Body object
    jointId_ = org.jointId_;

    parent_ = 0;
    sibling_ = 0;
    child_ = 0;

    T_ = org.T_;
    Tb_ = org.Tb_;
    Rs_ = org.Rs_;
    
    a_ = org.a_;
    jointType_ = org.jointType_;

    q_ = org.q_;
    dq_ = org.dq_;
    ddq_ = org.ddq_;
    u_ = org.u_;

    v_ = org.v_;
    w_ = org.w_;
    dv_ = org.dv_;
    dw_ = org.dw_;
    
    c_ = org.c_;
    wc_ = org.wc_;
    m_ = org.m_;
    I_ = org.I_;
    Jm2_ = org.Jm2_;

    F_ext_ = org.F_ext_;

    q_upper_ = org.q_upper_;
    q_lower_ = org.q_lower_;
    dq_upper_ = org.dq_upper_;
    dq_lower_ = org.dq_lower_;

    //! \todo add the mode for doing deep copy of the shape object
    visualShape_ = org.visualShape_;
    collisionShape_ = org.collisionShape_;
}


Link::~Link()
{
    Link* link = child();
    while(link){
        Link* linkToDelete = link;
        link = link->sibling();
        delete linkToDelete;
    }
}


void Link::prependChild(Link* link)
{
    if(link->parent_){
        link->parent_->removeChild(link);
    }
    link->sibling_ = child_;
    child_ = link;
    link->parent_ = this;
}


void Link::appendChild(Link* link)
{
    if(link->parent_){
        link->parent_->removeChild(link);
    }
    if(!child_){
        child_ = link;
        link->sibling_ = 0;
    } else {
        Link* lastChild = child_;
        while(lastChild->sibling_){
            lastChild = lastChild->sibling_;
        }
        lastChild->sibling_ = link;
        link->sibling_ = 0;
    }
    link->parent_ = this;
}


/**
   A child link is removed from the link.
   The detached child link is *not* deleted by this function.
   If a link given by the parameter is not a child of the link, false is returned.
*/
bool Link::removeChild(Link* childToRemove)
{
    bool removed = false;

    Link* link = child_;
    Link* prevSibling = 0;
    while(link){
        if(link == childToRemove){
            if(prevSibling){
                prevSibling->sibling_ = link->sibling_;
            } else {
                child_ = link->sibling_;
            }
            removed = true;
            break;
        }
        prevSibling = link;
        link = link->sibling_;
    }
    if(removed){
        childToRemove->parent_ = 0;
        childToRemove->sibling_ = 0;
    }
    return removed;
}


void Link::setName(const std::string& name)
{
    name_ = name;
}


void Link::setShape(SgNodePtr shape)
{
    visualShape_ = shape;
    collisionShape_ = shape;
}

void Link::setVisualShape(SgNodePtr shape)
{
    visualShape_ = shape;
}


void Link::setCollisionShape(SgNodePtr shape)
{
    collisionShape_ = shape;
}
