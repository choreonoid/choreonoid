#include "JointTraverse.h"
#include "Body.h"
#include <cnoid/CloneMap>

using namespace cnoid;

JointTraverse::JointTraverse()
{

}


JointTraverse::JointTraverse(Body* body)
    : linkTraverse_(body->linkTraverse())
{
    joints_.reserve(body->numJoints());
    for(auto& joint : body->joints()){
        joints_.push_back(joint);
    }
}


JointTraverse::JointTraverse(const JointTraverse& org, CloneMap* cloneMap)
    : linkTraverse_(org.linkTraverse_, cloneMap)
{
    if(!cloneMap){
        joints_ = org.joints_;
    } else {
        joints_.reserve(org.joints_.size());
        for(auto& joint : org.joints_){
            joints_.push_back(joint->clone(cloneMap));
        }
    }
}


void JointTraverse::clear()
{
    linkTraverse_.clear();
    joints_.clear();
}


bool JointTraverse::appendJoint(Link* joint)
{
    if(joint->hasActualJoint()){
        joints_.push_back(joint);
        return true;
    }
    return false;
}


bool JointTraverse::remove(Link* link)
{
    bool removed = linkTraverse_.remove(link);

    auto it = std::find(joints_.begin(), joints_.end(), link);
    if(it != joints_.end()){
        joints_.erase(it);
        removed = true;
    }

    return removed;
}


    


    

