#include "JointTraverse.h"
#include "Body.h"
#include <cnoid/CloneMap>
#include <algorithm>
#include <numeric>

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


JointTraverse::JointTraverse(Link* baseLink, bool toUpper, bool toLower)
    : linkTraverse_(baseLink, toUpper, toLower)
{
    joints_.reserve(linkTraverse_.numLinks() - 1);
    for(auto& link : linkTraverse_){
        if(link->hasActualJoint()){
            joints_.push_back(link);
        }
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
            joints_.push_back(cloneMap->getClone<Link>(joint));
        }
    }
}


void JointTraverse::clear()
{
    linkTraverse_.clear();
    joints_.clear();
    invalidateIdOrderCache();
}


bool JointTraverse::appendJoint(Link* joint)
{
    if(joint->hasActualJoint()){
        joints_.push_back(joint);
        invalidateIdOrderCache();
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
        invalidateIdOrderCache();
        removed = true;
    }

    return removed;
}


void JointTraverse::updateIdOrderIndices() const
{
    if(idOrderIndices_.empty() && !joints_.empty()){
        int n = joints_.size();
        idOrderIndices_.resize(n);
        std::iota(idOrderIndices_.begin(), idOrderIndices_.end(), 0);
        std::sort(idOrderIndices_.begin(), idOrderIndices_.end(),
                  [this](int a, int b){
                      return joints_[a]->jointId() < joints_[b]->jointId();
                  });
    }
}


Link* JointTraverse::jointAtIdOrder(int index) const
{
    updateIdOrderIndices();
    return joints_[idOrderIndices_[index]];
}


int JointTraverse::jointIndexAtIdOrder(int index) const
{
    updateIdOrderIndices();
    return idOrderIndices_[index];
}


    


    

