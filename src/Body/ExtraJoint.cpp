#include "ExtraJoint.h"
#include "Body.h"
#include <cnoid/CloneMap>

using namespace std;
using namespace cnoid;


ExtraJoint::ExtraJoint(const ExtraJoint& org, CloneMap* cloneMap)
    : type_(org.type_),
      axis_(org.axis_)
{
    for(int i = 0; i < 2; ++i){
        T[i] = org.T[i];
    }
    if(cloneMap){
        for(int i = 0; i < 2; ++i){
            if(org.links[i]){
                links[i] = cloneMap->getClone<Link>(org.links[i]);
            }
        }
        info_ = CloneMap::getClone(org.info_.get(), cloneMap);
    } else {
        for(int i = 0; i < 2; ++i){
            links[i] = org.links[i];
        }
        info_ = org.info_;
    }
}


ExtraJoint* ExtraJoint::doClone(CloneMap* cloneMap) const
{
    return new ExtraJoint(*this, cloneMap);
}


std::string ExtraJoint::bodyName(int which) const
{
    if(auto link = links[which]){
        if(auto body = link->body()){
            return body->name();
        }
    }
    return string();
}

