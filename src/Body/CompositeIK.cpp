/*!
  @file
  @author Shin'ichiro Nakaoka
*/

#include "CompositeIK.h"
#include "Link.h"

using namespace std;
using namespace cnoid;


CompositeIK::CompositeIK()
{
    targetLink_ = 0;
    hasAnalyticalIK_ = false;
}


CompositeIK::CompositeIK(Body* body, Link* targetLink)
{
    reset(body, targetLink);
}


CompositeIK::~CompositeIK()
{

}


void CompositeIK::reset(Body* body, Link* targetLink)
{
    body_ = body;
    targetLink_ = targetLink;
    hasAnalyticalIK_ = false;
    paths.clear();
}
   

bool CompositeIK::addBaseLink(Link* baseLink)
{
    if(baseLink && targetLink_){
        auto path = getCustomJointPath(body_, targetLink_, baseLink);
        if(path){
            hasAnalyticalIK_ = paths.empty() ? path->hasAnalyticalIK() : (hasAnalyticalIK_ && path->hasAnalyticalIK());
            paths.push_back(path);
            return true;
        }
    }
    return false;
}


void CompositeIK::setMaxIKerror(double e)
{
    for(size_t i=0; i < paths.size(); ++i){
        paths[i]->setNumericalIKmaxIKerror(e);
    }
}


bool CompositeIK::calcInverseKinematics(const Position& T)
{
    const int n = body_->numJoints();

    Position T0 = targetLink_->T();
    q0.resize(n);
    for(int i=0; i < n; ++i){
        q0[i] = body_->joint(i)->q();
    }

    targetLink_->setPosition(T);

    bool solved = true;
    size_t pathIndex;
    for(pathIndex=0; pathIndex < paths.size(); ++pathIndex){
        JointPath& path = *paths[pathIndex];
        Link* link = path.endLink();
        Position T_end = link->T();
        solved = path.setBaseLinkGoal(T).calcInverseKinematics(T_end);
        link->setPosition(T_end);
        if(!solved){
            break;
        }
    }

    if(!solved){
        targetLink_->setPosition(T0);
        for(int i=0; i < n; ++i){
            body_->joint(i)->q() = q0[i];
        }
        for(size_t i=0; i < pathIndex; ++i){
            paths[i]->calcForwardKinematics();
        }
    }

    return solved;
}
