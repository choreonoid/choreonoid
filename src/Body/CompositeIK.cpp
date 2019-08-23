/*!
  @file
  @author Shin'ichiro Nakaoka
*/

#include "CompositeIK.h"
#include "Body.h"
#include "Link.h"
#include "JointPath.h"

using namespace std;
using namespace cnoid;


CompositeIK::CompositeIK()
{
    targetLink_ = nullptr;
    hasAnalyticalIK_ = false;
}


CompositeIK::CompositeIK(Body* body, Link* targetLink)
{
    reset(body, targetLink);
}


void CompositeIK::reset(Body* body, Link* targetLink)
{
    body_ = body;
    targetLink_ = targetLink;
    hasAnalyticalIK_ = false;
    paths.clear();
    remainingLinkTraverse.reset();
}
   

bool CompositeIK::addBaseLink(Link* baseLink)
{
    if(baseLink && targetLink_){
        auto path = JointPath::getCustomPath(body_, baseLink, targetLink_);
        if(path){
            hasAnalyticalIK_ = paths.empty() ? path->hasAnalyticalIK() : (hasAnalyticalIK_ && path->hasAnalyticalIK());
            paths.push_back(path);
            remainingLinkTraverse.reset();
            return true;
        }
    }
    return false;
}


Link* CompositeIK::baseLink(int index) const
{
    return paths[index]->baseLink();
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

    bool solved = true;
    size_t pathIndex = 0;
    while(true){
        JointPath& path = *paths[pathIndex];
        solved = path.calcInverseKinematics(T);
        if(!solved){
            break;
        }
        ++pathIndex;
        if(pathIndex < paths.size()){
            targetLink_->setPosition(T0);
        } else {
            break;
        }
    }

    if(solved){
        targetLink_->setPosition(T);

    } else {
        targetLink_->setPosition(T0);
        for(int i=0; i < n; ++i){
            body_->joint(i)->q() = q0[i];
        }
        for(size_t i=0; i <= pathIndex; ++i){
            paths[i]->calcForwardKinematics();
        }
    }

    return solved;
}


bool CompositeIK::calcRemainingPartForwardKinematicsForInverseKinematics()
{
    if(!remainingLinkTraverse){
        remainingLinkTraverse = make_shared<LinkTraverse>(targetLink_, true, true);
        for(auto& jointPath : paths){
            for(auto& link : jointPath->linkPath()){
                remainingLinkTraverse->remove(link);
            }
        }
        remainingLinkTraverse->prependRootAdjacentLinkToward(targetLink_);
    }
    remainingLinkTraverse->calcForwardKinematics();
    return true;
}

    

