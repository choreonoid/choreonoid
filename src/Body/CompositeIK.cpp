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
        JointPathPtr path = getCustomJointPath(body_, targetLink_, baseLink);
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


bool CompositeIK::calcInverseKinematics(const Vector3& p, const Matrix3& R)
{
    const int n = body_->numJoints();

    Vector3 p0 = targetLink_->p();
    Matrix3 R0 = targetLink_->R();
    q0.resize(n);
    for(int i=0; i < n; ++i){
        q0[i] = body_->joint(i)->q();
    }

    targetLink_->p() = p;
    targetLink_->R() = R;
    bool solved = true;
    size_t pathIndex;
    for(pathIndex=0; pathIndex < paths.size(); ++pathIndex){
        JointPath& path = *paths[pathIndex];
        Link* link = path.endLink();
        Position T0 = link->T();
        //solved = path.setGoal(p, R, link->p(), link->R()).calcInverseKinematics();
        solved = path.calcInverseKinematics(p, R, link->p(), link->R());
        if(!solved){
            link->setPosition(T0);
            break;
        }
    }

    if(!solved){
        targetLink_->p() = p0;
        targetLink_->R() = R0;
        for(int i=0; i < n; ++i){
            body_->joint(i)->q() = q0[i];
        }
        for(size_t i=0; i < pathIndex; ++i){
            paths[i]->calcForwardKinematics();
        }
    }

    return solved;
}
