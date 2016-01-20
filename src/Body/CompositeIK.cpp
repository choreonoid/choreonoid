/*!
  @file
  @author Shin'ichiro Nakaoka
*/

#include "CompositeIK.h"
#include "Link.h"
#include "JointPath.h"

using namespace std;
using namespace boost;
using namespace cnoid;


CompositeIK::CompositeIK()
{
    targetLink_ = 0;
    isAnalytical_ = false;
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
    isAnalytical_ = false;
    pathList.clear();
}
   

bool CompositeIK::addBaseLink(Link* baseLink)
{
    if(baseLink && targetLink_){
        JointPathPtr path = getCustomJointPath(body_, targetLink_, baseLink);
        if(path){
            isAnalytical_ = pathList.empty() ? path->hasAnalyticalIK() : (isAnalytical_ && path->hasAnalyticalIK());
            PathInfo info;
            info.path = path;
            info.endLink = baseLink;
            pathList.push_back(info);
            return true;
        }
    }
    return false;
}


void CompositeIK::setMaxIKerror(double e)
{
    for(size_t i=0; i < pathList.size(); ++i){
        pathList[i].path->setNumericalIKmaxIKerror(e);
    }
}


bool CompositeIK::hasAnalyticalIK() const
{
    return isAnalytical_;
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
    for(size_t i=0; i < pathList.size(); ++i){
        PathInfo& info = pathList[i];
        info.p_given = info.endLink->p();
        info.R_given = info.endLink->R();
        //solved = info.path->setGoal(p, R, info.p_given, info.R_given).calcInverseKinematics();
        solved = info.path->calcInverseKinematics(p, R, info.p_given, info.R_given);
        if(!solved){
            break;
        }
        info.endLink->p() = info.p_given;
        info.endLink->R() = info.R_given;
    }

    if(!solved){
        targetLink_->p() = p0;
        targetLink_->R() = R0;
        for(int i=0; i < n; ++i){
            body_->joint(i)->q() = q0[i];
        }
        for(size_t i=0; i < pathList.size(); ++i){
            PathInfo& info = pathList[i];
            info.path->calcForwardKinematics();
            info.endLink->p() = info.p_given;
            info.endLink->R() = info.R_given;
        }
    }

    return solved;
}
