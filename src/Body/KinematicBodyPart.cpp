#include "KinematicBodyPart.h"

using namespace std;
using namespace cnoid;


KinematicBodyPart::KinematicBodyPart(Body* body)
    : body_(body),
      jointTraverse_(make_shared<JointTraverse>(body))
{

}


KinematicBodyPart::KinematicBodyPart(shared_ptr<JointTraverse> jointTraverse)
    : jointTraverse(jointTraverse)
{

}


KinematicBodyPart::KinematicBodyPart(LinkKinematicsKit* linkKinematicsKit)
    : body_(linkKinematicsKit->body()),
      linkKinematicsKit(linkKinematicsKit)
{

}


const std::vector<LinkPtr>& KinematicBodyPart::joints() const
{
    if(jointTraverse_){
        return jointTraverse_->joints();
    } else if(linkKinematicsKit_){
        return linkKinematicsKit_->joints();
    }
    return emptyJoints_;
}


void KinematicBodyPart::calcForwardKinematics(bool calcVelocity = false, bool calcAcceleration = false) const
{
    if(jointTraverse_){
        jointTraverse_->calcForwardKinematics(calcVelocity, calcAcceleration);
    } else if(linkKinematicsKit_){
        if(auto jointPath = linkKinematicsKit_->jointPath()){
            jointPath->calcForwardKinematics(calcVelocity, calcAcceleration);
        }
    }
}

    

