#include "KinematicBodyPart.h"
#include "JointPath.h"

using namespace std;
using namespace cnoid;


KinematicBodyPart::KinematicBodyPart()
{

}


KinematicBodyPart::KinematicBodyPart(std::shared_ptr<JointTraverse> traverse)
{
    setJointTraverse(traverse);
}


KinematicBodyPart::KinematicBodyPart(LinkKinematicsKit* kit)
{
    setLinkKinematicsKit(kit);
}


KinematicBodyPart::KinematicBodyPart(const KinematicBodyPart& org, CloneMap* cloneMap)
{
    if(org.jointTraverse_){
        setJointTraverse(make_shared<JointTraverse>(*org.jointTraverse_, cloneMap));
    } else if(org.linkKinematicsKit_){
        setLinkKinematicsKit(org.linkKinematicsKit_->clone(cloneMap));
    }
}


Referenced* KinematicBodyPart::doClone(CloneMap* cloneMap) const
{
    return new KinematicBodyPart(*this, cloneMap);
}


void KinematicBodyPart::setJointTraverse(std::shared_ptr<JointTraverse> traverse)
{
    body_ = traverse->body();
    jointTraverse_ = traverse;
    linkKinematicsKit_.reset();
}


void KinematicBodyPart::setLinkKinematicsKit(LinkKinematicsKit* kit)
{
    body_ = kit->body();
    linkKinematicsKit_ = kit;
    jointTraverse_.reset();
}


int KinematicBodyPart::numJoints() const
{
    if(jointTraverse_){
        return jointTraverse_->numJoints();
    } else if(linkKinematicsKit_){
        if(auto jointPath = linkKinematicsKit_->jointPath()){
            return jointPath->numJoints();
        }
    }
    return 0;
}


const std::vector<LinkPtr>& KinematicBodyPart::joints() const
{
    if(jointTraverse_){
        return jointTraverse_->joints();
    } else if(linkKinematicsKit_){
        if(auto jointPath = linkKinematicsKit_->jointPath()){
            return jointPath->joints();
        }
    }
    return emptyJoints;
}


void KinematicBodyPart::calcForwardKinematics(bool calcVelocity, bool calcAcceleration) const
{
    if(jointTraverse_){
        jointTraverse_->calcForwardKinematics(calcVelocity, calcAcceleration);
    } else if(linkKinematicsKit_){
        if(auto jointPath = linkKinematicsKit_->jointPath()){
            jointPath->calcForwardKinematics(calcVelocity, calcAcceleration);
        }
    }
}
