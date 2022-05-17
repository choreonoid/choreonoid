#ifndef CNOID_BODY_KINEMATIC_BODY_PART_H
#define CNOID_BODY_KINEMATIC_BODY_PART_H

#include "JointTraverse.h"
#include "LinkKinematicsKit.h"
#include "exportdecl.h"

namespace cnoid {

class CNOID_EXPORT KinematicBodyPart : public Referenced
{
public:
    KinematicBodyPart(Body* body);
    KinematicBodyPart(shared_ptr<JointTraverse> jointTraverse);
    KinematicBodyPart(LinkKinematicsKit* linkKinematicsKit);
    
    Body* body() { return body_; }
    const Body* body() const { return body_; }

    bool isLinkKinematicsKit() const { return static_cast<bool>(linkKinematicsKit_); }
    LinkKinematicsKitPtr linkKinematicsKit() { return linkKinematicsKit_; }
    bool isJointTraverse() const { return static_cast<bool>(jointTraverse_); }
    std::shared_ptr<JointTraverse> jointTraverse() { return jointTraverse_; }

    const std::vector<LinkPtr>& joints() const;
    void calcForwardKinematics(bool calcVelocity = false, bool calcAcceleration = false) const;

private:
    BodyPtr body_;
    // Either one of the following is valid
    std::shared_ptr<JointTraverse> jointTraverse_;
    LinkKinematicsKitPtr linkKinematicsKit_;
    std::vector<LinkPtr> emptyJoints_;
    ScopedConnectionSet connections;
};

typedef ref_ptr<KinematicBodyPart> KinematicBodyPartPtr;


}

#endif


