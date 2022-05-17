#ifndef CNOID_BODY_KINEMATIC_BODY_PART_H
#define CNOID_BODY_KINEMATIC_BODY_PART_H

#include "Body.h"
#include "JointTraverse.h"
#include "LinkKinematicsKit.h"
#include <cnoid/Signal>
#include "exportdecl.h"

namespace cnoid {

class CloneMap;

class CNOID_EXPORT KinematicBodyPart : public ClonableReferenced
{
public:
    KinematicBodyPart();
    KinematicBodyPart(std::shared_ptr<JointTraverse> jointTraverse);
    KinematicBodyPart(LinkKinematicsKit* linkKinematicsKit);

    KinematicBodyPart* clone(CloneMap* cloneMap) const {
        return static_cast<KinematicBodyPart*>(doClone(cloneMap));
    }

    void setJointTraverse(std::shared_ptr<JointTraverse> traverse);
    void setLinkKinematicsKit(LinkKinematicsKit* kit);

    Body* body() { return body_; }
    const Body* body() const { return body_; }

    bool isLinkKinematicsKit() const { return static_cast<bool>(linkKinematicsKit_); }
    LinkKinematicsKitPtr linkKinematicsKit() { return linkKinematicsKit_; }
    bool isJointTraverse() const { return static_cast<bool>(jointTraverse_); }
    std::shared_ptr<JointTraverse> jointTraverse() { return jointTraverse_; }

    int numJoints() const;
    const std::vector<LinkPtr>& joints() const;
    void calcForwardKinematics(bool calcVelocity = false, bool calcAcceleration = false) const;

    SignalProxy<void()> sigFrameSetChange() { return sigFrameSetChange_; }
    SignalProxy<void(const Isometry3& T_frameCoordinate)> sigPositionError();

protected:
    KinematicBodyPart(const KinematicBodyPart& org, CloneMap* cloneMap);
    virtual Referenced* doClone(CloneMap* cloneMap) const override;

private:
    BodyPtr body_;
    // Either one of the following is valid
    std::shared_ptr<JointTraverse> jointTraverse_;
    LinkKinematicsKitPtr linkKinematicsKit_;
    std::vector<LinkPtr> emptyJoints;
    Signal<void()> sigFrameSetChange_;
};

typedef ref_ptr<KinematicBodyPart> KinematicBodyPartPtr;


}

#endif
