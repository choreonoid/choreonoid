#ifndef CNOID_BODY_KINEMATIC_BODY_SET_H
#define CNOID_BODY_KINEMATIC_BODY_SET_H

#include "JointTraverse.h"
#include "LinkKinematicsKit.h"
#include <cnoid/GeneralId>
#include <cnoid/ConnectionSet>
#include <functional>
#include "exportdecl.h"

namespace cnoid {

class CNOID_EXPORT KinematicBodySet : public ClonableReferenced
{
public:
    KinematicBodySet();
    KinematicBodySet(const KinematicBodySet& org, CloneMap* cloneMap);

    void setBodyPart(int index, LinkKinematicsKit* linkKinematicsKit);
    void setBodyPart(int index, std::shared_ptr<JointTraverse> jointTraverse);
    void clearBodyPart(int index);
    void clear();
    bool empty() const { return bodyParts_.empty(); }
    int maxIndex() const { return bodyParts_.size() - 1; }
    std::vector<int> validBodyPartIndices() const;
    void setMainBodyPartIndex(int index) { mainBodyPartIndex_ = index; }
    int mainBodyPartIndex() const { return mainBodyPartIndex_; }

    class BodyPart : public Referenced
    {
    public:
        bool isLinkKinematicsKit() const { return static_cast<bool>(linkKinematicsKit_); }
        LinkKinematicsKitPtr linkKinematicsKit() { return linkKinematicsKit_; }
        bool isJointTraverse() const { return static_cast<bool>(jointTraverse_); }
        std::shared_ptr<JointTraverse> jointTraverse() { return jointTraverse_; }

    private:
        // Either one of the following is valid
        LinkKinematicsKitPtr linkKinematicsKit_;
        std::shared_ptr<JointTraverse> jointTraverse_;
        
        ScopedConnectionSet connections;

        friend class KinematicBodySet;
    };

    BodyPart* bodyPart(int index) { return bodyParts_[index]; }
    const BodyPart* bodyPart(int index) const { return bodyParts_[index]; }
    BodyPart* mainBodyPart() {
        return (mainBodyPartIndex_ >= 0) ? bodyParts_[mainBodyPartIndex_] : nullptr;
    }
    const BodyPart* mainBodyPart() const {
        return const_cast<KinematicBodySet*>(this)->mainBodyPart();
    }

    SignalProxy<void()> sigUpdated() { return sigUpdated_; }
    void notifyUpdate() { sigUpdated_(); }
    
    //! The signal is emitted when any sub kinematics kit emits the corresponding signal.
    SignalProxy<void()> sigFrameSetChange() { return sigFrameSetChange_; }

    //! The signal is emitted when any sub kinematics kit emits the corresponding signal.
    SignalProxy<void(const Isometry3& T_frameCoordinate)> sigPositionError();

protected:
    typedef std::function<BodyPart*()> CreateBodyPartFunc;
    typedef std::function<void(BodyPart* newBodyPart, BodyPart* orgBodyPart, CloneMap* cloneMap)> CopyBodyPartFunc;
    
    KinematicBodySet(CreateBodyPartFunc createBodyPart, CopyBodyPartFunc copyBodyPart);
    
    virtual Referenced* doClone(CloneMap* cloneMap) const override;

    void copyBodyPart(BodyPart* newBodyPart, BodyPart* orgBodyPart, CloneMap* cloneMap);
    void initializeBodyPart(BodyPart* bodyPart, LinkKinematicsKit* kinematicsKit);
    void initializeBodyPart(BodyPart* bodyPart, std::shared_ptr<JointTraverse> jointTraverse);
    BodyPart* findOrCreateBodyPart(int index);

private:
    std::vector<ref_ptr<BodyPart>> bodyParts_;
    int mainBodyPartIndex_;

    // Function objects are used instead of virtual functions so that the functions can be used in the constructor.
    CreateBodyPartFunc createBodyPartFunc;
    CopyBodyPartFunc copyBodyPartFunc;

    Signal<void()> sigUpdated_;
    Signal<void()> sigFrameSetChange_;
    Signal<void(const Isometry3& T_frameCoordinate)> sigPositionError_;
};

typedef ref_ptr<KinematicBodySet> KinematicBodySetPtr;
    
}

#endif
