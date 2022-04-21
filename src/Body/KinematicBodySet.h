#ifndef CNOID_BODY_KINEMATIC_BODY_SET_H
#define CNOID_BODY_KINEMATIC_BODY_SET_H

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
    ~KinematicBodySet();

    void setBodyPart(const GeneralId& partId, LinkKinematicsKit* kit);
    void clear();
    void clearBodyPart(const GeneralId& partId);
    
    int numKinematicBodyParts() const;
    LinkKinematicsKit* kinematicsKit(const GeneralId& partId);
    void setMainBodyPartId(const GeneralId& partId);
    const GeneralId& mainBodyPartId() const;
    LinkKinematicsKit* mainKinematicsKit();

    //! The signal is emitted when any sub kinematics kit emits the same signal.
    SignalProxy<void()> sigFrameSetChange();

    //! The signal is emitted when any sub kinematics kit emits the same signal.
    SignalProxy<void(const Isometry3& T_frameCoordinate)> sigPositionError();

    class BodyPart : public Referenced
    {
        LinkKinematicsKitPtr kinematicsKit;
        ScopedConnectionSet connections;
        friend class KinematicBodySet;
    };

protected:
    typedef std::function<BodyPart*()> CreateBodyPartFunc;
    typedef std::function<void(BodyPart* newBodyPart, BodyPart* orgBodyPart, CloneMap* cloneMap)> CopyBodyPartFunc;
    
    KinematicBodySet(CreateBodyPartFunc createBodyPart, CopyBodyPartFunc copyBodyPart);
    
    virtual Referenced* doClone(CloneMap* cloneMap) const override;

    void copyBodyPart(BodyPart* newBodyPart, BodyPart* orgBodyPart, CloneMap* cloneMap);
    void initializeBodyPart(BodyPart* bodyPart, LinkKinematicsKit* kinematicsKit);
    BodyPart* findBodyPart(const GeneralId& partId);
    BodyPart* findOrCreateBodyPart(const GeneralId& partId);

private:
    class Impl;
    Impl* impl;
};

typedef ref_ptr<KinematicBodySet> KinematicBodySetPtr;
    
}

#endif
