#ifndef CNOID_BODY_LINK_KINEMATICS_KIT_SET_H
#define CNOID_BODY_LINK_KINEMATICS_KIT_SET_H

#include "LinkKinematicsKit.h"
#include <cnoid/GeneralId>
#include <cnoid/ConnectionSet>
#include <unordered_map>
#include "exportdecl.h"

namespace cnoid {

class CNOID_EXPORT LinkKinematicsKitSet : public ClonableReferenced
{
public:
    LinkKinematicsKitSet();
    ~LinkKinematicsKitSet();

    void clearKinematicsKits();
    bool setKinematicsKit(const GeneralId& id, LinkKinematicsKit* kit);
    int numKinematicsKits() const { return kitMap.size(); }
    LinkKinematicsKit* kinematicsKit(const GeneralId& partId);
    void setMainPart(const GeneralId& partId) { mainPartId_ = partId; }
    const GeneralId& mainPartId() const { return mainPartId_; }
    LinkKinematicsKit* mainKinematicsKit();

    //! The signal is emitted when any sub kinematics kit emits the same signal.
    SignalProxy<void()> sigFrameSetChange() { return sigFrameSetChange_; }

    //! The signal is emitted when any sub kinematics kit emits the same signal.
    SignalProxy<void(const Isometry3& T_frameCoordinate)> sigPositionError();

protected:
    LinkKinematicsKitSet(const LinkKinematicsKitSet& org, CloneMap* cloneMap);
    virtual Referenced* doClone(CloneMap* cloneMap) const override;

private:
    struct KitInfo
    {
        LinkKinematicsKitPtr kinematicsKit;
        ScopedConnectionSet connections;
    };
    std::unordered_map<GeneralId, KitInfo> kitMap;
    GeneralId mainPartId_;
    Signal<void()> sigFrameSetChange_;
    Signal<void(const Isometry3& T_frameCoordinate)> sigPositionError_;
};

typedef ref_ptr<LinkKinematicsKitSet> LinkKinematicsKitSetPtr;
    
}

#endif
