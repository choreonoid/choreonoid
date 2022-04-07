#ifndef CNOID_BODY_LINK_KINEMATICS_KIT_SET_H
#define CNOID_BODY_LINK_KINEMATICS_KIT_SET_H

#include "LinkKinematicsKit.h"
#include <cnoid/ConnectionSet>
#include "exportdecl.h"

namespace cnoid {

class CNOID_EXPORT LinkKinematicsKitSet : public ClonableReferenced
{
public:
    LinkKinematicsKitSet();
    ~LinkKinematicsKitSet();

    void clearKinematicsKits();
    void addKinematicsKit(LinkKinematicsKit* kit);
    int numKinematicsKits() const { return elements.size(); }
    LinkKinematicsKit* kinematicsKit(int index) { return elements[index].kinematicsKit; }
    void setMainKinematicsKit(int index) { mainKinematicsKitIndex_ = index; }
    int mainKinematicsKitIndex() const { return mainKinematicsKitIndex_; }
    LinkKinematicsKit* mainKinematicsKit() {
        return mainKinematicsKitIndex_ >= 0 ? elements[mainKinematicsKitIndex_].kinematicsKit : nullptr;
    }

    //! The signal is emitted when any sub kinematics kit emits the same signal.
    SignalProxy<void()> sigFrameSetChange() { return sigFrameSetChange_; }

    //! The signal is emitted when any sub kinematics kit emits the same signal.
    SignalProxy<void(const Isometry3& T_frameCoordinate)> sigPositionError();

protected:
    LinkKinematicsKitSet(const LinkKinematicsKitSet& org, CloneMap* cloneMap);
    virtual Referenced* doClone(CloneMap* cloneMap) const override;

private:
    struct Element
    {
        LinkKinematicsKitPtr kinematicsKit;
        ConnectionSet connections;
    };
    std::vector<Element> elements;
    int mainKinematicsKitIndex_;
    Signal<void()> sigFrameSetChange_;
    Signal<void(const Isometry3& T_frameCoordinate)> sigPositionError_;
};

typedef ref_ptr<LinkKinematicsKitSet> LinkKinematicsKitSetPtr;
    
}

#endif
