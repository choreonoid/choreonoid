#ifndef CNOID_BODY_PLUGIN_KINEMATIC_BODY_ITEM_SET_H
#define CNOID_BODY_PLUGIN_KINEMATIC_BODY_ITEM_SET_H

#include "BodyItemKinematicsKit.h"
#include <cnoid/KinematicBodySet>
#include <cnoid/ConnectionSet>
#include <map>
#include <vector>
#include "exportdecl.h"

namespace cnoid {

class BodyItem;
class Link;
typedef ref_ptr<Link> LinkPtr;

/**
   KinematicBodyItemSet is the BodyItem-level counterpart of KinematicBodySet:
   each "body part" slot is implemented by a BodyItemKinematicsKit and refers
   to a BodyItem in the project tree. The class also tracks the lifetime and
   model structure of each member BodyItem, and exposes onBodyItemModelUpdated()
   as a hook for derived classes to react when a member's joint configuration
   actually changes. See KinematicBodySet for the meaning of "part" and the
   overall ensemble abstraction.
*/
class CNOID_EXPORT KinematicBodyItemSet : public KinematicBodySet
{
public:
    KinematicBodyItemSet();

    void setBodyItemPart(int index, BodyItemKinematicsKit* kinematicsKit) {
        setBodyPart(index, kinematicsKit);
    }
    virtual void setBodyPart(int index, BodyKinematicsKit* kinematicsKit) override;
    virtual void removeBodyPart(int index) override;

    BodyItemKinematicsKit* bodyItemPart(int index){
        return static_cast<BodyItemKinematicsKit*>(bodyPart(index));
    }
    const BodyItemKinematicsKit* bodyItemPart(int index) const {
        return static_cast<const BodyItemKinematicsKit*>(bodyPart(index));
    }
    BodyItem* bodyItem(int index) {
        if(auto part = bodyItemPart(index)){
            return part->bodyItem();
        }
        return nullptr;
    }
    const BodyItem* bodyItem(int index) const {
        return const_cast<KinematicBodyItemSet*>(this)->bodyItem(index);
    }

    int indexOf(const BodyItem* bodyItem) const;
    bool contains(const BodyItem* bodyItem) const;

    BodyItemKinematicsKit* mainBodyItemPart() {
        return static_cast<BodyItemKinematicsKit*>(mainBodyPart());
    }
    const BodyItemKinematicsKit* mainBodyItemPart() const {
        return static_cast<const BodyItemKinematicsKit*>(mainBodyPart());
    }
    BodyItem* mainBodyItem() {
        return bodyItem(mainBodyPartIndex());
    }
    const BodyItem* mainBodyItem() const {
        return bodyItem(mainBodyPartIndex());
    }

protected:
    KinematicBodyItemSet(const KinematicBodyItemSet& org, CloneMap* cloneMap);
    virtual Referenced* doClone(CloneMap* cloneMap) const override;
    virtual bool onBodyItemModelUpdated(int index, BodyItem* bodyItem, int flags);

private:
    struct BodyItemEntry {
        ScopedConnectionSet connections;
        std::vector<LinkPtr> jointSignature;
    };

    static std::vector<LinkPtr> captureJointSignature(BodyItemKinematicsKit* kit);
    void onBodyItemModelUpdate(int index, int flags);

    std::map<int, BodyItemEntry> bodyItemEntryMap;
};

typedef ref_ptr<KinematicBodyItemSet> KinematicBodyItemSetPtr;

}

#endif
