#ifndef CNOID_BODY_PLUGIN_KINEMATIC_BODY_ITEM_SET_H
#define CNOID_BODY_PLUGIN_KINEMATIC_BODY_ITEM_SET_H

#include "BodyItem.h"
#include <cnoid/KinematicBodySet>
#include "exportdecl.h"

namespace cnoid {

class BodyItem;

class CNOID_EXPORT KinematicBodyItemSet : public KinematicBodySet
{
public:
    KinematicBodyItemSet();
    void setBodyItemPart(int index, BodyItem* bodyItem, LinkKinematicsKit* linkKinematicsKit);
    void setBodyItemPart(int index, BodyItem* bodyItem, std::shared_ptr<JointTraverse> jointTraverse);
    void clearBodyItemPart(int index) { clearBodyPart(index); }

    class BodyItemPart : public KinematicBodySet::BodyPart
    {
    public:
        BodyItem* bodyItem() { return bodyItem_.lock(); }
        const BodyItem* bodyItem() const { return bodyItem_.lock(); }

    private:
        weak_ref_ptr<BodyItem> bodyItem_;
        ScopedConnectionSet bodyItemConnections;
        
        friend class KinematicBodyItemSet;
    };

    BodyItemPart* bodyItemPart(int index){
        return static_cast<BodyItemPart*>(bodyPart(index));
    }
    const BodyItemPart* bodyItemPart(int index) const {
        return static_cast<const BodyItemPart*>(bodyPart(index));
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
    BodyItemPart* mainBodyItemPart() {
        return static_cast<BodyItemPart*>(mainBodyPart());
    }
    const BodyItemPart* mainBodyItemPart() const {
        return static_cast<const BodyItemPart*>(mainBodyPart());
    }

protected:
    KinematicBodyItemSet(const KinematicBodyItemSet& org, CloneMap* cloneMap);
    virtual Referenced* doClone(CloneMap* cloneMap) const override;

private:
    void copyBodyPart(BodyPart* newBodyPart, BodyPart* orgBodyPart, CloneMap* cloneMap);    
};

typedef ref_ptr<KinematicBodyItemSet> KinematicBodyItemSetPtr;

}

#endif
