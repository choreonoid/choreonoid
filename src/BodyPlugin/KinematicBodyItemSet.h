#ifndef CNOID_BODY_PLUGIN_KINEMATIC_BODY_ITEM_SET_H
#define CNOID_BODY_PLUGIN_KINEMATIC_BODY_ITEM_SET_H

#include <cnoid/KinematicBodySet>
#include "exportdecl.h"

namespace cnoid {

class BodyItem;

class CNOID_EXPORT KinematicBodyItemSet : public KinematicBodySet
{
public:
    KinematicBodyItemSet();
    void setBodyItem(const GeneralId& partId, BodyItem* bodyItem);
    BodyItem* bodyItem(const GeneralId& partId);

protected:
    KinematicBodyItemSet(const KinematicBodyItemSet& org, CloneMap* cloneMap);
    virtual Referenced* doClone(CloneMap* cloneMap) const override;

private:
    void copyBodyPart(BodyPart* newBodyPart, BodyPart* orgBodyPart, CloneMap* cloneMap);    
    void initializeBodyPart(BodyPart* bodyPart, BodyItem* bodyItem);
};

typedef ref_ptr<KinematicBodyItemSet> KinematicBodyItemSetPtr;

}

#endif
