#ifndef CNOID_BODY_PLUGIN_BODY_ITEM_KINEMATIC_KIT_H
#define CNOID_BODY_PLUGIN_BODY_ITEM_KINEMATIC_KIT_H

#include "BodyItem.h"
#include <cnoid/BodyKinematicsKit>
#include "exportdecl.h"

namespace cnoid {

class BodyItem;

class CNOID_EXPORT BodyItemKinematicsKit : public BodyKinematicsKit
{
public:
    BodyItemKinematicsKit(BodyItem* bodyItem);
    
    BodyItem* bodyItem() { return bodyItem_.lock(); }
    const BodyItem* bodyItem() const { return bodyItem_.lock(); }

protected:
    BodyItemKinematicsKit(const BodyItemKinematicsKit& org, CloneMap* cloneMap);
    virtual Referenced* doClone(CloneMap* cloneMap) const override;

private:
    weak_ref_ptr<BodyItem> bodyItem_;
};

typedef ref_ptr<BodyItemKinematicsKit> BodyItemKinematicsKitPtr;

}

#endif

    
    



