#ifndef CNOID_BODY_PLUGIN_CONTROLLER_LOG_ITEM_H
#define CNOID_BODY_PLUGIN_CONTROLLER_LOG_ITEM_H

#include <cnoid/ReferencedObjectSeqItem>
#include "exportdecl.h"

namespace cnoid {

class CNOID_EXPORT ControllerLogItem : public ReferencedObjectSeqItem
{
public:
    static void initializeClass(ExtensionManager* ext);
            
    ControllerLogItem();
    ControllerLogItem(const ControllerLogItem& org);

    std::shared_ptr<ReferencedObjectSeq> log() { return seq(); }
    void resetLog() { resetSeq(); }

protected:
    virtual Item* doDuplicate() const override;
};

typedef ref_ptr<ControllerLogItem> ControllerLogItemPtr;

}

#endif

