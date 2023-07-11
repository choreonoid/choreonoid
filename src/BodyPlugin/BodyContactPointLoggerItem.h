#ifndef CNOID_BODY_PLUGIN_BODY_CONTACT_POINT_LOGGER_ITEM_H
#define CNOID_BODY_PLUGIN_BODY_CONTACT_POINT_LOGGER_ITEM_H

#include "BodyContactPointLogItem.h"
#include <cnoid/ControllerItem>
#include <cnoid/RenderableItem>
#include "exportdecl.h"

namespace cnoid {

class CNOID_EXPORT BodyContactPointLoggerItem : public ControllerItem, public RenderableItem
{
public:
    static void initializeClass(ExtensionManager* ext);
    
    BodyContactPointLoggerItem();

    virtual bool initialize(ControllerIO* io) override;
    virtual ReferencedObjectSeqItem* createLogItem() override;
    virtual void output() override;

    void setLogFrameToVisualize(BodyContactPointLogItem::LogFrame* logFrame);

    virtual SgNode* getScene() override;

    class Impl;

protected:
    BodyContactPointLoggerItem(const BodyContactPointLoggerItem& org);
    virtual Item* doCloneItem(CloneMap* cloneMap) const override;

private:
    Impl* impl;
};

typedef ref_ptr<BodyContactPointLoggerItem> BodyContactPointLoggerItemPtr;

}

#endif
