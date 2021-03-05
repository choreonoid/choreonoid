#ifndef CNOID_BODY_PLUGIN_BODY_CONTACT_POINT_LOGGER_ITEM_H
#define CNOID_BODY_PLUGIN_BODY_CONTACT_POINT_LOGGER_ITEM_H

#include <cnoid/ControllerItem>
#include <cnoid/RenderableItem>

namespace cnoid {

class BodyContactPointLoggerItem : public ControllerItem, public RenderableItem
{
public:
    static void initializeClass(ExtensionManager* ext);
    
    BodyContactPointLoggerItem();
    BodyContactPointLoggerItem(const BodyContactPointLoggerItem& org);

    virtual bool initialize(ControllerIO* io) override;
    virtual ControllerLogItem* createLogItem() override;
    virtual void log() override;

    virtual SgNode* getScene() override;

    class Impl;

protected:
    virtual Item* doDuplicate() const override;

private:
    Impl* impl;
};

typedef ref_ptr<BodyContactPointLoggerItem> BodyContactPointLoggerItemPtr;

}

#endif
