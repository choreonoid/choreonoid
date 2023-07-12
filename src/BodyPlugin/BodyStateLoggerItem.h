#ifndef CNOID_BODY_PLUGIN_BODY_STATE_LOGGER_ITEM_H
#define CNOID_BODY_PLUGIN_BODY_STATE_LOGGER_ITEM_H

#include <cnoid/ControllerItem>
#include "exportdecl.h"

namespace cnoid {

class CNOID_EXPORT BodyStateLoggerItem : public ControllerItem
{
public:
    static void initializeClass(ExtensionManager* ext);
    
    BodyStateLoggerItem();

    virtual bool initialize(ControllerIO* io) override;
    virtual bool start() override;
    virtual void input() override;
    virtual void stop() override;

protected:
    BodyStateLoggerItem(const BodyStateLoggerItem& org);
    virtual Item* doCloneItem(CloneMap* cloneMap) const override;
    virtual void doPutProperties(PutPropertyFunction& putProperty) override;
    virtual bool store(Archive& archive) override;
    virtual bool restore(const Archive& archive) override;
    
private:
    class Impl;
    Impl* impl;
};

typedef ref_ptr<BodyStateLoggerItem> BodyStateLoggerItemPtr;

}

#endif
