/**
   @author Shin'ichiro Nakaoka
*/

#ifndef CNOID_OPENRTM_PLUGIN_BODY_IO_RTC_ITEM_H
#define CNOID_OPENRTM_PLUGIN_BODY_IO_RTC_ITEM_H

#include <cnoid/ControllerItem>
#include "exportdecl.h"

namespace cnoid {

class BodyIoRTCItemImpl;

class CNOID_EXPORT BodyIoRTCItem : public ControllerItem
{
public:
    static void initialize(ExtensionManager* ext);
        
    BodyIoRTCItem();
    BodyIoRTCItem(const BodyIoRTCItem& org);
    virtual ~BodyIoRTCItem();
        
    virtual bool initialize(ControllerItemIO* io) override;
    virtual double timeStep() const override;
    virtual void input() override;
    virtual bool control() override;
    virtual void output() override;
    virtual void stop() override;

protected:
    virtual void onConnectedToRoot() override;
    virtual void onDisconnectedFromRoot() override;
    virtual void onPositionChanged() override;
    virtual Item* doDuplicate() const override;
    virtual void doPutProperties(PutPropertyFunction& putProperty) override;
    virtual bool store(Archive& archive) override;
    virtual bool restore(const Archive& archive) override;
        
private:
    BodyIoRTCItemImpl* impl;
};
        
typedef ref_ptr<BodyIoRTCItem> BodyIoRTCItemPtr;

}

#endif
