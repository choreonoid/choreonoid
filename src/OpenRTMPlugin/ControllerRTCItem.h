/**
   @author Shin'ichiro Nakaoka
*/

#ifndef CNOID_OPENRTM_PLUGIN_CONTROLLER_RTC_ITEM_H
#define CNOID_OPENRTM_PLUGIN_CONTROLLER_RTC_ITEM_H

#include <cnoid/ControllerItem>
#include "exportdecl.h"

namespace cnoid {

class ControllerRTCItemImpl;

class CNOID_EXPORT ControllerRTCItem : public ControllerItem
{
public:
    static void initialize(ExtensionManager* ext);
        
    ControllerRTCItem();
    ControllerRTCItem(const ControllerRTCItem& org);
    virtual ~ControllerRTCItem();
        
    virtual bool start() override;
    virtual double timeStep() const override;
    virtual void input() override;
    virtual bool control() override;
    virtual void output() override;
    virtual void stop() override;

protected:
    bool createRTC();
    void deleteRTC(bool waitToBeDeleted);
    
    virtual std::string defaultRTCInstanceName() const;
    virtual void onConnectedToRoot() override;
    virtual void onDisconnectedFromRoot() override;
    virtual Item* doDuplicate() const override;
    virtual void doPutProperties(PutPropertyFunction& putProperty) override;
    virtual bool store(Archive& archive) override;
    virtual bool restore(const Archive& archive) override;
        
private:
    ControllerRTCItemImpl* impl;
    friend class ControllerRTCItemImpl;
};
        
typedef ref_ptr<ControllerRTCItem> ControllerRTCItemPtr;

}

#endif
