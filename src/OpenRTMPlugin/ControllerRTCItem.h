/**
   @author Shin'ichiro Nakaoka
*/

#ifndef CNOID_OPENRTM_PLUGIN_CONTROLLER_RTC_ITEM_H
#define CNOID_OPENRTM_PLUGIN_CONTROLLER_RTC_ITEM_H

#include <cnoid/ControllerItem>
#include <rtm/RTObject.h>
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

    void setRTCModule(const std::string& name);
    void setRTCInstanceName(const std::string& name);
    void setExecContextType(int which);
    void setPeriodicRate(int rate);
    
    RTC::RtcBase* rtc();
    std::string rtcModuleName() const;
    std::string rtcInstanceName() const;

    using ControllerItem::initialize;
    virtual bool start() override;
    virtual double timeStep() const override;
    virtual void input() override;
    virtual bool control() override;
    virtual void output() override;
    virtual void stop() override;

protected:
    void useOnlySimulationExecutionContext();
    bool createRTCmain(bool isBodyIORTC = false);

    virtual Item* doDuplicate() const override;
    virtual void onConnectedToRoot() override;
    virtual void onDisconnectedFromRoot() override;
    virtual std::string getDefaultRTCInstanceName() const;
    virtual bool createRTC();
    virtual void deleteRTC(bool waitToBeDeleted);
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
