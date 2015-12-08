/**
   @author 
*/

#ifndef CNOID_OPENRTM_PLUGIN_RTC_ITEM_H_INCLUDED
#define CNOID_OPENRTM_PLUGIN_RTC_ITEM_H_INCLUDED

#include <cnoid/Item>
#include <cnoid/Process>
#include <rtm/Manager.h>
#include "exportdecl.h"

using namespace std;
using namespace RTC;

namespace cnoid {

class MessageView;
typedef map<string, string> PropertyMap;

class RTComponent
{
public:
    RTComponent(string moduleName);
    RTComponent(string moduleName, PropertyMap& properties);
    ~RTComponent();
    void deleteRTC(bool waitToBeDeleted);
    RtcBase* rtc() { return rtc_; };
    bool isValid() const;
    const std::string& name() const { return componentName; }
        
private:
    RTObject_var rtcRef;
    RtcBase* rtc_;
    string moduleName;
    Process rtcProcess;
    string componentName;
    MessageView* mv;

    void init(string moduleName, PropertyMap& properties);
    bool createRTC(PropertyMap& properties);
    void setupModules(string& fileName, string& initFuncName, string& componentName, PropertyMap& properties );
    void createProcess(string& command, PropertyMap& properties);
    void onReadyReadServerProcessOutput();
};

class CNOID_EXPORT RTCItem : public Item
{
public:
    static void initialize(ExtensionManager* ext);
        
    RTCItem();
    RTCItem(const RTCItem& org);
    virtual ~RTCItem();
    
    enum PERIODIC_TYPE {
        PERIODIC_EXECUTION_CONTEXT = 0,
        SYNCH_EXT_TRIGGER,
        EXT_TRIG_EXECUTION_CONTEXT,
        CHOREONOID_EXECUTION_CONTEXT,
        N_PERIODIC_TYPE
    };
           
    void setModuleName(const std::string& name);
    void setPeriodicType(int type);
    void setPeriodicRate(int rate);

protected:
    virtual void onPositionChanged();
    virtual void onDisconnectedFromRoot();
    virtual Item* doDuplicate() const;
    virtual void doPutProperties(PutPropertyFunction& putProperty);
    virtual bool store(Archive& archive);
    virtual bool restore(const Archive& archive);
        
private:
    ostream& os;
    MessageView* mv;
    string moduleName;
    RTComponent* rtcomp;
    Selection periodicType;
    int periodicRate;
    int oldType;
    PropertyMap properties;
};
        
typedef ref_ptr<RTCItem> RTCItemPtr;
}

#endif
