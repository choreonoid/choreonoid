#ifndef CNOID_OPENRTM_PLUGIN_RTC_ITEM_H
#define CNOID_OPENRTM_PLUGIN_RTC_ITEM_H

#include <cnoid/Item>
#include <cnoid/Selection>
#include <cnoid/stdx/filesystem>
#include <map>
#include "exportdecl.h"

namespace RTC {

class RTObject_impl;
typedef RTObject_impl RtcBase;

}

namespace cnoid {

class MessageView;
typedef std::map<std::string, std::string> PropertyMap;

class RTComponentImpl;

class RTComponent
{
public:
    RTComponent(const stdx::filesystem::path& modulePath, PropertyMap& properties);
    ~RTComponent();
    void deleteRTC();
    RTC::RtcBase* rtc();
    bool isValid() const;
    const std::string& name() const;
    void activate();

private:
    RTComponentImpl* impl;
};

class CNOID_EXPORT RTCItem : public Item
{
public:
    static void initialize(ExtensionManager* ext);

    RTCItem();
    RTCItem(const RTCItem& org);
    virtual ~RTCItem();

    enum PERIODIC_TYPE
    {
        PERIODIC_EXECUTION_CONTEXT = 0,
        SYNCH_EXT_TRIGGER,
        EXT_TRIG_EXECUTION_CONTEXT,
        SIMULATION_EXECUTION_CONTEXT,
        N_PERIODIC_TYPE
    };

    enum BaseDirectoryType
    {
        NO_BASE_DIRECTORY,
        RTC_DIRECTORY,
        PROJECT_DIRECTORY,
        N_BASE_DIRECTORY_TYPES
    };

    void setModuleName(const std::string& name);
    void setPeriodicType(int type);
    void setPeriodicRate(int rate);
    void setBaseDirectoryType(int base);
    void setActivationEnabled(bool on);
    bool isActivationEnabled() const { return isActivationEnabled_; }

protected:
    virtual void onConnectedToRoot() override;
    virtual void onDisconnectedFromRoot() override;
    virtual Item* doDuplicate() const override;
    virtual void doPutProperties(PutPropertyFunction& putProperty) override;
    virtual bool store(Archive& archive) override;
    virtual bool restore(const Archive& archive) override;

private:
    std::ostream& os;
    MessageView* mv;
    std::string moduleName;
    RTComponent* rtcomp;
    Selection periodicType;
    int oldPeriodicType;
    int periodicRate;
    PropertyMap properties;
    Selection baseDirectoryType;
    int oldBaseDirectoryType;
    stdx::filesystem::path rtcDirectory;
    stdx::filesystem::path modulePath;
    bool isActivationEnabled_;

    void deleteRTCInstance();
    void updateRTCInstance(bool forceUpdate = true);
    bool convertAbsolutePath();
};

typedef ref_ptr<RTCItem> RTCItemPtr;

}

#endif
