/**
   @author Shizuko Hattori
   @author Shin'ichiro Nakaoka
*/

#ifndef CNOID_OPENRTM_PLUGIN_BODY_RTC_ITEM_H
#define CNOID_OPENRTM_PLUGIN_BODY_RTC_ITEM_H

#include <cnoid/ControllerItem>
#include <cnoid/BasicSensors>
#include "../exportdecl.h"

namespace cnoid {

class Body;
class BodyRTCItemImpl;

class CNOID_EXPORT BodyRTCItem : public ControllerItem
{
public:
    static void initialize(ExtensionManager* ext);
        
    BodyRTCItem();
    BodyRTCItem(const BodyRTCItem& org);
    virtual ~BodyRTCItem();

    virtual bool initialize(ControllerIO* io) override;
    virtual bool start() override;
    virtual double timeStep() const override;
    virtual void input() override;
    virtual bool control() override;
    virtual void output() override;
    virtual void stop() override;

    const Body* body() const;
    const DeviceList<ForceSensor>& forceSensors() const;
    const DeviceList<RateGyroSensor>& rateGyroSensors() const;
    const DeviceList<AccelerationSensor>& accelerationSensors() const;

    double controlTime() const;
       
    enum ConfigMode {
        CONF_FILE_MODE = 0,
        CONF_ALL_MODE,
        N_CONFIG_MODES
    };

    enum BaseDirectoryType {
        NO_BASE_DIRECTORY,
        RTC_DIRECTORY,
        PROJECT_DIRECTORY,
        N_BASE_DIRECTORY_TYPES
    };

    void setControllerModule(const std::string& name);
    void setConfigFile(const std::string& filename);
    void setConfigMode(int mode);
    void setPeriodicRate(double freq);
    void setAutoConnectionMode(bool on); 
    void setBaseDirectoryType(int type);

#ifdef ENABLE_SIMULATION_PROFILING
    virtual void getProfilingNames(std::vector<std::string>& profilingNames);
    virtual void getProfilingTimes(std::vector<double>& profilingTimes);
#endif

protected:
    virtual void onPositionChanged() override;
    virtual void onDisconnectedFromRoot() override;
    virtual Item* doDuplicate() const override;
    virtual void doPutProperties(PutPropertyFunction& putProperty) override;
    virtual bool store(Archive& archive) override;
    virtual bool restore(const Archive& archive) override;
        
private:
    BodyRTCItemImpl* impl;
};
        
typedef ref_ptr<BodyRTCItem> BodyRTCItemPtr;

}

#endif
