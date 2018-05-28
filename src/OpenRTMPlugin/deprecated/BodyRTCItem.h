/**
   @author Shizuko Hattori
   @author Shin'ichiro Nakaoka
*/

#ifndef CNOID_OPENRTM_PLUGIN_BODY_RTC_ITEM_H
#define CNOID_OPENRTM_PLUGIN_BODY_RTC_ITEM_H

#include "VirtualRobotRTC.h"
#include "../RTCItem.h"
#include <cnoid/ControllerItem>
#include <cnoid/BasicSensorSimulationHelper>
#include <cnoid/Body>
#include <boost/filesystem.hpp>
#ifdef ENABLE_SIMULATION_PROFILING
#include <cnoid/TimeMeasure>
#endif
#include "../exportdecl.h"

namespace cnoid {

class MessageView;

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

    const BodyPtr& body() const { return simulationBody; };
    const DeviceList<ForceSensor>& forceSensors() const { return forceSensors_; }
    const DeviceList<RateGyroSensor>& rateGyroSensors() const { return gyroSensors_; }
    const DeviceList<AccelerationSensor>& accelerationSensors() const { return accelSensors_; }

    double controlTime() const { return controlTime_; }
       
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
    BodyPtr simulationBody;
    DeviceList<ForceSensor> forceSensors_;
    DeviceList<RateGyroSensor> gyroSensors_;
    DeviceList<AccelerationSensor> accelSensors_;
    double timeStep_;

    // The world time step is used if the following values are 0
    double executionCycleProperty;
    double executionCycle;
    double executionCycleCounter;
        
    const ControllerIO* io;
    double controlTime_;
    std::ostream& os;

    std::string bodyName;
    RTC::CorbaNaming* naming;
    BridgeConf* bridgeConf;
    VirtualRobotRTC* virtualRobotRTC;
    RTC::ExecutionContextService_var virtualRobotEC;
    OpenRTM::ExtTrigExecutionContextService_var virtualRobotExtEC;

    Selection configMode;
    bool autoConnect;
    RTComponent* rtcomp;

    typedef std::map<std::string, RTC::PortService_var> PortMap;

    struct RtcInfo
    {
        RTC::RTObject_var rtcRef;
        PortMap portMap;
        RTC::ExecutionContextService_var execContext;
        OpenRTM::ExtTrigExecutionContextService_var execContextExt;
        double timeRate;
        double timeRateCounter;
    };
    typedef std::shared_ptr<RtcInfo> RtcInfoPtr;

    typedef std::map<std::string, RtcInfoPtr> RtcInfoMap;
    RtcInfoMap rtcInfoMap;
    typedef std::vector<RtcInfoPtr> RtcInfoVector;
    RtcInfoVector rtcInfoVector;

    std::string moduleName;
    std::string moduleFileName;
    std::string confFileName;
    std::string instanceName;
    Selection baseDirectoryType;
    boost::filesystem::path rtcDirectory;
    MessageView* mv;

    void createRTC(BodyPtr body);
    void setdefaultPort(BodyPtr body);
    void activateComponents();
    void deactivateComponents();
    void detectRtcs();
    void setupRtcConnections();
    RtcInfoPtr addRtcVectorWithConnection(RTC::RTObject_var new_rtcRef);
    void makePortMap(RtcInfoPtr& rtcInfo);
    int connectPorts(RTC::PortService_var outPort, RTC::PortService_var inPort);

    void setInstanceName(const std::string& name);
    void deleteModule(bool waitToBeDeleted);

#ifdef ENABLE_SIMULATION_PROFILING
    double bodyRTCTime;
    double controllerTime;
    TimeMeasure timer;
#endif

};
        
typedef ref_ptr<BodyRTCItem> BodyRTCItemPtr;

}

#endif
