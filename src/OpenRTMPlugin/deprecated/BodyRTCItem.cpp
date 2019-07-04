/**
   @author Shizuko Hattori
   @author Shin'ichiro Nakaoka
*/

#include "BodyRTCItem.h"
#include "VirtualRobotRTC.h"
#include "../RTCItem.h"
#include "../OpenRTMUtil.h"
#include <cnoid/Body>
#include <cnoid/BodyItem>
#include <cnoid/Link>
#include <cnoid/BasicSensorSimulationHelper>
#include <cnoid/ControllerIO>
#include <cnoid/ItemManager>
#include <cnoid/Archive>
#include <cnoid/FileUtil>
#include <cnoid/ExecutablePath>
#include <cnoid/MessageView>
#include <cnoid/Sleep>
#include <cnoid/ProjectManager>
#ifdef ENABLE_SIMULATION_PROFILING
#include <cnoid/TimeMeasure>
#endif
#include <rtm/CorbaNaming.h>
#include <fmt/format.h>
#include "../LoggerUtil.h"
#include "../gettext.h"

using namespace std;
using namespace cnoid;
using namespace RTC;
using fmt::format;
namespace filesystem = cnoid::stdx::filesystem;

namespace {

const bool TRACE_FUNCTIONS = false;

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

}

namespace cnoid {

class BodyRTCItemImpl
{
public:
    BodyRTCItem* self;

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

    typedef std::map<std::string, RtcInfoPtr> RtcInfoMap;
    RtcInfoMap rtcInfoMap;
    typedef std::vector<RtcInfoPtr> RtcInfoVector;
    RtcInfoVector rtcInfoVector;

    std::string moduleName;
    std::string moduleFileName;
    std::string confFileName;
    std::string instanceName;
    Selection baseDirectoryType;
    filesystem::path rtcDirectory;
    MessageView* mv;

#ifdef ENABLE_SIMULATION_PROFILING
    double bodyRTCTime;
    double controllerTime;
    TimeMeasure timer;
#endif

    BodyRTCItemImpl(BodyRTCItem* self);
    BodyRTCItemImpl(BodyRTCItem* self, const BodyRTCItemImpl& org);
    void createRTC(BodyPtr body);
    void setdefaultPort(BodyPtr body);
    void onPositionChanged();
    bool initialize(ControllerIO* io);
    bool start();
    bool control();
    void setConfigFile(const std::string& name);
    void setConfigMode(int mode);
    void setControllerModule(const std::string& name);
    void setInstanceName(const std::string& name);
    void setBaseDirectoryType(int type);
    void doPutProperties(PutPropertyFunction& putProperty);
    bool store(Archive& archive);
    bool restore(const Archive& archive);
    void makePortMap(RtcInfoPtr& rtcInfo);
    void activateComponents();
    void deactivateComponents();
    void detectRtcs();
    void setupRtcConnections();
    RtcInfoPtr addRtcVectorWithConnection(RTC::RTObject_var new_rtcRef);
    int connectPorts(RTC::PortService_var outPort, RTC::PortService_var inPort);
    void deleteModule(bool waitToBeDeleted);
};

}
    

void BodyRTCItem::initialize(ExtensionManager* ext)
{
    static bool initialized = false;
    if(!initialized){
        ext->itemManager().registerClass<BodyRTCItem>(N_("BodyRTCItem"));
        ext->itemManager().addCreationPanel<BodyRTCItem>();
        initialized = true;
    }
}


BodyRTCItem::BodyRTCItem()
{
    impl = new BodyRTCItemImpl(this);
}


BodyRTCItemImpl::BodyRTCItemImpl(BodyRTCItem* self)
    : self(self),
      os(MessageView::instance()->cout()),
      configMode(BodyRTCItem::N_CONFIG_MODES, CNOID_GETTEXT_DOMAIN_NAME),
      baseDirectoryType(BodyRTCItem::N_BASE_DIRECTORY_TYPES, CNOID_GETTEXT_DOMAIN_NAME)
{
    self->setName("BodyRTC");
    
    io = 0;
    virtualRobotRTC = 0;
    rtcomp = 0;
    bridgeConf = 0;
    moduleName.clear();
    bodyName.clear();
    instanceName.clear();
    mv = MessageView::instance();

    configMode.setSymbol(BodyRTCItem::CONF_FILE_MODE,  N_("Use Configuration File"));
    configMode.setSymbol(BodyRTCItem::CONF_ALL_MODE,  N_("Create Default Port"));
    configMode.select(BodyRTCItem::CONF_ALL_MODE);
    autoConnect = false;

    baseDirectoryType.setSymbol(BodyRTCItem::RTC_DIRECTORY, N_("RTC directory"));
    baseDirectoryType.setSymbol(BodyRTCItem::PROJECT_DIRECTORY, N_("Project directory"));
    baseDirectoryType.select(BodyRTCItem::RTC_DIRECTORY);
    rtcDirectory = filesystem::path(executableTopDirectory()) / CNOID_PLUGIN_SUBDIR / "rtc";

    executionCycleProperty = 0.0;
}


BodyRTCItem::BodyRTCItem(const BodyRTCItem& org)
    : ControllerItem(org)
{
    impl = new BodyRTCItemImpl(this, *org.impl);
}

      
BodyRTCItemImpl::BodyRTCItemImpl(BodyRTCItem* self, const BodyRTCItemImpl& org)
    : os(MessageView::instance()->cout()),
      configMode(org.configMode),
      baseDirectoryType(org.baseDirectoryType)
{
    io = 0;
    virtualRobotRTC = org.virtualRobotRTC;
    rtcomp = org.rtcomp;
    bridgeConf = org.bridgeConf;
    moduleName = org.moduleName;
    instanceName = org.instanceName;
    bodyName = org.bodyName;
    autoConnect = org.autoConnect;
    mv = MessageView::instance();
    executionCycleProperty = org.executionCycleProperty;
    rtcDirectory = org.rtcDirectory;
}


BodyRTCItem::~BodyRTCItem()
{
    delete impl;
}


void BodyRTCItemImpl::createRTC(BodyPtr body)
{
    DDEBUG("BodyRTCItem::createRTC");

    bridgeConf = new BridgeConf();

    filesystem::path projectDir(ProjectManager::instance()->currentProjectDirectory());

    if(configMode.is(BodyRTCItem::CONF_ALL_MODE)){
        setdefaultPort(body);
    } else if(configMode.is(BodyRTCItem::CONF_FILE_MODE)){
        filesystem::path confPath;
        if(!confFileName.empty()){
            confPath = confFileName;
        } else if(!moduleName.empty()){
            confPath = moduleName+".conf";
        }

        if(!confPath.empty()){
            if(!confPath.is_absolute()){
                if(baseDirectoryType.is(BodyRTCItem::RTC_DIRECTORY)){
                    confPath = rtcDirectory / confPath;
                } else if(baseDirectoryType.is(BodyRTCItem::PROJECT_DIRECTORY)){
                    if(projectDir.empty()){
                        mv->putln(_("Please save the project."));
                        return;
                    } else {
                        confPath = projectDir / confPath;
                   }
                }
            }
            std::string confFileName0 = getNativePathString(confPath);
            try {
                if(bridgeConf->loadConfigFile(confFileName0.c_str())){
                    mv->putln(format(_("Config File \"{}\" has been loaded."), confFileName0));
                } else {
                    mv->putln(format(_("Cannot find or open \"{}\"."), confFileName0));
                }
            }
            catch (...) {
                mv->putln(format(_("Cannot find or open \"{}\"."), confFileName0), MessageView::ERROR);
            }
        }
    }

    ModuleInfoList& moduleInfoList = bridgeConf->moduleInfoList;
    ModuleInfoList::iterator it;
    for(it=moduleInfoList.begin(); it != moduleInfoList.end(); ++it){
        mv->putln(format(_("Loading Module....  \"{}\""), it->fileName));
    }
    bridgeConf->setupModules();

    if(!moduleName.empty()){
        ModuleInfoList& moduleInfoList = bridgeConf->moduleInfoList;
        ModuleInfoList::iterator it;
        for(it = moduleInfoList.begin(); it != moduleInfoList.end(); ++it){
            if(it->componentName == moduleName){
                break;
            }
        }
        if(it == moduleInfoList.end()){
            PropertyMap prop;
            prop["exec_cxt.periodic.type"] = "ChoreonoidExecutionContext";
            prop["exec_cxt.periodic.rate"] = "1000000";

            filesystem::path modulePath(moduleName);
            if(!modulePath.is_absolute()){
                if(baseDirectoryType.is(BodyRTCItem::RTC_DIRECTORY)){
                    modulePath = rtcDirectory / modulePath;
                } else if(baseDirectoryType.is(BodyRTCItem::PROJECT_DIRECTORY)){
                    if(projectDir.empty()){
                        mv->putln(_("Please save the project."));
                        return;
                    } else {
                        modulePath = projectDir / modulePath;
                    }
                }
            }
            rtcomp = new RTComponent(modulePath, prop);
        }
    }

    RTC::Manager& rtcManager = RTC::Manager::instance();
    string bodyName(body->name());
    if(instanceName.empty()){
        instanceName = bodyName;
    }
    int i=2;
    while(rtcManager.getComponent(instanceName.c_str()) != NULL){
        stringstream ss;
        ss << bodyName << "(" << i << ")";
        instanceName = ss.str();
        i++;
    }
#if defined(OPENRTM_VERSION11)
    string param("VirtualRobot?instance_name={}&exec_cxt.periodic.type=ChoreonoidExecutionContext&exec_cxt.periodic.rate=1000000");
#elif defined(OPENRTM_VERSION12)
    string param("VirtualRobot?instance_name={}&execution_contexts=ChoreonoidExecutionContext()&exec_cxt.periodic.type=ChoreonoidExecutionContext&exec_cxt.periodic.rate=1000000&exec_cxt.sync_activation=NO&exec_cxt.sync_deactivation=NO");
    DDEBUG("New Parameter 01");
#endif
    RtcBase* rtc = createManagedRTC(format(param, instanceName));
    mv->putln(format(_("RTC \"{}\" has been created."), instanceName));
    virtualRobotRTC = dynamic_cast<VirtualRobotRTC*>(rtc);
    virtualRobotRTC->createPorts(bridgeConf);

    virtualRobotExtEC = OpenRTM::ExtTrigExecutionContextService::_nil();
    virtualRobotEC = RTC::ExecutionContextService::_nil();
    RTC::ExecutionContextList_var eclist = virtualRobotRTC->get_owned_contexts();
    for(CORBA::ULong i=0; i < eclist->length(); ++i){
        if(!CORBA::is_nil(eclist[i])){
            virtualRobotEC = RTC::ExecutionContextService::_narrow(eclist[i]);
            break;
        }
    }
    
}

void BodyRTCItemImpl::setdefaultPort(BodyPtr body)
{
    PortInfoMap& outPortInfoMap = bridgeConf->outPortInfos;
    PortInfo portInfo;
    portInfo.dataOwnerNames.clear();
    portInfo.dataTypeId = JOINT_VALUE;
    portInfo.portName = "q";
    portInfo.stepTime = 0;
    outPortInfoMap.insert(make_pair(portInfo.portName, portInfo));
    portInfo.dataTypeId = JOINT_TORQUE;
    portInfo.portName = "u_out";
    portInfo.stepTime = 0;
    outPortInfoMap.insert(make_pair(portInfo.portName, portInfo));

    for(size_t i=0; i < forceSensors_.size(); ++i){
        if(Device* sensor = forceSensors_[i]){
            portInfo.dataTypeId = FORCE_SENSOR;
            portInfo.dataOwnerNames.clear();
            portInfo.dataOwnerNames.push_back(sensor->name());
            portInfo.portName = sensor->name();
            portInfo.stepTime = 0;
            outPortInfoMap.insert(make_pair(portInfo.portName, portInfo));
        }
    }
    for(size_t i=0; i < gyroSensors_.size(); ++i){
        if(Device* sensor = gyroSensors_[i]){
            portInfo.dataTypeId = RATE_GYRO_SENSOR;
            portInfo.dataOwnerNames.clear();
            portInfo.dataOwnerNames.push_back(sensor->name());
            portInfo.portName = sensor->name();
            portInfo.stepTime = 0;
            outPortInfoMap.insert(make_pair(portInfo.portName, portInfo));
        }
    }
    for(size_t i=0; i < accelSensors_.size(); ++i){
        if(Device* sensor = accelSensors_[i]){
            portInfo.dataTypeId = ACCELERATION_SENSOR;
            portInfo.dataOwnerNames.clear();
            portInfo.dataOwnerNames.push_back(sensor->name());
            portInfo.portName = sensor->name();
            portInfo.stepTime = 0;
            outPortInfoMap.insert(make_pair(portInfo.portName, portInfo));
        }
    }

    PortInfoMap& inPortInfoMap = bridgeConf->inPortInfos;
    portInfo.dataOwnerNames.clear();
    portInfo.dataTypeId = JOINT_TORQUE;
    portInfo.portName = "u_in";
    portInfo.stepTime = 0;
    inPortInfoMap.insert(make_pair(portInfo.portName, portInfo));
}

void BodyRTCItem::onPositionChanged()
{
    impl->onPositionChanged();
}


void BodyRTCItemImpl::onPositionChanged()
{
    // create or recreate an RTC corresponding to the body
    // The target body can be detected like this:

    BodyItem* ownerBodyItem = self->findOwnerItem<BodyItem>();
    if(ownerBodyItem){
        Body* body = ownerBodyItem->body();
        if(bodyName != body->name()){
            forceSensors_ = body->devices<ForceSensor>().getSortedById();
            gyroSensors_ = body->devices<RateGyroSensor>().getSortedById();
            accelSensors_ = body->devices<AccelerationSensor>().getSortedById();
            bodyName = body->name();
            deleteModule(true);
            createRTC(body);
        }
    } else {
        deleteModule(false);
        bodyName.clear();
    }
}


void BodyRTCItem::onDisconnectedFromRoot()
{
    // This is not necessary because onPositionChanged() is also called
    // when the item is disconnected from the root
    impl->deleteModule(false);
}


Item* BodyRTCItem::doDuplicate() const
{
    return new BodyRTCItem(*this);
}


bool BodyRTCItem::initialize(ControllerIO* io)
{
    return impl->initialize(io);
}


bool BodyRTCItemImpl::initialize(ControllerIO* io)
{
    this->io = io;
    simulationBody = io->body();
    timeStep_ = io->timeStep();
    controlTime_ = io->currentTime();

    for(auto joint : simulationBody->joints()){
        if(joint->isRevoluteJoint() || joint->isPrismaticJoint()){
            joint->setActuationMode(Link::JOINT_EFFORT);
        }
    }

    forceSensors_ = simulationBody->devices<ForceSensor>().getSortedById();
    gyroSensors_ = simulationBody->devices<RateGyroSensor>().getSortedById();
    accelSensors_ = simulationBody->devices<AccelerationSensor>().getSortedById();

    executionCycle = (executionCycleProperty > 0.0) ? executionCycleProperty : timeStep_;
    executionCycleCounter = executionCycle;

    return true;
}


bool BodyRTCItem::start()
{
    return impl->start();
}


bool BodyRTCItemImpl::start()
{
    bool isReady = true;
    
    if(rtcomp && !rtcomp->isValid()){
        mv->putln(format(_("RTC \"{}\" is not ready."), rtcomp->name()));
        isReady = false;
    }

    if(virtualRobotRTC) {
        virtualRobotRTC->initialize(simulationBody);
        if(!virtualRobotRTC->checkOutPortStepTime(timeStep_)){
            mv->putln(_("Output interval must be longer than the control interval."));
            isReady = false;
        }
    }

    if(isReady){
        rtcInfoVector.clear();
        detectRtcs();
        setupRtcConnections();
        activateComponents();
    }

#ifdef ENABLE_SIMULATION_PROFILING
    bodyRTCTime = 0.0;
#endif

    return isReady;
}


double BodyRTCItem::timeStep() const
{
    return impl->timeStep_;
}


void BodyRTCItem::input()
{
    impl->controlTime_ = impl->io->currentTime();

    // write the state of simulationBody to out-ports
    impl->virtualRobotRTC->inputDataFromSimulator(this);
}


bool BodyRTCItem::control()
{
    return impl->control();
}


bool BodyRTCItemImpl::control()
{
    // tick the execution context of the connected RTCs
    virtualRobotRTC->writeDataToOutPorts(controlTime_, timeStep_);

    if(!CORBA::is_nil(virtualRobotEC)){
        executionCycleCounter += timeStep_;
        if(executionCycleCounter + timeStep_ / 2.0 > executionCycle){

#ifdef ENABLE_SIMULATION_PROFILING
            timer.begin();
#endif

            if(CORBA::is_nil(virtualRobotExtEC)){
                virtualRobotExtEC = OpenRTM::ExtTrigExecutionContextService::_narrow(virtualRobotEC);
            }
            if(!CORBA::is_nil(virtualRobotExtEC)){
                virtualRobotExtEC->tick();
            }

#ifdef ENABLE_SIMULATION_PROFILING
            bodyRTCTime = timer.measure();
#endif

            executionCycleCounter -= executionCycle;
        }
    }

#ifdef ENABLE_SIMULATION_PROFILING
    timer.begin();
#endif

    for(RtcInfoVector::iterator p = rtcInfoVector.begin(); p != rtcInfoVector.end(); ++p){
        RtcInfoPtr& rtcInfo = *p;
        if(!CORBA::is_nil(rtcInfo->execContext)){
            rtcInfo->timeRateCounter += rtcInfo->timeRate;
            if(rtcInfo->timeRateCounter + rtcInfo->timeRate/2.0 > 1.0){
                if (CORBA::is_nil(rtcInfo->execContextExt)) {
                    rtcInfo->execContextExt = OpenRTM::ExtTrigExecutionContextService::_narrow(rtcInfo->execContext);
                }
                if (!CORBA::is_nil(rtcInfo->execContextExt)) {
                    rtcInfo->execContextExt->tick();
                    rtcInfo->timeRateCounter -= 1.0;
                }
            }
        }
    }

#ifdef ENABLE_SIMULATION_PROFILING
    controllerTime = timer.measure();
#endif

    virtualRobotRTC->readDataFromInPorts();

    return true;
}


void BodyRTCItem::output()
{
    // read in-ports and write the values to simulationBody
    impl->virtualRobotRTC->outputDataToSimulator(impl->simulationBody);
}

    
void BodyRTCItem::stop()
{
    // deactivate the RTC
    impl->deactivateComponents();
}


const Body* BodyRTCItem::body() const
{
    return impl->simulationBody;
};


const DeviceList<ForceSensor>& BodyRTCItem::forceSensors() const
{
    return impl->forceSensors_;
}


const DeviceList<RateGyroSensor>& BodyRTCItem::rateGyroSensors() const
{
    return impl->gyroSensors_;
}


const DeviceList<AccelerationSensor>& BodyRTCItem::accelerationSensors() const
{
    return impl->accelSensors_;
}


double BodyRTCItem::controlTime() const
{
    return impl->controlTime_;
}


void BodyRTCItem::setControllerModule(const std::string& name)
{
    impl->setControllerModule(name);
}


void BodyRTCItemImpl::setControllerModule(const std::string& name)
{
    if(name != moduleName){

        filesystem::path modulePath(name);
        if(modulePath.is_absolute()){
            baseDirectoryType.select(BodyRTCItem::NO_BASE_DIRECTORY);
            if(modulePath.parent_path() == rtcDirectory){
                baseDirectoryType.select(BodyRTCItem::RTC_DIRECTORY);
                modulePath = modulePath.filename();
            } else {
                filesystem::path projectDir(ProjectManager::instance()->currentProjectDirectory());
                if(!projectDir.empty() && (modulePath.parent_path() == projectDir)){
                    baseDirectoryType.select(BodyRTCItem::BodyRTCItem::PROJECT_DIRECTORY);
                    modulePath = modulePath.filename();
                }
            }
        }
        moduleName = modulePath.string();

        BodyItem* ownerBodyItem = self->findOwnerItem<BodyItem>();
        if(ownerBodyItem){
            BodyPtr body = ownerBodyItem->body();
            deleteModule(true);
            createRTC(body);
        }
    }
}


void BodyRTCItem::setAutoConnectionMode(bool on)
{
    impl->autoConnect = on;
}


void BodyRTCItem::setConfigFile(const std::string& name)
{
    impl->setConfigFile(name);
}


void BodyRTCItemImpl::setConfigFile(const std::string& name)
{
    if(name != confFileName){
        confFileName = name;
        if(configMode.is(BodyRTCItem::CONF_ALL_MODE))
            return;
        BodyItem* ownerBodyItem = self->findOwnerItem<BodyItem>();
        if(ownerBodyItem){
            BodyPtr body = ownerBodyItem->body();
            deleteModule(true);
            createRTC(body);
        }
    }
}


void BodyRTCItem::setConfigMode(int mode)
{
    impl->setConfigMode(mode);
}


void BodyRTCItemImpl::setConfigMode(int mode)
{
    if(mode != configMode.which()){
        configMode.select(mode);
        BodyItem* ownerBodyItem = self->findOwnerItem<BodyItem>();
        if(ownerBodyItem){
            BodyPtr body = ownerBodyItem->body();
            deleteModule(true);
            createRTC(body);
        }
    }
}


void BodyRTCItem::setPeriodicRate(double freq)
{
    impl->executionCycleProperty = 1.0 / freq;
}


void BodyRTCItemImpl::setInstanceName(const std::string& name)
{
    if(instanceName!=name){
        instanceName = name;
        BodyItem* ownerBodyItem = self->findOwnerItem<BodyItem>();
        if(ownerBodyItem){
            BodyPtr body = ownerBodyItem->body();
            deleteModule(true);
            createRTC(body);
        }
    }
}


void BodyRTCItem::setBaseDirectoryType(int type)
{
    impl->setBaseDirectoryType(type);
}


void BodyRTCItemImpl::setBaseDirectoryType(int type)
{
    if(type != baseDirectoryType.which()){
        baseDirectoryType.select(type);
        BodyItem* ownerBodyItem = self->findOwnerItem<BodyItem>();
        if(ownerBodyItem){
            BodyPtr body = ownerBodyItem->body();
            deleteModule(true);
            createRTC(body);
        }
    }
}


void BodyRTCItem::doPutProperties(PutPropertyFunction& putProperty)
{
    ControllerItem::doPutProperties(putProperty);
    impl->doPutProperties(putProperty);
}


void BodyRTCItemImpl::doPutProperties(PutPropertyFunction& putProperty)
{
    putProperty(_("Auto Connect"), autoConnect, changeProperty(autoConnect));
    putProperty(_("RTC Instance name"), instanceName,
                [&](const string& name){ setInstanceName(name); return true; });
    putProperty.decimals(3)(_("Periodic rate"), executionCycleProperty,
                            changeProperty(executionCycleProperty));

    FilePathProperty moduleProperty(
        moduleName,
        { format(_("RT-Component module (*{})"), DLL_SUFFIX) });

    FilePathProperty confFileProperty(
        confFileName,
        { _("RTC cconfiguration file (*.conf)") });

    if(baseDirectoryType.is(BodyRTCItem::RTC_DIRECTORY)){
        moduleProperty.setBaseDirectory(rtcDirectory.string());
        confFileProperty.setBaseDirectory(moduleProperty.baseDirectory());
    } else if(baseDirectoryType.is(BodyRTCItem::PROJECT_DIRECTORY)){
        moduleProperty.setBaseDirectory(ProjectManager::instance()->currentProjectDirectory());
        confFileProperty.setBaseDirectory(moduleProperty.baseDirectory());
    }
    putProperty(_("Base directory"), baseDirectoryType,
                [&](int which){ setBaseDirectoryType(which); return true; });

    putProperty(_("Controller module"), moduleProperty,
                [&](const string& name){ setControllerModule(name); return true; });

    putProperty(_("Configuration mode"), configMode,
                [&](int which){ setConfigMode(which); return true; });

    putProperty(_("Configuration file"), confFileProperty,
                [&](const string& name){ setConfigFile(name); return true; });
}


bool BodyRTCItem::store(Archive& archive)
{
    if(!ControllerItem::store(archive)){
        return false;
    }
    return impl->store(archive);
}


bool BodyRTCItemImpl::store(Archive& archive)
{
    archive.writeRelocatablePath("moduleName", moduleName);
    archive.writeRelocatablePath("confFileName", confFileName);
    archive.write("configurationMode", configMode.selectedSymbol(), DOUBLE_QUOTED);
    archive.write("autoConnect", autoConnect);
    archive.write("instanceName", instanceName, DOUBLE_QUOTED);
    archive.write("bodyPeriodicRate", executionCycleProperty);
    archive.write("baseDirectory", baseDirectoryType.selectedSymbol(), DOUBLE_QUOTED);

    return true;
}


bool BodyRTCItem::restore(const Archive& archive)
{
    DDEBUG("BodyRTCItem::restore");

    if(!ControllerItem::restore(archive)){
        return false;
    }
    return impl->restore(archive);
}


bool BodyRTCItemImpl::restore(const Archive& archive)
{
    string value;
    if(archive.read("moduleName", value)){
        filesystem::path path(archive.expandPathVariables(value));
        moduleName = getNativePathString(path);
    }
    if(archive.read("confFileName", value)){
        filesystem::path path(archive.expandPathVariables(value));
        confFileName = getNativePathString(path);
    }
    if(archive.read("configurationMode", value)){
        configMode.select(value);
    }
    if(archive.read("baseDirectory", value) || archive.read("RelativePathBase", value)){
        baseDirectoryType.select(value);
    }

    if (!archive.read("autoConnect", autoConnect)){
        archive.read("AutoConnect", autoConnect);
    }
    if(!archive.read("instanceName", instanceName)) {
        archive.read("InstanceName", instanceName);
    }
    archive.read("bodyPeriodicRate", executionCycleProperty);

    return true;
}


// Detects the RTC specified in the config file and the RTC already connected to the robot.
void BodyRTCItemImpl::detectRtcs()
{
    RTC::Manager& rtcManager = RTC::Manager::instance();
    
    string nameServer = rtcManager.getConfig()["corba.nameservers"];
    int comPos = nameServer.find(",");
    if (comPos < 0){
        comPos = nameServer.length();
    }
    nameServer = nameServer.substr(0, comPos);
    naming = new RTC::CorbaNaming(rtcManager.getORB(), nameServer.c_str());

    for(TimeRateMap::iterator it = bridgeConf->timeRateMap.begin(); it != bridgeConf->timeRateMap.end(); ++it){
        RTC::RTObject_var rtcRef;
        string rtcName = it->first;
        if( rtcName != "") {
            string rtcNamingName = rtcName + ".rtc";
            CORBA::Object_var objRef;
            try {
                objRef = naming->resolve(rtcNamingName.c_str());
            } catch(const CosNaming::NamingContext::NotFound &ex) {

            }
            if(CORBA::is_nil(objRef)) {
                mv->putln(format(_("{} is not found."), rtcName));
            } else {
                rtcRef = RTC::RTObject::_narrow(objRef);
                if(CORBA::is_nil(rtcRef)){
                    mv->putln(format(_("{} is not an RTC object."), rtcName));
                }
            }
        }
        if (!CORBA::is_nil(rtcRef)) {
            addRtcVectorWithConnection(rtcRef);
        }
    }

    mv->putln(_("setup RT components"));
    rtcInfoMap.clear();
    for(size_t i=0; i < bridgeConf->portConnections.size(); ++i){

        PortConnection& connection = bridgeConf->portConnections[i];

        for(int i=0; i<2; i++){
            RTC::RTObject_var rtcRef = 0;
            string rtcName;

            if(!connection.InstanceName[i].empty()){
                rtcName = connection.InstanceName[i];
            } else {
                if(i==1){
                    if(rtcomp && rtcomp->rtc()){
                        RTC::RtcBase* rtcServant = rtcomp->rtc();
                        rtcName = rtcServant->getInstanceName();                                                 
                        rtcRef = rtcServant->getObjRef();
                    }
                } else {
                    continue;
                }
            }
            
            RtcInfoMap::iterator it = rtcInfoMap.find(rtcName);
            if(it == rtcInfoMap.end()){
                if(!rtcRef){
                    string rtcNamingName = rtcName + ".rtc";
                    CORBA::Object_var objRef;
                    try {
                        objRef = naming->resolve(rtcNamingName.c_str());
                    } catch(const CosNaming::NamingContext::NotFound &ex) {

                    }
                    if(CORBA::is_nil(objRef)){
                        mv->putln(format(_("{} is not found."), rtcName));
                    } else {
                        rtcRef = RTC::RTObject::_narrow(objRef);
                        if(CORBA::is_nil(rtcRef)){
                            mv->putln(format(_("{} is not an RTC object."), rtcName));
                        }
                    }
                }
                if(!CORBA::is_nil(rtcRef)){
                    RtcInfoPtr rtcInfo = addRtcVectorWithConnection(rtcRef);
                    rtcInfoMap.insert(make_pair(rtcName,rtcInfo));
                }
            }
        }
    }

    RTC::RTCList_var rtcList = virtualRobotRTC->getConnectedRtcs();
    for(CORBA::ULong i=0; i < rtcList->length(); ++i){
        addRtcVectorWithConnection(rtcList[i]);
    }
}


void BodyRTCItemImpl::makePortMap(RtcInfoPtr& rtcInfo)
{
    RTC::PortServiceList_var ports = rtcInfo->rtcRef->get_ports();
    for(CORBA::ULong i=0; i < ports->length(); ++i){
        RTC::PortProfile_var profile = ports[i]->get_port_profile();
        std::string portName(profile->name);
        string::size_type index = portName.rfind(".");
        if (index != string::npos) portName = portName.substr(index+1);
        rtcInfo->portMap[portName] = ports[i];
    }
}

/// Create a port map of new_rtcRef and register it in rtcInfoVector.
RtcInfoPtr BodyRTCItemImpl::addRtcVectorWithConnection(RTC::RTObject_var new_rtcRef)
{
    RtcInfoVector::iterator it = rtcInfoVector.begin();
    for( ; it != rtcInfoVector.end(); ++it){
        if((*it)->rtcRef->_is_equivalent(new_rtcRef))
            return *it;
    }

    RtcInfoPtr rtcInfo(new RtcInfo());
    rtcInfo->rtcRef = new_rtcRef;
    makePortMap(rtcInfo);
    string rtcName = (string)rtcInfo->rtcRef->get_component_profile()->instance_name;

    if (bridgeConf->timeRateMap.size() == 0 ) {
        rtcInfo->timeRate = 1.0;
        rtcInfo->timeRateCounter = 0.0;
    } else {
        TimeRateMap::iterator p = bridgeConf->timeRateMap.find(rtcName);
        if ( p != bridgeConf->timeRateMap.end() ) {
            rtcInfo->timeRate = (double)p->second;
            rtcInfo->timeRateCounter = 1.0 - rtcInfo->timeRate;
        } else {
            rtcInfo->timeRate = 0.0;
            rtcInfo->timeRateCounter = 0.0;
        }
        mv->putln(format(_("periodic-rate ({0}) = {1} "), rtcName, rtcInfo->timeRate));
    }
    rtcInfoVector.push_back(rtcInfo);

    RTC::ExecutionContextList_var eclist = rtcInfo->rtcRef->get_owned_contexts();
    for(CORBA::ULong i=0; i < eclist->length(); ++i){
        if(!CORBA::is_nil(eclist[i])){
            rtcInfo->execContext = RTC::ExecutionContextService::_narrow(eclist[i]);
            SDOPackage::NVList& properties = rtcInfo->rtcRef->get_component_profile()->properties;
            const char* ec_type(0);
            NVUtil::find(properties, "exec_cxt.periodic.type") >>= ec_type;
            if(!CORBA::is_nil(rtcInfo->execContext)){
                mv->putln(_("detected the ExecutionContext"));
            }
            break;
        }
    }
    return rtcInfo;
}

void BodyRTCItemImpl::activateComponents()
{
    for(RtcInfoVector::iterator p = rtcInfoVector.begin(); p != rtcInfoVector.end(); ++p){
        RtcInfoPtr& rtcInfo = *p;
        if(!CORBA::is_nil(rtcInfo->execContext)){
            if( RTC::PRECONDITION_NOT_MET == rtcInfo->execContext->activate_component(rtcInfo->rtcRef) ){
                rtcInfo->execContext->reset_component(rtcInfo->rtcRef);
                rtcInfo->execContext->activate_component(rtcInfo->rtcRef);
            }
        }
    }

    RTC::ExecutionContextList_var eclist = virtualRobotRTC->get_owned_contexts();
    for(CORBA::ULong i=0; i < eclist->length(); ++i){
        if(!CORBA::is_nil(eclist[i])){
            OpenRTM::ExtTrigExecutionContextService_var execContext = OpenRTM::ExtTrigExecutionContextService::_narrow(eclist[i]);
            if(!CORBA::is_nil(execContext)){
                if(RTC::PRECONDITION_NOT_MET == execContext->activate_component(virtualRobotRTC->getObjRef())){
                    execContext->reset_component(virtualRobotRTC->getObjRef());
                    execContext->tick();
                    execContext->activate_component(virtualRobotRTC->getObjRef());
                }
                execContext->tick();
            }
            break;
        }
    }
}


void BodyRTCItemImpl::deactivateComponents()
{
    std::vector<RTC::ExecutionContextService_var> vecExecContext;

    for(RtcInfoVector::iterator p = rtcInfoVector.begin(); p != rtcInfoVector.end(); ++p){
        RtcInfoPtr& rtcInfo = *p;
        if(!CORBA::is_nil(rtcInfo->execContext)){
            rtcInfo->execContext->deactivate_component(rtcInfo->rtcRef);
            vecExecContext.push_back(rtcInfo->execContext);
        }
    }

    RTC::ExecutionContextList_var eclist = virtualRobotRTC->get_owned_contexts();
    for(CORBA::ULong i=0; i < eclist->length(); ++i){
        if(!CORBA::is_nil(eclist[i])){
            RTC::ExecutionContextService_var execContext = RTC::ExecutionContextService::_narrow(eclist[i]);
            if(!CORBA::is_nil(execContext)){
                execContext->deactivate_component(virtualRobotRTC->getObjRef());
                vecExecContext.push_back(execContext);
            }
            break;
        }
    }

    for(std::vector<RTC::ExecutionContextService_var>::iterator iter = vecExecContext.begin();
        iter != vecExecContext.end(); ++iter){
        if(!CORBA::is_nil(*iter)){
            OpenRTM::ExtTrigExecutionContextService_var extEC = OpenRTM::ExtTrigExecutionContextService::_narrow(*iter);
            if(!CORBA::is_nil(extEC)){
                extEC->tick();
            }
        }
    }
}


void BodyRTCItemImpl::setupRtcConnections()
{
    for(size_t i=0; i < bridgeConf->portConnections.size(); ++i){

        const PortConnection& connection = bridgeConf->portConnections[i];

        string instanceName0 = connection.InstanceName[0];
        string instanceName1 = connection.InstanceName[1];
        if(instanceName1.empty()){
            if(rtcomp && rtcomp->rtc()){
                instanceName1 = rtcomp->rtc()->getInstanceName();
            }
        }

        bool instance0isRobot = false;
        RtcInfoPtr rtcInfo0;
        RtcInfoPtr rtcInfo1; 

        if(!instanceName0.empty()){
            RtcInfoMap::iterator p = rtcInfoMap.find(instanceName0);
            if(p != rtcInfoMap.end()){
                rtcInfo0 = p->second;
            }
        } else {
            instance0isRobot = true;
            instanceName0 = virtualRobotRTC->getInstanceName();
        }
        if(!instanceName0.empty()){
            RtcInfoMap::iterator p = rtcInfoMap.find(instanceName1);
            if(p != rtcInfoMap.end()){
                rtcInfo1 = p->second;
            }
        } else {
            continue;
        }

        if( ( rtcInfo0 || instance0isRobot ) && rtcInfo1){
            RTC::PortService_var portRef0;
            if(!instance0isRobot){
                PortMap::iterator q = rtcInfo0->portMap.find(connection.PortName[0]);
                if(q == rtcInfo0->portMap.end()){
                    mv->putln(format(_("{0} does not have a port {1}."), instanceName0, connection.PortName[0]));
                    continue;
                }
                portRef0 = q->second;
            }

            RTC::PortService_var portRef1;
            PortMap::iterator q = rtcInfo1->portMap.find(connection.PortName[1]);
            if(q == rtcInfo1->portMap.end()){
                mv->putln(format(_("{0} does not have a port {1}."), instanceName1, connection.PortName[1]));
                continue;
            }
            portRef1 = q->second;
            RTC::PortProfile_var profile = portRef1->get_port_profile();
            const char* type;
            if (!(NVUtil::find(profile->properties, "port.port_type") >>= type)) {
                continue;
            }
            bool port1isIn = false;
            if(!std::strcmp(type,"DataInPort")){
                port1isIn = true;
            }

            if(instance0isRobot){
                PortHandlerPtr robotPortHandler;
                if(port1isIn){
                    robotPortHandler = virtualRobotRTC->getOutPortHandler(connection.PortName[0]);
                } else {
                    robotPortHandler = virtualRobotRTC->getInPortHandler(connection.PortName[0]);
                }
                if(!robotPortHandler){
                    mv->putln(format(_("The robot does not have a port named {}."), connection.PortName[0]));
                    continue;
                }
                portRef0 = robotPortHandler->portRef;
                if(CORBA::is_nil(portRef0)){
                    continue;
                }
            }

            int connected = false;
            if(port1isIn){
                mv->putln(
                    format(_("connect {0}:{1} --> {2}:{3}"),
                           instanceName0, connection.PortName[0], instanceName1, connection.PortName[1]));
                connected = connectPorts(portRef0, portRef1);
            }else{
                mv->putln(
                    format(_("connect {0}:{1} <-- {2}:{3}"),
                           instanceName0, connection.PortName[0], instanceName1, connection.PortName[1]));
                connected = connectPorts(portRef1, portRef0);
            }

            if(!connected){
                mv->putln( _("Connection was successful."));
            } else if(connected == -1){
                mv->putln(_("Connection failed."));
            } else if(connected == 1){
                mv->putln(_(" It has already been connected."));
            }
        }
    }

    if(configMode.is(BodyRTCItem::CONF_ALL_MODE) && autoConnect){
        std::vector<RTC::RTObject_var> rtcRefs;
        if(!moduleName.empty()){
            if(rtcomp && rtcomp->rtc()){
                rtcRefs.push_back(rtcomp->rtc()->getObjRef());
            }
        }
        if(rtcRefs.empty()){
            CosNaming::BindingList_var bl;
            CosNaming::BindingIterator_var bi;
            naming->list(naming->getRootContext(), 100, bl, bi);
            CORBA::ULong len(bl->length());
            for(CORBA::ULong i = 0; i < len; ++i){
                string name(naming->toString(bl[i].binding_name));
                if(name != instanceName+".rtc" && name.find(".rtc") != string::npos){
                    RTC::RTObject_var rtcRef;
                    try {
                        rtcRef = RTC::RTObject::_narrow(naming->resolve(bl[i].binding_name));
                    } catch(const CosNaming::NamingContext::NotFound &ex) {
                        
                    }
                    rtcRefs.push_back(rtcRef);
                }
            }
        }

        PortInfoMap& outPortInfoMap = bridgeConf->outPortInfos;
        for(PortInfoMap::iterator it=outPortInfoMap.begin(); it!=outPortInfoMap.end(); it++){
            string robotPortName(it->first);

            for(std::vector<RTC::RTObject_var>::iterator it = rtcRefs.begin(); it != rtcRefs.end(); it++){
                RTC::PortServiceList_var ports = (*it)->get_ports();
                for(CORBA::ULong i=0; i < ports->length(); ++i){
                    RTC::PortProfile_var profile = ports[i]->get_port_profile();
                    const char* type;
                    if (!(NVUtil::find(profile->properties, "port.port_type") >>= type)){
                        break;
                    }
                    if(!std::strcmp(type,"DataInPort")){
                        std::string portName(profile->name);
                        string::size_type index = portName.rfind(".");
                        if (index != string::npos) portName = portName.substr(index+1);
                        if(portName==robotPortName || (robotPortName=="u_out" && portName=="u") ||
                           (robotPortName=="u_out" && portName=="u_in")){
                            RtcInfoPtr rtcInfo = addRtcVectorWithConnection(*it);
                            PortMap::iterator q = rtcInfo->portMap.find(portName);
                            RTC::PortService_var controllerPortRef = q->second;
                            PortHandlerPtr robotPortHandler = virtualRobotRTC->getOutPortHandler(robotPortName);
                            RTC::PortService_var  robotPortRef = robotPortHandler->portRef;
                            if(!CORBA::is_nil(robotPortRef)){
                                int connected = connectPorts(robotPortRef, controllerPortRef);
                                mv->putln(
                                    format(_("connect {0}:{1} --> {2}:{3}"),
                                           virtualRobotRTC->getInstanceName(), robotPortName,
                                           string((*it)->get_component_profile()->instance_name), portName));
                                if(!connected){
                                    mv->putln(_("Connection was successful."));
                                } else if(connected == -1){
                                    mv->putln(_("Connection failed."));
                                } else if(connected == 1){
                                    mv->putln(_(" It has already been connected."));
                                }
                            }
                        }
                    }
                }
            }
        }

        PortInfoMap& inPortInfoMap = bridgeConf->inPortInfos;
        for(PortInfoMap::iterator it=inPortInfoMap.begin(); it!=inPortInfoMap.end(); it++){
            string robotPortName(it->first);

            for(std::vector<RTC::RTObject_var>::iterator it = rtcRefs.begin(); it != rtcRefs.end(); it++){
                RTC::PortServiceList_var ports = (*it)->get_ports();
                for(CORBA::ULong i=0; i < ports->length(); ++i){
                    RTC::PortProfile_var profile = ports[i]->get_port_profile();
                    const char* type;
                    if(!(NVUtil::find(profile->properties, "port.port_type") >>= type)){
                        break;
                    }
                    if(!std::strcmp(type,"DataOutPort")){
                        std::string portName(profile->name);
                        string::size_type index = portName.rfind(".");
                        if (index != string::npos) portName = portName.substr(index+1);
                        if(portName==robotPortName || (robotPortName=="u_in" && portName=="u") ||
                           (robotPortName=="u_in" && portName=="u_out")){
                            RtcInfoPtr rtcInfo = addRtcVectorWithConnection(*it);
                            PortMap::iterator q = rtcInfo->portMap.find(portName);
                            RTC::PortService_var controllerPortRef = q->second;
                            PortHandlerPtr robotPortHandler = virtualRobotRTC->getInPortHandler(robotPortName);
                            RTC::PortService_var  robotPortRef = robotPortHandler->portRef;
                            if(!CORBA::is_nil(robotPortRef)){
                                int connected = connectPorts(controllerPortRef, robotPortRef);
                                mv->putln(
                                    format(_("connect {0}:{1} <-- {2}:{3}"),
                                           virtualRobotRTC->getInstanceName(), robotPortName,
                                           string((*it)->get_component_profile()->instance_name), portName));
                                if(!connected){
                                    mv->putln(_("Connection was successful."));
                                } else if(connected == -1){
                                    mv->putln(_("Connection failed."));
                                } else if(connected == 1){
                                    mv->putln(_(" It has already been connected."));
                                }
                            }
                        }
                    }
                }
            }
        }
    }
}

int BodyRTCItemImpl::connectPorts(RTC::PortService_var outPort, RTC::PortService_var inPort)
{
    RTC::ConnectorProfileList_var connectorProfiles = inPort->get_connector_profiles();
    for(CORBA::ULong i=0; i < connectorProfiles->length(); ++i){
        RTC::ConnectorProfile& connectorProfile = connectorProfiles[i];
        RTC::PortServiceList& connectedPorts = connectorProfile.ports;

        for(CORBA::ULong j=0; j < connectedPorts.length(); ++j){
            RTC::PortService_ptr connectedPortRef = connectedPorts[j];
            if(connectedPortRef->_is_equivalent(outPort)){
                return 1;
            }
        }
    }
    // connect ports
    RTC::ConnectorProfile cprof;
    cprof.connector_id = "";
    cprof.name = CORBA::string_dup("connector0");
    cprof.ports.length(2);
    cprof.ports[0] = RTC::PortService::_duplicate(inPort);
    cprof.ports[1] = RTC::PortService::_duplicate(outPort);

    CORBA_SeqUtil::push_back(cprof.properties,
                             NVUtil::newNV("dataport.dataflow_type",
                                           "Push"));
    CORBA_SeqUtil::push_back(cprof.properties,
                             NVUtil::newNV("dataport.interface_type",
                                           "corba_cdr"));
    CORBA_SeqUtil::push_back(cprof.properties,
                             NVUtil::newNV("dataport.subscription_type",
                                           "flush"));
    RTC::ReturnCode_t result = inPort->connect(cprof);

    if(result == RTC::RTC_OK)
        return 0;
    else
        return -1;
}


void BodyRTCItemImpl::deleteModule(bool waitToBeDeleted)
{
    RTC::Manager& rtcManager = RTC::Manager::instance();
    
    if(TRACE_FUNCTIONS){
        cout << "BodyRTCItem::deleteModule()" << endl;
    }
    
    std::vector<string> deleteList;
    if(bridgeConf){
        ModuleInfoList& moduleInfoList = bridgeConf->moduleInfoList;
        ModuleInfoList::iterator it;
        for(it=moduleInfoList.begin(); it != moduleInfoList.end(); ++it){
            if(it->isLoaded){
                if(it->rtcServant){
                    deleteList.push_back(it->rtcServant->getInstanceName());
                    it->rtcServant->exit();
                    mv->putln(format(_("delete {}"), it->rtcServant->getInstanceName()));
                }
            }
        }
        delete bridgeConf;
        bridgeConf = 0;
    }
    
    if(rtcomp){
        rtcomp->deleteRTC();
        delete rtcomp;
        rtcomp = 0;
    }

    if(virtualRobotRTC){
        deleteList.push_back(virtualRobotRTC->getInstanceName());
        mv->putln(format(_("delete {}"), virtualRobotRTC->getInstanceName()));
        cnoid::deleteRTC(virtualRobotRTC);
        virtualRobotRTC = 0;
    }

    if(waitToBeDeleted){
        std::vector<string> remainder;
        for(int i=0; i < 100; i++){
            remainder.clear();
            for(std::vector<string>::iterator it=deleteList.begin(); it!=deleteList.end(); it++){
                RTC::RtcBase* component = rtcManager.getComponent((*it).c_str());
                if(component){
                    remainder.push_back(*it);
                }
            }
            if(remainder.empty()){
                return;
            }
            msleep(20);
        }
        for(std::vector<string>::iterator it=remainder.begin(); it!=remainder.end(); it++){
            mv->putln(format(_("{} cannot be deleted."), *it));
        }
    }

    if(TRACE_FUNCTIONS){
        cout << "End of BodyRTCItem::deleteModule()" << endl;
    }
}


#ifdef ENABLE_SIMULATION_PROFILING
void BodyRTCItem::getProfilingNames(vector<string>& profilingNames)
{
    impl->profilingNames.push_back("    BodyRTC calculation time");
    impl->profilingNames.push_back("    Controller calculation time");
}


void BodyRTCItem::getProfilingTimes(vector<double>& profilingToimes)
{
    impl->profilingToimes.push_back(impl->bodyRTCTime);
    impl->profilingToimes.push_back(impl->controllerTime);
}
#endif
