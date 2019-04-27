/**
   @author Shin'ichiro Nakaoka
*/

#include "ControllerRTCItem.h"
#include "OpenRTMUtil.h"
#include "LoggerUtil.h"
#include <cnoid/ItemManager>
#include <cnoid/MessageView>
#include <cnoid/ProjectManager>
#include <cnoid/Archive>
#include <cnoid/Sleep>
#include <cnoid/ExecutablePath>
#include <cnoid/FileUtil>
#include <cnoid/CorbaUtil>
#include <fmt/format.h>
#include "gettext.h"

using namespace std;
using namespace cnoid;
using fmt::format;
namespace filesystem = boost::filesystem;

namespace {

enum ExecContextType {
    SIMULATION_EXECUTION_CONTEXT = 0,
    SIMULATION_PERIODIC_EXECUTION_CONTEXT,
    N_EXEC_CONTEXT_TYPES,
    NO_EXECUTION_CONTEXT
};

}

namespace cnoid {

class ControllerRTCItemImpl
{
public:
    ControllerRTCItem* self;
    RTC::RtcBase* rtc;

    RTC::ExecutionContextService_var execContext;
    OpenRTM::ExtTrigExecutionContextService_var extTrigExecContext;
    int currentExecContextType;

    int periodicRateProperty;
    int periodicRate;
    double executionCycle;
    double executionCycleCounter;

    std::string moduleNameProperty;
    std::string moduleName;
    std::string moduleFilename;
    std::string rtcInstanceNameProperty;
    std::string rtcInstanceName;

    enum BaseDirectoryType {
        NO_BASE_DIRECTORY,
        RTC_DIRECTORY,
        PROJECT_DIRECTORY,
        N_BASE_DIRECTORY_TYPES
    };
    Selection baseDirectoryType;

    filesystem::path rtcDirectory;

    Selection execContextType;
    bool useOnlySimulationExecutionContext = false;

    MessageView* mv;
    
    ControllerRTCItemImpl(ControllerRTCItem* self);
    ControllerRTCItemImpl(ControllerRTCItem* self, const ControllerRTCItemImpl& org);
    void setBaseDirectoryType(int type);
    void setRTCModule(const std::string& name);
    std::string getModuleFilename();
    bool createRTCmain(bool isBodyIORTC = false);
    void deleteRTC(bool waitToBeDeleted);
    bool start();
    void stop();
    void doPutProperties(PutPropertyFunction& putProperty);
    bool store(Archive& archive);
    bool restore(const Archive& archive);
};

}


void ControllerRTCItem::initialize(ExtensionManager* ext)
{
    static bool initialized = false;
    if(!initialized){
        ext->itemManager().registerClass<ControllerRTCItem>(N_("ControllerRTCItem"));
        ext->itemManager().addCreationPanel<ControllerRTCItem>();
        initialized = true;
    }
}


ControllerRTCItem::ControllerRTCItem()
{
    impl = new ControllerRTCItemImpl(this);
}


ControllerRTCItemImpl::ControllerRTCItemImpl(ControllerRTCItem* self)
    : self(self),
      baseDirectoryType(N_BASE_DIRECTORY_TYPES, CNOID_GETTEXT_DOMAIN_NAME),
      execContextType(N_EXEC_CONTEXT_TYPES, CNOID_GETTEXT_DOMAIN_NAME),
      mv(MessageView::instance())
{
    rtc = nullptr;
    currentExecContextType = NO_EXECUTION_CONTEXT;
    
    periodicRateProperty = 0;

    baseDirectoryType.setSymbol(NO_BASE_DIRECTORY, N_("None"));
    baseDirectoryType.setSymbol(RTC_DIRECTORY, N_("RTC directory"));
    baseDirectoryType.setSymbol(PROJECT_DIRECTORY, N_("Project directory"));
    baseDirectoryType.select(RTC_DIRECTORY);

    rtcDirectory = filesystem::path(executableTopDirectory()) / CNOID_PLUGIN_SUBDIR / "rtc";

    execContextType.setSymbol(SIMULATION_EXECUTION_CONTEXT,  N_("SimulationExecutionContext"));
    execContextType.setSymbol(SIMULATION_PERIODIC_EXECUTION_CONTEXT,  N_("SimulationPeriodicExecutionContext"));
    execContextType.select(SIMULATION_EXECUTION_CONTEXT);

}


ControllerRTCItem::ControllerRTCItem(const ControllerRTCItem& org)
    : ControllerItem(org)
{
    impl = new ControllerRTCItemImpl(this, *org.impl);
}


ControllerRTCItemImpl::ControllerRTCItemImpl(ControllerRTCItem* self, const ControllerRTCItemImpl& org)
    : ControllerRTCItemImpl(self)
{
    baseDirectoryType = org.baseDirectoryType;
    rtcDirectory = org.rtcDirectory;
    moduleNameProperty = org.moduleNameProperty;
    rtcInstanceNameProperty = org.rtcInstanceNameProperty;
    periodicRateProperty = org.periodicRateProperty;
    execContextType = org.execContextType;
    useOnlySimulationExecutionContext = org.useOnlySimulationExecutionContext;
}


ControllerRTCItem::~ControllerRTCItem()
{
    delete impl;
}


Item* ControllerRTCItem::doDuplicate() const
{
    return new ControllerRTCItem(*this);
}


void ControllerRTCItem::onConnectedToRoot()
{
    createRTC();
}


void ControllerRTCItem::onDisconnectedFromRoot()
{
    deleteRTC(false);
}


void ControllerRTCItemImpl::setBaseDirectoryType(int type)
{
    if(type != baseDirectoryType.which()){
        baseDirectoryType.select(type);
        self->createRTC();
    }
}


void ControllerRTCItem::setRTCModule(const std::string& name)
{
    impl->setRTCModule(name);
}


void ControllerRTCItemImpl::setRTCModule(const std::string& name)
{
    if(name != moduleNameProperty){
        filesystem::path modulePath(name);
        if(modulePath.is_absolute()){
            baseDirectoryType.select(NO_BASE_DIRECTORY);
            if(modulePath.parent_path() == rtcDirectory){
                baseDirectoryType.select(RTC_DIRECTORY);
                modulePath = modulePath.filename();
            } else {
                filesystem::path projectDir(ProjectManager::instance()->currentProjectDirectory());
                if(!projectDir.empty() && (modulePath.parent_path() == projectDir)){
                    baseDirectoryType.select(PROJECT_DIRECTORY);
                    modulePath = modulePath.filename();
                }
            }
        }
        moduleNameProperty = modulePath.string();
        self->createRTC();
    }
}


void ControllerRTCItem::setRTCInstanceName(const std::string& name)
{
    if(name != impl->rtcInstanceNameProperty){
        impl->rtcInstanceNameProperty = name;
        createRTC();
    }
}


void ControllerRTCItem::setExecContextType(int which)
{
    DDEBUG_V("ControllerRTCItem::setExecContextType %d", which);

    if(which != impl->execContextType.which()){
        impl->execContextType.select(which);

#if defined(OPENRTM_VERSION11)
        createRTC();

#elif defined(OPENRTM_VERSION12)
        
        if(CORBA::is_nil(impl->execContext)){ // Is this check valid?
            return;
        }

        RTC::ReturnCode_t ret = impl->execContext->stop();
        DDEBUG_V("ControllerRTCItem::setExecContextType stop %d", ret);
        RTC::ExecutionContextList_var eclist = impl->rtc->get_owned_contexts();
        for(CORBA::ULong index = 0; index < eclist->length(); ++index){
            if(!CORBA::is_nil(eclist[index])){
                RTC::ExecutionContextService_var execContext = RTC::ExecutionContextService::_narrow(eclist[index]);
                if(!CORBA::is_nil(execContext)){
                    // Why is the following condition checked???
                    if(which == index){
                        impl->execContext = execContext; // Is this direct assignment valid?
                        impl->execContext->start();
                        DDEBUG_V("ControllerRTCItem::setExecContextType start %d", index);
                    }
                }
            }
        }
#endif
    }
}


void ControllerRTCItem::setPeriodicRate(int rate)
{
    if(rate != impl->periodicRateProperty){
        impl->periodicRateProperty = rate;
        createRTC();
    }
}


RTC::RtcBase* ControllerRTCItem::rtc()
{
    return impl->rtc;
}


std::string ControllerRTCItem::rtcModuleName() const
{
    return impl->moduleName;
}


std::string ControllerRTCItem::rtcInstanceName() const
{
    return impl->rtcInstanceName;
}


void ControllerRTCItem::useOnlySimulationExecutionContext()
{
    impl->useOnlySimulationExecutionContext = true;
    setExecContextType(SIMULATION_EXECUTION_CONTEXT);
}


std::string ControllerRTCItemImpl::getModuleFilename()
{
    if(moduleNameProperty.empty()){
        return string();
    }
    
    filesystem::path path(moduleNameProperty);

    moduleName = path.stem().string();
    
    if(!path.is_absolute()){
        if(baseDirectoryType.is(RTC_DIRECTORY)){
            path = rtcDirectory / path;
        } else if(baseDirectoryType.is(PROJECT_DIRECTORY)){
            string projectDir = ProjectManager::instance()->currentProjectDirectory();
            if(!projectDir.empty()){
                path = filesystem::path(projectDir) / path;
            } else {
                mv->putln(
                    format(_("The rtc of {} cannot be generated because the project directory "
                             "is not determined. Please save the project as a project file."),
                           self->name()),
                    MessageView::ERROR);
                return string();
            }
        }
    }

    string ext = path.extension().string();
    if(ext.empty()){
        path += ".";
        path += DLL_EXTENSION;
    }
        
    if(filesystem::exists(path)){
        return path.string();
    } else {
        mv->putln(
            format(_("RTC module file \"{0}\" of {1} does not exist."), path.string(), self->name()),
            MessageView::ERROR);
    }
    
    return string();
}


std::string ControllerRTCItem::getDefaultRTCInstanceName() const
{
    return impl->moduleName;
}


bool ControllerRTCItem::createRTC()
{
    if(impl->createRTCmain()){
        impl->mv->putln(format(_("BodyIoRTC \"{}\" has been created."), impl->rtcInstanceName));
        return true;
    }
    return false;
}


bool ControllerRTCItem::createRTCmain(bool isBodyIORTC)
{
    return impl->createRTCmain(isBodyIORTC);
}


bool ControllerRTCItemImpl::createRTCmain(bool isBodyIORTC) {
  DDEBUG_V("ControllerRTCItemImpl::createRTCmain:%d", isBodyIORTC);
    if(rtc){
        self->deleteRTC(true);
    }
    
    string moduleFilename = getModuleFilename();
    if(moduleFilename.empty()){
        return false;
    }
    string initFuncName = moduleName + "Init";
    auto& manager = RTC::Manager::instance();
    manager.load(moduleFilename.c_str(), initFuncName.c_str());

    auto instanceBaseName = rtcInstanceNameProperty;
    if(instanceBaseName.empty()){
        instanceBaseName = self->getDefaultRTCInstanceName();
    }
    if(instanceBaseName.empty()){
        mv->putln(
            format(_("The RTC instance name of {} is not specified."), self->name()),
            MessageView::ERROR);
        return false;
    }
    rtcInstanceName = instanceBaseName;

    const int maxNumInstances = 10000;
    int index = 2;
    for(int index = 2; index < maxNumInstances; ++index){
        if(manager.getComponent(rtcInstanceName.c_str()) == nullptr){
            break;
        }
        rtcInstanceName = format("{0}({1})", instanceBaseName, index++);
    }
    if(index >= maxNumInstances){
        mv->putln(
            format(_("RTC \"{0}\" of {1} is not created because more than "
                     "{2} existing instances have the same base name \"{3}\"."),
                   moduleName, self->name(), maxNumInstances, instanceBaseName),
            MessageView::ERROR);
        return false;
    }

    string option;
    if(periodicRateProperty > 0){
        periodicRate = periodicRateProperty;

#if defined(OPENRTM_VERSION11)
        option =
            format("instance_name={0}&exec_cxt.periodic.type={1}&exec_cxt.periodic.rate={2}",
                   rtcInstanceName, execContextType.selectedSymbol(), periodicRate);
        DDEBUG("ControllerRTCItemImpl::createRTCmain OPENRTM_VERSION11");

#elif defined(OPENRTM_VERSION12)
        if(isBodyIORTC){
            option =
                format("instance_name={0}&execution_contexts=SimulationExecutionContext()&"
                       "exec_cxt.periodic.type={1}&exec_cxt.periodic.rate={2}&"
                       "exec_cxt.sync_activation=NO&exec_cxt.sync_deactivation=NO",
                       rtcInstanceName, execContextType.selectedSymbol(), periodicRate);
            DDEBUG_V("ControllerRTCItemImpl::createRTCmain isBodyIORTC=TRUE  %s", option.c_str());
        } else {
            option =
                format("instance_name={0}&"
                        "execution_contexts=SimulationExecutionContext(),SimulationPeriodicExecutionContext()&"
                        "exec_cxt.periodic.type={1}&exec_cxt.periodic.rate={2}&"
                        "exec_cxt.sync_activation=NO&exec_cxt.sync_deactivation=NO",
                       rtcInstanceName, execContextType.selectedSymbol(), periodicRate);
            DDEBUG_V("ControllerRTCItemImpl::createRTCmain isBodyIORTC=FALSE  %s", option.c_str());
        }
#endif
    } else {
        periodicRate = 0;

#if defined(OPENRTM_VERSION11)
          option =
              format("instance_name={0}&exec_cxt.periodic.type={1}",
                     rtcInstanceName, execContextType.selectedSymbol());
          DDEBUG("ControllerRTCItemImpl::createRTCmain OPENRTM_VERSION11");

#elif defined(OPENRTM_VERSION12)
        if(isBodyIORTC){
            option =
                format("instance_name={0}&execution_contexts=SimulationExecutionContext()&"
                       "exec_cxt.periodic.type={1}&exec_cxt.sync_activation=NO&exec_cxt.sync_deactivation=NO",
                       rtcInstanceName, execContextType.selectedSymbol());
            DDEBUG_V("ControllerRTCItemImpl::createRTCmain isBodyIORTC=TRUE  %s", option.c_str());
        } else {
            option =
                format("instance_name={0}&"
                       "execution_contexts=SimulationExecutionContext(),SimulationPeriodicExecutionContext()&"
                       "exec_cxt.periodic.type={1}&exec_cxt.sync_activation=NO&exec_cxt.sync_deactivation=NO",
                       rtcInstanceName, execContextType.selectedSymbol());
            DDEBUG_V("ControllerRTCItemImpl::createRTCmain isBodyIORTC=FALSE  %s", option.c_str());
        }
#endif
    }
    rtc = createManagedRTC(moduleName + "?" + option);

    if(!rtc){
        mv->putln(
            format(_("RTC \"{0}\" of {1} cannot be created by the RTC manager.\n"
                     " RTC module file: \"{2}\"\n"
                     " Init function: {3}\n"
                     " option: {4}"),
                   moduleName, self->name(), moduleFilename, initFuncName, option),
            MessageView::ERROR);
        return false;
    }

    if(periodicRate == 0){
        periodicRate = QString::fromStdString(string(rtc->getProperties()["exec_cxt.periodic.rate"])).toInt();  
    }

    currentExecContextType = execContextType.which();
    execContext = RTC::ExecutionContextService::_nil();
    RTC::ExecutionContextList_var eclist = rtc->get_owned_contexts();

#if defined(OPENRTM_VERSION11)
    for(CORBA::ULong i=0; i < eclist->length(); ++i){
        if(!CORBA::is_nil(eclist[i])){
            execContext = RTC::ExecutionContextService::_narrow(eclist[i]);
            break;
        }
    }
#elif defined(OPENRTM_VERSION12)
    execContext = RTC::ExecutionContextService::_narrow(eclist[currentExecContextType]);
#endif
    extTrigExecContext = OpenRTM::ExtTrigExecutionContextService::_narrow(execContext);

    if(currentExecContextType == SIMULATION_EXECUTION_CONTEXT){
        if(CORBA::is_nil(extTrigExecContext)){
            currentExecContextType = NO_EXECUTION_CONTEXT;
        }
    }

    return true;
}


void ControllerRTCItem::deleteRTC(bool waitToBeDeleted)
{
    impl->deleteRTC(waitToBeDeleted);
}


void ControllerRTCItemImpl::deleteRTC(bool waitToBeDeleted)
{
    if(rtc){
        bool deleted = cnoid::deleteRTC(rtc);
        rtc = 0;

        if(waitToBeDeleted){
            RTC::Manager& manager = RTC::Manager::instance();
            for(int i=0; i < 100; i++){
                if(!manager.getComponent(rtcInstanceName.c_str())){
                    deleted = true;
                    break;
                }
                msleep(20);
            }
        }
        if(!deleted){
            mv->putln(
                format(_("RTC instance {0} of {1} cannot be deleted."),
                       rtcInstanceName, self->name()),
                MessageView::WARNING);
        }
    }
}


bool ControllerRTCItem::start()
{
    return impl->start();
}


bool ControllerRTCItemImpl::start()
{
    bool isReady = false;
    
    if(rtc && currentExecContextType != NO_EXECUTION_CONTEXT){

        RTC::ReturnCode_t result = RTC::RTC_OK;
        RTC::LifeCycleState state = execContext->get_component_state(rtc->getObjRef());

        bool doStateChange = false;
        if(state == RTC::ERROR_STATE){
            doStateChange = true;
            result = execContext->reset_component(rtc->getObjRef());
        } else if(state == RTC::ACTIVE_STATE){
            doStateChange = true;
            result = execContext->deactivate_component(rtc->getObjRef());
        }

        if(doStateChange && currentExecContextType == SIMULATION_EXECUTION_CONTEXT){
            extTrigExecContext->tick();
        }

        if(result == RTC::RTC_OK){
            if(execContext->activate_component(rtc->getObjRef()) == RTC::RTC_OK){
                isReady = true;
            }
        }
    }
    
    return isReady;
}


double ControllerRTCItem::timeStep() const
{
    return 0.0;
}


void ControllerRTCItem::input()
{

}


bool ControllerRTCItem::control()
{
    if(impl->currentExecContextType == SIMULATION_EXECUTION_CONTEXT){
        impl->extTrigExecContext->tick();
    }
    return true;
}


void ControllerRTCItem::output()
{

}


void ControllerRTCItem::stop()
{
    impl->stop();
}


void ControllerRTCItemImpl::stop()
{
    RTC::LifeCycleState state = execContext->get_component_state(rtc->getObjRef());
    if(state == RTC::ERROR_STATE){
        execContext->reset_component(rtc->getObjRef());
    } else {
        execContext->deactivate_component(rtc->getObjRef());
    }
    if(currentExecContextType == SIMULATION_EXECUTION_CONTEXT){
        extTrigExecContext->tick();
    }
}


void ControllerRTCItem::doPutProperties(PutPropertyFunction& putProperty)
{
    ControllerItem::doPutProperties(putProperty);
    impl->doPutProperties(putProperty);
}


void ControllerRTCItemImpl::doPutProperties(PutPropertyFunction& putProperty)
{
    FilePathProperty moduleProperty(
        moduleNameProperty,
        { format(_("RT-Component module (*{})"), DLL_SUFFIX) });

    if(baseDirectoryType.is(RTC_DIRECTORY)){
        moduleProperty.setBaseDirectory(rtcDirectory.string());
    } else if(baseDirectoryType.is(PROJECT_DIRECTORY)){
        moduleProperty.setBaseDirectory(ProjectManager::instance()->currentProjectDirectory());
    }

    putProperty(_("RTC module"), moduleProperty,
                [&](const string& name){ setRTCModule(name); return true; });
    putProperty(_("Base directory"), baseDirectoryType,
                [&](int which){ setBaseDirectoryType(which); return true; });
    
    putProperty(_("RTC Instance name"), rtcInstanceNameProperty,
                [&](const string& name) { self->setRTCInstanceName(name); return true; });

    if(!useOnlySimulationExecutionContext){
        putProperty(_("Execution context"), execContextType,
                    [&](int which){ self->setExecContextType(which); return true; });
    }

    putProperty.decimals(3)(_("Periodic rate"), periodicRateProperty,
                            [&](int rate){ self->setPeriodicRate(rate); return true; });
}


bool ControllerRTCItem::store(Archive& archive)
{
    return ControllerItem::store(archive) && impl->store(archive);
}


bool ControllerRTCItemImpl::store(Archive& archive)
{
    archive.writeRelocatablePath("module", moduleNameProperty);
    archive.write("baseDirectory", baseDirectoryType.selectedSymbol(), DOUBLE_QUOTED);
    archive.write("instanceName", rtcInstanceNameProperty, DOUBLE_QUOTED);
    if(!useOnlySimulationExecutionContext){
        archive.write("executionContext", execContextType.selectedSymbol(), DOUBLE_QUOTED);
    }
    archive.write("periodicRate", periodicRateProperty);
    return true;
}


bool ControllerRTCItem::restore(const Archive& archive)
{
    return ControllerItem::restore(archive) && impl->restore(archive);
}


bool ControllerRTCItemImpl::restore(const Archive& archive)
{
    DDEBUG("ControllerRTCItemImpl::restore");
    string value;
    if(archive.read("module", value) || archive.read("moduleName", value)){
        filesystem::path path(archive.expandPathVariables(value));
        moduleNameProperty = path.make_preferred().string();
    }
    if(archive.read("baseDirectory", value) || archive.read("pathBaseType", value)){
        baseDirectoryType.select(value);
    }
    archive.read("instanceName", rtcInstanceNameProperty);

    if(!useOnlySimulationExecutionContext){
        if(archive.read("executionContext", value)){
            if(!execContextType.select(value)){
                // For the backward compatibility
                if(value == "ChoreonoidExecutionContext"){
                    execContextType.select(SIMULATION_EXECUTION_CONTEXT);
                } else if(value == "PeriodicExecutionContext"){
                    execContextType.select(SIMULATION_PERIODIC_EXECUTION_CONTEXT);
                }
            }
        }
    }
    archive.read("periodicRate", periodicRateProperty);
    
    return true;
}
