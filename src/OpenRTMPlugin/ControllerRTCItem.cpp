/**
   @author Shin'ichiro Nakaoka
*/

#include "ControllerRTCItem.h"
#include "OpenRTMUtil.h"
#include <cnoid/ItemManager>
#include <cnoid/MessageView>
#include <cnoid/ProjectManager>
#include <cnoid/Archive>
#include <cnoid/Sleep>
#include <cnoid/ExecutablePath>
#include <cnoid/FileUtil>
#include "gettext.h"

using namespace std;
using namespace cnoid;
using boost::format;
namespace filesystem = boost::filesystem;

#if ( defined ( WIN32 ) || defined ( _WIN32 ) || defined(__WIN32__) || defined(__NT__) )
#define DLL_EXTENSION ".dll"
#elif defined(__APPLE__)
#define DLL_EXTENSION ".dylib"
#else
#define DLL_EXTENSION ".so"
#endif

namespace cnoid {

class ControllerRTCItemImpl
{
public:
    ControllerRTCItem* self;
    RTC::RtcBase* rtc = 0;
    OpenRTM::ExtTrigExecutionContextService_var execContext;
    bool isChoreonoidExecutionContext;

    int periodicRateProperty;
    int periodicRate;
    double executionCycle;
    double executionCycleCounter;

    std::string moduleNameProperty;
    std::string moduleName;
    std::string moduleFilename;
    std::string rtcInstanceNameProperty;
    std::string rtcInstanceName;

    enum RelativePathBaseType {
        RTC_DIRECTORY,
        PROJECT_DIRECTORY,
        N_RELATIVE_PATH_BASE_TYPES
    };
    Selection relativePathBaseType;

    enum ExecContextType {
        PERIODIC_EXECUTION_CONTEXT,
        CHOREONOID_EXECUTION_CONTEXT,
        N_EXEC_CONTEXT_TYPES
    };
    Selection execContextType;
    bool useOnlyChoreonoidExecutionContext = false;

    MessageView* mv;
    
    ControllerRTCItemImpl(ControllerRTCItem* self);
    ControllerRTCItemImpl(ControllerRTCItem* self, const ControllerRTCItemImpl& org);
    std::string getModuleFilename();
    bool createRTCmain();
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
      mv(MessageView::instance()),
      relativePathBaseType(N_RELATIVE_PATH_BASE_TYPES, CNOID_GETTEXT_DOMAIN_NAME),
      execContextType(N_EXEC_CONTEXT_TYPES, CNOID_GETTEXT_DOMAIN_NAME)
{
    relativePathBaseType.setSymbol(RTC_DIRECTORY, N_("RTC directory"));
    relativePathBaseType.setSymbol(PROJECT_DIRECTORY, N_("Project directory"));
    relativePathBaseType.select(RTC_DIRECTORY);

    execContextType.setSymbol(PERIODIC_EXECUTION_CONTEXT,  N_("PeriodicExecutionContext"));
    execContextType.setSymbol(CHOREONOID_EXECUTION_CONTEXT,  N_("ChoreonoidExecutionContext"));
    execContextType.select(CHOREONOID_EXECUTION_CONTEXT);

    periodicRateProperty = 0;
}


ControllerRTCItem::ControllerRTCItem(const ControllerRTCItem& org)
    : ControllerItem(org)
{
    impl = new ControllerRTCItemImpl(this, *org.impl);
}


ControllerRTCItemImpl::ControllerRTCItemImpl(ControllerRTCItem* self, const ControllerRTCItemImpl& org)
    : ControllerRTCItemImpl(self)
{
    relativePathBaseType = org.relativePathBaseType;
    moduleNameProperty = org.moduleNameProperty;
    rtcInstanceNameProperty = org.rtcInstanceNameProperty;
    periodicRateProperty = org.periodicRateProperty;
    execContextType = org.execContextType;
    useOnlyChoreonoidExecutionContext = org.useOnlyChoreonoidExecutionContext;
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


void ControllerRTCItem::setRelativePathBaseType(int which)
{
    if(which != impl->relativePathBaseType.which()){
        impl->relativePathBaseType.select(which);
        createRTC();
    }
}


void ControllerRTCItem::setRTCModule(const std::string& name)
{
    if(name != impl->moduleNameProperty){
        impl->moduleNameProperty = name;
        createRTC();
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
    if(which != impl->execContextType.which()){
        impl->execContextType.select(which);
        createRTC();
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


void ControllerRTCItem::useOnlyChoreonoidExecutionContext()
{
    impl->useOnlyChoreonoidExecutionContext = true;
    setExecContextType(ControllerRTCItemImpl::CHOREONOID_EXECUTION_CONTEXT);
}


std::string ControllerRTCItemImpl::getModuleFilename()
{
    if(moduleNameProperty.empty()){
        return string();
    }
    
    filesystem::path path(moduleNameProperty);

    moduleName = path.stem().string();
    
    if(!checkAbsolute(path)){
        if(relativePathBaseType.is(RTC_DIRECTORY)){
            path = filesystem::path(executableTopDirectory()) / CNOID_PLUGIN_SUBDIR / "rtc" / path;
        } else {
            string projectFile = ProjectManager::instance()->currentProjectFile();
            if(!projectFile.empty()){
                path = filesystem::path(projectFile).parent_path() / path;
            } else {
                mv->putln(MessageView::ERROR,
                          format(_("The rtc of %1% cannot be generated because the project directory is not determined."
                                   " Please save the project as a project file."))
                          % self->name());
                return string();
            }
        }
    }

    string ext = path.extension().string();
    if(ext.empty()){
        path += DLL_EXTENSION;
    }
        
    if(filesystem::exists(path)){
        return path.string();
    } else {
        mv->putln(MessageView::ERROR,
                  format(_("RTC module file \"%1%\" of %2% does not exist."))
                  % path.string() % self->name());
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
        impl->mv->putln(fmt(_("BodyIoRTC \"%1%\" has been created.")) % impl->rtcInstanceName);
        return true;
    }
    return false;
}


bool ControllerRTCItem::createRTCmain()
{
    return impl->createRTCmain();
}


bool ControllerRTCItemImpl::createRTCmain()
{
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
        mv->putln(MessageView::ERROR,
                  format(_("The RTC instance name of %1% is not specified."))
                  % self->name());
        return false;
    }
    rtcInstanceName = instanceBaseName;

    const int maxNumInstances = 10000;
    int index = 2;
    for(int index = 2; index < maxNumInstances; ++index){
        if(manager.getComponent(rtcInstanceName.c_str()) == nullptr){
            break;
        }
        rtcInstanceName = str(format("%1%(%2%)") % instanceBaseName % index++);
    }
    if(index >= maxNumInstances){
        mv->putln(MessageView::ERROR,
                  format(_("RTC \"%1%\" of %2% is not created because more than "
                           "%3% existing instances have the same base name \"%4%\"."))
                  % moduleName % self->name() % maxNumInstances % instanceBaseName);
        return false;
    }

    string option;
    if(periodicRateProperty > 0){
        periodicRate = periodicRateProperty;
        option = 
            str(format("instance_name=%1%&exec_cxt.periodic.type=%2%&exec_cxt.periodic.rate=%3%")
                % rtcInstanceName % execContextType.selectedSymbol() % periodicRate);
    } else {
        periodicRate = 0;
        option = 
            str(format("instance_name=%1%&exec_cxt.periodic.type=%2%")
                % rtcInstanceName % execContextType.selectedSymbol());
    }
    rtc = createManagedRTC((moduleName + "?" + option).c_str());

    if(!rtc){
        mv->putln(MessageView::ERROR,
                  format(_("RTC \"%1%\" of %2% cannot be created by the RTC manager.\n"
                           " RTC module file: \"%3%\"\n"
                           " Init function: %4%\n"
                           " option: %5%"))
                  % moduleName % self->name() % moduleFilename % initFuncName % option);
        return false;
    }

    if(periodicRate == 0){
        periodicRate = boost::lexical_cast<int>(rtc->getProperties()["exec_cxt.periodic.rate"]);
    }

    execContext = OpenRTM::ExtTrigExecutionContextService::_nil();
    isChoreonoidExecutionContext = false; 
    RTC::ExecutionContextList_var eclist = rtc->get_owned_contexts();
    for(CORBA::ULong i=0; i < eclist->length(); ++i){
        if(!CORBA::is_nil(eclist[i])){
            execContext = OpenRTM::ExtTrigExecutionContextService::_narrow(eclist[i]);
            isChoreonoidExecutionContext = execContextType.is(CHOREONOID_EXECUTION_CONTEXT);
            break;
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
            mv->putln(MessageView::WARNING,
                      format(_("RTC instance %1% of %2% cannot be deleted."))
                      % rtcInstanceName % self->name());
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
    
    if(rtc){
        if(!CORBA::is_nil(execContext)){
            RTC::ReturnCode_t result = RTC::RTC_OK;
            RTC::LifeCycleState state = execContext->get_component_state(rtc->getObjRef());
            if(state == RTC::ERROR_STATE){
                result = execContext->reset_component(rtc->getObjRef());
                execContext->tick();
            } else if(state == RTC::ACTIVE_STATE){
                result = execContext->deactivate_component(rtc->getObjRef());
                execContext->tick();
            }
            if(result == RTC::RTC_OK){
                result = execContext->activate_component(rtc->getObjRef());
                execContext->tick();
            }
            if(result == RTC::RTC_OK){
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
    if(impl->isChoreonoidExecutionContext){
        impl->execContext->tick();
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
    execContext->tick();
}


void ControllerRTCItem::doPutProperties(PutPropertyFunction& putProperty)
{
    ControllerItem::doPutProperties(putProperty);
    impl->doPutProperties(putProperty);
}


void ControllerRTCItemImpl::doPutProperties(PutPropertyFunction& putProperty)
{
    FileDialogFilter filter;
    filter.push_back(string(_("RT-Component module file")) + DLLSFX);
    string dir;
    if(!moduleNameProperty.empty() && checkAbsolute(filesystem::path(moduleNameProperty))){
        dir = filesystem::path(moduleNameProperty).parent_path().generic_string();
    } else if(relativePathBaseType.is(RTC_DIRECTORY)) {
        dir = (filesystem::path(executableTopDirectory()) / CNOID_PLUGIN_SUBDIR / "rtc").generic_string();
    }
    putProperty(_("RTC module"), FilePath(moduleNameProperty, filter, dir),
                [&](const std::string& name){ self->setRTCModule(name); return true; });

    putProperty(_("Relative path base"), relativePathBaseType,
                [&](int which){ self->setRelativePathBaseType(which); return true; });
    
    putProperty(_("RTC Instance name"), rtcInstanceNameProperty,
                [&](const std::string& name) { self->setRTCInstanceName(name); return true; });

    if(!useOnlyChoreonoidExecutionContext){
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
    archive.writeRelocatablePath("moduleName", moduleNameProperty);
    archive.write("pathBaseType", relativePathBaseType.selectedSymbol(), DOUBLE_QUOTED);
    archive.write("instanceName", rtcInstanceNameProperty, DOUBLE_QUOTED);
    if(!useOnlyChoreonoidExecutionContext){
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
    string value;
    if(archive.read("moduleName", value)){
        filesystem::path path(archive.expandPathVariables(value));
        moduleNameProperty = getNativePathString(path);
    }
    string symbol;
    if(archive.read("pathBaseType", symbol)){
        relativePathBaseType.select(symbol);
    }
    archive.read("instanceName", rtcInstanceNameProperty);

    if(!useOnlyChoreonoidExecutionContext){
        if(archive.read("executionContext", symbol)){
            execContextType.select(symbol);
        }
    }
    archive.read("periodicRate", periodicRateProperty);
    return true;
}
