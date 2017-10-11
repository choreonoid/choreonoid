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

    enum BaseDirectoryType {
        NO_BASE_DIRECTORY,
        RTC_DIRECTORY,
        PROJECT_DIRECTORY,
        N_BASE_DIRECTORY_TYPES
    };
    Selection baseDirectoryType;

    filesystem::path rtcDirectory;

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
    void setBaseDirectoryType(int type);
    void setRTCModule(const std::string& name);
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
      baseDirectoryType(N_BASE_DIRECTORY_TYPES, CNOID_GETTEXT_DOMAIN_NAME),
      execContextType(N_EXEC_CONTEXT_TYPES, CNOID_GETTEXT_DOMAIN_NAME)
{
    baseDirectoryType.setSymbol(NO_BASE_DIRECTORY, N_("None"));
    baseDirectoryType.setSymbol(RTC_DIRECTORY, N_("RTC directory"));
    baseDirectoryType.setSymbol(PROJECT_DIRECTORY, N_("Project directory"));
    baseDirectoryType.select(RTC_DIRECTORY);

    rtcDirectory = filesystem::path(executableTopDirectory()) / CNOID_PLUGIN_SUBDIR / "rtc";

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
    baseDirectoryType = org.baseDirectoryType;
    rtcDirectory = org.rtcDirectory;
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
    
    if(!path.is_absolute()){
        if(baseDirectoryType.is(RTC_DIRECTORY)){
            path = rtcDirectory / path;
        } else if(baseDirectoryType.is(PROJECT_DIRECTORY)){
            string projectDir = ProjectManager::instance()->currentProjectDirectory();
            if(!projectDir.empty()){
                path = filesystem::path(projectDir) / path;
            } else {
                mv->putln(MessageView::ERROR,
                          format(_("The rtc of %1% cannot be generated because the project directory "
                                   "is not determined. Please save the project as a project file."))
                          % self->name());
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
    FilePathProperty moduleProperty(
        moduleNameProperty,
        { str(format(_("RT-Component module (*%1%)")) % DLL_SUFFIX) });

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
    archive.writeRelocatablePath("module", moduleNameProperty);
    archive.write("baseDirectory", baseDirectoryType.selectedSymbol(), DOUBLE_QUOTED);
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
    if(archive.read("module", value) || archive.read("moduleName", value)){
        filesystem::path path(archive.expandPathVariables(value));
        moduleNameProperty = path.make_preferred().string();
    }
    if(archive.read("baseDirectory", value) || archive.read("pathBaseType", value)){
        baseDirectoryType.select(value);
    }
    archive.read("instanceName", rtcInstanceNameProperty);

    if(!useOnlyChoreonoidExecutionContext){
        if(archive.read("executionContext", value)){
            execContextType.select(value);
        }
    }
    archive.read("periodicRate", periodicRateProperty);
    
    return true;
}
