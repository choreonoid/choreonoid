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
#include <rtm/RTObject.h>
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

    MessageView* mv;
    
    ControllerRTCItemImpl(ControllerRTCItem* self);
    ControllerRTCItemImpl(ControllerRTCItem* self, const ControllerRTCItemImpl& org);
    bool setRelativePathBaseType(int which);
    bool setRTCModule(const std::string& name);
    bool setRTCInstanceName(const std::string& name);
    bool setExecContextType(int which);
    std::string getModuleFilename();
    bool createRTC();
    bool createRTCMain();
    void deleteRTC(bool waitToBeDeleted);
    void deleteRTC(RTC::RtcBase* rtc, bool waitToBeDeleted);
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

    periodicRateProperty = 1000;
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
    impl->createRTC();
}


void ControllerRTCItem::onDisconnectedFromRoot()
{
    impl->deleteRTC(false);
}


bool ControllerRTCItemImpl::setRelativePathBaseType(int which)
{
    if(which != relativePathBaseType.which()){
        relativePathBaseType.select(which);
        createRTC();
    }
    return true;
}


bool ControllerRTCItemImpl::setRTCModule(const std::string& name)
{
    if(name != moduleNameProperty){
        moduleNameProperty = name;
        createRTC();
    }
    return true;
}


bool ControllerRTCItemImpl::setRTCInstanceName(const std::string& name)
{
    if(name != rtcInstanceNameProperty){
        rtcInstanceNameProperty = name;
        createRTC();
    }
    return true;
}


bool ControllerRTCItemImpl::setExecContextType(int which)
{
    if(which != execContextType.which()){
        execContextType.select(which);
        createRTC();
    }
    return true;
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
        path /= DLL_EXTENSION;
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


std::string ControllerRTCItem::defaultRTCInstanceName() const
{
    return impl->moduleName;
}


bool ControllerRTCItem::createRTC()
{
    impl->createRTCMain();
}


bool ControllerRTCItemImpl::createRTC()
{
    if(createRTCMain()){
        mv->putln(fmt(_("BodyIoRTC \"%1%\" has been created.")) % rtcInstanceName);
        return true;
    }
    return false;
}


bool ControllerRTCItemImpl::createRTCMain()
{
    if(rtc){
        deleteRTC(true);
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
        instanceBaseName = self->defaultRTCInstanceName();
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

    string option = 
        str(format("instance_name=%1%&exec_cxt.periodic.type=%2%&exec_cxt.periodic.rate=%3%")
            % rtcInstanceName % execContextType.selectedSymbol() % periodicRateProperty);
    RTC::RtcBase* rtc = createManagedRTC((moduleName + "?" + option).c_str());
    
    if(!rtc){
        mv->putln(MessageView::ERROR,
                  format(_("RTC \"%1%\" of %2% cannot be created by the RTC manager.\n"
                           " RTC module file: \"%3%\"\n"
                           " Init function: %4%\n"
                           " option: %5%"))
                  % moduleName % self->name() % moduleFilename % initFuncName % option);
        return false;
    }

    execContext = OpenRTM::ExtTrigExecutionContextService::_nil();
    isChoreonoidExecutionContext = false; 
    RTC::ExecutionContextList_var eclist = rtc->get_owned_contexts();
    for(CORBA::ULong i=0; i < eclist->length(); ++i){
        if(!CORBA::is_nil(eclist[i])){
            execContext = OpenRTM::ExtTrigExecutionContextService::_narrow(eclist[i]);
            isChoreonoidExecutionContext = execContextType.is(CHOREONOID_EXECUTION_CONTEXT);
            /*
            SDOPackage::NVList& properties = rtc->get_component_profile()->properties;
            const char* ec_type(0);
            NVUtil::find(properties, "exec_cxt.periodic.type") >>= ec_type;
            cout << "ec_type: " << ec_type << endl;
            */
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
            if(RTC::PRECONDITION_NOT_MET == execContext->activate_component(rtc->getObjRef())){
                execContext->reset_component(rtc->getObjRef());
                execContext->tick();
                execContext->activate_component(rtc->getObjRef());
            }
            execContext->tick();
            isReady = true;
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
    execContext->deactivate_component(rtc->getObjRef());
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
                [&](const std::string& name){ return setRTCModule(name); });

    putProperty(_("Relative path base"), relativePathBaseType,
                [&](int which){ return setRelativePathBaseType(which); });
    
    putProperty(_("RTC Instance name"), rtcInstanceNameProperty,
                [&](const std::string& name) { return setRTCInstanceName(name); });

    putProperty(_("Execution context"), execContextType,
                [&](int which){ return setExecContextType(which); });

    putProperty.decimals(3)(_("Periodic rate"), periodicRateProperty,
                            changeProperty(periodicRateProperty));
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
    archive.write("executionContext", execContextType.selectedSymbol(), DOUBLE_QUOTED);
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
    if(archive.read("executionContext", symbol)){
        execContextType.select(symbol);
    }
    archive.read("periodicRate", periodicRateProperty);
    return true;
}
