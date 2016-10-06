/**
   @author Shin'ichiro Nakaoka
*/

#include "BodyIoRTCItem.h"
#include "OpenRTMUtil.h"
#include <cnoid/BodyIoRTC>
#include <cnoid/ItemManager>
#include <cnoid/BodyItem>
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

namespace {

const bool TRACE_FUNCTIONS = false;

}

namespace cnoid {

class BodyIoRTCItemImpl
{
public:
    BodyIoRTCItem* self;
    BodyItem* bodyItem = 0;
    BodyIoRTC* bodyIoRTC = 0;
    OpenRTM::ExtTrigExecutionContextService_var execContext;

    double executionCycleProperty;
    double executionCycle;
    double executionCycleCounter;

    std::string moduleNameProperty;
    std::string moduleName;
    std::string moduleFilename;
    std::string rtcInstanceNameProperty;
    std::string rtcInstanceName;

    enum PathBase {
        RTC_DIRECTORY,
        PROJECT_DIRECTORY,
        N_PATH_BASE
    };
    Selection relativePathBaseType;

    MessageView* mv;
    
    BodyIoRTCItemImpl(BodyIoRTCItem* self);
    BodyIoRTCItemImpl(BodyIoRTCItem* self, const BodyIoRTCItemImpl& org);
    void setBodyItem(BodyItem* newBodyItem, bool forceReset);
    bool setRelativePathBaseType(int which);
    bool setRTCModule(const std::string& name);
    bool setRTCInstanceName(const std::string& name);
    std::string getModuleFilename();
    bool resetBodyIoRTC();
    void deleteBodyIoRTC(bool waitToBeDeleted);
    void deleteRTC(RTC::RtcBase* rtc, bool waitToBeDeleted);
    bool start();
    void stop();
    void doPutProperties(PutPropertyFunction& putProperty);
    bool store(Archive& archive);
    bool restore(const Archive& archive);
};

}


void BodyIoRTCItem::initialize(ExtensionManager* ext)
{
    static bool initialized = false;
    if(!initialized){
        ext->itemManager().registerClass<BodyIoRTCItem>(N_("BodyIoRTCItem"));
        ext->itemManager().addCreationPanel<BodyIoRTCItem>();
        initialized = true;
    }
}


BodyIoRTCItem::BodyIoRTCItem()
{
    impl = new BodyIoRTCItemImpl(this);
}


BodyIoRTCItemImpl::BodyIoRTCItemImpl(BodyIoRTCItem* self)
    : self(self),
      mv(MessageView::instance()),
      relativePathBaseType(N_PATH_BASE, CNOID_GETTEXT_DOMAIN_NAME)
{
    relativePathBaseType.setSymbol(RTC_DIRECTORY, N_("RTC directory"));
    relativePathBaseType.setSymbol(PROJECT_DIRECTORY, N_("Project directory"));
    relativePathBaseType.select(RTC_DIRECTORY);

    executionCycleProperty = 0.0;
}


BodyIoRTCItem::BodyIoRTCItem(const BodyIoRTCItem& org)
    : ControllerItem(org)
{
    impl = new BodyIoRTCItemImpl(this, *org.impl);
}


BodyIoRTCItemImpl::BodyIoRTCItemImpl(BodyIoRTCItem* self, const BodyIoRTCItemImpl& org)
    : BodyIoRTCItemImpl(self)
{
    relativePathBaseType = org.relativePathBaseType;
    moduleNameProperty = org.moduleNameProperty;
    rtcInstanceNameProperty = org.rtcInstanceNameProperty;
    executionCycleProperty = org.executionCycleProperty;
}


BodyIoRTCItem::~BodyIoRTCItem()
{
    delete impl;
}


Item* BodyIoRTCItem::doDuplicate() const
{
    return new BodyIoRTCItem(*this);
}


void BodyIoRTCItem::onConnectedToRoot()
{

}


void BodyIoRTCItem::onDisconnectedFromRoot()
{
    impl->deleteBodyIoRTC(false);
    impl->bodyItem = 0;
}


void BodyIoRTCItem::onPositionChanged()
{
    impl->setBodyItem(findOwnerItem<BodyItem>(), false);
}


void BodyIoRTCItemImpl::setBodyItem(BodyItem* newBodyItem, bool forceReset)
{
    if(newBodyItem != bodyItem || forceReset){
        bodyItem = newBodyItem;
        resetBodyIoRTC();
    }
}


bool BodyIoRTCItemImpl::setRelativePathBaseType(int which)
{
    if(which != relativePathBaseType.which()){
        relativePathBaseType.select(which);
        resetBodyIoRTC();
    }
    return true;
}


bool BodyIoRTCItemImpl::setRTCModule(const std::string& name)
{
    if(name != moduleNameProperty){
        moduleNameProperty = name;
        resetBodyIoRTC();
    }
    return true;
}


bool BodyIoRTCItemImpl::setRTCInstanceName(const std::string& name)
{
    if(name != rtcInstanceNameProperty){
        rtcInstanceNameProperty = name;
        resetBodyIoRTC();
    }
    return true;
}


std::string BodyIoRTCItemImpl::getModuleFilename()
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


bool BodyIoRTCItemImpl::resetBodyIoRTC()
{
    if(!bodyItem){
        return false;
    }
    
    if(bodyIoRTC){
        deleteBodyIoRTC(true);
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
        instanceBaseName = bodyItem->name();
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
        str(format("instance_name=%1%&exec_cxt.periodic.type=ChoreonoidExecutionContext&exec_cxt.periodic.rate=1000000")
            % rtcInstanceName);
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

    bodyIoRTC = dynamic_cast<BodyIoRTC*>(rtc);
    if(!bodyIoRTC){
        mv->putln(MessageView::ERROR,
                  format(_("RTC \"%1%\" of %2% cannot be used as a BodyIoRTC because it is not derived from it."))
                  % moduleName % self->name());
        deleteRTC(rtc, false);
        return false;
    }
    if(bodyIoRTC->onInitialize(bodyItem->body()) != RTC::RTC_OK){
        mv->putln(MessageView::ERROR,
                  format(_("RTC \"%1%\" of %2% failed to initialize."))
                  % moduleName % self->name());
        deleteBodyIoRTC(true);
        return false;
    }

    mv->putln(fmt(_("BodyIoRTC \"%1%\" has been created.")) % rtcInstanceName);

    execContext = OpenRTM::ExtTrigExecutionContextService::_nil();
    RTC::ExecutionContextList_var eclist = bodyIoRTC->get_owned_contexts();
    for(CORBA::ULong i=0; i < eclist->length(); ++i){
        if(!CORBA::is_nil(eclist[i])){
            execContext = OpenRTM::ExtTrigExecutionContextService::_narrow(eclist[i]);
            break;
        }
    }

    return true;
}


void BodyIoRTCItemImpl::deleteBodyIoRTC(bool waitToBeDeleted)
{
    deleteRTC(bodyIoRTC, waitToBeDeleted);
    bodyIoRTC = 0;
}


void BodyIoRTCItemImpl::deleteRTC(RTC::RtcBase* rtc, bool waitToBeDeleted)
{
    if(rtc){
        bool deleted = false;
        deleted = cnoid::deleteRTC(rtc);

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


bool BodyIoRTCItem::initialize(ControllerItemIO* io)
{
    if(impl->bodyIoRTC){
        return impl->bodyIoRTC->initializeSimulation(io);
    }
    return false;
}


bool BodyIoRTCItem::start()
{
    return impl->start();
}


bool BodyIoRTCItemImpl::start()
{
    bool isReady = false;
    
    if(bodyIoRTC){
        if(bodyIoRTC->startSimulation()){
            if(!CORBA::is_nil(execContext)){
                if(RTC::PRECONDITION_NOT_MET == execContext->activate_component(bodyIoRTC->getObjRef())){
                    execContext->reset_component(bodyIoRTC->getObjRef());
                    execContext->tick();
                    execContext->activate_component(bodyIoRTC->getObjRef());
                }
                execContext->tick();
                //bodyIoRTC->inputFromSimulator();
                isReady = true;
            }
        }
    }
    
    return isReady;
}


double BodyIoRTCItem::timeStep() const
{
    return 0.0;
}


void BodyIoRTCItem::input()
{
    impl->bodyIoRTC->inputFromSimulator();
}


bool BodyIoRTCItem::control()
{
    if(!CORBA::is_nil(impl->execContext)){
        impl->execContext->tick();
    }

    return true;
}


void BodyIoRTCItem::output()
{
    impl->bodyIoRTC->outputToSimulator();
}


void BodyIoRTCItem::stop()
{
    impl->stop();
}


void BodyIoRTCItemImpl::stop()
{
    bodyIoRTC->stopSimulation();
    execContext->deactivate_component(bodyIoRTC->getObjRef());
    execContext->tick();
}


void BodyIoRTCItem::doPutProperties(PutPropertyFunction& putProperty)
{
    ControllerItem::doPutProperties(putProperty);
    impl->doPutProperties(putProperty);
}


void BodyIoRTCItemImpl::doPutProperties(PutPropertyFunction& putProperty)
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

    putProperty.decimals(3)(_("Periodic rate"), executionCycleProperty,
                            changeProperty(executionCycleProperty));
}


bool BodyIoRTCItem::store(Archive& archive)
{
    return ControllerItem::store(archive) && impl->store(archive);
}


bool BodyIoRTCItemImpl::store(Archive& archive)
{
    archive.writeRelocatablePath("moduleName", moduleNameProperty);
    archive.write("pathBaseType", relativePathBaseType.selectedSymbol(), DOUBLE_QUOTED);
    archive.write("instanceName", rtcInstanceNameProperty, DOUBLE_QUOTED);
    archive.write("bodyPeriodicRate", executionCycleProperty);
    return true;
}


bool BodyIoRTCItem::restore(const Archive& archive)
{
    return ControllerItem::restore(archive) && impl->restore(archive);
}


bool BodyIoRTCItemImpl::restore(const Archive& archive)
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
    archive.read("bodyPeriodicRate", executionCycleProperty);
    return true;
}
