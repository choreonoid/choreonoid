/**
   @author Shin'ichiro Nakaoka
*/

#include "BodyIoRTCItem.h"
#include "OpenRTMUtil.h"
#include <cnoid/BodyIoRTC>
#include <cnoid/ItemManager>
#include <cnoid/BodyItem>
#include <cnoid/MessageView>
#include <cnoid/Archive>
#include <cnoid/Sleep>
#include <cnoid/FileUtil>
#include <cnoid/ExecutablePath>
#include <cnoid/ProjectManager>
#include <rtm/CorbaNaming.h>
#include <boost/regex.hpp>
#include "gettext.h"

using namespace std;
using namespace cnoid;
using boost::format;
namespace filesystem = boost::filesystem;

namespace {

const bool TRACE_FUNCTIONS = false;

}

namespace cnoid {

class BodyIoRTCItemImpl
{
public:
    BodyIoRTCItem* self;
    MessageView* mv;
    BodyItem* bodyItem = 0;

    std::string moduleNameProperty;
    std::string moduleName;
    std::string moduleFilename;
    std::string rtcInstanceName;
    BodyIoRTC* bodyIoRTC = 0;
    OpenRTM::ExtTrigExecutionContextService_var execContext;

    double executionCycleProperty;
    double executionCycle;
    double executionCycleCounter;

    enum PathBase {
        RTC_DIRECTORY,
        PROJECT_DIRECTORY,
        N_PATH_BASE
    };
    Selection pathBase;
    int oldPathBase;
    
    BodyIoRTCItemImpl(BodyIoRTCItem* self);
    BodyIoRTCItemImpl(BodyIoRTCItem* self, const BodyIoRTCItemImpl& org);
    std::string getModuleFilename();
    bool createBodyIoRTC();

    void deleteBodyIoRTC(bool waitToBeDeleted);
    void deleteRTC(RTC::RtcBase*& rtc, bool waitToBeDeleted);        
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
      pathBase(N_PATH_BASE, CNOID_GETTEXT_DOMAIN_NAME)
{
    pathBase.setSymbol(RTC_DIRECTORY, N_("RTC directory"));
    pathBase.setSymbol(PROJECT_DIRECTORY, N_("Project directory"));
    pathBase.select(RTC_DIRECTORY);
    oldPathBase = RTC_DIRECTORY;

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
    pathBase = org.pathBase;
    oldPathBase = org.oldPathBase;
    moduleNameProperty = org.moduleNameProperty;
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
    // This is not necessary because onPositionChanged() is also called
    // when the item is disconnected from the root
    impl->deleteBodyIoRTC(false);
}


void BodyIoRTCItem::onPositionChanged()
{
    // create or recreate an RTC corresponding to the body
    // The target body can be detected like this:

#if 0
    BodyItem* ownerBodyItem = findOwnerItem<BodyItem>();
    if(ownerBodyItem){
        Body* body = ownerBodyItem->body();
        if(bodyName != body->name()){
            bodyName = body->name();
            deleteModule(true);
            createRTC(body);
        }
    } else {
        deleteModule(false);
        bodyName.clear();
    }
#endif
}


std::string BodyIoRTCItemImpl::getModuleFilename()
{
    if(moduleNameProperty.empty()){
        return string();
    }
    
    filesystem::path path(moduleNameProperty);

    moduleName = path.leaf().basename();
    
    if(!checkAbsolute(path)){
        if(pathBase.is(RTC_DIRECTORY)){
            modulePath =
                filesystem::path(executableTopDirectory()) / CNOID_PLUGIN_SUBDIR / "rtc" / path;
        } else {
            const string& projectFileName = ProjectManager::instance()->getProjectFileName();
            if(projectFileName.empty()){{
                    mv->putln(_("Please save the project."));
                    return string();
                } else {
                    path = filesystem::path(projectFileName).parent_path() / path;
                }
            }
        }
    }

    string ext = path.extension();
    if(ext.empty()){
        path /= DLL_EXTENSION;
    } else if(ext != DLL_EXTENSION){
        return false;
    }
        
    if(filesystem::exists(path)){
        return getNativePathString(path);
    }

    mv->putln(fmt(_("A file of RTC \"%1%\" does not exist.")) % componentName);
    return string();
}


bool BodyIoRTCItemImpl::createBodyIoRTC()
{
    if(!bodyItem){
        return false;
    }
    
    string moduleFilename = getModuleFilename();
    if(moduleFilename.empty()){
        return false;
    }
    string initFuncName = moduleName + "Init";
    auto& manager = RTC::Manager::instance();
    manager.load(moduleFilename.c_str(), initFuncName.c_str());

    auto instanceBaseName = bodyItem->name();
    if(instanceBaseName.empty()){
        instanceBaseName = bodyItem->body->modelName();
    }
    if(instanceBaseName.empty()){
        instanceBaseName = self->name();
    }
    if(instanceBaseName.empty()){
        instanceBaseName = "BodyIO";
    }
    rtcInstanceName = instanceBaseName;

    const int maxNumInstances = 10000;
    int index = 2;
    for(int index = 2; index < maxNumInstances; ++index){
        if(manager.getComponent(rtcInstanceName.c_str()) == nullptr){
            break;
        }
        rtcInstanceName = str(format("%1%(%2%)") % instanceBaseName & index++);
    }
    if(index >= maxNumInstances){
        mv->putln(MessageView::ERROR,
                  _("RTC \"%1%\" of %2% is not created because more than %3% existing instances have the same base name \"%4%\".")
                  % moduleName % self->name() % maxNumInstances % instanceBaseName);
        return false;
    }

    string option = 
        str(format("instance_name=%1%&exec_cxt.periodic.type=ChoreonoidExecutionContext&exec_cxt.periodic.rate=1000000")
            % rtcInstanceName);
    RtcBase* rtc = createManagedRTC((moduleName + "?" + option).c_str());
    
    if(!rtc){
        mv->putln(MessageView::ERROR,
                  _("RTC \"%1%\" of %2% cannot be created by the RTC manager.\n"
                    " RTC module file: \"%3%\"\n"
                    " Init function: %4%\n"
                    " option: %5%")
                  % moduleName % self->name() % moduleFilename % initFuncName % option);
        return false;
    }

    bodyIoRTC = dynamic_cast<BodyIoRTC*>(rtc);
    if(!bodyIoRTC){
        mv->putln(MessageView::ERROR,
                  _("RTC \"%1%\" of %2% cannot be used as BodyIoRTC because it is not derived from it.")
                  % moduleName % self->name());
        deleteRTC(rtc, false);
        return false;
    }

    mv->putln(fmt(_("BodyIoRTC \"%1%\" has been created.")) % rtcInstanceName);

    execContext = OpenRTM::ExtTrigExecutionContextService::_nil();
    RTC::ExecutionContextList_var eclist = virtualRobotRTC->get_owned_contexts();
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
}


void BodyIoRTCItemImpl::deleteRTC(RTC::RtcBase*& rtc, bool waitToBeDeleted)
{
    if(rtc){
        bool deleted = false;
        deleted = cnoid::deleteRTC(rtc);
        rtc = 0;

        if(waitToBeDeleted){
            RTC::Manager& manager = RTC::Manager::instance();
            for(int i=0; i < 100; i++){
                if(!manager.getComponent(rtcInstanceName)){
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
    return false;
}


double BodyIoRTCItem::timeStep() const
{
    return 0.0;
}


void BodyIoRTCItem::input()
{

}


bool BodyIoRTCItem::control()
{
    return true;
}


void BodyIoRTCItem::output()
{

}


void BodyIoRTCItem::stop()
{

}


void BodyIoRTCItem::doPutProperties(PutPropertyFunction& putProperty)
{

}


bool BodyIoRTCItem::store(Archive& archive)
{
    return true;
}


bool BodyIoRTCItem::restore(const Archive& archive)
{
    return true;
}

