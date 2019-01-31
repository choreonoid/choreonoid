/**
   @author Shin'ichiro Nakaoka
*/

#include "OpenHRPInterpreterServiceItem.h"
#include <cnoid/corba/OpenHRP/3.1/InterpreterService.hh>
#include <cnoid/ItemManager>
#include <cnoid/Archive>
#include <cnoid/MessageView>
#include <cnoid/OpenRTMUtil>
#include <cnoid/LazyCaller>
#include <cnoid/ScriptItem>
#include <rtm/DataFlowComponentBase.h>
#include <rtm/CorbaPort.h>
#include <fmt/format.h>
#include "gettext.h"

using namespace std;
using namespace std::placeholders;
using namespace cnoid;
using fmt::format;

namespace {
const bool TRACE_FUNCTIONS = false;

class InterpreterService_impl
    : public virtual POA_OpenHRP::InterpreterService,
      public virtual PortableServer::RefCountServantBase
{
public:
    char* interpret(const char* expr);
    void interpretMain(const char* expr);
        
    OpenHRPInterpreterServiceItemImpl* itemImpl;    
    string result;
};

class InterpreterRTC : public RTC::DataFlowComponentBase
{
public:
    InterpreterRTC(RTC::Manager* manager);
    virtual ~InterpreterRTC();
    virtual RTC::ReturnCode_t onInitialize();

    RTC::CorbaPort interpreterServicePort;
    InterpreterService_impl interpreterService;
};
}

namespace cnoid {

class OpenHRPInterpreterServiceItemImpl
{
public:
    OpenHRPInterpreterServiceItem* self;
    InterpreterRTC* rtc;
    string rtcInstanceName;
    Connection scriptItemUpdateConnection;
    ScriptItem* scriptItem;
    bool isScriptItemBackgroundMode;
    bool forceMainThreadExecution;
    bool doPutScriptTextToInterpret;
    ostream& os;

    OpenHRPInterpreterServiceItemImpl(OpenHRPInterpreterServiceItem* self);
    OpenHRPInterpreterServiceItemImpl(
        OpenHRPInterpreterServiceItem* self, const OpenHRPInterpreterServiceItemImpl& org);
    ~OpenHRPInterpreterServiceItemImpl();

    void setRTCinstanceName(const std::string& name);
    bool createRTC();
    bool deleteRTC();
    void onScriptItemUpdated();
};
typedef OpenHRPInterpreterServiceItemImpl ItemImpl;
}


void OpenHRPInterpreterServiceItem::initializeClass(ExtensionManager* ext)
{
    static const char* spec[] = {
        "implementation_id", "OpenHRPInterpreterService",
        "type_name",         "OpenHRPInterpreterService",
        "description",       "Component for accessing the python interpreter executing a script",
        "version",           "1.0",
        "vendor",            "AIST",
        "category",          "Choreonoid",
        "activity_type",     "DataFlowComponent",
        "max_instance",      "100",
        "language",          "C++",
        "lang_type",         "compile",
        ""
    };

    RTC::Properties profile(spec);
    RTC::Manager::instance().registerFactory(
        profile, RTC::Create<InterpreterRTC>, RTC::Delete<InterpreterRTC>);

    ext->itemManager()
        .registerClass<OpenHRPInterpreterServiceItem>(N_("OpenHRPInterpreterServiceItem"))
        .addCreationPanel<OpenHRPInterpreterServiceItem>();
}


OpenHRPInterpreterServiceItem::OpenHRPInterpreterServiceItem()
{
    setName("OpenHRPInterpreterService");
    impl = new ItemImpl(this);
}


ItemImpl::OpenHRPInterpreterServiceItemImpl(OpenHRPInterpreterServiceItem* self)
    : self(self),
      os(MessageView::instance()->cout())
{
    rtc = 0;
    scriptItem = 0;
    isScriptItemBackgroundMode = false;
    forceMainThreadExecution = false;
    doPutScriptTextToInterpret = false;
}


OpenHRPInterpreterServiceItem::OpenHRPInterpreterServiceItem(const OpenHRPInterpreterServiceItem& org)
    : Item(org)
{
    impl = new ItemImpl(this, *org.impl);
}


ItemImpl::OpenHRPInterpreterServiceItemImpl(OpenHRPInterpreterServiceItem* self, const ItemImpl& org)
    : self(self),
      forceMainThreadExecution(org.forceMainThreadExecution),
      os(MessageView::instance()->cout())
{
    rtc = 0;
    scriptItem = 0;
    isScriptItemBackgroundMode = org.isScriptItemBackgroundMode;
    doPutScriptTextToInterpret = org.doPutScriptTextToInterpret;
}
    

OpenHRPInterpreterServiceItem::~OpenHRPInterpreterServiceItem()
{
    delete impl;
}


OpenHRPInterpreterServiceItemImpl::~OpenHRPInterpreterServiceItemImpl()
{
    deleteRTC();
}


Item* OpenHRPInterpreterServiceItem::doDuplicate() const
{
    return new OpenHRPInterpreterServiceItem(*this);
}


void OpenHRPInterpreterServiceItem::setRTCInstanceName(const std::string& name)
{
    impl->setRTCinstanceName(name);
}
    

void ItemImpl::setRTCinstanceName(const std::string& name)
{
    if(rtcInstanceName != name){
        rtcInstanceName = name;
        if(self->findRootItem()){
            createRTC();
        }
    }
}


bool ItemImpl::createRTC()
{
    if(rtc){
        deleteRTC();
    }
    if(rtc || rtcInstanceName.empty()){
        return false;
    }
    
    string param("OpenHRPInterpreterService?"
                 "instance_name={}&"
                 "exec_cxt.periodic_type=PeriodicExecutionContext&"
                 "exec_cxt.periodic.rate=10");
    
    rtc = dynamic_cast<InterpreterRTC*>(cnoid::createManagedRTC(format(param, rtcInstanceName)));
    
    if(rtc){
        rtc->interpreterService.itemImpl = this;
        
        os << format(_("RTC \"{0}\" of \"{1}\" has been created."),
                     rtcInstanceName, self->name()) << endl;
    } else {
        os << format(_("RTC \"{0}\" of \"{1}\" cannot be created."),
                     rtcInstanceName, self->name()) << endl;
    }
    
    return (rtc != 0);
}


bool ItemImpl::deleteRTC()
{
    if(rtc){
        if(cnoid::deleteRTC(rtc)){
            os << format(_("RTC \"{0}\" of \"{1}\" has been deleted."),
                         rtcInstanceName, self->name()) << endl;
            rtc = 0;
        } else {
            os << format(_("RTC \"{0}\" of \"{1}\" cannot be deleted."),
                         rtcInstanceName, self->name()) << endl;
        }
    }

    return (rtc == 0);
}


void OpenHRPInterpreterServiceItem::onConnectedToRoot()
{
    impl->createRTC();
}


void OpenHRPInterpreterServiceItem::onPositionChanged()
{
    impl->scriptItemUpdateConnection.disconnect();
    impl->scriptItem = findOwnerItem<ScriptItem>();
    if(impl->scriptItem){
        impl->onScriptItemUpdated();
        impl->scriptItemUpdateConnection =
            impl->scriptItem->sigUpdated().connect(std::bind(&ItemImpl::onScriptItemUpdated, impl));
    }
}


void ItemImpl::onScriptItemUpdated()
{
    // This property is stored to avoid the access from a background thread
    isScriptItemBackgroundMode = scriptItem->isBackgroundMode();
}


void OpenHRPInterpreterServiceItem::onDisconnectedFromRoot()
{
    impl->deleteRTC();
}


void OpenHRPInterpreterServiceItem::doPutProperties(PutPropertyFunction& putProperty)
{
    putProperty(_("RTC Instance name"), impl->rtcInstanceName,
                std::bind(&ItemImpl::setRTCinstanceName, impl, _1), true);
    putProperty(_("Force main thread execution"), impl->forceMainThreadExecution,
                changeProperty(impl->forceMainThreadExecution));
    putProperty(_("Put script text to interpret"), impl->doPutScriptTextToInterpret,
                changeProperty(impl->doPutScriptTextToInterpret));
}


bool OpenHRPInterpreterServiceItem::store(Archive& archive)
{
    archive.write("rtcInstance", impl->rtcInstanceName);
    archive.write("forceMainThreadExecution", impl->forceMainThreadExecution);
    archive.write("putScriptText", impl->doPutScriptTextToInterpret);
    return true;
}


bool OpenHRPInterpreterServiceItem::restore(const Archive& archive)
{
    impl->setRTCinstanceName(archive.get("rtcInstance", impl->rtcInstanceName));
    archive.read("forceMainThreadExecution", impl->forceMainThreadExecution);
    archive.read("putScriptText", impl->doPutScriptTextToInterpret);
    return true;
}


InterpreterRTC::InterpreterRTC(RTC::Manager* manager)
    : RTC::DataFlowComponentBase(manager),
      interpreterServicePort("InterpreterService")
{

}
      

InterpreterRTC:: ~InterpreterRTC()
{

}


RTC::ReturnCode_t InterpreterRTC::onInitialize()
{
    interpreterServicePort.registerProvider("service0", "InterpreterService", interpreterService);
    addPort(interpreterServicePort);
    return RTC::RTC_OK;
}


char* InterpreterService_impl::interpret(const char* expr)
{
    result.clear();

    if(!itemImpl->isScriptItemBackgroundMode || itemImpl->forceMainThreadExecution){
        callSynchronously(std::bind(&InterpreterService_impl::interpretMain, this, expr));
    } else {
        interpretMain(expr);
    }

    CORBA::String_var ret(result.c_str());
    return ret._retn();;
}


void InterpreterService_impl::interpretMain(const char* expr)
{
    ostream& os = MessageView::instance()->cout();

    Item* item = itemImpl->self;

    if(itemImpl->doPutScriptTextToInterpret){
        os << format(_("{0}: interpret(\"{1}\")"), item->name(), expr) << endl;
    }

    ScriptItem* scriptItem = item->findOwnerItem<ScriptItem>();
    if(!scriptItem){
        os << format(_("The owner script item of {} is not found. The interpret function cannot be executed."),
                     item->name()) << endl;
    } else {
        if(scriptItem->isRunning()){
            os << format(_("Owner script item \"{}\" is running now. The interpret function cannot be executed."),
                         scriptItem->name()) << endl;
        } else {
            if(!scriptItem->executeCode(expr)){
                os << _("Executing the script given to the interpret function failed.") << endl;
            } else {
                if(!scriptItem->waitToFinish()){
                    os << _("The script does not return.") << endl;
                } else {
                    result = scriptItem->resultString();
                    if(itemImpl->doPutScriptTextToInterpret){
                        if(!result.empty()){
                            os << format(_("{0}: interpret() returns {1}."), item->name(), result) << endl;
                        } else {
                            os << format(_("{}: interpret() finished."), item->name()) << endl;
                        }
                    }
                }
            }
        }
    }
}
