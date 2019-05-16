/*!
  @file
  @author Shin'ichiro Nakaoka
*/

#include "RTCItem.h"
#include "ControllerRTCItem.h"
#include "BodyIoRTCItem.h"
#include "SimulationExecutionContext.h"
#include "SimulationPeriodicExecutionContext.h"
#include "OpenRTMUtil.h"
#include "RTSNameServerView.h"
#include "RTSPropertiesView.h"
#include "RTSConfigurationView.h"
#include "RTMImageView.h"

#ifdef ENABLE_NEW_RT_SYSTEM_ITEM_IMPLEMENTATION
#include "experimental/RTSDiagramViewEx.h"
#include "experimental/RTSystemItemEx.h"
#else
#include "RTSDiagramView.h"
#include "RTSystemItem.h"
#endif

#include "RTSCommonUtil.h"
#include "BodyStateSubscriberRTCItem.h"
#include "deprecated/VirtualRobotRTC.h"
#include "deprecated/BodyRTCItem.h"
#include "deprecated/ChoreonoidExecutionContext.h"
#include "deprecated/ChoreonoidPeriodicExecutionContext.h"
#include "deprecated/PointCloudSubscriberRTCItem.h"
#include <cnoid/Plugin>
#include <cnoid/ItemManager>
#include <cnoid/Archive>
#include <cnoid/MessageView>
#include <cnoid/MenuManager>
#include <cnoid/CorbaPlugin>
#include <cnoid/SimulationBar>
#include <cnoid/Sleep>
#include <cnoid/FileUtil>
#include <QTcpSocket>
#include <cnoid/AppConfig>
#include <fmt/format.h>
#include <rtm/ComponentActionListener.h>
#include <set>

#include "LoggerUtil.h"

#include "gettext.h"

using namespace std;
using namespace cnoid;
using fmt::format;
namespace filesystem = boost::filesystem;

namespace {

class ManagerEx : public RTC::Manager
{
public:
    RTM::ManagerServant* servant()
    {
        return m_mgrservant;
    }
};

ManagerEx* manager;

std::set<RTC::RTObject_impl*> managedComponents;

Signal<void()> sigAboutToFinalizeRTM_;

class PostComponentShutdownListenr : public RTC::PostComponentActionListener
{
    RTC::RTObject_impl* rtc;
public:
    PostComponentShutdownListenr(RTC::RTObject_impl* rtc) : rtc(rtc) {}
    virtual void operator()(RTC::UniqueId ec_id, RTC::ReturnCode_t ret) {}
};

SettingDialog* settingInstance = 0;

class OpenRTMPlugin : public Plugin
{
    MessageView* mv;
    Action* deleteRTCsOnSimulationStartCheck;
    Connection connectionToSigSimulaionAboutToStart;

public:
    OpenRTMPlugin() : Plugin("OpenRTM")
    {
        require("Body");
        require("Corba");
        precede("Corba");

#ifdef LOG_OUT
        LoggerUtil::startLog(LogLevel::LOG_DEBUG, "Log");
#endif
    }

    template<typename ExecutionContextType>
    void registerExecutionContext(const char* name)
    {
#if defined(OPENRTM_VERSION11)
        if (manager->registerECFactory(
            name,
            RTC::ECCreate<ExecutionContextType>,
            RTC::ECDelete<ExecutionContextType>)) {
#elif defined(OPENRTM_VERSION12)
        if (RTC::ExecutionContextFactory::instance().addFactory(
            name,
            ::coil::Creator<::RTC::ExecutionContextBase, ExecutionContextType>,
            ::coil::Destructor<::RTC::ExecutionContextBase, ExecutionContextType>) == 0) {
#endif
            mv->putln(format(_("{} has been registered."), name));
        } else {
            mv->putln(format(_("Failed to register {}."), name), MessageView::WARNING);
        }
    }

    virtual bool initialize()
    {
        DDEBUG("initialize");
        string configFile = DEFAULT_CONF_FILENAME;
        string loggerEnable = "logger.enable: NO";
        string logLeval = "logger.log_level: WARN";
        MappingPtr appVars = AppConfig::archive()->openMapping("OpenRTM");
        if (appVars) {
            bool outputLog = appVars->get("outputLog", false);
            if (outputLog) {
                loggerEnable = "logger.enable: YES";
                logLeval = "logger.log_level:" + appVars->get("logLevel", "INFO");
            }
            //
            configFile = appVars->get("defaultSetting", DEFAULT_CONF_FILENAME);
        } else {
            appVars->write("defaultSetting", DEFAULT_CONF_FILENAME, DOUBLE_QUOTED);
            appVars->write("defaultVendor", "AIST", DOUBLE_QUOTED);
            appVars->write("defaultVersion", "1.0.0", DOUBLE_QUOTED);

#if defined(OPENRTM_VERSION12)
            appVars->write("heartBeatPeriod", 500);
#endif
            appVars->write("outputLog", false);
            appVars->write("logLevel", "INFO");
        }
        DDEBUG_V("configFile : %s", configFile.c_str());

        bool isConfFileSpecified = filesystem::exists(configFile);

        const char* argv[] = {
            "choreonoid",
            "-o", "manager.shutdown_on_nortcs: NO",
            "-o", "manager.shutdown_auto: NO",
            "-o", "naming.formats: %n.rtc",
            "-o", loggerEnable.c_str(),
            "-o", logLeval.c_str(),
#ifdef Q_OS_WIN32
            // To reduce the startup time on Windows
            "-o", "corba.args: -ORBclientCallTimeOutPeriod 100",
#endif
#if defined(OPENRTM_VERSION12)
            "-i",
#endif
            "-f", configFile.c_str(),
        };

#ifdef Q_OS_WIN32
#if defined(OPENRTM_VERSION11)
        int numArgs = 13;
#elif defined(OPENRTM_VERSION12)
        int numArgs = 14;
#endif
#else
        int numArgs = 11;
#endif
        if(isConfFileSpecified){
            numArgs += 2;
        }

        mv = MessageView::mainInstance();

        cnoid::checkOrInvokeCorbaNameServer();

        for (unsigned int index = 0; index < numArgs; index++) {
            DDEBUG_V("argv[%d]=%s", index, argv[index]);
        }

        manager = static_cast<ManagerEx*>(RTC::Manager::init(numArgs, const_cast<char**>(argv)));

        RTM::Manager_ptr servantRef = manager->servant()->getObjRef();
        if (CORBA::is_nil(servantRef)) {
            manager->servant()->createINSManager();
        }

        registerExecutionContext<SimulationExecutionContext>("SimulationExecutionContext");
        registerExecutionContext<SimulationPeriodicExecutionContext>("SimulationPeriodicExecutionContext");

        // Deprecated
        registerExecutionContext<ChoreonoidExecutionContext>("ChoreonoidExecutionContext");
        registerExecutionContext<ChoreonoidPeriodicExecutionContext>("ChoreonoidPeriodicExecutionContext");

        manager->activateManager();

#ifdef Q_OS_WIN32
        omniORB::setClientCallTimeout(0); // reset the global timeout setting?
#endif

        if (!cnoid::takeOverCorbaPluginInitialization(manager->getORB())) {
            return false;
        }

        BodyIoRTCItem::initialize(this);
        ControllerRTCItem::initialize(this);
        RTCItem::initialize(this);
        BodyRTCItem::initialize(this);

        VirtualRobotRTC::registerFactory(manager, "VirtualRobot");

        manager->runManager(true);

        menuManager().setPath("/Tools/OpenRTM").addItem(_("Delete unmanaged RT components"))
            ->sigTriggered().connect([&]() { deleteUnmanagedRTCs(true); });

        deleteRTCsOnSimulationStartCheck =
            menuManager().setPath("/Options/OpenRTM").addCheckItem(
                _("Delete unmanaged RT components on starting a simulation"));
        deleteRTCsOnSimulationStartCheck->sigToggled().connect(
            [&](bool on) { onDeleteRTCsOnSimulationStartToggled(on); });

        if (!settingInstance) {
            settingInstance = new SettingDialog();

            menuManager().setPath("/Tools/OpenRTM");
            menuManager().addItem(_("Preferences"))
                ->sigTriggered().connect([]() { settingInstance->show(); });
        }

        setProjectArchiver(
            [&](Archive& archive) { return store(archive); },
            [&](const Archive& archive) { restore(archive); });

        NameServerInfo info = RTCCommonUtil::getManagerAddress();
        if (!info.hostAddress.empty()) {
            NameServerManager::instance()->getNCHelper()->setLocation(info.hostAddress, info.portNo);
            DDEBUG_V("Init ncHelper host:%s, port:%d", info.hostAddress.c_str(), info.portNo);
        }

        RTSNameServerView::initializeClass(this);
        RTSPropertiesView::initializeClass(this);
        RTSConfigurationView::initializeClass(this);
        RTMImageView::initializeClass(this);

#ifdef ENABLE_NEW_RT_SYSTEM_ITEM_IMPLEMENTATION
#ifdef ENABLE_BACKGROUND_STATE_DETECTION
        RTSystemExt2Item::initializeClass(this);
        RTSDiagramExt2View::initializeClass(this);
#else
        RTSystemItemEx::initializeClass(this);
        RTSDiagramViewEx::initializeClass(this);
#endif
#else
        RTSystemItem::initializeClass(this);
        RTSDiagramView::initializeClass(this);
#endif

        BodyStateSubscriberRTCItem::initializeClass(this);
        PointCloudSubscriberRTCItem::initializeClass(this);

        DDEBUG("initialize Finished");

        return true;
    }


    bool store(Archive& archive)
    {
        archive.write("deleteUnmanagedRTCsOnStartingSimulation", deleteRTCsOnSimulationStartCheck->isChecked());
        return true;
    }


    void restore(const Archive& archive)
    {
        bool checked = deleteRTCsOnSimulationStartCheck->isChecked();
        if (!archive.read("deleteUnmanagedRTCsOnStartingSimulation", checked)) {
            // for reading the old version format
            const Archive& oldNode = *archive.findSubArchive("OpenRTMPlugin");
            if (oldNode.isValid()) {
                oldNode.read("deleteUnmanagedRTCsOnStartingSimulation", checked);
            }
        }
        deleteRTCsOnSimulationStartCheck->setChecked(checked);
    }



    void onDeleteRTCsOnSimulationStartToggled(bool on)
    {
        connectionToSigSimulaionAboutToStart.disconnect();
        if (on) {
            connectionToSigSimulaionAboutToStart =
                SimulationBar::instance()->sigSimulationAboutToStart().connect(
                    [&](SimulatorItem*) { deleteUnmanagedRTCs(false); });
        }
    }


    void onSimulationAboutToStart()
    {
        if (deleteUnmanagedRTCs(false) > 0) {
            mv->flush();
        }
    }

    int deleteUnmanagedRTCs(bool doPutMessageWhenNoUnmanagedComponents)
    {
        DDEBUG("deleteUnmanagedRTCs");
        int n = cnoid::numUnmanagedRTCs();

        if (n == 0) {
            if (doPutMessageWhenNoUnmanagedComponents) {
                mv->notify("There are no RT components which are not managed by Choreonoid.");
            }
        } else {
            if (n == 1) {
                mv->notify(_("An RT component which is not managed by Choreonoid is being deleted."));
            } else {
                mv->notify(
                    format(_("{} RT components which are not managed by Choreonoid are being deleted."),
                           n));
            }
            mv->flush();
            cnoid::deleteUnmanagedRTCs();
            if (n == 1) {
                mv->notify(_("The unmanaged RT component has been deleted."));
            } else {
                mv->notify(_("The unmanaged RT components have been deleted."));
            }
        }

        return n;
    }


    virtual bool finalize()
    {
        DDEBUG("finalize");
        sigAboutToFinalizeRTM_();

        connectionToSigSimulaionAboutToStart.disconnect();

        std::vector<RTC::RTObject_impl*> rtcs = manager->getComponents();
        for (size_t i = 0; i < rtcs.size(); ++i) {
            RTC::RTObject_impl* rtc = rtcs[i];
            RTC::ExecutionContextList_var eclist = rtc->get_participating_contexts();
            if (eclist->length() > 0) {
                for (CORBA::ULong j = 0; j < eclist->length(); ++j) {
                    if (!CORBA::is_nil(eclist[j])) {
                        eclist[j]->remove_component(rtc->getObjRef());
                    }
                }
            }
        }

        // delete all the components owned by exisiting BodyRTCItems
        itemManager().detachAllManagedTypeItemsFromRoot();

        cnoid::deleteUnmanagedRTCs();

        manager->shutdown();
        manager->unloadAll();

        return true;
    }

};

}

CNOID_IMPLEMENT_PLUGIN_ENTRY(OpenRTMPlugin);


SignalProxy<void()> cnoid::sigAboutToFinalizeRTM()
{
    return sigAboutToFinalizeRTM_;
}


/*
RTM::Manager_ptr cnoid::getRTCManagerServant()
{
    return RTM::Manager::_duplicate(manager->servant()->getObjRef());
}
*/


RTC::RTObject_impl* cnoid::createManagedRTC(const std::string& comp_args)
{
    DDEBUG("createManagedRTC");
    RTC::RTObject_impl* rtc = manager->createComponent(comp_args.c_str());
    if (rtc) {
        managedComponents.insert(rtc);
        //rtc->addPostComponentActionListener(POST_ON_SHUTDOWN, new PostComponentShutdownListenr(rtc));
    }
    return rtc;
}


CNOID_EXPORT int cnoid::numUnmanagedRTCs()
{
    int n = 0;
    std::vector<RTC::RTObject_impl*> rtcs = manager->getComponents();
    for (size_t i = 0; i < rtcs.size(); ++i) {
        RTC::RTObject_impl* rtc = rtcs[i];
        if (managedComponents.find(rtc) == managedComponents.end()) {
            ++n;
        }
    }
    return n;
}


CNOID_EXPORT int cnoid::deleteUnmanagedRTCs()
{
    DDEBUG("deleteUnmanagedRTCs");
    int numDeleted = 0;

    std::vector<RTC::RTObject_impl*> rtcs = manager->getComponents();

    //  deactivate
    for (size_t i = 0; i < rtcs.size(); ++i) {
        RTC::RTObject_impl* rtc = rtcs[i];
        if (managedComponents.find(rtc) == managedComponents.end()) {
            RTC::ExecutionContextList_var eclist = rtc->get_owned_contexts();
            if (eclist->length() > 0 && !CORBA::is_nil(eclist[0])) {
                eclist[0]->deactivate_component(rtc->getObjRef());
                OpenRTM::ExtTrigExecutionContextService_var execContext = OpenRTM::ExtTrigExecutionContextService::_narrow(eclist[0]);
                if (!CORBA::is_nil(execContext)) {
                    execContext->tick();
                }
            }
            eclist = rtc->get_participating_contexts();
            if (eclist->length() > 0 && !CORBA::is_nil(eclist[0])) {
                eclist[0]->deactivate_component(rtc->getObjRef());
                OpenRTM::ExtTrigExecutionContextService_var execContext = OpenRTM::ExtTrigExecutionContextService::_narrow(eclist[0]);
                if (!CORBA::is_nil(execContext)) {
                    execContext->tick();
                }
            }
        }
    }

#if defined(OPENRTM_VERSION11)
    for (size_t i = 0; i < rtcs.size(); ++i) {
        RTC::RTObject_impl* rtc = rtcs[i];
        if (managedComponents.find(rtc) == managedComponents.end()) {
            RTC::ExecutionContextList_var eclist = rtc->get_participating_contexts();
            for (CORBA::ULong j = 0; j < eclist->length(); ++j) {
                if (!CORBA::is_nil(eclist[j])) {
                    eclist[j]->remove_component(rtc->getObjRef());
                }
            }
            RTC::PortServiceList_var ports = rtc->get_ports();
            for (CORBA::ULong j = 0; j < ports->length(); ++j) {
                ports[j]->disconnect_all();
            }
            RTC::ExecutionContextList_var myEClist = rtc->get_owned_contexts();
            if (myEClist->length() > 0 && !CORBA::is_nil(myEClist[0])) {
                OpenRTM::ExtTrigExecutionContextService_var myEC = OpenRTM::ExtTrigExecutionContextService::_narrow(myEClist[0]);
                RTC::RTCList rtcs = myEC->get_profile()->participants;
                for (size_t i = 0; i < rtcs.length(); ++i) {
                    myEC->remove_component(rtcs[i]);
                }
            }
        }
    }
#elif defined(OPENRTM_VERSION12)
    typedef std::map<RTC::ExecutionContextService_var, std::set<RTC::RTObject_impl*>> RemoveMap;
    RemoveMap removeMap;
    for (int i = 0; i < rtcs.size(); ++i) {
        RTC::RTObject_impl* rtc = rtcs[i];
        RTC::ExecutionContextList_var eclist = rtc->get_participating_contexts();
        for (CORBA::ULong j = 0; j < eclist->length(); ++j) {
            if (!CORBA::is_nil(eclist[j])) {
                RTC::ExecutionContextService_var execContext = RTC::ExecutionContextService::_narrow(eclist[j]);
                RemoveMap::iterator it = removeMap.find(execContext);
                if (it == removeMap.end()) {
                    std::set<RTC::RTObject_impl*> rtcs;
                    rtcs.insert(rtc);
                    removeMap.insert(make_pair(execContext, rtcs));
                } else {
                    it->second.insert(rtc);
                }
            }
        }
    }
    for (RemoveMap::iterator it = removeMap.begin(); it != removeMap.end(); ++it) {
        std::set<RTC::RTObject_impl*> removeRTCs = it->second;
        RTC::ExecutionContextService_var ec = it->first;
        for (std::set<RTC::RTObject_impl*>::iterator it = removeRTCs.begin(); it != removeRTCs.end(); ++it) {
            ec->remove_component((*it)->getObjRef());
        }
        if (!CORBA::is_nil(ec)) {
            OpenRTM::ExtTrigExecutionContextService_var execContext = OpenRTM::ExtTrigExecutionContextService::_narrow(ec);
            if (!CORBA::is_nil(execContext)) {
                execContext->tick();
            }
        }
    }
#endif

    for (size_t i = 0; i < rtcs.size(); ++i) {
        RTC::RTObject_impl* rtc = rtcs[i];
        if (managedComponents.find(rtc) == managedComponents.end()) {
            //std::cout << rtc->getInstanceName() << std::endl;
            rtc->exit();
            manager->cleanupComponents();
            ++numDeleted;
        }
    }
    return numDeleted;
}


bool cnoid::deleteRTC(RTC::RtcBase* rtc)
{
    DDEBUG("deleteRTC");

    bool deleted = false;

    if (rtc) { // Is the exception handler necessary here?
        try {
            RTC::ExecutionContextList_var eclist = rtc->get_participating_contexts();
            for (CORBA::ULong i = 0; i < eclist->length(); ++i) {
                if (!CORBA::is_nil(eclist[i])) {
                    eclist[i]->remove_component(rtc->getObjRef());
#if defined(OPENRTM_VERSION12)
                    OpenRTM::ExtTrigExecutionContextService_var execContext = OpenRTM::ExtTrigExecutionContextService::_narrow(eclist[i]);
                    if (!CORBA::is_nil(execContext)) {
                        execContext->tick();
                    }
#endif
                }
                RTC::ExecutionContextList_var myEClist = rtc->get_owned_contexts();
                if (myEClist->length() > 0 && !CORBA::is_nil(myEClist[0])) {
#if defined(OPENRTM_VERSION11)
                    OpenRTM::ExtTrigExecutionContextService_var myEC = OpenRTM::ExtTrigExecutionContextService::_narrow(myEClist[0]);
                    RTC::RTCList rtcs = myEC->get_profile()->participants;
                    for (size_t i = 0; i < rtcs.length(); ++i) {
                        myEC->remove_component(rtcs[i]);
                    }
#elif defined(OPENRTM_VERSION12)
                    RTC::ExecutionContextService_var myEC = RTC::ExecutionContextService::_narrow(myEClist[0]);
                    if (!CORBA::is_nil(myEC)) {
                        RTC::RTCList rtcs = myEC->get_profile()->participants;
                        for (int i = 0; i < rtcs.length(); ++i) {
                            myEC->remove_component(rtcs[i]);
                        }
                        OpenRTM::ExtTrigExecutionContextService_var execContext = OpenRTM::ExtTrigExecutionContextService::_narrow(myEC);
                        if (!CORBA::is_nil(execContext)) {
                            execContext->tick();
                        }
                    }
#endif
                }
            }

            rtc->exit();
            manager->cleanupComponents();
            managedComponents.erase(rtc);
            deleted = true;

        } catch (CORBA::SystemException& ex) {
            MessageView::instance()->putln(
                format(_("CORBA {0} ({1}), {2} in cnoid::deleteRTC()."),
                       ex._name(), ex._rep_id(), ex.NP_minorString()),
                MessageView::WARNING);
        }
    }

    DDEBUG("deleteRTC End");

    return deleted;
}


bool cnoid::isManagedRTC(RTC::RTObject_ptr rtc)
{
    for (auto itr = managedComponents.begin(); itr != managedComponents.end(); ++itr) {
        if (rtc->_is_equivalent((*itr)->getObjRef())) {
            return true;
        }
    }
    return false;
}


namespace cnoid {

template<> CORBA::Object::_ptr_type findRTCService<CORBA::Object>(RTC::RTObject_ptr rtc, const std::string& name)
{
    CORBA::Object_ptr service = CORBA::Object::_nil();

    RTC::PortServiceList ports;
    ports = *(rtc->get_ports());

    RTC::ComponentProfile* cprof;
    cprof = rtc->get_component_profile();
    std::string portname = std::string(cprof->instance_name) + "." + name;

    for (unsigned int i = 0; i < ports.length(); i++) {
        RTC::PortService_var port = ports[i];
        RTC::PortProfile* prof = port->get_port_profile();
        if (std::string(prof->name) == portname) {
            RTC::ConnectorProfile connProfile;
            connProfile.name = "noname";
            connProfile.connector_id = "";
            connProfile.ports.length(1);
            connProfile.ports[0] = port;
            connProfile.properties = 0;
            port->connect(connProfile);

            const char* ior = 0;
            connProfile.properties[0].value >>= ior;
            if (ior) {
                service = getORB()->string_to_object(ior);
            }
            port->disconnect(connProfile.connector_id);
            break;
        }
    }

    return service;
}

}
