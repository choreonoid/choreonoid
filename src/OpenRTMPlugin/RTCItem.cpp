/**
   @author Shizuko Hattori
   @author Shin'ichiro Nakaoka
*/

#include "RTCItem.h"
#include "OpenRTMUtil.h"
#include <cnoid/ItemManager>
#include <cnoid/FileUtil>
#include <cnoid/ExecutablePath>
#include <cnoid/MessageView>
#include <cnoid/ProjectManager>
#include <cnoid/CorbaUtil>
#include <rtm/RTObject.h>
#include <fmt/format.h>
#include "gettext.h"

#include "LoggerUtil.h"

using namespace std;
using namespace cnoid;
using fmt::format;
namespace filesystem = boost::filesystem;

namespace {
const bool TRACE_FUNCTIONS = false;
}


void RTCItem::initialize(ExtensionManager* ext)
{
    static bool initialized = false;
    if (!initialized) {
        ext->itemManager().registerClass<RTCItem>(N_("RTCItem"));
        ext->itemManager().addCreationPanel<RTCItem>();
        initialized = true;
    }
}


RTCItem::RTCItem()
    : os(MessageView::instance()->cout()),
    periodicType(N_PERIODIC_TYPE, CNOID_GETTEXT_DOMAIN_NAME),
    baseDirectoryType(N_BASE_DIRECTORY_TYPES, CNOID_GETTEXT_DOMAIN_NAME)
{
    rtcomp = 0;
    moduleName.clear();
    mv = MessageView::instance();

    periodicRate = 1000;
    periodicType.setSymbol(PERIODIC_EXECUTION_CONTEXT, N_("PeriodicExecutionContext"));

#if defined(OPENRTM_VERSION11)
    periodicType.setSymbol(SYNCH_EXT_TRIGGER, N_("SynchExtTriggerEC"));
    periodicType.setSymbol(EXT_TRIG_EXECUTION_CONTEXT, N_("ExtTrigExecutionContext"));
    periodicType.setSymbol(SIMULATION_EXECUTION_CONTEXT, N_("SimulationExecutionContext"));
    periodicType.select(PERIODIC_EXECUTION_CONTEXT);
#endif

    oldPeriodicType = periodicType.which();

    properties.clear();
    properties.insert(make_pair(string("exec_cxt.periodic.type"), periodicType.selectedSymbol()));
    stringstream ss;
    ss << periodicRate;
    properties.insert(make_pair(string("exec_cxt.periodic.rate"), ss.str()));

    baseDirectoryType.setSymbol(RTC_DIRECTORY, N_("RTC directory"));
    baseDirectoryType.setSymbol(PROJECT_DIRECTORY, N_("Project directory"));
    baseDirectoryType.select(RTC_DIRECTORY);
    oldBaseDirectoryType = baseDirectoryType.which();

    rtcDirectory = filesystem::path(executableTopDirectory()) / CNOID_PLUGIN_SUBDIR / "rtc";

    isActivationEnabled_ = false;
}


RTCItem::RTCItem(const RTCItem& org)
    : Item(org),
    os(MessageView::instance()->cout()),
    periodicType(org.periodicType),
    baseDirectoryType(org.baseDirectoryType),
    rtcDirectory(org.rtcDirectory)
{
    rtcomp = org.rtcomp;
    moduleName = org.moduleName;
    mv = MessageView::instance();
    periodicRate = org.periodicRate;
    oldPeriodicType = org.oldPeriodicType;
    properties = org.properties;
    oldBaseDirectoryType = org.oldBaseDirectoryType;
    isActivationEnabled_ = org.isActivationEnabled_;
}

RTCItem::~RTCItem()
{

}

void RTCItem::deleteRTCInstance()
{
    if (rtcomp) {
        rtcomp->deleteRTC();
        delete rtcomp;
        rtcomp = 0;
    }
}

void RTCItem::updateRTCInstance(bool forceUpdate)
{
    if (rtcomp && forceUpdate) {
        deleteRTCInstance();
    }
    if (!rtcomp) {
        if (convertAbsolutePath()) {
            rtcomp = new RTComponent(modulePath, properties);
            if (isActivationEnabled_) {
                rtcomp->activate();
            }
        }
    }
}

void RTCItem::onConnectedToRoot()
{
    DDEBUG("RTCItem::onConnectedToRoot");
    updateRTCInstance(false);
}


void RTCItem::onDisconnectedFromRoot()
{
    deleteRTCInstance();
}


Item* RTCItem::doDuplicate() const
{
    return new RTCItem(*this);
}

void RTCItem::setModuleName(const std::string& name)
{
    DDEBUG_V("RTCItem::setModuleName %s", name.c_str());
    if (moduleName != name) {
        moduleName = name;
        updateRTCInstance();
    }
}

void RTCItem::setPeriodicType(int type)
{
    DDEBUG_V("RTCItem::setPeriodicType %d", type);
    if (oldPeriodicType != type) {
        oldPeriodicType = type;
        properties["exec_cxt.periodic.type"] = periodicType.symbol(type);
        updateRTCInstance();
    }
    if (convertAbsolutePath()) rtcomp = new RTComponent(modulePath, properties);
}

void RTCItem::setPeriodicRate(int rate)
{
    DDEBUG_V("RTCItem::setPeriodicRate %d", rate);
    if (periodicRate != rate) {
        periodicRate = rate;
        stringstream ss;
        ss << periodicRate;
        properties["exec_cxt.periodic.rate"] = ss.str();
        updateRTCInstance();
    }
}

void RTCItem::setBaseDirectoryType(int base)
{
    DDEBUG_V("RTCItem::setBaseDirectoryType %d", base);
    baseDirectoryType.select(base);
    if (oldBaseDirectoryType != base) {
        oldBaseDirectoryType = base;
        updateRTCInstance();
    }
}

void RTCItem::setActivationEnabled(bool on)
{
    if (on != isActivationEnabled_) {
        isActivationEnabled_ = on;
        if (on && rtcomp) {
            rtcomp->activate();
        }
    }
}

void RTCItem::doPutProperties(PutPropertyFunction& putProperty)
{
    Item::doPutProperties(putProperty);

    FilePathProperty moduleProperty(
        moduleName,
        { format(_("RT-Component module (*{})"), DLL_SUFFIX) });

    if (baseDirectoryType.is(RTC_DIRECTORY)) {
        moduleProperty.setBaseDirectory(rtcDirectory.string());
    } else if (baseDirectoryType.is(PROJECT_DIRECTORY)) {
        moduleProperty.setBaseDirectory(ProjectManager::instance()->currentProjectDirectory());
    }

    putProperty(_("RTC module"), moduleProperty,
                [&](const string& name) { setModuleName(name); return true; });
    putProperty(_("Base directory"), baseDirectoryType,
                [&](int which) { setBaseDirectoryType(which); return true; });
    putProperty(_("Execution context"), periodicType,
                [&](int which) { setPeriodicType(which); return true; });
    putProperty(_("Periodic rate"), periodicRate,
                [&](int rate) { setPeriodicRate(rate); return true; });
    putProperty(_("Activation"), isActivationEnabled_, [&](bool on) { setActivationEnabled(on); return true; });
}


bool RTCItem::store(Archive& archive)
{
    if (!Item::store(archive)) {
        return false;
    }
    archive.writeRelocatablePath("module", moduleName);
    archive.write("baseDirectory", baseDirectoryType.selectedSymbol(), DOUBLE_QUOTED);
    archive.write("periodicType", periodicType.selectedSymbol());
    archive.write("periodicRate", periodicRate);
    archive.write("activation", isActivationEnabled_);
    return true;
}


bool RTCItem::restore(const Archive& archive)
{
    DDEBUG("RTCItem::restore");

    if (!Item::restore(archive)) {
        return false;
    }
    string value;
    if (archive.read("module", value) || archive.read("moduleName", value)) {
        filesystem::path path(archive.expandPathVariables(value));
        moduleName = path.make_preferred().string();
    }
    if (archive.read("baseDirectory", value) || archive.read("RelativePathBase", value)) {
        baseDirectoryType.select(value);
        oldBaseDirectoryType = baseDirectoryType.selectedIndex();
    }

    if (archive.read("periodicType", value)) {
        periodicType.select(value);
        oldPeriodicType = periodicType.selectedIndex();
        properties["exec_cxt.periodic.type"] = value;
    }
    if (archive.read("periodicRate", periodicRate)) {
        stringstream ss;
        ss << periodicRate;
        properties["exec_cxt.periodic.rate"] = ss.str();
    }
    archive.read("activation", isActivationEnabled_);

    return true;
}


bool RTCItem::convertAbsolutePath()
{
    modulePath = moduleName;
    if (!modulePath.is_absolute()) {
        if (baseDirectoryType.is(RTC_DIRECTORY)) {
            modulePath = rtcDirectory / modulePath;
        } else if (baseDirectoryType.is(PROJECT_DIRECTORY)) {
            string projectDir = ProjectManager::instance()->currentProjectDirectory();
            if (projectDir.empty()) {
                mv->putln(_("Please save the project."));
                return false;
            } else {
                modulePath = filesystem::path(projectDir) / modulePath;
            }
        }
    }
    return true;
}


RTComponent::RTComponent(const filesystem::path& modulePath, PropertyMap& prop)
{
    DDEBUG("RTComponent::RTComponent");
    rtc_ = nullptr;
    rtcRef = RTC::RTObject::_nil();
    if (modulePath.empty()) return;
    init(modulePath, prop);
}


RTComponent::~RTComponent()
{
    deleteRTC();
}


void RTComponent::init(const filesystem::path& modulePath_, PropertyMap& prop)
{
    DDEBUG("RTComponent::init");
    mv = MessageView::instance();
    modulePath = modulePath_;
    createRTC(prop);
}


bool  RTComponent::createRTC(PropertyMap& prop)
{
    DDEBUG("RTComponent::createRTC");

    string moduleNameLeaf = modulePath.leaf().string();
    size_t i = moduleNameLeaf.rfind('.');
    if (i != string::npos) {
        componentName = moduleNameLeaf.substr(0, i);
    } else {
        componentName = moduleNameLeaf;
    }

    string actualFilename;

    if (filesystem::exists(modulePath)) {
        actualFilename = getNativePathString(modulePath);
        if (modulePath.extension() == DLL_SUFFIX) {
            string initFunc(componentName + "Init");
            setupModules(actualFilename, initFunc, componentName, prop);
        } else {
            createProcess(actualFilename, prop);
        }
    } else {
        filesystem::path exePath(modulePath.string() + EXEC_SUFFIX);
        if (filesystem::exists(exePath)) {
            actualFilename = getNativePathString(exePath);
            createProcess(actualFilename, prop);
        } else {
            filesystem::path dllPath(modulePath.string() + DLL_SUFFIX);
            if (filesystem::exists(dllPath)) {
                actualFilename = getNativePathString(dllPath);
                string initFunc(componentName + "Init");
                setupModules(actualFilename, initFunc, componentName, prop);
            } else {
                mv->putln(format(_("A file of RTC \"{}\" does not exist."), componentName));
            }
        }
    }

    bool created = isValid();

    if (created) {
        mv->putln(format(_("RTC \"{0}\" has been created from \"{1}\"."), componentName, actualFilename));
    } else {
        mv->putln(format(_("RTC \"{}\" cannot be created."), componentName));
    }

    return created;
}


void RTComponent::setupModules(string& fileName, string& initFuncName, string& componentName, PropertyMap& prop)
{
    DDEBUG("RTComponent::setupModules");

    RTC::Manager& rtcManager = RTC::Manager::instance();

    rtcManager.load(fileName.c_str(), initFuncName.c_str());

    string option("?");
    for (PropertyMap::iterator it = prop.begin(); it != prop.end(); ) {
        option += it->first + "=" + it->second;
        if (++it != prop.end()) {
            option += "&";
        }
    }
    rtc_ = createManagedRTC((componentName + option).c_str());

    if (!rtc_) {
        mv->putln(
            format(_("RTC \"{0}\" cannot be created by the RTC manager.\n"
                     " RTC module file: \"{1}\"\n"
                     " Init function: {2}\n"
                     " option: {3}"),
                   componentName, fileName, initFuncName, option));
    }
}


void RTComponent::onReadyReadServerProcessOutput()
{
    mv->put(QString(rtcProcess.readAll()));
}


bool RTComponent::isValid() const
{
    return (rtc_ || rtcProcess.state() != QProcess::NotRunning);
}


void RTComponent::createProcess(string& command, PropertyMap& prop)
{
    DDEBUG("RTComponent::createProcess");

    QStringList argv;
    argv.push_back(QString("-o"));
    argv.push_back(QString("naming.formats: %n.rtc"));
    argv.push_back(QString("-o"));
    argv.push_back(QString("logger.enable: NO"));
    for (PropertyMap::iterator it = prop.begin(); it != prop.end(); it++) {
        argv.push_back(QString("-o"));
        argv.push_back(QString(string(it->first + ":" + it->second).c_str()));
    }
    if (rtcProcess.state() != QProcess::NotRunning) {
        rtcProcess.kill();
        rtcProcess.waitForFinished(100);
    }
#ifdef _WIN32
    rtcProcess.start(QString("\"") + command.c_str() + "\"", argv);
#else
    rtcProcess.start(command.c_str(), argv);
#endif
    if (!rtcProcess.waitForStarted()) {
        mv->putln(format(_("RT Component process \"{}\" cannot be executed."), command));
    } else {
        mv->putln(format(_("RT Component process \"{}\" has been executed."), command));
        rtcProcess.sigReadyReadStandardOutput().connect(
          std::bind(&RTComponent::onReadyReadServerProcessOutput, this));
    }
}


void RTComponent::deleteRTC()
{
    if (TRACE_FUNCTIONS) {
        cout << "BodyRTComponent::deleteRTC()" << endl;
    }

    if (rtc_) {
        string rtcName(rtc_->getInstanceName());
        mv->putln(format(_("delete {}"), rtcName));
        if (!cnoid::deleteRTC(rtc_)) {
            mv->putln(format(_("{} cannot be deleted."), rtcName));
        }
        rtc_ = nullptr;

    } else if (rtcProcess.state() != QProcess::NotRunning) {
        mv->putln(format(_("delete {}"), componentName));
        rtcProcess.kill();
        rtcProcess.waitForFinished(100);
    }

    if (TRACE_FUNCTIONS) {
        cout << "End of BodyRTCItem::deleteModule()" << endl;
    }
}

void RTComponent::activate()
{
    if (rtc_) {
        RTC::ExecutionContextList_var eclist = rtc_->get_owned_contexts();
        for (CORBA::ULong i = 0; i < eclist->length(); ++i) {
            if (!CORBA::is_nil(eclist[i])) {
                eclist[i]->activate_component(rtc_->getObjRef());
                break;
            }
        }
    }
}
