/**
   @author Shizuko Hattori
   @author Shin'ichiro Nakaoka
*/

#include "RTCItem.h"
#include "BridgeConf.h"
#include "OpenRTMUtil.h"
#include <cnoid/BodyItem>
#include <cnoid/Link>
#include <cnoid/Sensor>
#include <cnoid/ItemManager>
#include <cnoid/Archive>
#include <cnoid/FileUtil>
#include <cnoid/ExecutablePath>
#include <cnoid/MessageView>
#include <cnoid/Sleep>
#include <rtm/CorbaNaming.h>
#include <boost/bind.hpp>
#include <boost/regex.hpp>
#include "gettext.h"


using namespace boost;
using namespace cnoid;


namespace {
const bool TRACE_FUNCTIONS = false;
}


void RTCItem::initialize(ExtensionManager* ext)
{
    static bool initialized = false;
    if(!initialized){
        ext->itemManager().registerClass<RTCItem>(N_("RTCItem"));
        ext->itemManager().addCreationPanel<RTCItem>();
        initialized = true;
    }
}


RTCItem::RTCItem()
    : os(MessageView::instance()->cout()),
      periodicType(N_PERIODIC_TYPE, CNOID_GETTEXT_DOMAIN_NAME)
{
    rtcomp = 0;
    moduleName.clear();
    mv = MessageView::instance();

    periodicRate = 1000;
    periodicType.setSymbol(PERIODIC_EXECUTION_CONTEXT,  N_("PeriodicExecutionContext"));
    periodicType.setSymbol(SYNCH_EXT_TRIGGER,  N_("SynchExtTriggerEC"));
    periodicType.setSymbol(EXT_TRIG_EXECUTION_CONTEXT,  N_("ExtTrigExecutionContext"));
    periodicType.setSymbol(CHOREONOID_EXECUTION_CONTEXT,  N_("ChoreonoidExecutionContext"));
    periodicType.select(PERIODIC_EXECUTION_CONTEXT);
    oldType = PERIODIC_EXECUTION_CONTEXT;
    properties.clear();
    properties.insert(make_pair(string("exec_cxt.periodic.type"), periodicType.selectedSymbol()));
    stringstream ss;
    ss << periodicRate;
    properties.insert(make_pair(string("exec_cxt.periodic.rate"), ss.str()));
}


RTCItem::RTCItem(const RTCItem& org)
    : os(MessageView::instance()->cout()),
      Item(org),
      periodicType(org.periodicType)
{
    rtcomp = org.rtcomp;
    moduleName = org.moduleName;
    mv = MessageView::instance();
    periodicRate = org.periodicRate;
    oldType = org.oldType;
    properties = org.properties;
}

RTCItem::~RTCItem()
{
    
}


void RTCItem::onPositionChanged()
{
    if(!rtcomp){
        rtcomp = new RTComponent(moduleName, properties);
    }
}


void RTCItem::onDisconnectedFromRoot()
{
    if(rtcomp){
        rtcomp->deleteRTC(false);
        delete rtcomp;
        rtcomp = 0;
    }
}


ItemPtr RTCItem::doDuplicate() const
{
    return new RTCItem(*this);
}


void RTCItem::setModuleName(const std::string& name)
{
    if(moduleName!=name){
        moduleName = name;
        if(rtcomp){
            delete rtcomp;
        }
        rtcomp = new RTComponent(moduleName, properties);
    }
}


void RTCItem::setPeriodicType(int type)
{
    if(oldType != type){
        oldType = type;
        properties["exec_cxt.periodic.type"] = periodicType.symbol(type);
        if(rtcomp){
            delete rtcomp;
        }
        rtcomp = new RTComponent(moduleName, properties);
    }
}


void RTCItem::setPeriodicRate(int rate)
{
    if(periodicRate!=rate){
        periodicRate = rate;
        stringstream ss;
        ss << periodicRate;
        properties["exec_cxt.periodic.rate"] = ss.str();
        if(rtcomp){
            delete rtcomp;
        }
        rtcomp = new RTComponent(moduleName, properties);
    }
}


void RTCItem::doPutProperties(PutPropertyFunction& putProperty)
{
    Item::doPutProperties(putProperty);
    putProperty(_("RTC module Name"), moduleName,
                boost::bind(&RTCItem::setModuleName, this, _1), true);
    putProperty(_("Periodic type"), periodicType,
                boost::bind((bool(Selection::*)(int))&Selection::select, &periodicType, _1));
    setPeriodicType(periodicType.selectedIndex());
    putProperty(_("Periodic Rate"), periodicRate,
                boost::bind(&RTCItem::setPeriodicRate, this, _1), true);
}


bool RTCItem::store(Archive& archive)
{
    if(!Item::store(archive)){
        return false;
    }
    archive.writeRelocatablePath("moduleName", moduleName);
    archive.write("periodicType", periodicType.selectedSymbol());
    archive.write("periodicRate", periodicRate);
    return true;
}


bool RTCItem::restore(const Archive& archive)
{
    if(!Item::restore(archive)){
        return false;
    }
    string value;
    if(archive.read("moduleName", value)){
        moduleName = archive.expandPathVariables(value);
    }
    string symbol;
    if(archive.read("periodicType", symbol)){
        periodicType.select(symbol);
        oldType = periodicType.selectedIndex();
    }
    archive.read("periodicRate", periodicRate);
    return true;
}


RTComponent::RTComponent(string moduleName_)
{
    rtc_ = 0;
    rtcRef = 0;
    if(moduleName_.empty()){
        return;
    }
    PropertyMap prop;
    prop.clear();
    init(moduleName_, prop);
}


RTComponent::RTComponent(string moduleName_, PropertyMap& prop)
{
    rtc_ = 0;
    rtcRef = 0;
    if(moduleName_.empty()){
        return;
    }
    init(moduleName_, prop);
}


RTComponent::~RTComponent()
{
    deleteRTC(true);
}


void RTComponent::init(string moduleName_, PropertyMap& prop)
{
    mv = MessageView::instance();
    moduleName = moduleName_;
    createRTC(prop);
}


bool  RTComponent::createRTC(PropertyMap& prop)
{   
    filesystem::path modulePath(moduleName);
    string moduleNameLeaf = modulePath.leaf().string();
    size_t i = moduleNameLeaf.rfind('.');
    if(i != string::npos){
        componentName = moduleNameLeaf.substr(0, i);
    } else {
        componentName = moduleNameLeaf;
    }

    if(!checkAbsolute(modulePath)){
        modulePath =
            filesystem::path(executableTopDirectory()) /
            CNOID_PLUGIN_SUBDIR / "rtc" / modulePath;
    }

    string actualFilename;

    if(filesystem::exists(modulePath)){
        actualFilename = getNativePathString(modulePath);
        if(modulePath.extension() == SUFFIX_SHARED_EXT){
            string initFunc(componentName + "Init");
            setupModules(actualFilename, initFunc, componentName, prop);          
        } else {
            createProcess(actualFilename, prop);
        }   
    } else {
        filesystem::path exePath(modulePath.string() + SUFFIX_EXE_EXT);
        if(filesystem::exists(exePath)){
            actualFilename = getNativePathString(exePath);
            createProcess(actualFilename, prop);
        } else {
            filesystem::path dllPath(modulePath.string() + SUFFIX_SHARED_EXT);
            if(filesystem::exists(dllPath)){
                actualFilename = getNativePathString(dllPath);
                string initFunc(componentName + "Init");
                setupModules(actualFilename, initFunc, componentName, prop);
            } else {
                mv->putln(fmt(_("A file of RTC \"%1%\" does not exist.")) % componentName);
            }
        }
    }

    bool created = isValid();

    if(created){
        mv->putln(fmt(_("RTC \"%1%\" has been created from \"%2%\".")) % componentName % actualFilename);
    } else {
        mv->putln(fmt(_("RTC \"%1%\" cannot be created.")) % componentName);
    }

    return created;
}


void RTComponent::setupModules(string& fileName, string& initFuncName, string& componentName, PropertyMap& prop)
{
    RTC::Manager& rtcManager = RTC::Manager::instance();

    rtcManager.load(fileName.c_str(), initFuncName.c_str());

    string option("?");
    for(PropertyMap::iterator it = prop.begin(); it != prop.end(); ){
        option += it->first + "=" + it->second;
        if(++it != prop.end()){
            option += "&";
        }
    }
    rtc_ = createManagedRTC((componentName + option).c_str());

    if(!rtc_){
        mv->putln(fmt(_("RTC \"%1%\" cannot be created by the RTC manager.\n"
                        " RTC module file: \"%2%\"\n"
                        " Init function: %3%\n"
                        " option: %4%"))
                  % componentName % fileName % initFuncName % option);
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
    QStringList argv;
    argv.push_back(QString("-o"));
    argv.push_back(QString("naming.formats: %n.rtc"));
    argv.push_back(QString("-o"));
    argv.push_back(QString("logger.enable: NO"));
    for(PropertyMap::iterator it = prop.begin(); it != prop.end(); it++){
        argv.push_back(QString("-o"));
        argv.push_back(QString(string(it->first+":"+it->second).c_str()));
    }
    if(rtcProcess.state() != QProcess::NotRunning){
        rtcProcess.kill();
        rtcProcess.waitForFinished(100);
    }
#ifdef _WIN32
    rtcProcess.start(QString("\"") + command.c_str() + "\"", argv );
#else
    rtcProcess.start(command.c_str(), argv);
#endif
    if(!rtcProcess.waitForStarted()){
        mv->putln(fmt(_("RT Component process \"%1%\" cannot be executed.")) % command);
    } else {
        mv->putln(fmt(_("RT Component process \"%1%\" has been executed.")) % command );
        rtcProcess.sigReadyReadStandardOutput().connect(
            boost::bind(&RTComponent::onReadyReadServerProcessOutput, this));
    }
}


void RTComponent::deleteRTC(bool waitToBeDeleted)
{
    if(TRACE_FUNCTIONS){
        cout << "BodyRTComponent::deleteRTC()" << endl;
    }
    
    if(rtc_){
        string rtcName(rtc_->getInstanceName());
        mv->putln(fmt(_("delete %1%")) % rtcName);
        if(!cnoid::deleteRTC(rtc_, true)){
            mv->putln(fmt(_("%1% cannot be deleted.")) % rtcName);
        }
        rtc_ = 0;

    } else if(rtcProcess.state() != QProcess::NotRunning){
        mv->putln(fmt(_("delete %1%")) % componentName);
        rtcProcess.kill();
        rtcProcess.waitForFinished(100);
    }
   
    if(TRACE_FUNCTIONS){
        cout << "End of BodyRTCItem::deleteModule()" << endl;
    }
}
