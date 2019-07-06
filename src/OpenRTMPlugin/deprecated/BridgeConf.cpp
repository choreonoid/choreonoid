/**
   \file
   \author Shin'ichiro Nakaoka
*/

#include "BridgeConf.h"
#include "../OpenRTMUtil.h"
#include "../LoggerUtil.h"
#include <cnoid/Config>
#include <cnoid/stdx/filesystem>
#include <fmt/format.h>
#include <regex>
#include <iostream>

using namespace std;
using namespace cnoid;
namespace program_options = boost::program_options;
namespace filesystem = cnoid::stdx::filesystem;
using fmt::format;


BridgeConf::BridgeConf() :
    options("Allowed options"),
    commandLineOptions("Allowed options")
{   
    isReady_ = true;
}

BridgeConf::~BridgeConf()
{

}

bool BridgeConf::loadConfigFile(const char* confFileName) 
{
    isReady_ = false;
    initOptionsDescription();
    initLabelToDataTypeMap();

    ifstream ifs(confFileName);
    if (ifs.fail()) {
        return false;
    }
    program_options::store(program_options::parse_config_file(ifs, options), vmap);
    isProcessingConfigFile = true;
    program_options::notify(vmap);
    parseOptions();
    isReady_ = true;
    return true;
}

void BridgeConf::initOptionsDescription()
{
    options.add_options()

        ("name-server",
         program_options::value<string>()->default_value("127.0.0.1:2809"),
         "Nameserver used by OpenHRP (hostname:port)")

        ("server-name",
         program_options::value<string>()->default_value("ControllerBridge"),
         "Name of the OpenHRP controller factory server")

        ("robot-name",
         program_options::value<string>()->default_value(""),
         "Name of the virtual robot RTC type")

        ("module",
         program_options::value<vector<string> >(),
         "Module filename")

        ("out-port",
         program_options::value<vector<string> >(),
         "Set an out-port that transfers data from the simulator to the controller")

        ("in-port",
         program_options::value<vector<string> >(),
         "Set an in-port transfers data from the controller to the simulator")

        ("connection",
         program_options::value<vector<string> >(),
         "Port connection setting")

        ("periodic-rate",
         program_options::value<vector<string> >(), 
         "Periodic rate of execution context (INSTANCE_NAME:TIME_RATE[<=1.0])");

    commandLineOptions.add(options).add_options()

        ("config-file",
         program_options::value<string>(), 
         "configuration file of the controller bridge")
    
        ("help,h", "Show this help");
}


void BridgeConf::initLabelToDataTypeMap()
{
    LabelToDataTypeIdMap& m = labelToDataTypeIdMap;
  
    m["JOINT_VALUE"]         = JOINT_VALUE;
    m["JOINT_VELOCITY"]      = JOINT_VELOCITY;
    m["JOINT_ACCELERATION"]  = JOINT_ACCELERATION;
    m["JOINT_TORQUE"]        = JOINT_TORQUE;
    m["EXTERNAL_FORCE"]      = EXTERNAL_FORCE;
    m["ABS_TRANSFORM"]       = ABS_TRANSFORM;
    m["ABS_VELOCITY"]        = ABS_VELOCITY;
    m["ABS_ACCELERATION"]    = ABS_ACCELERATION;
    m["FORCE_SENSOR"]        = FORCE_SENSOR;
    m["RATE_GYRO_SENSOR"]    = RATE_GYRO_SENSOR;
    m["ACCELERATION_SENSOR"] = ACCELERATION_SENSOR;
    m["RANGE_SENSOR"]        = RANGE_SENSOR;
    m["CONSTRAINT_FORCE"]    = CONSTRAINT_FORCE;
    m["RATE_GYRO_SENSOR2"]    = RATE_GYRO_SENSOR2;
    m["ACCELERATION_SENSOR2"] = ACCELERATION_SENSOR2;
    m["ABS_TRANSFORM2"]       = ABS_TRANSFORM2;
    m["LIGHT"]                = LIGHT;
    m["CAMERA_IMAGE"]         = CAMERA_IMAGE;
    m["CAMERA_RANGE"]         = CAMERA_RANGE;
}


void BridgeConf::parseCommandLineOptions(int argc, char* argv[])
{
    isProcessingConfigFile = false;

    program_options::parsed_options parsed = 
        program_options::command_line_parser(argc, argv).options(commandLineOptions).allow_unregistered().run();      
    program_options::store(parsed, vmap); 

    if(vmap.count("help")){
        cout << commandLineOptions << endl;
    } else {

        if(vmap.count("config-file")){
            string fileName(vmap["config-file"].as<string>());
            ifstream ifs(fileName.c_str());
      
            if (ifs.fail()) {
                throw invalid_argument(string("cannot open the config file"));
            }
            program_options::store(program_options::parse_config_file(ifs, options), vmap);
      
            isProcessingConfigFile = true;
        }
        program_options::notify(vmap);
    
        parseOptions();

        isReady_ = true;
    }
}


void BridgeConf::parseOptions()
{
    if(vmap.count("module")){
        vector<string> values = vmap["module"].as<vector<string> >();
        for(size_t i=0; i < values.size(); ++i){
            string modulePath( values[i] );
            if(filesystem::path(modulePath).extension().empty()){
                modulePath += string( SUFFIX_SHARED_EXT );
            }
            addModuleInfo( modulePath );
        }
    }

    if(vmap.count("out-port")){
        setPortInfos("out-port", outPortInfos);
    }

    if(vmap.count("in-port")){
        setPortInfos("in-port", inPortInfos);
    }

    if(vmap.count("connection")){
        vector<string> values = vmap["connection"].as<vector<string> >();
        for(size_t i=0; i < values.size(); ++i){
            addPortConnection(values[i]);
        }
    }

    string server(expandEnvironmentVariables(vmap["name-server"].as<string>()));
    nameServerIdentifier = string("corbaloc:iiop:") + server + "/NameService";

    controllerName = expandEnvironmentVariables(vmap["server-name"].as<string>());

    virtualRobotRtcTypeName = expandEnvironmentVariables(vmap["robot-name"].as<string>());
    if(virtualRobotRtcTypeName==""){
        virtualRobotRtcTypeName=controllerName;
        virtualRobotRtcTypeName.append("(Robot)");
    }

    if(vmap.count("periodic-rate")){
        vector<string> values = vmap["periodic-rate"].as<vector<string> >();
        for(size_t i=0; i < values.size(); ++i){
            addTimeRateInfo(values[i]);
        }
    }
}


void BridgeConf::setPortInfos(const char* optionLabel, PortInfoMap& portInfos)
{
    vector<string> ports = vmap[optionLabel].as<vector<string> >();
    for(size_t i=0; i < ports.size(); ++i){
        vector<string> parameters;
        extractParameters(ports[i], parameters);
        int n = parameters.size();
        if(n < 2 || n > 4){
            throw invalid_argument(string("invalid in port setting"));
        }
        PortInfo info;

        info.portName = parameters[0];
        int j;
        for(j=1; j<3; j++){
					if (parameters.size() <= j) {
						throw invalid_argument(string("invalid in port setting"));
					}
					LabelToDataTypeIdMap::iterator it = labelToDataTypeIdMap.find(parameters[j]);
            if(it == labelToDataTypeIdMap.end() ){ // Handle as identification name because it is not a property name
                if(j==2)    // Error because there is no property name by the third
                    throw invalid_argument(string("invalid data type"));
                string st=parameters[j];
                vector<string> owners;
                extractParameters(st, owners, ',');
                for(size_t i=0; i<owners.size(); i++){
                    bool digit=true;
                    for(string::iterator itr=owners[i].begin(); itr!=owners[i].end(); itr++){
                        if(!isdigit(*itr)){
                            digit = false;             
                            break;
                        }
                    }
                    if(digit){
                        // Since the identification name is numeric, it becomes image data.
                        // If two or more are specified, an error occurs.
                        info.dataOwnerId = atoi(owners[i].c_str());
                        info.dataOwnerNames.clear();
                        if(owners.size() > 1){
                            throw invalid_argument(string("invalid VisionSensor setting"));
                        }
                    }else{  // Identification name is name
                        info.dataOwnerId = -1;
                        info.dataOwnerNames.push_back(owners[i]);
                    }
                }
            }else{  // Property name
                info.dataTypeId = it->second;
                j++;
                break;
            }
        }
        if(j<n){ // If there is still a parameter, handle it as the sampling time
            info.stepTime = atof(parameters[j].c_str());
        }else{
            info.stepTime = 0;
        }

        // When the property is CONSTRAINT_FORCE, there must not be two or more identification names
        if(info.dataTypeId==CONSTRAINT_FORCE && info.dataOwnerNames.size() > 1 )
            throw invalid_argument(string("invalid in port setting"));
        portInfos.insert(make_pair(info.portName, info));
    }
}


void BridgeConf::addPortConnection(const std::string& value)
{
    vector<string> parameters;
    extractParameters(value, parameters);
    int n = parameters.size();
    if(n < 2 || n > 4){
        throw std::invalid_argument(string("Invalied port connection"));
    }

    PortConnection connection;
    if(parameters.size() == 2){
        connection.PortName[0] = parameters[0];
        connection.PortName[1] = parameters[1];
    } else if(parameters.size() == 3){
        connection.PortName[0] = parameters[0];
        connection.InstanceName[1] = parameters[1];
        connection.PortName[1] = parameters[2];
    }else if(parameters.size() == 4 ){
        connection.InstanceName[0] = parameters[0];
        connection.PortName[0] = parameters[1];
        connection.InstanceName[1] = parameters[2];
        connection.PortName[1] = parameters[3];
    }

    portConnections.push_back(connection);
}

void BridgeConf::setPreLoadModuleInfo()
{
    RTC::Manager& rtcManager = RTC::Manager::instance();
    std::vector<RTC::RtcBase*> components = rtcManager.getComponents();
    for(std::vector<RTC::RtcBase*>::iterator it=components.begin(); it != components.end(); ++it){ 
        ModuleInfo info;
        info.componentName = (*it)->get_component_profile()->type_name;
        info.fileName = "";
        info.initFuncName = "";
        info.isLoaded = true;
        info.rtcServant = *it;
        moduleInfoList.push_back(info);
    }
}

void BridgeConf::addModuleInfo(const std::string& value)
{
    vector<string> parameters;
    extractParameters(value, parameters);

    if(parameters.size() == 0 || parameters.size() > 2){
        throw std::invalid_argument(std::string("invalid module set"));
    } else {
        ModuleInfo info;
        info.fileName = parameters[0];
        info.componentName = filesystem::path(info.fileName).stem().string();
        if(parameters.size() == 1){
            info.initFuncName = info.componentName + "Init";
        } else {
            info.initFuncName = parameters[1];
        }
        info.isLoaded = false;
    
        moduleInfoList.push_back(info);
    }
}

void BridgeConf::addTimeRateInfo(const std::string& value) 
{
    vector<string> parameters;
    extractParameters(value, parameters);
    if (parameters.size() == 0 || parameters.size() > 2) {
        throw std::invalid_argument(std::string("invalid time rate set"));
    } else {
        timeRateMap.insert( std::map<std::string, double>::value_type(parameters[0], atof(parameters[1].c_str())) );
    }
}

void BridgeConf::setupModules() {
  DDEBUG("BridgeConf::setupModules");
    RTC::Manager& rtcManager = RTC::Manager::instance();
    ModuleInfoList::iterator moduleInfo = moduleInfoList.begin();
#if defined(OPENRTM_VERSION11)
    string param("{}?exec_cxt.periodic.type=ChoreonoidExecutionContext&exec_cxt.periodic.rate=1000000");
#elif defined(OPENRTM_VERSION12)
    string param("{}?execution_contexts=ChoreonoidExecutionContext(),OpenHRPExecutionContext()&exec_cxt.periodic.type=ChoreonoidExecutionContext&exec_cxt.periodic.rate=1000000&exec_cxt.sync_activation=NO&exec_cxt.sync_deactivation=NO");
#endif
    while(moduleInfo != moduleInfoList.end()){
        if(!moduleInfo->isLoaded){
            rtcManager.load(moduleInfo->fileName.c_str(), moduleInfo->initFuncName.c_str());
            moduleInfo->isLoaded = true;
            moduleInfo->rtcServant = cnoid::createManagedRTC(format(param, moduleInfo->componentName));
        }
        ++moduleInfo;
    }
}


const char* BridgeConf::getOpenHRPNameServerIdentifier()
{
    return nameServerIdentifier.c_str();
}


const char* BridgeConf::getControllerName()
{
    return controllerName.c_str();
}


const char* BridgeConf::getVirtualRobotRtcTypeName()
{
    return virtualRobotRtcTypeName.c_str();
}

void BridgeConf::extractParameters(const std::string& str, std::vector<std::string>& result, const char delimiter)
{
    string::size_type sepPos = 0;
    string::size_type nowPos = 0;
  
    while (sepPos != string::npos) {
        sepPos = str.find(delimiter, nowPos);

        if(isProcessingConfigFile){
            result.push_back(expandEnvironmentVariables(str.substr(nowPos, sepPos-nowPos)));
        } else {
            result.push_back(str.substr(nowPos, sepPos-nowPos));
        }
      
        nowPos = sepPos+1;
    }
}


std::string BridgeConf::expandEnvironmentVariables(const std::string& str)
{
    regex variablePattern("\\$([A-z][A-z_0-9]*)");
    
    match_results<string::const_iterator> result; 
    regex_constants::match_flag_type flags = regex_constants::match_default; 
    
    string::const_iterator start, end;
    std::string str_(str);
    start = str_.begin();
    end = str_.end();
    int pos = 0;

    vector<pair<int, int>> results;

    while(regex_search(start, end, result, variablePattern, flags)){
        results.push_back(std::make_pair(pos+result.position(1), result.length(1)));

        // seek to the remaining part
        start = result[0].second;
        pos += result.length(0);
        flags |= regex_constants::match_prev_avail; 
    }
    // replace the variables in reverse order
    while (!results.empty()) {
        int begin = results.back().first;
        int length = results.back().second;

        string envName = str.substr(begin, length);
        char* envValue = getenv(envName.c_str());

        if(envValue){
            str_.replace(begin-1, length+1, string(envValue));
        }
        results.pop_back();
    }

    return str_;
}
