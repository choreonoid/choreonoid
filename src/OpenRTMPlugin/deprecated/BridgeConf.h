#ifndef OPENHRP_BRIDGE_CONF_H
#define OPENHRP_BRIDGE_CONF_H

#if ( defined ( WIN32 ) || defined ( _WIN32 ) || defined(__WIN32__) || defined(__NT__) )
#define SUFFIX_SHARED_EXT   ".dll"
#define SUFFIX_EXE_EXT      ".exe"
#elif defined(__APPLE__)
#define SUFFIX_SHARED_EXT   ".dylib"
#define SUFFIX_EXE_EXT      ".app"
#else
#define SUFFIX_SHARED_EXT   ".so"
#define SUFFIX_EXE_EXT      ""
#endif

#include <map>
#include <list>
#include <vector>
#include <string>
#include <boost/program_options.hpp>
#include <rtm/Manager.h>
#include <rtm/RTObject.h>

enum DataTypeId {
    INVALID_DATA_TYPE = 0,
    JOINT_VALUE,
    JOINT_VELOCITY,
    JOINT_ACCELERATION,
    JOINT_TORQUE,
    EXTERNAL_FORCE,
    ABS_TRANSFORM,
    ABS_VELOCITY,
    ABS_ACCELERATION,
    FORCE_SENSOR,
    RATE_GYRO_SENSOR,
    ACCELERATION_SENSOR,
    RANGE_SENSOR,
    CONSTRAINT_FORCE,
    RATE_GYRO_SENSOR2,
    ACCELERATION_SENSOR2,
    ABS_TRANSFORM2,
    LIGHT,
    CAMERA_IMAGE,
    CAMERA_RANGE
};

struct PortInfo {
    std::string portName;
    DataTypeId dataTypeId;
    std::vector<std::string> dataOwnerNames; // link name or sensor name
    int dataOwnerId;           // sensor id
    double stepTime;
};
    
typedef std::map<std::string, PortInfo> PortInfoMap;
    

struct PortConnection {
    std::string InstanceName[2];
    std::string PortName[2];
    //std::string robotPortName;
    //std::string controllerInstanceName;
    //std::string controllerPortName;
};
typedef std::vector<PortConnection> PortConnectionList;
    
    
struct ModuleInfo {
    std::string fileName;
    std::string componentName;
    std::string initFuncName;
    bool isLoaded;
    RTC::RtcBase* rtcServant;
};
typedef std::list<ModuleInfo> ModuleInfoList;
    
typedef std::map<std::string, double> TimeRateMap;

class BridgeConf
{
      
public:
    BridgeConf();
    ~BridgeConf();

    bool loadConfigFile(const char* confFileName);
    bool isReady() { return isReady_; }
      
    const char* getOpenHRPNameServerIdentifier();
    const char* getControllerName();
    const char* getVirtualRobotRtcTypeName();

    void setupModules();

    typedef std::map<std::string, DataTypeId> LabelToDataTypeIdMap;
    LabelToDataTypeIdMap labelToDataTypeIdMap;
      
    PortInfoMap outPortInfos;
    PortInfoMap inPortInfos;
      
    ModuleInfoList moduleInfoList;

    PortConnectionList portConnections;

    TimeRateMap timeRateMap;

private:
      
    boost::program_options::variables_map vmap;
    boost::program_options::options_description options;
    boost::program_options::options_description commandLineOptions;
      
    bool isReady_;
    bool isProcessingConfigFile;
      
    std::string virtualRobotRtcTypeName;
    std::string controllerName;
    std::string nameServerIdentifier;

    void initOptionsDescription();
    void initLabelToDataTypeMap();

    void parseCommandLineOptions(int argc, char* argv[]);
    void parseOptions();
    void setPortInfos(const char* optionLabel, PortInfoMap& portInfos);
    void addPortConnection(const std::string& value);
    
    void setPreLoadModuleInfo();
    void addModuleInfo(const std::string& value);
    void addTimeRateInfo(const std::string& value);
    
    void extractParameters(const std::string& str, std::vector<std::string>& result, const char delimiter = ':');
    std::string expandEnvironmentVariables(const std::string& str);
};

#endif 
