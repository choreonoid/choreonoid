#ifndef CNOID_OPENRTM_PLUGIN_PROFILE_HANDLER_H_INCLUDED
#define CNOID_OPENRTM_PLUGIN_PROFILE_HANDLER_H_INCLUDED

#include <string>
#include <vector>
#include <map>
#include <memory>
#include <ostream>
#include <cnoid/Referenced>
#include <cnoid/EigenTypes>
#include <cnoid/IdPair>
#include <rtm/idl/SDOPackage.hh>
#include <pugixml.hpp>

namespace cnoid {

class Property
{
public:
    std::string name;
    std::string value;
};

class ExecutionContext
{
public:
    std::string id;
    std::string kind;
    double rate;
    std::vector<Property> propertyList;
};

class ConfigurationSet
{
public:
    std::string id;
    std::vector<Property> dataList;
};

class ServicePort
{
public:
    std::string name;
    std::vector<Property> propertyList;
};

class DataPort
{
public:
    std::string name;
    std::string direction;
    std::vector<Property> propertyList;
};

class Component
{
public:
    std::string id;
    std::string instanceName;
    std::string pathUri;
    std::string activeConfigurationSet;
    bool isRequired;
    int posX;
    int posY;
    bool isRegisteredInRtmDefaultNameServer;
    //
    std::vector<DataPort> dataPortList;
    std::vector<ServicePort> servicePortList;
    std::vector<ConfigurationSet> configList;
    std::vector<ExecutionContext> ecList;
    std::vector<Property> propertyList;

    Component()
    {
        activeConfigurationSet = "default";
        posX = 0;
        posY = 0;
    }
};
/////
class TargetPort
{
public:
    std::string portName;
    std::string instanceName;
    std::string componentId;
    std::string pathId;

    std::vector<Property> propertyList;
};

class PortConnector
{
public:
    std::string connectorId;
    std::string name;

    std::vector<Property> propertyList;
    Vector2 pos[6];

    TargetPort source;
    TargetPort target;
};

class DataPortConnector : public PortConnector
{
public:
    std::string dataType;
    std::string interfaceType;
    std::string dataflowType;
    std::string subscriptionType;
    double pushInterval;
};
/////
class ServicePortConnector :public PortConnector
{
public:
    std::string transMethod;
};

class RtsProfile
{
public:
    std::string id;
    std::vector<Component> compList;
    std::vector<DataPortConnector> dataConnList;
    std::vector<ServicePortConnector> serviceConnList;
};

class RTSystem;

class RTSComp;
typedef cnoid::ref_ptr<RTSComp> RTSCompPtr;

class RTSPort;
class RTSConnection;
typedef cnoid::IdPair<RTSPort*> RTSPortPair;
typedef ref_ptr<RTSConnection> RTSConnectionPtr;
typedef std::map<RTSPortPair, RTSConnectionPtr> RTSConnectionMap;

class ProfileHandler
{
public:
    ProfileHandler() {};
    ~ProfileHandler() {};

    static bool restoreRtsProfile(std::string targetFile, RTSystem* rts);
    static bool getRtsProfileInfo(std::string targetFile, std::string& vendorName, std::string& version);

    static void saveRtsProfile(
            const std::string& targetFile, std::string& systemId,
            std::map<std::string, cnoid::RTSCompPtr>& comps, RTSConnectionMap& connections,
            std::ostream& os);

private:
    static bool parseProfile(std::string targetFile, RtsProfile& profile);
    static void parseConfigurationSet(pugi::xml_node& comp, Component& proComp);
    static TargetPort parseTargetPort(const pugi::xml_node& targetPort);
    static void parseConnectorPosition(const pugi::xml_node& targetCon, PortConnector& profile);

    static RTSPort* getTargetPort(std::string& sourceRtc, std::string& sourcePort, RTSystem* rts);
    /////
    static bool writeProfile(const std::string& targetFile, RtsProfile& profile, std::ostream& os);
    static void writeComponent(std::vector<Component>& compList, pugi::xml_node& parent);
    static void writeDataPort(std::vector<DataPort>& portList, pugi::xml_node& parent);
    static void writeServicePort(std::vector<ServicePort>& portList, pugi::xml_node& parent);
    static void writeConfigurationSet(std::vector<ConfigurationSet>& configList, pugi::xml_node& parent);
    static void writeExecutionContext(std::vector<ExecutionContext>& ecList, pugi::xml_node& parent);
    static void writeLocation(Component& target, pugi::xml_node& parent);
    static void writeProperty(std::vector<Property>& propList, pugi::xml_node& parent);

    static void writeDataConnector(std::vector<DataPortConnector>& connList, pugi::xml_node& parent);
    static void writeServiceConnector(std::vector<ServicePortConnector>& connList, pugi::xml_node& parent);
    static void writeTargetPort(TargetPort& target, std::string tag, pugi::xml_node& parent);

    static void copyNVListToProperty(SDOPackage::NVList& source, std::vector<Property>& target);
    static void buildPortInfo(RTSPort* port, Component& compProf, std::string direction);
    static TargetPort buildTargetPortInfo(RTSPort* sourcePort);
    static void buildPosition(const RTSConnection* connect, int offsetX, int offsetY, std::vector<Property>& propList);

    static void appendStringValue(std::vector<Property>& target, std::string& name, std::string& value);
    static void removePropertyByValue(std::vector<Property>& target, const std::string& name);

};

}

#endif 
