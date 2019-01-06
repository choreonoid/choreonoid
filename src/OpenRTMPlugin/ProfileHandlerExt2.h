#ifndef CNOID_OPENRTM_PLUGIN_PROFILE_HANDLER_EXT_H_INCLUDED
#define CNOID_OPENRTM_PLUGIN_PROFILE_HANDLER_EXT_H_INCLUDED

#include <string>
#include <vector>
#include <map>
#include <memory>
#include <ostream>
#include <cnoid/Referenced>
#include <cnoid/IdPair>
#include <rtm/idl/SDOPackage.hh>
#include <pugixml.hpp>

#include "ProfileHandler.h"

namespace cnoid {

class RTSystemExt2Item;

class RTSCompExt2;
typedef cnoid::ref_ptr<RTSCompExt2> RTSCompExt2Ptr;

class RTSPortExt2;
class RTSConnectionExt2;
typedef cnoid::IdPair<RTSPortExt2*> RTSPortExt2Pair;
typedef ref_ptr<RTSConnectionExt2> RTSConnectionExt2Ptr;
typedef std::map<RTSPortExt2Pair, RTSConnectionExt2Ptr> RTSConnectionExt2Map;

class ProfileHandlerExt2
{
public:
    ProfileHandlerExt2() {};
    ~ProfileHandlerExt2() {};

    static bool restoreRtsProfile(std::string targetFile, RTSystemExt2Item* impl);
    static bool getRtsProfileInfo(std::string targetFile, std::string& vendorName, std::string& version);

    static void saveRtsProfile(
            const std::string& targetFile, std::string& systemId,
            std::map<std::string, cnoid::RTSCompExt2Ptr>& comps, RTSConnectionExt2Map& connections,
            std::ostream& os);

private:
    static bool parseProfile(std::string targetFile, RtsProfile& profile);
    static void parseConfigurationSet(pugi::xml_node& comp, Component& proComp);
    static TargetPort parseTargetPort(const pugi::xml_node& targetPort);
    static void parseConnectorPosition(const pugi::xml_node& targetCon, PortConnector& profile);

    static RTSPortExt2* getTargetPort(std::string& sourceRtc, std::string& sourcePort, RTSystemExt2Item* impl);
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
    static void buildPortInfo(RTSPortExt2* port, Component& compProf, std::string direction);
    static TargetPort buildTargetPortInfo(RTSPortExt2* sourcePort);
    static void buildPosition(const RTSConnectionExt2* connect, int offsetX, int offsetY, std::vector<Property>& propList);

    static void appendStringValue(std::vector<Property>& target, std::string& name, std::string& value);
    static void removePropertyByValue(std::vector<Property>& target, const std::string& name);
};

}

#endif 
