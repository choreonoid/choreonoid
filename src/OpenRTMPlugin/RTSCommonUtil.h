/*!
 * @brief  This is a definition of RTSystemEditorPlugin.
 * @author Hisashi Ikari
 * @file
 */
#ifndef CNOID_OPENRTM_PLUGIN_RTS_COMMON_UTIL_H_INCLUDED
#define CNOID_OPENRTM_PLUGIN_RTS_COMMON_UTIL_H_INCLUDED

#include <vector>
#include "RTSystemItem.h"

using namespace std;

namespace cnoid {

/*!
  * @brief Common processing of RTS.
  */
class TypeComparer
{
public:
    virtual std::string match(std::string type1, std::string type2) = 0;
};

class DataTypeComparer : public TypeComparer
{
public:
    std::string match(std::string type1, std::string type2);
};

class ignoreCaseComparer : public TypeComparer
{
public:
    std::string match(std::string type1, std::string type2);
};

struct NameServerInfo
{
    std::string hostAddress;
    int portNo;
    bool isRtmDefaultNameServer;

    NameServerInfo()
    {
        this->hostAddress = "";
        this->portNo = -1;
        this->isRtmDefaultNameServer = false;
    };
    NameServerInfo(std::string host, int port, bool isRtmDefaultNameServer)
        : hostAddress(host), portNo(port), isRtmDefaultNameServer(isRtmDefaultNameServer)
    {
    };
};

class RTCCommonUtil
{
public:
    static void splitPortName(std::string& value);
    static void splitPortName(std::string& value, std::vector<std::string>& result);

    static std::vector<std::string> split(const std::string &str, char delim);
    static bool isAllowAnyDataType(RTSPort* source, RTSPort* target);

    static std::vector<std::string> getAllowDataTypes(RTSPort* source, RTSPort* target);
    static std::vector<std::string> getAllowInterfaceTypes(RTSPort* source, RTSPort* target);
    static std::vector<std::string> getAllowDataflowTypes(RTSPort* source, RTSPort* target);
    static std::vector<std::string> getAllowSubscriptionTypes(RTSPort* source, RTSPort* target);

    static bool isIFR(std::string type);
    static bool compareIgnoreCase(const std::string& lhs, const std::string& rhs);

    static NameServerInfo getManagerAddress();

private:
    static std::vector<std::string> getAllowList(std::vector<std::string>& source, std::vector<std::string>& target, TypeComparer& comparer);
    static bool isAnyString(std::string target);
    static bool isExistAny(std::vector<std::string> target);


};

struct ServerFullComparator
{
    NameServerInfo target_;

    ServerFullComparator(NameServerInfo target)
    {
        target_ = target;
    }
    bool operator()(const NameServerInfo elem) const
    {
        return (target_.hostAddress == elem.hostAddress 
                    && target_.portNo == elem.portNo
                    && target_.isRtmDefaultNameServer == elem.isRtmDefaultNameServer);
    }
};

struct ServerComparator
{
    string hostAddress_;
    int portNo_;

    ServerComparator(string hostAddress, int portNo)
    {
        hostAddress_ = hostAddress;
        portNo_ = portNo;
    }
    bool operator()(const NameServerInfo elem) const
    {
        return (hostAddress_ == elem.hostAddress && portNo_ == elem.portNo);
    }
};

class NameServerManager
{
public:
    static NameServerManager* instance();
    ~NameServerManager()
    {
        if (handler) {
            delete handler;
        }
        delete ncHelper;
    };

    inline NamingContextHelper* getNCHelper()
    {
        return ncHelper;
    }

    inline std::vector<NameServerInfo> getServerList()
    {
        return serverList;
    }
    void addNameServer(NameServerInfo source);
    void addRtmDefaultNameServer();
    bool isExistingNameServer(NameServerInfo source);
    bool isRtmDefaultNameServer(string hostAddress, int portNo);
    void removeNameServer(string target);

private:
    static NameServerManager* handler;
    NameServerManager()
    {
        ncHelper = new NamingContextHelper();
    };

    NamingContextHelper* ncHelper;
    std::vector<NameServerInfo> serverList;
};

};

#endif
