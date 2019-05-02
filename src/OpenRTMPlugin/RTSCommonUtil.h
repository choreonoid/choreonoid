/*!
 * @brief  This is a definition of RTSystemEditorPlugin.
 * @author Hisashi Ikari
 * @file
 */
#ifndef CNOID_OPENRTM_PLUGIN_RTS_COMMON_UTIL_H_INCLUDED
#define CNOID_OPENRTM_PLUGIN_RTS_COMMON_UTIL_H_INCLUDED

#include <cnoid/CorbaUtil>
#include <vector>
#include <string>
#include <memory>

using namespace std;

namespace {
// Old conf filename. This should be deprecated, but continue to use for a while
const char* DEFAULT_CONF_FILENAME = "./rtc.conf.choreonoid";

// New conf filename. It is desirable to use this.
//const char* DEFAUT_CONF_FILENAME = "./choreonoid.rtc.conf"

};

namespace cnoid {

class NamedValue
{
public:
    std::string name_;
    std::string value_;

    NamedValue(std::string name, std::string value)
    {
        name_ = name;
        value_ = value;
    };
};
typedef std::shared_ptr<NamedValue> NamedValuePtr;

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
        this->hostAddress = "localhost";
        this->portNo = 2809;
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

    static bool isIFR(std::string type);
    static bool compareIgnoreCase(const std::string& lhs, const std::string& rhs);

    static NameServerInfo getManagerAddress();
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
    void removeNameServer(NameServerInfo target);
    void clearNameServer();

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
