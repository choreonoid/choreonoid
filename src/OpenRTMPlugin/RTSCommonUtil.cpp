/*!
 * @brief  This is a definition of RTSystemEditorPlugin.
 * @author Hisashi Ikari
 * @file
 */
#include "RTSCommonUtil.h"
#include "LoggerUtil.h"
#include "gettext.h"
#include <rtm/Manager.h>
#include <QString>
#include <string>
#include <cassert>

using namespace std;

namespace cnoid {

string DataTypeComparer::match(string type1, string type2)
{
    bool isIFR1 = RTCCommonUtil::isIFR(type1);
    bool isIFR2 = RTCCommonUtil::isIFR(type2);
    // For IFR formats (1.1) and simple forms (1.0), default type comparison  
    if (isIFR1 == isIFR2) {
        if (RTCCommonUtil::compareIgnoreCase(type1, type2)) {
            return type1;
        }
        return "";
    }
    // When 1.1 / 1.0 is mixed, fuzzy comparison by backward matching  
    string ifrType;
    string oldType;
    if (isIFR1) {
        ifrType = type1;
        oldType = type2;
    } else if (isIFR2) {
        ifrType = type2;
        oldType = type1;
    }
    if (ifrType.empty()) {
        return "";
    }
    vector<string> ifr = RTCCommonUtil::split(ifrType, ':');
    vector<string> ifrSeg = RTCCommonUtil::split(ifr[1], '/');
    vector<string> oldSeg = RTCCommonUtil::split(oldType, ':');
    if (oldSeg.size() > ifrSeg.size()) {
        return "";
    }
    for (int i = 1; i <= oldSeg.size(); i++) {
        string s1 = oldSeg[oldSeg.size() - i];
        string s2 = ifrSeg[ifrSeg.size() - i];
        if (RTCCommonUtil::compareIgnoreCase(s1, s2)) {
            return "";
        }
    }
    // Using IFR format for ConnectorProfile when mixed 1.1 / 1.0  
    return ifrType;
}

string ignoreCaseComparer::match(string type1, string type2)
{
    if (RTCCommonUtil::compareIgnoreCase(type1, type2)) {
        return type1;
    }
    return "";
}
////////////////////
void RTCCommonUtil::splitPortName(string& value)
{
    if (string::npos == value.find(".")) {
        return;
    }
    vector<string> results = split(value, '.');
    assert(0 < results.size());
    value = results[results.size() - 1];
}


void RTCCommonUtil::splitPortName(string& value, vector<string>& result)
{
    result.clear();
    if (string::npos == value.find(".")) {
        result.push_back(value);
        return;
    }
    vector<string> temp;
    temp = split(value, '.');
    result = temp;
}

vector<string> RTCCommonUtil::split(const std::string &str, char delim)
{
    vector<std::string> res;
    size_t current = 0, found;

    while ((found = str.find_first_of(delim, current)) != std::string::npos) {
        res.push_back(string(str, current, found - current));
        current = found + 1;
    }
    res.push_back(string(str, current, str.size() - current));
    return res;
}

bool RTCCommonUtil::isIFR(string type)
{
    if (type.empty()) return false;
    vector<string> ifr = split(type, ':');
    if (ifr.size() == 3 && ifr[0] == "IDL") {
        return true;
    }
    return false;
}

bool RTCCommonUtil::compareIgnoreCase(const string& lhs, const string& rhs)
{
    QString left = QString::fromStdString(lhs);
    QString right = QString::fromStdString(rhs);

    return QString::compare(left, right, Qt::CaseInsensitive) == 0;
}

NameServerInfo RTCCommonUtil::getManagerAddress()
{
    NameServerInfo result;

    RTC::Manager* rtcManager = &RTC::Manager::instance();
    if (!rtcManager) return result;

    coil::Properties prop = rtcManager->getConfig();
    std::string val = prop.getProperty("corba.nameservers");
    vector<string> elems = RTCCommonUtil::split(val, ':');
    DDEBUG_V("RTCCommonUtil::getManagerAddress: %s %d", val.c_str(), elems.size());
    if (elems.size() == 0) return result;
    result.hostAddress = elems[0];
    result.portNo = 2809;
    if (1 < elems.size()) {
        result.portNo = QString::fromStdString(elems[1]).toInt();
    }
    result.isRtmDefaultNameServer = true;

    return result;
}

NameServerManager *NameServerManager::handler = NULL;

NameServerManager* NameServerManager::instance()
{
    if (!handler) {
        handler = new NameServerManager();
    }
    return handler;
}

void NameServerManager::addNameServer(NameServerInfo source)
{
    DDEBUG_V("NameServerManager::addServer %s, %d", source.hostAddress.c_str(), source.portNo);
    vector<NameServerInfo>::iterator serverItr = find_if(serverList.begin(), serverList.end(), ServerFullComparator(source));
    if (serverItr != serverList.end()) return;
    serverList.push_back(source);
}

void NameServerManager::addRtmDefaultNameServer()
{
    NameServerInfo info = RTCCommonUtil::getManagerAddress();
    NameServerInfo nsInfo(info.hostAddress, info.portNo, info.isRtmDefaultNameServer);
    addNameServer(nsInfo);
}

bool NameServerManager::isExistingNameServer(NameServerInfo source)
{
    vector<NameServerInfo>::iterator serverItr = find_if(serverList.begin(), serverList.end(), ServerComparator(source.hostAddress, source.portNo));
    if (serverItr != serverList.end()) return true;
    return false;
}

bool NameServerManager::isRtmDefaultNameServer(string hostAddress, int portNo)
{
    for(unsigned int index=0; index<serverList.size(); index++) {
        NameServerInfo info = serverList[index];
        if (info.hostAddress != hostAddress || info.portNo != portNo) continue;
        if(info.isRtmDefaultNameServer) {
            return true;
        }
    }
    return false;
}

void NameServerManager::removeNameServer(NameServerInfo target) {
    serverList.erase(remove_if(serverList.begin(), serverList.end(), ServerFullComparator(target)));
}

void NameServerManager::clearNameServer() {
    serverList.clear();
}

}
