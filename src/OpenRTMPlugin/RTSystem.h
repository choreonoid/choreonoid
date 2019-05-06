#ifndef CNOID_OPENRTM_PLUGIN_RT_SYSTEM_H
#define CNOID_OPENRTM_PLUGIN_RT_SYSTEM_H

#include "RTSCommonUtil.h"
#include "RTCWrapper.h"
#include <cnoid/Referenced>
#include <QPoint>

namespace cnoid {

class Item;

class RTSystem
{
public:
    virtual RTSComp* addRTSComp(const NamingContextHelper::ObjectInfo& info, const QPointF& pos) = 0;
    virtual RTSComp* nameToRTSComp(const std::string& name) = 0;
    virtual RTSConnection* addRTSConnection(
        const std::string& id, const std::string& name, RTSPort* sourcePort, RTSPort* targetPort,
        const std::vector<NamedValuePtr>& propList, const Vector2 pos[]) = 0;
    virtual void setVendorName(const std::string& name) = 0;
    virtual void setVersion(const std::string& version) = 0;
};

class PortInterface
{
public:
    std::string rtc_name;
    std::string port_name;
    std::string if_polarity;
    std::string if_tname;
    std::string if_iname;

    bool isRequiredPolarity()
    {
        return if_polarity == "required";
    };

    std::string toDispStr()
    {
        return rtc_name + ":" + if_tname + ":" + if_iname;
    };
    std::string toStr()
    {
        return rtc_name + ".port." + port_name + "." + if_polarity + "." + if_tname + "." + if_iname;
    };
};
typedef std::shared_ptr<PortInterface> PortInterfacePtr;

class RTSPort;
typedef ref_ptr<RTSPort> RTSPortPtr;

// Implementation is written in RTSystemItem.cpp
class RTSComp : public Referenced, public RTCWrapper
{
public:
    std::string name;
    std::string fullPath;
    std::string hostAddress;
    int portNo;
    bool isDefaultNS;
    std::vector<RTSPortPtr> inPorts;
    std::vector<RTSPortPtr> outPorts;

    Component profile;
    bool isAlive_;

    RTC_STATUS rtc_status_; // Ext
    RTSCompPtr orgComp_; // Ext2
    bool isSetRtc_; // Ext2
    RTC::RTObject_ptr rtcCheck_; // Ext2

    RTSComp(const std::string& name, const std::string& fullPath, RTC::RTObject_ptr rtc, Item* rtsItem, const QPointF& pos, const std::string& host, int port, bool isDefault);
    Item* rtsItem() { return rtsItem_; }
    void setRtc(RTC::RTObject_ptr rtc);
    RTSPort* nameToRTSPort(const std::string& name);
    const QPointF& pos() const { return pos_; }
    void setPos(const QPointF& p); // Non-Ext
    void moveToRelative(const QPointF& p); // Ext
    RTSCompPtr copyForChecking(); // Ext2
    bool connectionCheck();
    bool connectionCheckForChecking(); // Ext2

private:
    Item* rtsItem_;
    RTC::ExecutionContextList_var participatingExeContList;
    QPointF pos_;

    bool connectionCheckSub(RTSPort* rtsPort);
    bool connectionCheckSubForChecking(RTSPort* rtsPort); // Ext2
    bool getComponentPath(RTC::PortService_ptr source, std::string& out_path);
};

typedef ref_ptr<RTSComp> RTSCompPtr;

// Implementation is written in RTSystemItem.cpp
class RTSPort : public Referenced
{
public:
    RTSComp* rtsComp;
    std::string name;
    RTC::PortService_var port;
    bool isServicePort;
    bool isInPort;
    std::vector<PortInterfacePtr> interList;
    bool isConnected_; // Ext

    RTSPortPtr orgPort_; // Ext2

    RTSPort(const std::string& name, RTC::PortService_var port, RTSComp* parent);
    bool isConnected();
    bool checkConnected(); //  Ext2
    bool isConnectedWith(RTSPort* target);
    bool checkConnectablePort(RTSPort* target);
    std::vector<std::string> getDataTypes();
    std::vector<std::string> getInterfaceTypes();
    std::vector<std::string> getDataflowTypes();
    std::vector<std::string> getSubscriptionTypes();
    RTSPortPtr copyForChecking(); // Ext2

private:
    std::vector<std::string> getProperty(const std::string& key);
};

// Implementation is written in RTSystemItem.cpp
class RTSConnection : public Referenced
{
public:
    std::string id;
    std::string name;
    std::string sourceRtcName;
    std::string sourcePortName;
    std::string targetRtcName;
    std::string targetPortName;
    std::vector<NamedValuePtr> propList;
    RTSComp* srcRTC;
    RTSPort* sourcePort;
    RTSComp* targetRTC;
    RTSPort* targetPort;
    Vector2 position[6];
    bool setPos;
    bool isAlive_;

    DataPortConnector dataProfile;
    ServicePortConnector serviceProfile;

    RTSConnection(
        const std::string& id, const std::string& name, const std::string& sourceRtcName,
        const std::string& sourcePortName, const std::string& targetRtcName, const std::string& targetPortName);
    bool isAlive() { return isAlive_; };
    void setPosition(const Vector2 pos[]);

    /**
       \return true if the connection is newly established.
       false if the connection has already been established, or the connection failed.
    */
    bool connect();

    bool disconnect();

    RTSConnectionPtr copyForChecking(); // Ext2
};

typedef ref_ptr<RTSConnection> RTSConnectionPtr;

}

#endif

    

    
