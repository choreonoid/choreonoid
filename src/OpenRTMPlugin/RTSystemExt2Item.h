#ifndef CNOID_OPENRTM_PLUGIN_RTS_EXT_ITEM_H
#define CNOID_OPENRTM_PLUGIN_RTS_EXT_ITEM_H

#include <cnoid/Item>
#include <cnoid/EigenUtil>
#include <cnoid/IdPair>
#include <cnoid/CorbaUtil>
#include <rtm/idl/RTC.hh>
#include <rtm/NVUtil.h>
#include <rtm/CORBA_SeqUtil.h>
#include <QPoint>
#include <list>
#include <string>
#include "RTCWrapper.h"
#include "ProfileHandlerExt2.h"
#include "exportdecl.h"

namespace cnoid {

class RTSCompExt2;
class RTSystemExt2Item;
class RTSystemExt2ItemImpl;

class NamedValueExt2
{
public:
    std::string name_;
    std::string value_;

    NamedValueExt2(std::string name, std::string value)
    {
        name_ = name;
        value_ = value;
    };
};
typedef std::shared_ptr<NamedValueExt2> NamedValueExt2Ptr;

class PortInterfaceExt2
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
typedef std::shared_ptr<PortInterfaceExt2> PortInterfaceExt2Ptr;

typedef ref_ptr<RTSPortExt2> RTSPortExt2Ptr;

class RTSPortExt2 : public Referenced
{
public:
    RTSCompExt2* rtsComp;
    std::string name;
    RTC::PortService_var port;
    bool isServicePort;
    bool isInPort;
    std::vector<PortInterfaceExt2Ptr>  interList;
    bool isConnected_;

    RTSPortExt2Ptr orgPort_;

    RTSPortExt2(const std::string& name, RTC::PortService_var port, RTSCompExt2* parent);
    bool checkConnected();
    bool isConnectedWith(RTSPortExt2* target);
    bool checkConnectablePort(RTSPortExt2* target);
    std::vector<std::string> getDataTypes();
    std::vector<std::string> getInterfaceTypes();
    std::vector<std::string> getDataflowTypes();
    std::vector<std::string> getSubscriptionTypes();
    RTSPortExt2Ptr copyForChecking();

private:
    std::vector<std::string> getProperty(const std::string& key);
};


class RTSConnectionExt2 : public Referenced
{
public:
    std::string id;
    std::string name;
    std::string sourceRtcName;
    std::string sourcePortName;
    std::string targetRtcName;
    std::string targetPortName;
    std::vector<NamedValueExt2Ptr> propList;
    RTSCompExt2* srcRTC;
    RTSPortExt2* sourcePort;
    RTSCompExt2* targetRTC;
    RTSPortExt2* targetPort;
    Vector2 position[6];
    bool setPos;

    DataPortConnector dataProfile;
    ServicePortConnector serviceProfile;

    RTSConnectionExt2(
        const std::string& id, const std::string& name, const std::string& sourceRtcName,
        const std::string& sourcePortName, const std::string& targetRtcName, const std::string& targetPortName);
    bool isAlive() { return isAlive_; };
    void setPosition(const Vector2 pos[]);
    RTSConnectionExt2Ptr copyForChecking();

private:
    bool isAlive_;

    /**
       \return true if the connection is newly established.
       false if the connection has already been established, or the connection failed.
    */
    bool connect();

    bool disconnect();

    friend class RTSCompExt2;
    friend class RTSystemExt2Item;
    friend class RTSystemExt2ItemImpl;
};

typedef ref_ptr<RTSConnectionExt2> RTSConnectionExt2Ptr;

class RTSCompExt2 : public Referenced, public RTCWrapper
{
public:
    std::string name;
    std::string fullPath;
    std::string hostAddress;
    int portNo;
    bool isDefaultNS;
    std::vector<RTSPortExt2Ptr> inPorts;
    std::vector<RTSPortExt2Ptr> outPorts;

    Component profile;
    bool isAlive_;
    RTC_STATUS rtc_status_;

    RTSCompExt2Ptr orgComp_;
    bool isSetRtc_;
    RTC::RTObject_ptr rtcCheck_;

    RTSCompExt2(const std::string& name, const std::string& fullPath, RTC::RTObject_ptr rtc, RTSystemExt2Item* rts, const QPointF& pos, const std::string& host, int port, bool isDefault);
    RTSystemExt2Item* rts() { return rts_; }
    RTSPortExt2* nameToRTSPort(const std::string& name);
    const QPointF& pos() const { return pos_; }
    void moveToRelative(const QPointF& p);
    RTSCompExt2Ptr copyForChecking();

private:
    RTSystemExt2Item* rts_;
    RTC::ExecutionContextList_var participatingExeContList;
    QPointF pos_;

    void setRtc(RTC::RTObject_ptr rtc);
    bool connectionCheck();
    bool connectionCheckSub(RTSPortExt2* rtsPort);
    bool connectionCheckForChecking();
    bool connectionCheckSubForChecking(RTSPortExt2* rtsPort);
    bool getComponentPath(RTC::PortService_ptr source, std::string& out_path);

    friend class RTSystemExt2ItemImpl;
};

typedef ref_ptr<RTSCompExt2> RTSCompExt2Ptr;

/*!
 * @brief This is the RTSystem item.
 */
class CNOID_EXPORT RTSystemExt2Item : public Item
{
public:
    typedef cnoid::IdPair<RTSPortExt2*> RTSPortPair;
    typedef std::map<RTSPortPair, RTSConnectionExt2Ptr> RTSConnectionMap;
    RTSystemExt2Item();
    RTSystemExt2Item(const RTSystemExt2Item& org);
    virtual ~RTSystemExt2Item();
    static void initializeClass(ExtensionManager* ext);

    RTSCompExt2* addRTSComp(const std::string& name, const QPointF& pos);
    RTSCompExt2* addRTSComp(const NamingContextHelper::ObjectInfo& info, const QPointF& pos);
    void deleteRTSComp(const std::string& name);
    RTSCompExt2* nameToRTSComp(const std::string& name);
    std::map<std::string, RTSCompExt2Ptr>& rtsComps();

    RTSConnectionExt2* addRTSConnection(
        const std::string& id, const std::string& name,
        RTSPortExt2* sourcePort, RTSPortExt2* targetPort, const std::vector<NamedValueExt2Ptr>& propList,
        const Vector2 pos[]);
    void RTSCompToConnectionList(const RTSCompExt2* rtsComp, std::list<RTSConnectionExt2*>& rtsConnectionList, int mode = 0);
    RTSConnectionMap& rtsConnections();
    void disconnectAndRemoveConnection(RTSConnectionExt2* connection);

    bool loadRtsProfile(const std::string& filename);
    bool saveRtsProfile(const std::string& filename);

    void setVendorName(const std::string& name);
    void setVersion(const std::string& version);
    int stateCheck() const;

    void checkStatus();
    bool isCheckAtLoading();
    void onActivated();

    SignalProxy<void(bool)> sigStatusUpdate();

protected:
    virtual Item* doDuplicate() const override;
    virtual void doPutProperties(PutPropertyFunction& putProperty) override;
    virtual bool store(Archive& archive) override;
    virtual bool restore(const Archive& archive) override;

private:
    RTSystemExt2ItemImpl* impl;

    friend class RTSCompExt2;
};

typedef ref_ptr<RTSystemExt2Item> RTSystemItemExt2Ptr;
}

#endif
