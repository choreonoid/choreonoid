#ifndef CNOID_OPENRTM_PLUGIN_RTS_ITEM_H
#define CNOID_OPENRTM_PLUGIN_RTS_ITEM_H

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
#include "ProfileHandler.h"
#include "exportdecl.h"

namespace cnoid {

class RTSComp;
class RTSystemItem;
class RTSystemItemImpl;

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

class RTSPort : public Referenced
{
public:
    RTSComp* rtsComp;
    std::string name;
    RTC::PortService_var port;
    bool isServicePort;
    bool isInPort;
    std::vector<PortInterfacePtr>  interList;

    RTSPort(const std::string& name, RTC::PortService_var port, RTSComp* parent);
    bool isConnected();
    bool isConnectedWith(RTSPort* target);
    bool checkConnectablePort(RTSPort* target);
    std::vector<std::string> getDataTypes();
    std::vector<std::string> getInterfaceTypes();
    std::vector<std::string> getDataflowTypes();
    std::vector<std::string> getSubscriptionTypes();

private:
    std::vector<std::string> getProperty(const std::string& key);
};

typedef ref_ptr<RTSPort> RTSPortPtr;

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

    DataPortConnector dataProfile;
    ServicePortConnector serviceProfile;

    RTSConnection(
        const std::string& id, const std::string& name, const std::string& sourceRtcName,
        const std::string& sourcePortName, const std::string& targetRtcName, const std::string& targetPortName);
    bool isAlive() { return isAlive_; };
    void setPosition(const Vector2 pos[]);

private:
    bool isAlive_;

    /**
       \return true if the connection is newly established.
       false if the connection has already been established, or the connection failed.
    */
    bool connect();

    bool disconnect();

    friend class RTSComp;
    friend class RTSystemItem;
    friend class RTSystemItemImpl;
};

typedef ref_ptr<RTSConnection> RTSConnectionPtr;

class RTSComp : public Referenced, public RTCWrapper
{
public:
    std::string name;
    std::string fullPath;
    std::string hostAddress;
    int portNo;
    std::vector<RTSPortPtr> inPorts;
    std::vector<RTSPortPtr> outPorts;

    Component profile;
    bool isAlive_;

    RTSComp(const std::string& name, const std::string& fullPath, RTC::RTObject_ptr rtc, RTSystemItem* rts, const QPointF& pos, const std::string& host, int port);
    RTSystemItem* rts() { return rts_; }
    RTSPort* nameToRTSPort(const std::string& name);
    const QPointF& pos() const { return pos_; }
    void setPos(const QPointF& p);

private:
    RTSystemItem* rts_;
    RTC::ExecutionContextList_var participatingExeContList;
    QPointF pos_;

    void setRtc(RTC::RTObject_ptr rtc);
    bool connectionCheck();
    bool connectionCheckSub(RTSPort* rtsPort);
    bool getComponentPath(RTC::PortService_ptr source, std::string& out_path);

    friend class RTSystemItemImpl;
};

typedef ref_ptr<RTSComp> RTSCompPtr;

/*!
 * @brief This is the RTSystem item.
 */
class CNOID_EXPORT RTSystemItem : public Item
{
public:
    typedef cnoid::IdPair<RTSPort*> RTSPortPair;
    typedef std::map<RTSPortPair, RTSConnectionPtr> RTSConnectionMap;
    RTSystemItem();
    RTSystemItem(const RTSystemItem& org);
    virtual ~RTSystemItem();
    static void initializeClass(ExtensionManager* ext);

    RTSComp* addRTSComp(const std::string& name, const QPointF& pos);
    RTSComp* addRTSComp(const NamingContextHelper::ObjectInfo& info, const QPointF& pos);
    void deleteRTSComp(const std::string& name);
    bool compIsAlive(RTSComp* rtsComp);
    RTSComp* nameToRTSComp(const std::string& name);
    std::map<std::string, RTSCompPtr>& rtsComps();

    RTSConnection* addRTSConnection(
        const std::string& id, const std::string& name,
        RTSPort* sourcePort, RTSPort* targetPort, const std::vector<NamedValuePtr>& propList,
        const Vector2 pos[]);
    bool connectionCheck();
    void RTSCompToConnectionList(const RTSComp* rtsComp, std::list<RTSConnection*>& rtsConnectionList, int mode = 0);
    RTSConnectionMap& rtsConnections();
    void disconnectAndRemoveConnection(RTSConnection* connection);

    bool loadRtsProfile(const std::string& filename);
    bool saveRtsProfile(const std::string& filename);

    int pollingCycle() const;
    void setVendorName(const std::string& name);
    void setVersion(const std::string& version);
    int stateCheck() const;

    bool checkStatus();

    SignalProxy<void(int)> sigTimerPeriodChanged();
    SignalProxy<void(bool)> sigTimerChanged();
    SignalProxy<void(bool isRestored)> sigLoaded();

protected:
    virtual Item* doDuplicate() const override;
    virtual void doPutProperties(PutPropertyFunction& putProperty) override;
    virtual bool store(Archive& archive) override;
    virtual bool restore(const Archive& archive) override;

private:
    RTSystemItemImpl* impl;

    friend class RTSComp;
};

typedef ref_ptr<RTSystemItem> RTSystemItemPtr;
}

#endif
