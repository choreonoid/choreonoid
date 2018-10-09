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
#include "ProfileHandlerExt.h"
#include "exportdecl.h"

namespace cnoid {

class RTSCompExt;
class RTSystemExtItem;
class RTSystemExtItemImpl;

class NamedValueExt
{
public:
    std::string name_;
    std::string value_;

    NamedValueExt(std::string name, std::string value)
    {
        name_ = name;
        value_ = value;
    };
};
typedef std::shared_ptr<NamedValueExt> NamedValueExtPtr;

class PortInterfaceExt
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
typedef std::shared_ptr<PortInterfaceExt> PortInterfaceExtPtr;

class RTSPortExt : public Referenced
{
public:
    RTSCompExt* rtsComp;
    std::string name;
    RTC::PortService_var port;
    bool isServicePort;
    bool isInPort;
    std::vector<PortInterfaceExtPtr>  interList;
    bool isConnected_;

    RTSPortExt(const std::string& name, RTC::PortService_var port, RTSCompExt* parent);
    bool isConnected();
    bool isConnectedWith(RTSPortExt* target);
    bool checkConnectablePort(RTSPortExt* target);
    std::vector<std::string> getDataTypes();
    std::vector<std::string> getInterfaceTypes();
    std::vector<std::string> getDataflowTypes();
    std::vector<std::string> getSubscriptionTypes();

private:
    std::vector<std::string> getProperty(const std::string& key);
};

typedef ref_ptr<RTSPortExt> RTSPortExtPtr;

class RTSConnectionExt : public Referenced
{
public:
    std::string id;
    std::string name;
    std::string sourceRtcName;
    std::string sourcePortName;
    std::string targetRtcName;
    std::string targetPortName;
    std::vector<NamedValueExtPtr> propList;
    RTSCompExt* srcRTC;
    RTSPortExt* sourcePort;
    RTSCompExt* targetRTC;
    RTSPortExt* targetPort;
    Vector2 position[6];
    bool setPos;

    DataPortConnector dataProfile;
    ServicePortConnector serviceProfile;

    RTSConnectionExt(
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

    friend class RTSCompExt;
    friend class RTSystemExtItem;
    friend class RTSystemExtItemImpl;
};

typedef ref_ptr<RTSConnectionExt> RTSConnectionExtPtr;

class RTSCompExt : public Referenced, public RTCWrapper
{
public:
    std::string name;
    std::string fullPath;
    std::string hostAddress;
    int portNo;
    std::vector<RTSPortExtPtr> inPorts;
    std::vector<RTSPortExtPtr> outPorts;

    Component profile;
    bool isAlive_;
    RTC_STATUS rtc_status_;

    RTSCompExt(const std::string& name, const std::string& fullPath, RTC::RTObject_ptr rtc, RTSystemExtItem* rts, const QPointF& pos, const std::string& host, int port);
    RTSystemExtItem* rts() { return rts_; }
    RTSPortExt* nameToRTSPort(const std::string& name);
    const QPointF& pos() const { return pos_; }
    void moveToRelative(const QPointF& p);

private:
    RTSystemExtItem* rts_;
    RTC::ExecutionContextList_var participatingExeContList;
    QPointF pos_;

    void setRtc(RTC::RTObject_ptr rtc);
    bool connectionCheck();
    bool connectionCheckSub(RTSPortExt* rtsPort);
    bool getComponentPath(RTC::PortService_ptr source, std::string& out_path);

    friend class RTSystemExtItemImpl;
};

typedef ref_ptr<RTSCompExt> RTSCompExtPtr;

/*!
 * @brief This is the RTSystem item.
 */
class CNOID_EXPORT RTSystemExtItem : public Item
{
public:
    typedef cnoid::IdPair<RTSPortExt*> RTSPortPair;
    typedef std::map<RTSPortPair, RTSConnectionExtPtr> RTSConnectionMap;
    RTSystemExtItem();
    RTSystemExtItem(const RTSystemExtItem& org);
    virtual ~RTSystemExtItem();
    static void initializeClass(ExtensionManager* ext);

    RTSCompExt* addRTSComp(const std::string& name, const QPointF& pos);
    RTSCompExt* addRTSComp(const NamingContextHelper::ObjectInfo& info, const QPointF& pos);
    void deleteRTSComp(const std::string& name);
    bool compIsAlive(RTSCompExt* rtsComp);
    RTSCompExt* nameToRTSComp(const std::string& name);
    std::map<std::string, RTSCompExtPtr>& rtsComps();

    RTSConnectionExt* addRTSConnection(
        const std::string& id, const std::string& name,
        RTSPortExt* sourcePort, RTSPortExt* targetPort, const std::vector<NamedValueExtPtr>& propList,
        const Vector2 pos[]);
    bool connectionCheck();
    void RTSCompToConnectionList(const RTSCompExt* rtsComp, std::list<RTSConnectionExt*>& rtsConnectionList, int mode = 0);
    RTSConnectionMap& rtsConnections();
    void disconnectAndRemoveConnection(RTSConnectionExt* connection);

    bool loadRtsProfile(const std::string& filename);
    bool saveRtsProfile(const std::string& filename);

    void setVendorName(const std::string& name);
    void setVersion(const std::string& version);
    int stateCheck() const;

    void checkStatus();
    void onActivated();

    SignalProxy<void(bool isRestored)> sigLoaded();
    SignalProxy<void(bool)> sigStatusUpdate();

protected:
    virtual Item* doDuplicate() const override;
    virtual void doPutProperties(PutPropertyFunction& putProperty) override;
    virtual bool store(Archive& archive) override;
    virtual bool restore(const Archive& archive) override;

private:
    RTSystemExtItemImpl* impl;

    friend class RTSCompExt;
};

typedef ref_ptr<RTSystemExtItem> RTSystemItemExtPtr;
}

#endif
