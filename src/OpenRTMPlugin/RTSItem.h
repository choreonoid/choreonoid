/*!
 * @brief  This is a definition of RTSystemEditorPlugin.
 * @author Hisashi Ikari
 * @file
 */
#ifndef CNOID_OPENRTM_PLUGIN_RTS_ITEM_H
#define CNOID_OPENRTM_PLUGIN_RTS_ITEM_H

#include <cnoid/Item>
#include <cnoid/EigenUtil>
#include <cnoid/IdPair>
#include <rtm/idl/RTC.hh>
#include <rtm/NVUtil.h>
#include <rtm/CORBA_SeqUtil.h>
#include <QPoint>
#include <list>
#include "exportdecl.h"

namespace cnoid {

class RTSComp;
class RTSPort : public Referenced
{
public :
    RTSPort(const std::string& name, RTC::PortService_var port, RTSComp* parent);
    RTSComp* rtsComp;
    std::string name;
    RTC::PortService_var port;
    bool isServicePort;
    bool isInPort;

    bool connected();
    void stateCheck();
    bool checkConnectablePort(RTSPort* target);
    bool connectedWith(RTSPort* target);
};
typedef ref_ptr<RTSPort> RTSPortPtr;

class RTSConnection : public Referenced
{
public :
    RTSConnection(const std::string& id, const std::string& name, const std::string& sourceRtcName,
            const std::string& sourcePortName, const std::string& targetRtcName, const std::string& targetPortName);
    std::string id;
    std::string name;
    std::string sourceRtcName;
    std::string sourcePortName;
    std::string targetRtcName;
    std::string targetPortName;
    std::string dataflow;
    std::string sinterface;
    std::string subscription;
    float  pushRate;
    std::string pushPolicy;
    bool isAlive_;

    RTSComp* srcRTC;
    RTSPort* sourcePort;
    RTSComp* targetRTC;
    RTSPort* targetPort;

    Vector2 position[6];
    bool setPos;

    bool connect();
    bool disConnect();
    void setPosition(const Vector2 pos[]){
        for(int i=0; i<6; i++)
            position[i] = pos[i];
        setPos = true;
    }
    bool isAlive(){
        return isAlive_;
    };
};
typedef ref_ptr<RTSConnection> RTSConnectionPtr;

class RTSystemItemImpl;
class RTSComp : public Referenced
{
public :
    RTSComp(const std::string& name, RTC::RTObject_ptr rtc, RTSystemItemImpl* impl, const QPointF& pos);

    RTSystemItemImpl* impl;
    RTC::RTObject_ptr rtc_;
    std::string name;
    RTC::ExecutionContextList_var ownedExeContList;
    RTC::ExecutionContextList_var participatingExeContList;
    std::map<std::string, RTSPortPtr> inPorts;
    std::map<std::string, RTSPortPtr> outPorts;
    QPointF pos;

    bool isActive();
    bool connectionCheckSub(RTSPort* rtsPort);
    bool connectionCheck();
    void setRtc(RTC::RTObject_ptr rtc);

    RTSPort* nameToRTSPort(const std::string& name);

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
    static void initialize(ExtensionManager* ext);

    RTSComp* addRTSComp(const std::string& name, const QPointF& pos);
    void deleteRTSComp(const std::string& name);
    bool compIsAlive(RTSComp* rtsComp);
    RTSComp* nameToRTSComp(const std::string& name);
    RTSConnection* addRTSConnection(const std::string& id, const std::string& name,
            RTSPort* sourcePort, RTSPort* targetPort, const std::string& dataflow, const std::string& subscription);
    bool connectionCheck();
    void RTSCompToConnectionList(const RTSComp* rtsComp,
                                 std::list<RTSConnection*>& rtsConnectionList, int mode=0);
    RTSConnectionMap& rtsConnections();
   // std::map<std::string, RTSConnectionPtr>& deletedRtsConnections();
    void deleteRtsConnection(const RTSConnection* connection);
    std::map<std::string, RTSCompPtr>& rtsComps();

    bool autoConnection;

protected :
    virtual Item* doDuplicate() const;
    virtual void doPutProperties(PutPropertyFunction& putProperty);
    virtual bool store(Archive& archive);
    virtual bool restore(const Archive& archive);

private:
    RTSystemItemImpl* impl;
};

typedef ref_ptr<RTSystemItem> RTSystemItemPtr;
}

#endif
