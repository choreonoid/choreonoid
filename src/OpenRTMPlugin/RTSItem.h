/*!
 * @brief  This is a definition of RTSystemEditorPlugin.
 * @author Hisashi Ikari
 * @file
 */
#ifndef CNOID_OPENRTM_PLUGIN_RTS_ITEM_H_INCLUDED
#define CNOID_OPENRTM_PLUGIN_RTS_ITEM_H_INCLUDED

#include <cnoid/Item>
#include <cnoid/EigenUtil>
#include <cnoid/IdPair>
#include <rtm/idl/RTC.hh>
#include <rtm/NVUtil.h>
#include <rtm/CORBA_SeqUtil.h>
#include <QPoint>
#include "exportdecl.h"

using namespace std;
using namespace RTC;

namespace cnoid {

class RTSComp;
class RTSPort : public Referenced
{
public :
    RTSPort(const string& name, PortService_var port, RTSComp* parent);
    RTSComp* rtsComp;
    string name;
    PortService_var port;
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
    RTSConnection(const string& id, const string& name, const string& sourceRtcName,
            const string& sourcePortName, const string& targetRtcName, const string& targetPortName);
    string id;
    string name;
    string sourceRtcName;
    string sourcePortName;
    string targetRtcName;
    string targetPortName;
    string dataflow;
    string sinterface;
    string subscription;
    float  pushRate;
    string pushPolicy;
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
    RTSComp(const string& name, RTC::RTObject_ptr rtc, RTSystemItemImpl* impl, const QPointF& pos);

    RTSystemItemImpl* impl;
    RTObject_ptr rtc_;
    string name;
    ExecutionContextList_var ownedExeContList;
    ExecutionContextList_var participatingExeContList;
    map<string, RTSPortPtr> inPorts;
    map<string, RTSPortPtr> outPorts;
    QPointF pos;

    bool isActive();
    bool connectionCheckSub(RTSPort* rtsPort);
    bool connectionCheck();
    void setRtc(RTObject_ptr rtc);

    RTSPort* nameToRTSPort(const string& name);

};
typedef ref_ptr<RTSComp> RTSCompPtr;

/*!
 * @brief This is the RTSystem item.
 */
class CNOID_EXPORT RTSystemItem : public Item
{
public:
    typedef cnoid::IdPair<RTSPort*> RTSPortPair;
    typedef map<RTSPortPair, RTSConnectionPtr> RTSConnectionMap;
    RTSystemItem();
    RTSystemItem(const RTSystemItem& org);
    virtual ~RTSystemItem();
    static void initialize(ExtensionManager* ext);

    RTSComp* addRTSComp(const string& name, const QPointF& pos);
    void deleteRTSComp(const string& name);
    bool compIsAlive(RTSComp* rtsComp);
    RTSComp* nameToRTSComp(const string& name);
    RTSConnection* addRTSConnection(const string& id, const string& name,
            RTSPort* sourcePort, RTSPort* targetPort, const string& dataflow, const string& subscription);
    bool connectionCheck();
    void RTSCompToConnectionList(const RTSComp* rtsComp,
            list<RTSConnection*>& rtsConnectionList, int mode=0);
    RTSConnectionMap& rtsConnections();
   // map<string, RTSConnectionPtr>& deletedRtsConnections();
    void deleteRtsConnection(const RTSConnection* connection);
    map<string, RTSCompPtr>& rtsComps();

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
