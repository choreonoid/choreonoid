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

#include <string>
#include "OpenRTMItem.h"

#include "exportdecl.h"

namespace cnoid {

class RTSComp;

class NamedValue {
public:
	std::string name_;
	std::string value_;

	NamedValue(std::string name, std::string value) {
		name_ = name;
		value_ = value;
	};
};
typedef std::shared_ptr<NamedValue> NamedValuePtr;

class PortInterface {
public:
	std::string rtc_name;
	std::string port_name;
	std::string if_polarity;
	std::string if_tname;
	std::string if_iname;

	bool isRequiredPolarity() {
		return if_polarity == "required";
	};

	std::string toDispStr() {
		return rtc_name + ":" + if_tname + ":" + if_iname;
	};
	std::string toStr() {
		return rtc_name + ".port." + port_name + "." + if_polarity + "." + if_tname + "." + if_iname;
	};
};
typedef std::shared_ptr<PortInterface> PortInterfacePtr;

class RTSPort : public Referenced {
public :
    RTSPort(const std::string& name, RTC::PortService_var port, RTSComp* parent);
    RTSComp* rtsComp;
    std::string name;
    RTC::PortService_var port;
    bool isServicePort;
    bool isInPort;
		std::vector<PortInterfacePtr>  interList;

    bool connected();
    void stateCheck();
    bool checkConnectablePort(RTSPort* target);
    bool connectedWith(RTSPort* target);

		std::vector<std::string> getDataTypes();
		std::vector<std::string> getInterfaceTypes();
		std::vector<std::string> getDataflowTypes();
		std::vector<std::string> getSubscriptionTypes();

private:
		std::vector<std::string> getProperty(const std::string& key);

};
typedef ref_ptr<RTSPort> RTSPortPtr;

struct RTSPortComparator {
	std::string name_;

	RTSPortComparator(std::string value) {
		name_ = value;
	}
	bool operator()(const RTSPortPtr elem) const {
		return (name_ == elem->name);
	}
};


class RTSConnection : public Referenced {
public :
    RTSConnection(const std::string& id, const std::string& name, const std::string& sourceRtcName,
            const std::string& sourcePortName, const std::string& targetRtcName, const std::string& targetPortName);
    std::string id;
    std::string name;
    std::string sourceRtcName;
    std::string sourcePortName;
    std::string targetRtcName;
    std::string targetPortName;

		std::vector<NamedValuePtr> propList;

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
class RTSComp : public Referenced, public RTCWrapper {
public :
    RTSComp(const std::string& name, RTC::RTObject_ptr rtc, RTSystemItemImpl* impl, const QPointF& pos);

    RTSystemItemImpl* impl;
    std::string name;
    RTC::ExecutionContextList_var participatingExeContList;
		std::vector<RTSPortPtr> inPorts;
		std::vector<RTSPortPtr> outPorts;

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
class CNOID_EXPORT RTSystemItem : public Item {
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
		RTSConnection* addRTSConnection(const std::string& id, const std::string& name,
						RTSPort* sourcePort, RTSPort* targetPort, const std::vector<NamedValuePtr>& propList);
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
