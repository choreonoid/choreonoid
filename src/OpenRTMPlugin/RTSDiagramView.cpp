/*!
 * @author Shizuko Hattori
 * @file
 */
#include "RTSDiagramView.h"
#include "RTSNameServerView.h"
#include "RTSPropertiesView.h"
#include "RTSCommonUtil.h"
#include <cnoid/ViewManager>
#include <cnoid/MessageView>
#include <cnoid/MenuManager>
#include <cnoid/ConnectionSet>
#include <cnoid/Dialog>
#include <cnoid/ComboBox>
#include <cnoid/Timer>
#include <QGraphicsView>
#include <QVBoxLayout>
#include <QDropEvent>
#include <QGraphicsItemGroup>
#include <QPushButton>
#include <QLabel>
#include <QLineEdit>
#include <QStyleOptionGraphicsItem>
#include <boost/bind.hpp>
#include <boost/algorithm/string.hpp>
#include <rtm/idl/RTC.hh>
#include <rtm/NVUtil.h>
#include <rtm/CORBA_SeqUtil.h>
#include "gettext.h"

using namespace cnoid;
using namespace std;
using namespace RTC;

namespace cnoid {

class RTSConnectionLineItem : public QGraphicsLineItem
{
public:
    RTSConnectionLineItem(qreal x1, qreal y1, qreal x2, qreal y2) :
        QGraphicsLineItem(x1, y1, x2, y2),
        x1(x1), x2(x2), y1(y1), y2(y2) {
        QPen pen;
        pen.setColor(QColor("black"));
        pen.setWidth(1);
        setPen(pen);
        //setFlags(QGraphicsItem::ItemIsMovable);
        cx = (x1+x2) /2.0;
        cy = (y1+y2) /2.0;
    }

    void update(){
        setLine(x1,y1,x2,y2);
        cx = (x1+x2) /2.0;
        cy = (y1+y2) /2.0;
    }

    void setPos(qreal x1_, qreal y1_, qreal x2_, qreal y2_)
    { x1 = x1_; y1 = y1_; x2 = x2_; y2 = y2_; update(); }
    void setSx(qreal x_){ x1 = x_; update(); }
    void setEx(qreal x_){ x2 = x_; update(); }
    void setSy(qreal y_){ y1 = y_; update(); }
    void setEy(qreal y_){ y2 = y_; update(); }
    void moveX(qreal x_){ x1 = x2 = x_; update(); }
    void moveY(qreal y_){ y1 = y2 = y_; update(); }

    qreal x1,x2,y1,y2;
    qreal cx,cy;
};

class RTSConnectionMarkerItem : public QGraphicsRectItem
{
public:
    enum markerType { UNMOVABLE, HORIZONTAL, VERTIAL };
    markerType type;
    Signal<void(RTSConnectionMarkerItem*)> sigPositionChanged;
    static const int size = 6;

    RTSConnectionMarkerItem(qreal x, qreal y, markerType type_) :
        QGraphicsRectItem(x-size/2, y-size/2, size, size),
        type(type_){
        setBrush(QBrush(QColor("black")));
        setFlags(QGraphicsItem::ItemIsMovable);
    }

    void setPos(qreal x, qreal y){ setRect( x-size/2, y-size/2, size, size); }
    void move(qreal x, qreal y){
        switch(type){
        case UNMOVABLE:
            return;
        case HORIZONTAL:
            setPos(x, rect().center().y());
            sigPositionChanged(this);
            break;
        case VERTIAL:
            setPos( rect().center().x(), y);
            sigPositionChanged(this);
            break;
        }
    }

};

class RTSConnectionGItem : public QGraphicsItemGroup
{
public :
    enum lineType { THREEPARTS_TYPE, FIVEPARTS_TYPE };
    static const qreal xxoffset = 5;

    RTSConnectionGItem(bool sIsLeft, QPointF s, bool tIsLeft, QPointF t);
    ~RTSConnectionGItem();

    void paint (QPainter *painter, const QStyleOptionGraphicsItem *option, QWidget *widget){
        QStyleOptionGraphicsItem myoption(*option);
        myoption.state &= !QStyle::State_Selected;
        QGraphicsItemGroup::paint(painter, &myoption, widget);
    };
    QPainterPath shape() const {
        QPainterPath ret;
        for(int i=0; i<5; i++){
            if(line[i]){
                QGraphicsLineItem atari(line[i]->line());
                atari.setPen(QPen(QBrush(Qt::black), 10));
                ret.addPath(atari.shape());
            }
        }
        return ret;
    };
    QRectF boundingRect() const {
        QRectF rect;
        for(int i=0; i<5; i++){
            if(line[i]){
                QGraphicsLineItem atari(line[i]->line());
                atari.setPen(QPen(QBrush(Qt::black), 10));
                rect |= atari.boundingRect();
            }
        }
        return rect;
    };

    void showMarker(bool on);
    qreal firstLineX(qreal x);
    qreal endLineX(qreal x);
    int markerIndex(QGraphicsItem* gItem);
    RTSConnectionMarkerItem* findMarker(QGraphicsItem* gItem);
    void lineMove(RTSConnectionMarkerItem* marker_);
    bool changePortPos(bool isSource, QPointF s);
    lineType getType(qreal sx, qreal tx);

private:
    bool _sIsLeft;
    bool _tIsLeft;
    lineType type;
    RTSConnectionLineItem* line[5];
    RTSConnectionMarkerItem* marker[5];
    ConnectionSet signalConnections;

};

class RTSComp;

class RTSCompGItem : public QGraphicsItemGroup
{
public :
    RTSCompGItem(RTSComp* rtsComp) : rtsComp(rtsComp){ };
    QVariant itemChange ( GraphicsItemChange change, const QVariant & value );

    RTSComp* rtsComp;
};

class RTSPort : public Referenced
{
public :
    RTSPort(const string& name, PortService_var port, RTSComp* parent);
    RTSComp* rtsComp;
    string name;
    PortService_var port;
    bool isServicePort;
    bool isInPort;

    QGraphicsItemGroup* gItem;
    QGraphicsPolygonItem* polygon;
    QPointF pos;

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
    ~RTSConnection();
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

    RTSComp* srcRTC;
    RTSPort* sourcePort;
    RTSComp* targetRTC;
    RTSPort* targetPort;


    RTSConnectionGItem* gItem;

    bool connect();
    bool disConnect();
    void createGItem(bool sIsLeft, QPointF s, bool tIsLeft, QPointF t);
};
typedef ref_ptr<RTSConnection> RTSConnectionPtr;


class RTSComp : public Referenced
{
public :
    RTSComp(RTC::RTObject_ptr, RTSDiagramViewImpl* impl);
    ~RTSComp();

    RTSDiagramViewImpl* impl;
    RTObject_ptr rtc_;
    string name;
    ExecutionContextList_var ownedExeContList;
    ExecutionContextList_var participatingExeContList;
    map<string, RTSPortPtr> inPorts;
    map<string, RTSPortPtr> outPorts;

    RTSCompGItem* gItem;
    QGraphicsRectItem* rect;

    void createGItem(const QPointF& pos);
    bool isActive();
    void stateCheck();
    void connectionCheckSub(RTSPort* rtsPort);
    void connectionCheck();

    RTSPort* nameToRTSPort(const string& name);

    Signal<void(const RTSComp*)> sigPositionChanged;
    Connection positionChangeConnection;

private :
    int correctTextY();
};
typedef ref_ptr<RTSComp> RTSCompPtr;

class CreateConnectionDialog : public Dialog
{
public :
    CreateConnectionDialog();
    QLineEdit* nameLineEdit;
    ComboBox* interfaceCombo;
    ComboBox* dataflowCombo;
    ComboBox* subscriptionCombo;
};

#define STATE_CHECK_TIME 500  //msec
class RTSDiagramViewImpl : public QGraphicsView
{

public:
    RTSDiagramViewImpl(RTSDiagramView* self);
    ~RTSDiagramViewImpl();

    RTSDiagramView* self;
    QGraphicsScene  scene;
    //DroppableGraphicsView* view;
    NamingContextHelper ncHelper;
    Connection locationChangedConnection;
    Connection selectionChangedConnection;
    map<string, RTSCompPtr> rtsComps;
    map<string, RTSConnectionPtr> rtsConnections;
    map<string, RTSConnectionPtr> deletedRtsConnections;
    list<NamingContextHelper::ObjectInfo> selectionItems;
    list<RTSComp*> selectionRTCs;
    list<RTSConnection*> selectionRTSConnections;
    MenuManager menuManager;
    ConnectionSet connections;
    Timer timer;

    RTSPort* sourcePort;
    QGraphicsLineItem dragPortLine;
    RTSConnectionMarkerItem* targetMarker;

    void onLocationChanged(std::string host, int port);
    void addRTSComp(string name, const QPointF& pos);
    void deleteRTSComp(RTSComp* rtsComp);
    void deleteRTSConnections(RTSConnection* rtsConnection);
    void deleteSelectedRTSItem();
    void dragEnterEvent   (QDragEnterEvent* event);
    void dragMoveEvent    (QDragMoveEvent*  event);
    void dragLeaveEvent   (QDragLeaveEvent* event);
    void dropEvent        (QDropEvent*      event);
    void mouseMoveEvent   (QMouseEvent*     event);
    void mousePressEvent  (QMouseEvent*     event);
    void mouseReleaseEvent(QMouseEvent*     event);
    void keyPressEvent    (QKeyEvent*       event);
    void onItemSelectionChanged(const list<NamingContextHelper::ObjectInfo>& items);
    RTSPort* findTargetRTSPort(QPointF& pos);
    RTSConnectionMarkerItem* findConnectionMarker(QGraphicsItem* gItem);
    void createConnectionGItem(RTSConnection* rtsConnection);
    RTSComp* nameToRTSComp(const string& name);
    RTSComp* gItemToRTSComp(RTSCompGItem* gitem);
    RTSConnection* gItemToRTSConnection(QGraphicsItem* gItem);
    void RTSCompToConnectionList(const RTSComp* rtsComp, list<RTSConnection*>& rtsConnectionList, int mode=0);
    void onRTSCompSelectionChange();
    void onRTSCompPositionChanged(const RTSComp*);
    void onTime();
    void onActivated(bool on);
};

}


RTSConnectionGItem::RTSConnectionGItem(bool sIsLeft, QPointF s, bool tIsLeft, QPointF t) :
        _sIsLeft(sIsLeft), _tIsLeft(tIsLeft)
{
    qreal sx,tx;
    sx = firstLineX(s.x());
    tx = endLineX(t.x());
    type = getType(sx, tx);

    line[0] = new RTSConnectionLineItem(sx, s.y(), s.x(), s.y());
    addToGroup(line[0]);

    line[1] = new RTSConnectionLineItem(tx, t.y(), t.x(), t.y());
    addToGroup(line[1]);

    qreal centerX = (sx + tx)/2.0;
    qreal centerY = (s.y() + t.y())/2.0;
    if(type==THREEPARTS_TYPE){
        line[2] = new RTSConnectionLineItem(sx, s.y(), centerX, s.y());
        addToGroup(line[2]);
        line[3] = new RTSConnectionLineItem(centerX, s.y(), centerX, t.y());
        addToGroup(line[3]);
        line[4] = new RTSConnectionLineItem(centerX, t.y(), tx, t.y());
        addToGroup(line[4]);
        marker[0] = new RTSConnectionMarkerItem(line[0]->x2 , line[0]->y2, RTSConnectionMarkerItem::UNMOVABLE);
        marker[1] = new RTSConnectionMarkerItem(line[1]->x2 , line[1]->y2, RTSConnectionMarkerItem::UNMOVABLE);
        marker[2] = new RTSConnectionMarkerItem(line[3]->cx , line[3]->cy, RTSConnectionMarkerItem::HORIZONTAL);
        signalConnections.add(marker[2]->sigPositionChanged.connect(
                boost::bind(&RTSConnectionGItem::lineMove, this, _1)));
        marker[3] = marker[4] = 0;
    }else{
        line[2] = new RTSConnectionLineItem(sx, s.y(), sx, centerY);
        addToGroup(line[2]);
        line[3] = new RTSConnectionLineItem(sx, centerY, tx, centerY);
        addToGroup(line[3]);
        line[4] = new RTSConnectionLineItem(tx, centerY, tx, t.y());
        addToGroup(line[4]);
        marker[0] = new RTSConnectionMarkerItem(line[0]->x2, line[0]->y2, RTSConnectionMarkerItem::UNMOVABLE);
        marker[1] = new RTSConnectionMarkerItem(line[1]->x2, line[1]->y2, RTSConnectionMarkerItem::UNMOVABLE);
        marker[2] = new RTSConnectionMarkerItem(line[2]->cx, line[2]->cy, RTSConnectionMarkerItem::HORIZONTAL);
        marker[3] = new RTSConnectionMarkerItem(line[3]->cx, line[3]->cy, RTSConnectionMarkerItem::VERTIAL);
        marker[4] = new RTSConnectionMarkerItem(line[4]->cx, line[4]->cy, RTSConnectionMarkerItem::HORIZONTAL);
        signalConnections.add(marker[2]->sigPositionChanged.connect(
                        boost::bind(&RTSConnectionGItem::lineMove, this, _1)));
        signalConnections.add(marker[3]->sigPositionChanged.connect(
                        boost::bind(&RTSConnectionGItem::lineMove, this, _1)));
        signalConnections.add(marker[4]->sigPositionChanged.connect(
                        boost::bind(&RTSConnectionGItem::lineMove, this, _1)));
    }

    setFlags(QGraphicsItem::ItemIsSelectable );
}


RTSConnectionGItem::~RTSConnectionGItem()
{
    signalConnections.disconnect();

    for(int i=0; i<5; i++){
        if(marker[i]){
            if(marker[i]->scene())
                marker[i]->scene()->removeItem(marker[i]);
            delete marker[i];
        }
    }
}


qreal RTSConnectionGItem::firstLineX(qreal x){
    if(_sIsLeft)
        return x-xxoffset;
    else
        return x+xxoffset;
}


qreal RTSConnectionGItem::endLineX(qreal x){
    if(_tIsLeft)
        return x-xxoffset;
    else
        return x+xxoffset;
}


int RTSConnectionGItem::markerIndex(QGraphicsItem* gItem){
    for(int i=2; i<5; i++)
        if(marker[i] == gItem)
            return i;
    return -1;
}


RTSConnectionMarkerItem* RTSConnectionGItem::findMarker(QGraphicsItem* gItem){
    if(this==gItem || gItem==0)
        return 0;
    int i = markerIndex(gItem);
    if(i<0)
        return 0;
    return marker[i];
}


void RTSConnectionGItem::lineMove(RTSConnectionMarkerItem* marker_){
    int i = markerIndex(marker_);
    if(i<0)
        return;
    QPointF c = marker_->rect().center();

    if(type==RTSConnectionGItem::THREEPARTS_TYPE){
        if( i==2 ){
            line[2]->setEx(c.x());
            line[3]->moveX(c.x());
            line[4]->setSx(c.x());
        }
    }else{
        if( i==2 ){
            line[0]->setSx(c.x());
            line[2]->moveX(c.x());
            line[3]->setSx(c.x());
            marker[3]->setPos(line[3]->cx, line[3]->cy);
        }else if( i==3 ){
            line[2]->setEy(c.y());
            marker[2]->setPos(line[2]->cx, line[2]->cy);
            line[3]->moveY(c.y());
            line[4]->setSy(c.y());
            marker[4]->setPos(line[4]->cx, line[4]->cy);
        }else if( i==4 ){
            line[3]->setEx(c.x());
            marker[3]->setPos(line[3]->cx, line[3]->cy);
            line[4]->moveX(c.x());
            line[1]->setSx(c.x());
        }
    }
}


RTSConnectionGItem::lineType RTSConnectionGItem::getType(qreal sx, qreal tx){
    if((_sIsLeft && tx<sx) || (_tIsLeft &&  sx<tx) ||
            (!_sIsLeft && sx<tx) || (!_tIsLeft && tx<sx))
        return THREEPARTS_TYPE;
    else
        return FIVEPARTS_TYPE;
}


bool RTSConnectionGItem::changePortPos(bool isSource, QPointF s)
{
    if(isSource){
        qreal sx = firstLineX(s.x());
        lineType type_ = getType(sx, line[1]->x1);
        if(type != type_)
            return false;
        if(type_==THREEPARTS_TYPE){
            line[0]->setPos(sx, s.y(), s.x(), s.y());
            line[2]->setPos(sx, s.y(), line[2]->x2, s.y());
            line[3]->setSy(s.y());
            marker[0]->setPos(line[0]->x2 , line[0]->y2);
            marker[2]->setPos(line[3]->cx, line[3]->cy);
        }else{
            line[0]->setPos(line[2]->x1, s.y(), s.x(), s.y());
            line[2]->setSy(s.y());
            marker[0]->setPos(line[0]->x2 , line[0]->y2);
            marker[2]->setPos(line[2]->cx, line[2]->cy);
        }
    }else{
        qreal tx = endLineX(s.x());
        lineType type_ = getType(line[0]->x1, tx);
        if(type != type_)
            return false;
        if(type_==THREEPARTS_TYPE){
            line[1]->setPos(tx, s.y(), s.x(), s.y());
            line[4]->setPos(line[4]->x1, s.y(), tx, s.y());
            line[3]->setEy(s.y());
            marker[1]->setPos(line[1]->x2 , line[1]->y2);
            marker[2]->setPos(line[3]->cx, line[3]->cy);
        }else{
            line[1]->setPos(line[4]->x1, s.y(), s.x(), s.y());
            line[4]->setEy(s.y());
            marker[1]->setPos(line[1]->x2 , line[1]->y2);
            marker[4]->setPos(line[4]->cx, line[4]->cy);
        }
    }
    return true;

}


void RTSConnectionGItem::showMarker(bool on)
{
    if(on){
        for(int i=0; i<5; i++){
            if(marker[i] && scene()){
                scene()->addItem(marker[i]);
            }
        }
    }else{
        for(int i=0; i<5; i++){
            if(marker[i] && scene()){
                scene()->removeItem(marker[i]);
            }
        }
    }
}


RTSPort::RTSPort(const string& name_, PortService_var port_, RTSComp* parent)
    : rtsComp(parent)
{
    isInPort = true;
    name = name_;
    port = port_;
    gItem = new QGraphicsItemGroup;
}


bool RTSPort::connected()
{
    ConnectorProfileList_var connectorProfiles = port->get_connector_profiles();
    return connectorProfiles->length()!=0;
}


bool RTSPort::connectedWith(RTSPort* target)
{
    ConnectorProfileList_var connectorProfiles = port->get_connector_profiles();
    for(CORBA::ULong i=0; i < connectorProfiles->length(); ++i){
        ConnectorProfile& connectorProfile = connectorProfiles[i];
        PortServiceList& connectedPorts = connectorProfile.ports;

        for(CORBA::ULong j=0; j < connectedPorts.length(); ++j){
            PortService_ptr connectedPortRef = connectedPorts[j];
            if(connectedPortRef->_is_equivalent(target->port)){
                return true;
            }
        }
    }
    return false;
}


void RTSPort::stateCheck()
{
    this->polygon->setBrush(QBrush(QColor(connected() ? "lightgreen" : (isServicePort ? "lightblue" : "blue"))));
}


bool RTSPort::checkConnectablePort(RTSPort* target)
{
    if(rtsComp == target->rtsComp)
        return false;
    if((isInPort && target->isInPort) ||
       (isInPort && target->isServicePort) ||
       (!isInPort && !isServicePort && !target->isInPort) ||
       (isServicePort && !target->isServicePort))
        return false;

    return true;
}


RTSConnection::RTSConnection(const string& id, const string& name,
        const string& sourceRtcName,const string& sourcePortName,
        const string& targetRtcName, const string& targetPortName)
    : id(id), name(name), sourceRtcName(sourceRtcName), sourcePortName(sourcePortName),
      targetRtcName(targetRtcName), targetPortName(targetPortName)
{
    pushPolicy = "all";
    pushRate = 1000.0;
    sinterface = "corba_cdr";
    gItem = 0;
}


RTSConnection::~RTSConnection()
{
    if(gItem){
        gItem->scene()->removeItem(gItem);
        delete gItem;
    }
}


void RTSConnection::createGItem(bool sIsLeft, QPointF s, bool tIsLeft, QPointF t)
{
    gItem = new RTSConnectionGItem(sIsLeft, s, tIsLeft, t);
}


bool RTSConnection::connect()
{
    ConnectorProfile cprof;
    cprof.connector_id = CORBA::string_dup(id.c_str());
    cprof.name = CORBA::string_dup(name.c_str());
    cprof.ports.length(2);
    cprof.ports[0] = PortService::_duplicate(sourcePort->port);
    cprof.ports[1] = PortService::_duplicate(targetPort->port);

    CORBA_SeqUtil::push_back(cprof.properties,
            NVUtil::newNV("dataport.dataflow_type",
                    dataflow.c_str()));
    CORBA_SeqUtil::push_back(cprof.properties,
            NVUtil::newNV("dataport.interface_type",
                    sinterface.c_str()));
    CORBA_SeqUtil::push_back(cprof.properties,
            NVUtil::newNV("dataport.subscription_type",
                    subscription.c_str()));

    RTC::ReturnCode_t result = sourcePort->port->connect(cprof);
    if(result == RTC::RTC_OK){
        PortProfile_var portprofile = sourcePort->port->get_port_profile();
        ConnectorProfileList connections = portprofile->connector_profiles;
        for(CORBA::ULong i = 0; i < connections.length(); i++){
            ConnectorProfile& connector = connections[i];
            PortServiceList& connectedPorts = connector.ports;
            for(CORBA::ULong j=0; j < connectedPorts.length(); ++j){
                PortService_ptr connectedPortRef = connectedPorts[j];
                PortProfile_var connectedPortProfile = connectedPortRef->get_port_profile();
                string portName = string(connectedPortProfile->name);
                if(portName == targetPortName){
                    id = string(connector.connector_id);
                    return true;
                }
            }
        }
        return false;
    }else{
        return false;
    }
}


bool RTSConnection::disConnect()
{
    ReturnCode_t result = sourcePort->port->disconnect(id.c_str());
    if(result == RTC_OK)
        return true;
    else
        return false;
}


QVariant RTSCompGItem::itemChange ( GraphicsItemChange change, const QVariant & value )
{
    if (change == ItemPositionChange){
        for(map<string, RTSPortPtr>::iterator it = rtsComp->inPorts.begin();
                it != rtsComp->inPorts.end(); it++){
            it->second->pos += value.value<QPointF>()-pos();
        }
        for(map<string, RTSPortPtr>::iterator it = rtsComp->outPorts.begin();
                it != rtsComp->outPorts.end(); it++){
            it->second->pos += value.value<QPointF>()-pos();
        }
        rtsComp->sigPositionChanged(rtsComp);
    }
    return QGraphicsItem::itemChange(change, value);
}


RTSComp::RTSComp(RTC::RTObject_ptr rtc, RTSDiagramViewImpl* impl) :
        impl(impl)
{
    rtc_ = rtc;
    ComponentProfile_var cprofile = rtc->get_component_profile();
    name = cprofile->instance_name;

    ownedExeContList = rtc_->get_owned_contexts();
    participatingExeContList = rtc_->get_participating_contexts();

    PortServiceList_var portlist = rtc->get_ports();
    for (CORBA::ULong i = 0; i < portlist->length(); i++) {
        PortProfile_var portprofile = portlist[i]->get_port_profile();
        coil::Properties pproperties = NVUtil::toProperties(portprofile->properties);
        string portType = pproperties["port.port_type"];
        RTSPortPtr rtsPort = new RTSPort(string(portprofile->name), portlist[i], this);
        if (boost::iequals(portType, "CorbaPort")){
            rtsPort->isServicePort = true;
            rtsPort->isInPort = false;
            outPorts.insert(pair<string, RTSPortPtr>(rtsPort->name, rtsPort));
        }else{
            rtsPort->isServicePort = false;
            if(boost::iequals(portType, "DataInPort"))
                inPorts.insert(pair<string, RTSPortPtr>(rtsPort->name, rtsPort));
            else{
                rtsPort->isInPort = false;
                outPorts.insert(pair<string, RTSPortPtr>(rtsPort->name, rtsPort));
            }
        }
    }

    connectionCheck();

    gItem = new RTSCompGItem(this);

    positionChangeConnection = sigPositionChanged.connect(
            boost::bind(&RTSDiagramViewImpl::onRTSCompPositionChanged, impl, _1));
}


RTSComp::~RTSComp()
{
    gItem->scene()->removeItem(gItem);
    delete gItem;
    positionChangeConnection.disconnect();
}


bool RTSComp::isActive()
{
    for(CORBA::ULong i=0; i<ownedExeContList->length(); i++){
        if(ownedExeContList[i]->get_component_state(rtc_) == RTC::ACTIVE_STATE)
            return true;
    }
    for(CORBA::ULong i=0; i<participatingExeContList->length(); i++){
        if(participatingExeContList[i]->get_component_state(rtc_) == RTC::ACTIVE_STATE)
            return true;
    }
    return false;
}


int RTSComp::correctTextY()
{
    int numIn = inPorts.size();
    int numOut = outPorts.size();
    int numMax = (numIn < numOut ? numOut : numIn);
    if(!numMax)
        numMax = 1;
    return (25 * numMax) - 7 * (numMax - 1);
}


void RTSComp::createGItem(const QPointF& pos)
{
    QGraphicsTextItem * text = new QGraphicsTextItem(QString(name.c_str()));
    int height = correctTextY();
    text->setPos(0 + pos.x(), height + 5 + pos.y());
    gItem->addToGroup(text);

    rect = new QGraphicsRectItem(7, 7, 50, height);
    int rectX = (text->boundingRect().width() / 2 - 50 / 2) - 5;
    rect->setPos(rectX+pos.x(), pos.y());
    rect->setPen(QPen(QColor("darkgray")));
    gItem->addToGroup(rect);
    gItem->setFlags(QGraphicsItem::ItemIsMovable | QGraphicsItem::ItemIsSelectable | QGraphicsItem::ItemSendsScenePositionChanges);

    int i=0;
    for(map<string, RTSPortPtr>::iterator it = inPorts.begin();
            it != inPorts.end(); it++, i++){
        RTSPort* inPort = it->second.get();
        string portName = string(inPort->name);
        RTCCommonUtil::splitPortName(portName);
        text = new QGraphicsTextItem(QString(portName.c_str()));
        int r = 7*i;
        int x = rectX+pos.x() - text->boundingRect().width();
        int y = 25* i - 7*i - 3 + pos.y();
        text->setPos(x, y);
        gItem->addToGroup(text);
        inPort->pos = QPointF( rectX +  5 + pos.x(),  5 + 7 + 7 + (25 * i) - r + pos.y());
        inPort->polygon = new QGraphicsPolygonItem(QPolygonF(QVector<QPointF>()
                << QPointF( rectX +  0 + pos.x(),  0 + 7 + 7 + (25 * i) - r + pos.y())
                << QPointF( rectX + 10 + pos.x(),  0 + 7 + 7 + (25 * i) - r + pos.y())
                << QPointF( rectX + 10 + pos.x(), 10 + 7 + 7 + (25 * i) - r + pos.y())
                << QPointF( rectX +  0 + pos.x(), 10 + 7 + 7 + (25 * i) - r + pos.y())
                << QPointF( rectX +  5 + pos.x(),  5 + 7 + 7 + (25 * i) - r + pos.y())
                << QPointF( rectX +  0 + pos.x(),  0 + 7 + 7 + (25 * i) - r + pos.y())));
        inPort->polygon->setPen(QPen(QColor("red")));
        inPort->stateCheck();
        inPort->gItem->addToGroup(inPort->polygon);
        gItem->addToGroup(inPort->gItem);
    }

    i = 0;
    for(map<string, RTSPortPtr>::iterator it = outPorts.begin();
                it != outPorts.end(); it++, i++){
        RTSPort* outPort = it->second.get();
        string portName = string(outPort->name);
        RTCCommonUtil::splitPortName(portName);
        text = new QGraphicsTextItem(QString(portName.c_str()));
        int r = 7*i;
        int x = rectX+66+pos.x();
        int y = 25*i-r-3+pos.y();
        text->setPos(x, y);
        gItem->addToGroup(text);

        if(outPort->isServicePort){
            outPort->polygon = new QGraphicsPolygonItem(QPolygonF(QVector<QPointF>()
                << QPointF( rectX + 53 + pos.x(),  0 + 7 + 7 + (25 * i) - r + pos.y())
                << QPointF( rectX + 63 + pos.x(),  0 + 7 + 7 + (25 * i) - r + pos.y())
                << QPointF( rectX + 63 + pos.x(),  5 + 7 + 7 + (25 * i) - r + pos.y())
                << QPointF( rectX + 63 + pos.x(), 10 + 7 + 7 + (25 * i) - r + pos.y())
                << QPointF( rectX + 53 + pos.x(), 10 + 7 + 7 + (25 * i) - r + pos.y())
                << QPointF( rectX + 53 + pos.x(),  0 + 7 + 7 + (25 * i) - r + pos.y())));
            outPort->pos = QPointF( rectX + 63 + pos.x(),  5 + 7 + 7 + (25 * i) - r + pos.y());
        }else{
            outPort->polygon = new QGraphicsPolygonItem(QPolygonF(QVector<QPointF>()
                << QPointF( rectX + 53 + pos.x(),  0 + 7 + 7 + (25 * i) - r + pos.y())
                << QPointF( rectX + 60 + pos.x(),  0 + 7 + 7 + (25 * i) - r + pos.y())
                << QPointF( rectX + 65 + pos.x(),  5 + 7 + 7 + (25 * i) - r + pos.y())
                << QPointF( rectX + 60 + pos.x(), 10 + 7 + 7 + (25 * i) - r + pos.y())
                << QPointF( rectX + 53 + pos.x(), 10 + 7 + 7 + (25 * i) - r + pos.y())
                << QPointF( rectX + 53 + pos.x(),  0 + 7 + 7 + (25 * i) - r + pos.y())));
            outPort->pos = QPointF( rectX + 65 + pos.x(),  5 + 7 + 7 + (25 * i) - r + pos.y());
        }
        outPort->stateCheck();
        outPort->polygon->setPen(QPen(QColor("red")));
        outPort->gItem->addToGroup(outPort->polygon);
        gItem->addToGroup(outPort->gItem);
    }

    stateCheck();
}


void RTSComp::stateCheck()
{
    rect->setBrush(QBrush(QColor(isActive() ? "lightgreen" : "blue")));

    for(map<string, RTSPortPtr>::iterator it = inPorts.begin(); it != inPorts.end(); it++){
        it->second->stateCheck();
    }

    for(map<string, RTSPortPtr>::iterator it = outPorts.begin(); it != outPorts.end(); it++){
            it->second->stateCheck();
    }
}


void RTSComp::connectionCheckSub(RTSPort* rtsPort)
{
    PortProfile_var portprofile = rtsPort->port->get_port_profile();
    ConnectorProfileList connectorProfiles = portprofile->connector_profiles;
    for(CORBA::ULong i = 0; i < connectorProfiles.length(); i++){
        ConnectorProfile& connectorProfile = connectorProfiles[i];
        string id = string(connectorProfile.connector_id);
        map<string, RTSConnectionPtr>::iterator itr = impl->rtsConnections.find(id);
        if(itr!=impl->rtsConnections.end()){
            impl->deletedRtsConnections.erase(id);
            continue;
        }
        PortServiceList& connectedPorts = connectorProfile.ports;
        for(CORBA::ULong j=0; j < connectedPorts.length(); ++j){
            PortService_ptr connectedPortRef = connectedPorts[j];
            PortProfile_var connectedPortProfile = connectedPortRef->get_port_profile();
            string portName = string(connectedPortProfile->name);
            vector<string> target;
            RTCCommonUtil::splitPortName(portName, target);
            if(target[0] != name){
                RTSComp* targetRTC = impl->nameToRTSComp(target[0]);
                if(targetRTC){
                    RTSConnectionPtr rtsConnection = new RTSConnection(
                            id, string(connectorProfile.name),
                            name, string(portprofile->name),
                            target[0], portName);
                    coil::Properties properties = NVUtil::toProperties(connectorProfile.properties);
                    rtsConnection->dataflow = properties["dataport.dataflow_type"];
                    rtsConnection->subscription = properties["dataport.subscription_type"];
                    rtsConnection->srcRTC = this;
                    rtsConnection->sourcePort = nameToRTSPort(rtsConnection->sourcePortName);
                    rtsConnection->targetRTC = targetRTC;
                    rtsConnection->targetPort = targetRTC->nameToRTSPort(rtsConnection->targetPortName);
                    impl->rtsConnections.insert(pair<string, RTSConnectionPtr>(id, rtsConnection));
                }
            }
        }
    }
}


void RTSComp::connectionCheck()
{
    for(map<string, RTSPortPtr>::iterator it = inPorts.begin(); it != inPorts.end(); it++)
        connectionCheckSub(it->second.get());
    for(map<string, RTSPortPtr>::iterator it = outPorts.begin(); it != outPorts.end(); it++)
        connectionCheckSub(it->second.get());
}


RTSPort* RTSComp::nameToRTSPort(const string& name)
{
    map<string, RTSPortPtr>::iterator it = inPorts.find(name);
    if(it!=inPorts.end())
        return it->second.get();
    it = outPorts.find(name);
    if(it!=outPorts.end())
        return it->second.get();
    return 0;
}


void RTSDiagramViewImpl::dragEnterEvent(QDragEnterEvent *event)
{
    if(event->mimeData()->hasFormat("application/RTSNameServerItem")){
        event->acceptProposedAction();
    }
}


void RTSDiagramViewImpl::dragMoveEvent(QDragMoveEvent *event)
{
    if(event->mimeData()->hasFormat("application/RTSNameServerItem")){
        event->acceptProposedAction();
    }
}


void RTSDiagramViewImpl::dragLeaveEvent(QDragLeaveEvent *event)
{
    MessageView::instance()->putln(_("Drag and drop has been canceled. Please be operation again."));
}


void RTSDiagramViewImpl::dropEvent(QDropEvent *event)
{
    for(list<NamingContextHelper::ObjectInfo>::iterator it = selectionItems.begin();
            it != selectionItems.end(); it++){
        NamingContextHelper::ObjectInfo& info = *it;
        if(!info.isAlive)
            MessageView::instance()->putln((boost::format(_("%1% is not alive")) % info.id).str());
        else{
            addRTSComp(info.id, mapToScene(event->pos()));
        }
    }
}


void RTSDiagramViewImpl::onItemSelectionChanged(const list<NamingContextHelper::ObjectInfo>& items)
{
    selectionItems = items;
}


RTSPort* RTSDiagramViewImpl::findTargetRTSPort(QPointF& pos)
{
    for(map<string, RTSCompPtr>::iterator it = rtsComps.begin();
            it != rtsComps.end(); it++){
        map<string, RTSPortPtr>& inPorts = it->second->inPorts;
        for(map<string, RTSPortPtr>::iterator itr = inPorts.begin();
                itr != inPorts.end(); itr++)
            if(itr->second->gItem->sceneBoundingRect().contains(pos))
                return itr->second.get();
        map<string, RTSPortPtr>& outPorts = it->second->outPorts;
        for(map<string, RTSPortPtr>::iterator itr = outPorts.begin();
                itr != outPorts.end(); itr++)
            if(itr->second->gItem->sceneBoundingRect().contains(pos))
                return itr->second;
    }
    return 0;
}


RTSConnectionMarkerItem* RTSDiagramViewImpl::findConnectionMarker(QGraphicsItem* gItem)
{
    for(list<RTSConnection*>::iterator it = selectionRTSConnections.begin();
                it != selectionRTSConnections.end(); it++){
        RTSConnectionMarkerItem* marker = (*it)->gItem->findMarker(gItem);
        if(marker)
            return marker;
    }
    return 0;
}


void RTSDiagramViewImpl::mouseMoveEvent(QMouseEvent* event)
{
    QPointF pos = mapToScene(event->pos());

    if(sourcePort){
        QPointF sp = sourcePort->pos;
        dragPortLine.setLine(sp.x(), sp.y(), pos.x(), pos.y());
        RTSPort* port = findTargetRTSPort(pos);
        if(port && !sourcePort->checkConnectablePort(port))
            setCursor(Qt::ForbiddenCursor);
        else
            setCursor(Qt::PointingHandCursor);
    }else if(targetMarker){
        targetMarker->move(pos.x(), pos.y());
    }else {
        RTSPort* port = findTargetRTSPort(pos);
        if(port){
            setCursor(Qt::PointingHandCursor);
        }else{
            QGraphicsItem* gItem = scene.itemAt(pos.x(), pos.y());
            if(gItem){
                if(findConnectionMarker(gItem))
                    setCursor(Qt::ClosedHandCursor);
                else
                    setCursor(Qt::ArrowCursor);
            }else
                setCursor(Qt::ArrowCursor);
        }
    }
    QGraphicsView::mouseMoveEvent(event);
}


void RTSDiagramViewImpl::keyPressEvent(QKeyEvent *event)
{
   if (event->key() == Qt::Key_Delete) {
       deleteSelectedRTSItem();
    }
}


void RTSDiagramViewImpl::mousePressEvent(QMouseEvent* event)
{
    if(event->button() == Qt::LeftButton){
        QPointF pos = mapToScene(event->pos());
        sourcePort = findTargetRTSPort(pos);
        if(sourcePort){
            dragPortLine.setLine(pos.x(), pos.y(), pos.x(), pos.y());
            dragPortLine.setVisible(true);
            return;
        }

        QGraphicsItem* gItem = scene.itemAt(pos.x(), pos.y());
        if(gItem){
            targetMarker = findConnectionMarker(gItem);
            if(targetMarker){
                return;
            }
        }

        QGraphicsView::mousePressEvent(event);
    }

    if(event->button() == Qt::RightButton){
        if(!selectionRTCs.empty() || !selectionRTSConnections.empty()){
            menuManager.setNewPopupMenu(this);
            menuManager.addItem("Delete")
               ->sigTriggered().connect(boost::bind(&RTSDiagramViewImpl::deleteSelectedRTSItem, this));

            menuManager.popupMenu()->popup(event->globalPos());
        }
    }
}


void RTSDiagramViewImpl::mouseReleaseEvent(QMouseEvent *event)
{
    QPointF pos = mapToScene(event->pos());

    if(sourcePort){
        dragPortLine.setVisible(false);
        RTSPort* targetPort = findTargetRTSPort(pos);
        if(targetPort && sourcePort->checkConnectablePort(targetPort)){
            if(sourcePort->connectedWith(targetPort)){
                MessageView::instance()->putln(_("These are already connected."));
            }else{
                CreateConnectionDialog createDialog;
                createDialog.nameLineEdit->setText(QString((sourcePort->name+"_"+targetPort->name).c_str()));
                bool create = false;
                int result = createDialog.exec();
                if( result == QDialog::Accepted )
                    create = true;
                else if( result == QDialog::Rejected )
                    ;
                if(create){
                    string id = "";
                    string name = createDialog.nameLineEdit->text().toStdString();
                    string dataflow = createDialog.dataflowCombo->currentText().toStdString();
                    string subscription = createDialog.subscriptionCombo->currentText().toStdString();
                    RTSConnectionPtr rtsConnection = new RTSConnection(
                            id, name,
                            sourcePort->rtsComp->name, sourcePort->name,
                            targetPort->rtsComp->name, targetPort->name);
                    rtsConnection->dataflow = dataflow;
                    rtsConnection->subscription = subscription;
                    rtsConnection->srcRTC = sourcePort->rtsComp;
                    rtsConnection->sourcePort = sourcePort;
                    rtsConnection->targetRTC = targetPort->rtsComp;
                    rtsConnection->targetPort = targetPort;
                    if(rtsConnection->connect()){
                        createConnectionGItem(rtsConnection);
                        rtsConnections.insert(pair<string, RTSConnectionPtr>(rtsConnection->id, rtsConnection));
                    }
                }
            }
        }
        sourcePort = 0;
    }
    targetMarker = 0;
    setCursor(Qt::ArrowCursor);
    QGraphicsView::mouseReleaseEvent(event);
}


void RTSDiagramView::initializeClass(ExtensionManager* ext)
{
    ext->viewManager().registerClass<RTSDiagramView>(
        "RTSDiagramView", N_("RTC Diagram"), ViewManager::SINGLE_OPTIONAL);
}


RTSDiagramView* RTSDiagramView::instance()
{
    return ViewManager::findView<RTSDiagramView>();
}


RTSDiagramView::RTSDiagramView()
{
    impl = new RTSDiagramViewImpl(this);
}


void RTSDiagramView::onRTSCompSelectionChange()
{
    impl->onRTSCompSelectionChange();
}


RTSDiagramViewImpl::RTSDiagramViewImpl(RTSDiagramView* self)
    :self(self),
     sourcePort(0)
{
    self->setDefaultLayoutArea(View::CENTER);

    QVBoxLayout* vbox = new QVBoxLayout();
    setScene(&scene);
    vbox->addWidget(this);
    self->setLayout(vbox);
    setAcceptDrops(true);
    connect(&scene, SIGNAL(selectionChanged()), self, SLOT(onRTSCompSelectionChange()));

    RTSNameServerView* nsView = RTSNameServerView::instance();
    if(nsView){
        selectionItems = nsView->getSelection();
        if(!selectionChangedConnection.connected()){
            selectionChangedConnection = nsView->sigSelectionChanged().connect(
                    boost::bind(&RTSDiagramViewImpl::onItemSelectionChanged, this, _1));
        }
        if(!locationChangedConnection.connected()){
            locationChangedConnection = nsView->sigLocationChanged().connect(
                    boost::bind(&RTSDiagramViewImpl::onLocationChanged, this, _1, _2));
            ncHelper.setLocation(nsView->getHost(), nsView->getPort());
        }
    }

    timer.setSingleShot(true);
    connections.add(timer.sigTimeout().connect(
            boost::bind(&RTSDiagramViewImpl::onTime, this)));
    self->sigActivated().connect(boost::bind(&RTSDiagramViewImpl::onActivated, this, true));
    self->sigDeactivated().connect(boost::bind(&RTSDiagramViewImpl::onActivated, this ,false));

    QPen pen(Qt::DashDotLine);
    pen.setWidth(2);
    dragPortLine.setPen(pen);
    dragPortLine.setZValue(100);
    dragPortLine.setVisible(false);
    scene.addItem(&dragPortLine);
}


RTSDiagramView::~RTSDiagramView()
{
    delete impl;
}


RTSDiagramViewImpl::~RTSDiagramViewImpl()
{
    selectionChangedConnection.disconnect();
    locationChangedConnection.disconnect();
    connections.disconnect();
    disconnect(&scene, SIGNAL(selectionChanged()), self, SLOT(onRTSCompSelectionChange()));

}


void RTSDiagramViewImpl::onLocationChanged(string host, int port)
{
    ncHelper.setLocation(host, port);
}


void RTSDiagramViewImpl::addRTSComp(string name, const QPointF& pos)
{
    if(!nameToRTSComp(name)){
        RTC::RTObject_ptr rtc = ncHelper.findObject<RTC::RTObject>(name, "rtc");
        RTSCompPtr rtsComp = new RTSComp(rtc, this);
        rtsComps.insert(pair<string, RTSCompPtr>(name, rtsComp));

        rtsComp->createGItem(pos);
        scene.addItem(rtsComp->gItem);

        for(map<string, RTSConnectionPtr>::iterator itr = rtsConnections.begin();
                itr != rtsConnections.end(); itr++){
            if(!itr->second->gItem)
                createConnectionGItem(itr->second.get());
        }
    }
}


void RTSDiagramViewImpl::deleteRTSComp(RTSComp* rtsComp)
{
    list<RTSConnection*> rtsConnectionList;
    RTSCompToConnectionList(rtsComp, rtsConnectionList);
    for(list<RTSConnection*>::iterator itr = rtsConnectionList.begin();
            itr != rtsConnectionList.end(); itr++){
        rtsConnections.erase((*itr)->id);
    }
    rtsComps.erase(rtsComp->name);
}


void RTSDiagramViewImpl::deleteRTSConnections(RTSConnection* rtsConnection)
{
    rtsConnection->disConnect();
    rtsConnections.erase(rtsConnection->id);
}


void RTSDiagramViewImpl::deleteSelectedRTSItem()
{
    disconnect(&scene, SIGNAL(selectionChanged()), self, SLOT(onRTSCompSelectionChange()));
    for(list<RTSConnection*>::iterator it = selectionRTSConnections.begin();
            it != selectionRTSConnections.end(); it++){
        deleteRTSConnections(*it);
    }
    selectionRTSConnections.clear();
    for(list<RTSComp*>::iterator it= selectionRTCs.begin();
            it != selectionRTCs.end(); it++){
        deleteRTSComp(*it);
    }
    selectionRTCs.clear();
    connect(&scene, SIGNAL(selectionChanged()), self, SLOT(onRTSCompSelectionChange()));
}


void RTSDiagramViewImpl::createConnectionGItem(RTSConnection* rtsConnection)
{
    rtsConnection->createGItem(rtsConnection->sourcePort->isInPort,
            rtsConnection->sourcePort->pos,
            rtsConnection->targetPort->isInPort,
            rtsConnection->targetPort->pos);

    scene.addItem(rtsConnection->gItem);
    //rtsConnection->gItem->addToScene(&scene);
}


RTSComp* RTSDiagramViewImpl::nameToRTSComp(const string& name)
{
    map<string, RTSCompPtr>::iterator it = rtsComps.find(name);
    if(it==rtsComps.end())
        return 0;
    else
        return it->second.get();
}


RTSComp* RTSDiagramViewImpl::gItemToRTSComp(RTSCompGItem* gItem)
{
    for(map<string, RTSCompPtr>::iterator it = rtsComps.begin();
            it != rtsComps.end(); it++){
        if(it->second->gItem == gItem)
            return it->second.get();
    }
    return 0;
}


RTSConnection* RTSDiagramViewImpl::gItemToRTSConnection(QGraphicsItem* gItem)
{
    for(map<string, RTSConnectionPtr>::iterator it = rtsConnections.begin();
            it != rtsConnections.end(); it++){
        if(it->second->gItem == gItem)
        //if(it->second->gItem->contains(gItem))
            return it->second.get();
    }
    return 0;
}


void RTSDiagramViewImpl::onRTSCompSelectionChange()
{
    RTSComp* singleSelectedRTC = 0;
    if(selectionRTCs.size()==1)
        singleSelectedRTC = selectionRTCs.front();

    RTSConnection* singleSelectedConnection = 0;
    if(selectionRTSConnections.size()==1)
        singleSelectedConnection = selectionRTSConnections.front();

    for(list<RTSConnection*>::iterator it = selectionRTSConnections.begin();
            it != selectionRTSConnections.end(); it++){
            (*it)->gItem->showMarker(false);
    }
    selectionRTCs.clear();
    selectionRTSConnections.clear();

    QList<QGraphicsItem *> items = scene.selectedItems();
    for(QList<QGraphicsItem*>::iterator it = items.begin(); it != items.end(); it++){
        RTSComp* rtsComp = gItemToRTSComp(dynamic_cast<RTSCompGItem*>(*it));
        if(rtsComp){
            selectionRTCs.push_back(rtsComp);
        }else{
            RTSConnection* rtsConnection = gItemToRTSConnection(*it);
            if(rtsConnection){
                selectionRTSConnections.push_back(rtsConnection);
                rtsConnection->gItem->showMarker(true);
            }
        }
    }

    if(selectionRTCs.size()==1 && singleSelectedRTC != selectionRTCs.front()){
        RTSNameServerView* nsView = RTSNameServerView::instance();
        if(nsView){
            nsView->setSelection(selectionRTCs.front()->name);
        }
    }

    if(selectionRTSConnections.size()==1 && singleSelectedConnection != selectionRTSConnections.front()){
        RTSNameServerView* nsView = RTSNameServerView::instance();
        if(nsView){
            nsView->setSelection("");
        }
        RTSPropertiesView* propView = RTSPropertiesView::instance();
        if(propView){
            RTSConnection* rtsConnection = selectionRTSConnections.front();
            propView->showConnectionProperties(rtsConnection->sourcePort->port, rtsConnection->id);
        }
    }
}


void RTSDiagramViewImpl::RTSCompToConnectionList(const RTSComp* rtsComp,
        list<RTSConnection*>& rtsConnectionList, int mode)
{
    for(map<string, RTSConnectionPtr>::iterator it = rtsConnections.begin();
            it != rtsConnections.end(); it++){
        switch (mode){
        case 0:
        default :
            if(it->second->sourceRtcName == rtsComp->name || it->second->targetRtcName == rtsComp->name)
                rtsConnectionList.push_back(it->second);
            break;
        case 1:
            if(it->second->sourceRtcName == rtsComp->name)
                rtsConnectionList.push_back(it->second);
            break;
        case 2:
            if(it->second->targetRtcName == rtsComp->name)
                rtsConnectionList.push_back(it->second);
            break;
        }
    }
}


void RTSDiagramViewImpl::onRTSCompPositionChanged(const RTSComp* rtsComp)
{
    list<RTSConnection*> rtsConnectionList;

    RTSCompToConnectionList(rtsComp, rtsConnectionList, 1);
    for(list<RTSConnection*>::iterator it = rtsConnectionList.begin();
                it != rtsConnectionList.end(); it++){
        RTSConnectionGItem* gItem = (*it)->gItem;
        if(!gItem->changePortPos( true, (*it)->sourcePort->pos)){
            scene.removeItem(gItem);
            delete gItem;
            createConnectionGItem((*it));
        }
    }

    rtsConnectionList.clear();
    RTSCompToConnectionList(rtsComp, rtsConnectionList, 2);
    for(list<RTSConnection*>::iterator it = rtsConnectionList.begin();
            it != rtsConnectionList.end(); it++){
        RTSConnectionGItem* gItem = (*it)->gItem;
        if(!gItem->changePortPos( false, (*it)->targetPort->pos)){
            scene.removeItem(gItem);
            delete gItem;
            createConnectionGItem((*it));
        }
    }
}


void RTSDiagramViewImpl::onTime()
{
    for(map<string, RTSCompPtr>::iterator it = rtsComps.begin();
            it != rtsComps.end(); it++){
        if(!ncHelper.isObjectAlive(it->second->rtc_)){
            deleteRTSComp(it->second);
        }
    }

    deletedRtsConnections = rtsConnections;
    for(map<string, RTSCompPtr>::iterator it = rtsComps.begin();
                it != rtsComps.end(); it++){
        it->second->stateCheck();
        it->second->connectionCheck();
    }

    for(map<string, RTSConnectionPtr>::iterator itr = deletedRtsConnections.begin();
                itr != deletedRtsConnections.end(); itr++){
        rtsConnections.erase(itr->first);
    }
    deletedRtsConnections.clear();

    for(map<string, RTSConnectionPtr>::iterator itr = rtsConnections.begin();
            itr != rtsConnections.end(); itr++){
        if(!itr->second->gItem)
            createConnectionGItem(itr->second.get());
    }

    timer.start(STATE_CHECK_TIME);
}


void RTSDiagramViewImpl::onActivated(bool on)
{
    if(on)
        timer.start(STATE_CHECK_TIME);
    else
        timer.stop();
}


CreateConnectionDialog::CreateConnectionDialog()
{
    setWindowTitle("Connector Profile");

    QLabel* label1 = new QLabel(_("name : "));
    nameLineEdit = new QLineEdit();
    QHBoxLayout* hlayout1 = new QHBoxLayout();
    hlayout1->addWidget(label1);
    hlayout1->addWidget(nameLineEdit);

    QLabel* label2 = new QLabel(_("Interface Type : "));
    interfaceCombo = new ComboBox;
    interfaceCombo->addItem("corba_cdr");
    interfaceCombo->setCurrentIndex(0);
    QHBoxLayout* hlayout2 = new QHBoxLayout();
    hlayout2->addWidget(label2);
    hlayout2->addWidget(interfaceCombo);

    QLabel* label3 = new QLabel(_("Dataflow Type : "));
    dataflowCombo = new ComboBox;
    dataflowCombo->addItem("push");
    dataflowCombo->addItem("pull");
    dataflowCombo->setCurrentIndex(0);
    QHBoxLayout* hlayout3 = new QHBoxLayout();
    hlayout3->addWidget(label3);
    hlayout3->addWidget(dataflowCombo);

    QLabel* label4 = new QLabel(_("Subscription Type : "));
    subscriptionCombo = new ComboBox;
    subscriptionCombo->addItem("flush");
    subscriptionCombo->addItem("new");
    subscriptionCombo->addItem("periodic");
    subscriptionCombo->setCurrentIndex(0);
    QHBoxLayout* hlayout4 = new QHBoxLayout();
    hlayout4->addWidget(label4);
    hlayout4->addWidget(subscriptionCombo);

    QPushButton* okButton = new QPushButton(_("&OK"));
    okButton->setDefault(true);
    QPushButton* cancelButton = new QPushButton(_("&Cancel"));
    QHBoxLayout* hlayoute = new QHBoxLayout();
    hlayoute->addWidget(cancelButton);
    hlayoute->addWidget(okButton);

    QVBoxLayout* vlayout = new QVBoxLayout();
    vlayout->addLayout(hlayout1);
    vlayout->addLayout(hlayout2);
    vlayout->addLayout(hlayout3);
    vlayout->addLayout(hlayout4);
    vlayout->addLayout(hlayoute);
    setLayout(vlayout);

    connect(okButton,SIGNAL(clicked()), this, SLOT(accept()) );
    connect(cancelButton,SIGNAL(clicked()), this, SLOT(reject()) );
}

