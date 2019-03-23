/*!
 * @author Shizuko Hattori
 * @file
 */
#include "RTSDiagramExt2View.h"
#include "RTSNameServerView.h"
#include "RTSPropertiesView.h"
#include "RTSCommonUtil.h"
#include "OpenRTMUtil.h"
#include "PortConnectionDialogExt2.h"
#include "LoggerUtil.h"
#include <cnoid/ViewManager>
#include <cnoid/MessageView>
#include <cnoid/MenuManager>
#include <cnoid/ConnectionSet>
#include <cnoid/ItemList>
#include <cnoid/ItemTreeView>
#include <cnoid/RootItem>
#include <cnoid/SimulatorItem>
#include <cnoid/AppConfig>
#include <QGraphicsLineItem>
#include <QGraphicsOpacityEffect>
#include <QGraphicsScene>
#include <QGraphicsView>
#include <QBoxLayout>
#include <QDragEnterEvent>
#include <QMessageBox>
#include "gettext.h"

using namespace cnoid;
using namespace std;
using namespace std::placeholders;
using namespace RTC;

#define STATE_CHECK_TIME 1000  //msec
#define OPACITY 0.3

namespace {

class RTSConnectionLineExt2Item : public QGraphicsLineItem
{
public:
    RTSConnectionLineExt2Item(qreal x1, qreal y1, qreal x2, qreal y2)
        : QGraphicsLineItem(x1, y1, x2, y2), x1(x1), x2(x2), y1(y1), y2(y2)
    {
        QPen pen;
        pen.setColor(QColor("black"));
        pen.setWidth(1);
        setPen(pen);
        cx = (x1 + x2) / 2.0;
        cy = (y1 + y2) / 2.0;
    }

    ~RTSConnectionLineExt2Item()
    {
    }

    void update()
    {
        setLine(x1, y1, x2, y2);
        cx = (x1 + x2) / 2.0;
        cy = (y1 + y2) / 2.0;
    }

    void setPos(qreal x1_, qreal y1_, qreal x2_, qreal y2_)
    {
        x1 = x1_; y1 = y1_; x2 = x2_; y2 = y2_; update();
    }

    void setSx(qreal x_) { x1 = x_; update(); }
    void setEx(qreal x_) { x2 = x_; update(); }
    void setSy(qreal y_) { y1 = y_; update(); }
    void setEy(qreal y_) { y2 = y_; update(); }
    void moveX(qreal x_) { x1 = x2 = x_; update(); }
    void moveY(qreal y_) { y1 = y2 = y_; update(); }

    qreal x1, x2, y1, y2;
    qreal cx, cy;
};
//////////
class RTSConnectionMarkerExt2Item : public QGraphicsRectItem
{
public:
    enum markerType { UNMOVABLE, HORIZONTAL, VERTIAL };
    markerType type;
    Signal<void(RTSConnectionMarkerExt2Item*)> sigPositionChanged;
    static const int size = 6;

    RTSConnectionMarkerExt2Item(qreal x, qreal y, markerType type_)
        : QGraphicsRectItem(x - size / 2, y - size / 2, size, size),
        type(type_)
    {
        setBrush(QBrush(QColor("black")));
        if (type != UNMOVABLE) {
            setFlags(QGraphicsItem::ItemIsMovable);
        }
    }

    ~RTSConnectionMarkerExt2Item()
    {
    }

    void setPos(qreal x, qreal y) { setRect(x - size / 2, y - size / 2, size, size); }

    void move(qreal x, qreal y)
    {
        switch (type) {
            case UNMOVABLE:
                return;
            case HORIZONTAL:
                setPos(x, rect().center().y());
                sigPositionChanged(this);
                break;
            case VERTIAL:
                setPos(rect().center().x(), y);
                sigPositionChanged(this);
                break;
        }
    }
};
//////////
struct PortInfo {
    bool isLeft;
    QPointF portPos;
    QPointF compPos;
    double height;
    double width;
};

class RTSConnectionExt2GItem : public QGraphicsItemGroup, public Referenced
{
public:
    enum lineType { THREEPARTS_TYPE, FIVEPARTS_TYPE };
    const qreal xxoffset = 5;

    RTSConnectionExt2GItem(RTSConnectionExt2* rtsConnection, PortInfo source, PortInfo target);
    ~RTSConnectionExt2GItem();

    void paint(QPainter *painter, const QStyleOptionGraphicsItem *option, QWidget *widget)
    {
        QStyleOptionGraphicsItem myoption(*option);
        myoption.state &= !QStyle::State_Selected;
        QGraphicsItemGroup::paint(painter, &myoption, widget);
    };

    QPainterPath shape() const
    {
        QPainterPath ret;
        for (int i = 0; i < 5; ++i) {
            if (line[i]) {
                QGraphicsLineItem atari(line[i]->line());
                atari.setPen(QPen(QBrush(Qt::black), 10));
                ret.addPath(atari.shape());
            }
        }
        return ret;
    };

    QRectF boundingRect() const
    {
        QRectF rect;
        for (int i = 0; i < 5; ++i) {
            if (line[i]) {
                QGraphicsLineItem atari(line[i]->line());
                atari.setPen(QPen(QBrush(Qt::black), 10));
                rect |= atari.boundingRect();
            }
        }
        return rect;
    };

    void getLinePosition(Vector2 pos[])
    {
        pos[0](0) = line[0]->x2;
        pos[0](1) = line[0]->y2;
        for (int i = 2; i < 5; ++i) {
            pos[i - 1](0) = line[i]->x1;
            pos[i - 1](1) = line[i]->y1;
        }
        pos[4](0) = line[1]->x1;
        pos[4](1) = line[1]->y1;
        pos[5](0) = line[1]->x2;
        pos[5](1) = line[1]->y2;
    }

    void showMarker(bool on);
    qreal firstLineX(qreal x);
    qreal endLineX(qreal x);
    int markerIndex(QGraphicsItem* gItem);
    RTSConnectionMarkerExt2Item* findMarker(QGraphicsItem* gItem);
    void lineMove(RTSConnectionMarkerExt2Item* marker_);
    bool changePortPos(bool isSource, QPointF s);
    lineType getType(qreal sx, qreal tx);

    RTSConnectionExt2Ptr rtsConnection;

private:
    bool _sIsLeft;
    bool _tIsLeft;
    lineType type;
    RTSConnectionLineExt2Item* line[5];
    RTSConnectionMarkerExt2Item* marker[5];
    ConnectionSet signalConnections;
    QGraphicsOpacityEffect* effect;

    double calcLinePos(PortInfo source, PortInfo target);
};

typedef ref_ptr<RTSConnectionExt2GItem> RTSConnectionExt2GItemPtr;

RTSConnectionExt2GItem::RTSConnectionExt2GItem
(RTSConnectionExt2* rtsConnection, PortInfo source, PortInfo target)
    : rtsConnection(rtsConnection), _sIsLeft(source.isLeft), _tIsLeft(target.isLeft)
{
    effect = new QGraphicsOpacityEffect;
    effect->setOpacity(OPACITY);
    setGraphicsEffect(effect);

    if (rtsConnection->isAlive()) {
        effect->setEnabled(false);
    } else {
        effect->setEnabled(true);
    }

    if (rtsConnection->setPos) {
        Vector2& p0s = rtsConnection->position[0];
        Vector2& p0e = rtsConnection->position[1];
        line[0] = new RTSConnectionLineExt2Item(p0e(0), p0e(1), p0s(0), p0s(1));
        addToGroup(line[0]);
        Vector2& p1s = rtsConnection->position[4];
        Vector2& p1e = rtsConnection->position[5];
        line[1] = new RTSConnectionLineExt2Item(p1s(0), p1s(1), p1e(0), p1e(1));
        addToGroup(line[1]);
        for (int i = 2; i < 5; ++i) {
            Vector2& p0 = rtsConnection->position[i - 1];
            Vector2& p1 = rtsConnection->position[i];
            line[i] = new RTSConnectionLineExt2Item(p0(0), p0(1), p1(0), p1(1));
            addToGroup(line[i]);
        }
        if (line[3]->x1 == line[3]->x2) {
            type = THREEPARTS_TYPE;
            marker[0] = new RTSConnectionMarkerExt2Item(line[0]->x2, line[0]->y2, RTSConnectionMarkerExt2Item::UNMOVABLE);
            marker[1] = new RTSConnectionMarkerExt2Item(line[1]->x2, line[1]->y2, RTSConnectionMarkerExt2Item::UNMOVABLE);
            marker[2] = new RTSConnectionMarkerExt2Item(line[3]->cx, line[3]->cy, RTSConnectionMarkerExt2Item::HORIZONTAL);
            signalConnections.add(
                marker[2]->sigPositionChanged.connect(
                    std::bind(&RTSConnectionExt2GItem::lineMove, this, _1)));
            marker[3] = marker[4] = 0;
        } else {
            type = FIVEPARTS_TYPE;
            marker[0] = new RTSConnectionMarkerExt2Item(line[0]->x2, line[0]->y2, RTSConnectionMarkerExt2Item::UNMOVABLE);
            marker[1] = new RTSConnectionMarkerExt2Item(line[1]->x2, line[1]->y2, RTSConnectionMarkerExt2Item::UNMOVABLE);
            marker[2] = new RTSConnectionMarkerExt2Item(line[2]->cx, line[2]->cy, RTSConnectionMarkerExt2Item::HORIZONTAL);
            marker[3] = new RTSConnectionMarkerExt2Item(line[3]->cx, line[3]->cy, RTSConnectionMarkerExt2Item::VERTIAL);
            marker[4] = new RTSConnectionMarkerExt2Item(line[4]->cx, line[4]->cy, RTSConnectionMarkerExt2Item::HORIZONTAL);
            signalConnections.add(
                marker[2]->sigPositionChanged.connect(
                    std::bind(&RTSConnectionExt2GItem::lineMove, this, _1)));
            signalConnections.add(
                marker[3]->sigPositionChanged.connect(
                    std::bind(&RTSConnectionExt2GItem::lineMove, this, _1)));
            signalConnections.add(
                marker[4]->sigPositionChanged.connect(
                    std::bind(&RTSConnectionExt2GItem::lineMove, this, _1)));
        }
    } else {
        qreal sx, tx;
        sx = firstLineX(source.portPos.x());
        tx = endLineX(target.portPos.x());
        type = getType(sx, tx);

        line[0] = new RTSConnectionLineExt2Item(sx, source.portPos.y(), source.portPos.x(), source.portPos.y());
        addToGroup(line[0]);

        line[1] = new RTSConnectionLineExt2Item(tx, target.portPos.y(), target.portPos.x(), target.portPos.y());
        addToGroup(line[1]);

        qreal centerX = (sx + tx) / 2.0;
        qreal centerY = (source.portPos.y() + target.portPos.y()) / 2.0;
        if (type == THREEPARTS_TYPE) {
            line[2] = new RTSConnectionLineExt2Item(sx, source.portPos.y(), centerX, source.portPos.y());
            addToGroup(line[2]);
            line[3] = new RTSConnectionLineExt2Item(centerX, source.portPos.y(), centerX, target.portPos.y());
            addToGroup(line[3]);
            line[4] = new RTSConnectionLineExt2Item(centerX, target.portPos.y(), tx, target.portPos.y());
            addToGroup(line[4]);
            marker[0] = new RTSConnectionMarkerExt2Item(line[0]->x2, line[0]->y2, RTSConnectionMarkerExt2Item::UNMOVABLE);
            marker[1] = new RTSConnectionMarkerExt2Item(line[1]->x2, line[1]->y2, RTSConnectionMarkerExt2Item::UNMOVABLE);
            marker[2] = new RTSConnectionMarkerExt2Item(line[3]->cx, line[3]->cy, RTSConnectionMarkerExt2Item::HORIZONTAL);
            signalConnections.add(
                marker[2]->sigPositionChanged.connect(
                    std::bind(&RTSConnectionExt2GItem::lineMove, this, _1)));
            marker[3] = marker[4] = 0;

        } else {
            centerY = calcLinePos(source, target);

            line[2] = new RTSConnectionLineExt2Item(sx, source.portPos.y(), sx, centerY);
            addToGroup(line[2]);
            line[3] = new RTSConnectionLineExt2Item(sx, centerY, tx, centerY);
            addToGroup(line[3]);
            line[4] = new RTSConnectionLineExt2Item(tx, centerY, tx, target.portPos.y());
            addToGroup(line[4]);
            marker[0] = new RTSConnectionMarkerExt2Item(line[0]->x2, line[0]->y2, RTSConnectionMarkerExt2Item::UNMOVABLE);
            marker[1] = new RTSConnectionMarkerExt2Item(line[1]->x2, line[1]->y2, RTSConnectionMarkerExt2Item::UNMOVABLE);
            marker[2] = new RTSConnectionMarkerExt2Item(line[2]->cx, line[2]->cy, RTSConnectionMarkerExt2Item::HORIZONTAL);
            marker[3] = new RTSConnectionMarkerExt2Item(line[3]->cx, line[3]->cy, RTSConnectionMarkerExt2Item::VERTIAL);
            marker[4] = new RTSConnectionMarkerExt2Item(line[4]->cx, line[4]->cy, RTSConnectionMarkerExt2Item::HORIZONTAL);
            signalConnections.add(
                marker[2]->sigPositionChanged.connect(
                    std::bind(&RTSConnectionExt2GItem::lineMove, this, _1)));
            signalConnections.add(
                marker[3]->sigPositionChanged.connect(
                    std::bind(&RTSConnectionExt2GItem::lineMove, this, _1)));
            signalConnections.add(
                marker[4]->sigPositionChanged.connect(
                    std::bind(&RTSConnectionExt2GItem::lineMove, this, _1)));
        }

        Vector2 pos[6];
        getLinePosition(pos);
        rtsConnection->setPosition(pos);
    }

    setFlags(QGraphicsItem::ItemIsSelectable);
}

RTSConnectionExt2GItem::~RTSConnectionExt2GItem()
{
    signalConnections.disconnect();

    for (int i = 0; i < 5; ++i) {
        if (marker[i]) {
            delete marker[i];
        }
    }
}

qreal RTSConnectionExt2GItem::firstLineX(qreal x)
{
    if (_sIsLeft) {
        return x - xxoffset;
    } else {
        return x + xxoffset;
    }
}

qreal RTSConnectionExt2GItem::endLineX(qreal x)
{
    if (_tIsLeft) {
        return x - xxoffset;
    } else {
        return x + xxoffset;
    }
}

int RTSConnectionExt2GItem::markerIndex(QGraphicsItem* gItem)
{
    for (int i = 2; i < 5; ++i) {
        if (marker[i] == gItem) {
            return i;
        }
    }
    return -1;
}

RTSConnectionMarkerExt2Item* RTSConnectionExt2GItem::findMarker(QGraphicsItem* gItem)
{
    if (this == gItem || !gItem) {
        return nullptr;
    }
    int i = markerIndex(gItem);
    if (i < 0) {
        return nullptr;
    }
    return marker[i];
}

void RTSConnectionExt2GItem::lineMove(RTSConnectionMarkerExt2Item* marker_)
{
    int i = markerIndex(marker_);
    if (i < 0) {
        return;
    }
    QPointF c = marker_->rect().center();

    if (type == RTSConnectionExt2GItem::THREEPARTS_TYPE) {
        if (i == 2) {
            line[2]->setEx(c.x());
            line[3]->moveX(c.x());
            line[4]->setSx(c.x());
        }
    } else {
        if (i == 2) {
            line[0]->setSx(c.x());
            line[2]->moveX(c.x());
            line[3]->setSx(c.x());
            marker[3]->setPos(line[3]->cx, line[3]->cy);
        } else if (i == 3) {
            line[2]->setEy(c.y());
            marker[2]->setPos(line[2]->cx, line[2]->cy);
            line[3]->moveY(c.y());
            line[4]->setSy(c.y());
            marker[4]->setPos(line[4]->cx, line[4]->cy);
        } else if (i == 4) {
            line[3]->setEx(c.x());
            marker[3]->setPos(line[3]->cx, line[3]->cy);
            line[4]->moveX(c.x());
            line[1]->setSx(c.x());
        }
    }

    Vector2 pos[6];
    getLinePosition(pos);
    rtsConnection->setPosition(pos);
}

RTSConnectionExt2GItem::lineType RTSConnectionExt2GItem::getType(qreal sx, qreal tx)
{
    if ((_sIsLeft && tx < sx) || (_tIsLeft &&  sx < tx) || (!_sIsLeft && sx < tx) || (!_tIsLeft && tx < sx)) {
        return THREEPARTS_TYPE;
    } else {
        return FIVEPARTS_TYPE;
    }
}

bool RTSConnectionExt2GItem::changePortPos(bool isSource, QPointF s)
{
    if (isSource) {
        qreal sx = firstLineX(s.x());
        lineType type_ = getType(sx, line[1]->x1);
        if (type != type_) {
            return false;
        }
        if (type_ == THREEPARTS_TYPE) {
            line[0]->setPos(sx, s.y(), s.x(), s.y());
            line[2]->setPos(sx, s.y(), line[2]->x2, s.y());
            line[3]->setSy(s.y());
            marker[0]->setPos(line[0]->x2, line[0]->y2);
            marker[2]->setPos(line[3]->cx, line[3]->cy);
        } else {
            line[0]->setPos(line[2]->x1, s.y(), s.x(), s.y());
            line[2]->setSy(s.y());
            marker[0]->setPos(line[0]->x2, line[0]->y2);
            marker[2]->setPos(line[2]->cx, line[2]->cy);
        }
    } else {
        qreal tx = endLineX(s.x());
        lineType type_ = getType(line[0]->x1, tx);
        if (type != type_) {
            return false;
        }
        if (type_ == THREEPARTS_TYPE) {
            line[1]->setPos(tx, s.y(), s.x(), s.y());
            line[4]->setPos(line[4]->x1, s.y(), tx, s.y());
            line[3]->setEy(s.y());
            marker[1]->setPos(line[1]->x2, line[1]->y2);
            marker[2]->setPos(line[3]->cx, line[3]->cy);
        } else {
            line[1]->setPos(line[4]->x1, s.y(), s.x(), s.y());
            line[4]->setEy(s.y());
            marker[1]->setPos(line[1]->x2, line[1]->y2);
            marker[4]->setPos(line[4]->cx, line[4]->cy);
        }
    }

    Vector2 pos[6];
    this->getLinePosition(pos);
    rtsConnection->setPosition(pos);
    return true;
}

void RTSConnectionExt2GItem::showMarker(bool on)
{
    if (on) {
        for (int i = 0; i < 5; ++i) {
            if (marker[i] && scene()) {
                scene()->addItem(marker[i]);
            }
        }
    } else {
        for (int i = 0; i < 5; ++i) {
            if (marker[i] && scene()) {
                scene()->removeItem(marker[i]);
            }
        }
    }
}

double RTSConnectionExt2GItem::calcLinePos(PortInfo source, PortInfo target) {
    double result = 0.0;

    double srcTop = source.compPos.y();
    double srcBtm = source.compPos.y() + source.height;
    double trgTop = target.compPos.y();
    double trgBtm = target.compPos.y() + target.height;

    if (srcTop <= trgTop && trgTop <= srcBtm) {
        double diffBtm = trgBtm - source.portPos.y();
        double diffTop = srcTop - source.portPos.y();
        if (fabs(diffBtm) < fabs(diffTop)) {
            result = trgBtm + 30;
        } else {
            result = srcTop - 10;
        }

    } else if (trgTop < srcTop && srcTop < trgBtm) {
        double diffBtm = srcBtm - source.portPos.y();
        double diffTop = trgTop - source.portPos.y();
        if (fabs(diffBtm) < fabs(diffTop)) {
            result = srcBtm + 30;
        } else {
            result = trgTop - 10;
        }
        
    } else {
        result = (source.portPos.y() + target.portPos.y()) / 2.0;
    }

    return result;
}

//////////
class RTSCompExt2GItem;

class RTSPortExt2GItem : public QGraphicsItemGroup, public Referenced
{
public:
    enum portType { INPORT, OUTPORT, SERVICEPORT };
    RTSPortExt2* rtsPort;
    QGraphicsPolygonItem* polygon;
    QPointF pos;
    RTSCompExt2GItem* parent;

    RTSPortExt2GItem(RTSPortExt2* rtsPort, RTSCompExt2GItem* rtc);
    void create(const int posX, const int posY, portType type);
    void checkPortState();
    void setCandidate(bool isCand);

};

typedef ref_ptr<RTSPortExt2GItem> RTSPortExt2GItemPtr;

RTSPortExt2GItem::RTSPortExt2GItem(RTSPortExt2* rtsPort, RTSCompExt2GItem* rtc)
    : rtsPort(rtsPort), parent(rtc)
{

}

void RTSPortExt2GItem::create(const int posX, const int posY, portType type)
{
    switch (type) {

        case INPORT:
            this->pos = QPointF(posX + 5, 5 + 7 + 7 + posY);
            polygon = new QGraphicsPolygonItem(
                QPolygonF(QVector<QPointF>()
                    << QPointF(posX +  0, 0 + 7 + 7 + posY)
                    << QPointF(posX + 10, 0 + 7 + 7 + posY)
                    << QPointF(posX + 10, 10 + 7 + 7 + posY)
                    << QPointF(posX +  0, 10 + 7 + 7 + posY)
                    << QPointF(posX +  5, 5 + 7 + 7 + posY)
                    << QPointF(posX +  0, 0 + 7 + 7 + posY)));
            polygon->setPen(QPen(QColor("red")));
            checkPortState();
            addToGroup(polygon);
            break;

        case OUTPORT:
            polygon = new QGraphicsPolygonItem(
                QPolygonF(QVector<QPointF>()
                    << QPointF(posX + 53, 0 + 7 + 7 + posY)
                    << QPointF(posX + 60, 0 + 7 + 7 + posY)
                    << QPointF(posX + 65, 5 + 7 + 7 + posY)
                    << QPointF(posX + 60, 10 + 7 + 7 + posY)
                    << QPointF(posX + 53, 10 + 7 + 7 + posY)
                    << QPointF(posX + 53, 0 + 7 + 7 + posY)));
            this->pos = QPointF(posX + 65, 5 + 7 + 7 + posY);
            polygon->setPen(QPen(QColor("red")));
            checkPortState();
            addToGroup(polygon);
            break;

        case SERVICEPORT:
            polygon = new QGraphicsPolygonItem(
                QPolygonF(QVector<QPointF>()
                    << QPointF(posX + 53, 0 + 7 + 7 + posY)
                    << QPointF(posX + 63, 0 + 7 + 7 + posY)
                    << QPointF(posX + 63, 5 + 7 + 7 + posY)
                    << QPointF(posX + 63, 10 + 7 + 7 + posY)
                    << QPointF(posX + 53, 10 + 7 + 7 + posY)
                    << QPointF(posX + 53, 0 + 7 + 7 + posY)));
            this->pos = QPointF(posX + 63, 5 + 7 + 7 + posY);
            polygon->setPen(QPen(QColor("red")));
            checkPortState();
            addToGroup(polygon);
            break;
    }
}

void RTSPortExt2GItem::checkPortState()
{
    polygon->setBrush(QBrush(QColor(rtsPort->isConnected_ ? "lightgreen" : (rtsPort->isServicePort ? "lightblue" : "blue"))));
}

void RTSPortExt2GItem::setCandidate(bool isCand)
{
    if (isCand) {
        polygon->setPen(QPen(QColor("magenta"), 3));
    } else {
        polygon->setPen(QPen(QColor("red"), 1));
    }
}
//////////
class RTSCompExt2GItem : public QGraphicsItemGroup, public Referenced
{
public:
    RTSCompExt2GItem(RTSCompExt2* rtsComp, RTSDiagramExt2ViewImpl* impl, const QPointF& pos, const int interval);
    ~RTSCompExt2GItem();
    QVariant itemChange(GraphicsItemChange change, const QVariant & value);
    void create(const QPointF& pos, const int interval);
    void checkRTCState();
    void checkCandidate(RTSPortExt2GItem* sourcePort);
    void clearCandidate();
    int correctTextY(int interval);

    RTSDiagramExt2ViewImpl* impl;
    RTSCompExt2* rtsComp;
    map<string, RTSPortExt2GItemPtr> inPorts;
    map<string, RTSPortExt2GItemPtr> outPorts;
    QGraphicsRectItem* rect;
    QGraphicsOpacityEffect* effect;
    Signal<void(const RTSCompExt2GItem*)> sigPositionChanged;
    Connection positionChangeConnection;
};

typedef ref_ptr<RTSCompExt2GItem> RTSCompExt2GItemPtr;

QVariant RTSCompExt2GItem::itemChange(GraphicsItemChange change, const QVariant & value)
{
    if (change == ItemPositionChange) {
        rtsComp->moveToRelative(value.value<QPointF>() - pos());
        for (auto it = inPorts.begin(); it != inPorts.end(); ++it) {
            it->second->pos += value.value<QPointF>() - pos();
        }
        for (auto it = outPorts.begin(); it != outPorts.end(); ++it) {
            it->second->pos += value.value<QPointF>() - pos();
        }
        sigPositionChanged(this);
    }
    return QGraphicsItem::itemChange(change, value);
}

int RTSCompExt2GItem::correctTextY(int interval)
{
    int numIn = rtsComp->inPorts.size();
    int numOut = rtsComp->outPorts.size();
    int numMax = (numIn < numOut ? numOut : numIn);
    if (!numMax) {
        numMax = 1;
    }
    return (interval * numMax) - 7 * (numMax - 1);
}

void RTSCompExt2GItem::checkCandidate(RTSPortExt2GItem* sourcePort)
{
    if (!sourcePort->rtsPort) {
        return;
    }
    if (sourcePort->rtsPort->isInPort) {
        for (map<string, RTSPortExt2GItemPtr>::iterator it = outPorts.begin(); it != outPorts.end(); it++) {
            bool isCand = sourcePort->rtsPort->checkConnectablePort(it->second->rtsPort);
            it->second->setCandidate(isCand);
        }
    } else {
        for (map<string, RTSPortExt2GItemPtr>::iterator it = inPorts.begin(); it != inPorts.end(); it++) {
            bool isCand = sourcePort->rtsPort->checkConnectablePort(it->second->rtsPort);
            it->second->setCandidate(isCand);
        }
    }
}

void RTSCompExt2GItem::clearCandidate()
{
    for (map<string, RTSPortExt2GItemPtr>::iterator it = inPorts.begin(); it != inPorts.end(); it++) {
        it->second->setCandidate(false);
    }

    for (map<string, RTSPortExt2GItemPtr>::iterator it = outPorts.begin(); it != outPorts.end(); it++) {
        it->second->setCandidate(false);
    }
}

void RTSCompExt2GItem::checkRTCState()
{
    RTC_STATUS status = rtsComp->rtc_status_;
    if (status == RTC_STATUS::RTC_INACTIVE) {
        rect->setBrush(QBrush(QColor("blue")));
    } else if (status == RTC_STATUS::RTC_ACTIVE) {
        rect->setBrush(QBrush(QColor("lightgreen")));
    } else if (status == RTC_STATUS::RTC_ERROR) {
        rect->setBrush(QBrush(QColor("red")));
    }

    for (map<string, RTSPortExt2GItemPtr>::iterator it = inPorts.begin(); it != inPorts.end(); it++) {
        it->second->checkPortState();
    }

    for (map<string, RTSPortExt2GItemPtr>::iterator it = outPorts.begin(); it != outPorts.end(); it++) {
        it->second->checkPortState();
    }
}

}
//////////
namespace cnoid {

class RTSDiagramExt2ViewImpl : public QGraphicsView
{
public:
    RTSystemItemExt2Ptr currentRTSItem;
    ScopedConnection itemAddedConnection;
    ScopedConnection itemTreeViewSelectionChangedConnection;
    ScopedConnection connectionOfRTSystemItemDetachedFromRoot;

    ScopedConnection rtsLoadedConnection;
    ScopedConnection updateStatusConnection;

    map<string, RTSCompExt2GItemPtr> rtsComps;
    map<string, RTSConnectionExt2GItemPtr> rtsConnections;
    map<RTSPortExt2*, RTSPortExt2GItem*> rtsPortMap;
    list<NamingContextHelper::ObjectInfo> nsViewSelections;
    list<RTSCompExt2GItem*> selectionRTCs;
    list<RTSConnectionExt2GItem*> selectionRTSConnections;
    MenuManager menuManager;
    RTSPortExt2GItem* sourcePort;
    QGraphicsLineItem* dragPortLine;
    RTSConnectionMarkerExt2Item* targetMarker;
    int pollingPeriod;

    RTSDiagramExt2ViewImpl(RTSDiagramExt2View* self);
    ~RTSDiagramExt2ViewImpl();
    void setNewRTSItemDetector();
    void addRTSComp(NamingContextHelper::ObjectInfo& info, const QPointF& pos);
    void addRTSComp(RTSCompExt2* rtsComp);
    void deleteRTSComp(RTSCompExt2GItem* rtsComp);
    void deleteRTSConnection(RTSConnectionExt2GItem* rtsConnection);
    void deleteSelectedRTSItem();
    void dragEnterEvent(QDragEnterEvent* event);
    void dragMoveEvent(QDragMoveEvent* event);
    void dragLeaveEvent(QDragLeaveEvent* event);
    void dropEvent(QDropEvent* event);
    void mouseMoveEvent(QMouseEvent* event);
    void mousePressEvent(QMouseEvent* event);
    void mouseReleaseEvent(QMouseEvent* event);
    void wheelEvent(QWheelEvent* event);
    void keyPressEvent(QKeyEvent* event);
    void onnsViewItemSelectionChanged(const list<NamingContextHelper::ObjectInfo>& items);
    RTSPortExt2GItem* findTargetRTSPort(QPointF& pos);
    RTSCompExt2GItem* findTargetRTC(RTSPortExt2GItem* port);
    RTSConnectionMarkerExt2Item* findConnectionMarker(QGraphicsItem* gItem);
    void createConnectionGItem(RTSConnectionExt2* rtsConnection, RTSPortExt2GItem* sourcePort, RTSPortExt2GItem* targetPort);
    void onRTSCompSelectionChange();
    void onRTSCompPositionChanged(const RTSCompExt2GItem*);
    void onActivated(bool on);
    void onItemTreeViewSelectionChanged(const ItemList<RTSystemExt2Item>& items);
    void onRTSystemItemDetachedFromRoot();
    void setCurrentRTSItem(RTSystemExt2Item* item);
    void updateView();
    void updateRestoredView();

    void activateComponent();
    void deactivateComponent();
    void resetComponent();
    void finalizeComponent();
    void startExecutionContext();
    void stopExecutionContext();

    void onRTSystemLoaded();
    void onUpdateStatus(bool modified);

private:
    RTSDiagramExt2View* self;
    QGraphicsScene  scene;

    ScopedConnection nsViewSelectionChangedConnection;

    bool activated;
    int interval_;

    void updateStatus();
};

}
//////////
RTSCompExt2GItem::RTSCompExt2GItem(RTSCompExt2* rtsComp, RTSDiagramExt2ViewImpl* impl, const QPointF& pos, const int interval)
    : impl(impl), rtsComp(rtsComp)
{
    DDEBUG("RTSCompGItem::RTSCompGItem");
    effect = new QGraphicsOpacityEffect;
    effect->setOpacity(OPACITY);
    setGraphicsEffect(effect);
    if (!CORBA::is_nil(rtsComp->rtc_)) {
        DDEBUG("RTSCompGItem::RTSCompGItem rtc NOT NULL");
        rtsComp->isAlive_ = true;
        effect->setEnabled(false);
    } else {
        DDEBUG("RTSCompGItem::RTSCompGItem rtc NULL");
        rtsComp->isAlive_ = false;
        effect->setEnabled(true);
    }

    create(pos, interval);
    positionChangeConnection = sigPositionChanged.connect(
        std::bind(&RTSDiagramExt2ViewImpl::onRTSCompPositionChanged, impl, _1));
}

RTSCompExt2GItem::~RTSCompExt2GItem()
{
    positionChangeConnection.disconnect();
    for (auto it = inPorts.begin(); it != inPorts.end(); it++) {
        impl->rtsPortMap.erase(it->second->rtsPort);
    }
    for (auto it = outPorts.begin(); it != outPorts.end(); it++) {
        impl->rtsPortMap.erase(it->second->rtsPort);
    }
}

void RTSCompExt2GItem::create(const QPointF& pos, const int interval)
{
    QGraphicsTextItem * text = new QGraphicsTextItem(QString(rtsComp->name.c_str()));
    int height = correctTextY(interval);
    text->setPos(0 + pos.x(), height + 5 + pos.y());
    addToGroup(text);

    rect = new QGraphicsRectItem(7, 7, 50, height);
    int rectX = (text->boundingRect().width() / 2 - 50 / 2) - 5;
    rect->setPos(rectX + pos.x(), pos.y());
    rect->setPen(QPen(QColor("darkgray")));
    rect->setBrush(QBrush(QColor("lightgray")));
    addToGroup(rect);
    setFlags(QGraphicsItem::ItemIsMovable | QGraphicsItem::ItemIsSelectable | QGraphicsItem::ItemSendsScenePositionChanges);

    int i = 0;
    for (vector<RTSPortExt2Ptr>::iterator it = rtsComp->inPorts.begin(); it != rtsComp->inPorts.end(); it++, i++) {
        RTSPortExt2* inPort = *it;
        string portName = string(inPort->name);
        RTCCommonUtil::splitPortName(portName);
        text = new QGraphicsTextItem(QString(portName.c_str()));
        int x = rectX + pos.x() - text->boundingRect().width();
        int y = interval * i - 7 * i - 3 + pos.y();

        text->setPos(x, y);
        addToGroup(text);

        RTSPortExt2GItemPtr inPortGItem = new RTSPortExt2GItem(inPort, this);
        inPortGItem->create(rectX + pos.x(), y + 3, RTSPortExt2GItem::INPORT);
        addToGroup(inPortGItem);
        inPorts[inPort->name] = inPortGItem;
        impl->rtsPortMap[inPort] = inPortGItem.get();
    }

    i = 0;
    for (vector<RTSPortExt2Ptr>::iterator it = rtsComp->outPorts.begin(); it != rtsComp->outPorts.end(); it++, i++) {
        RTSPortExt2* outPort = *it;
        string portName = string(outPort->name);
        RTCCommonUtil::splitPortName(portName);
        text = new QGraphicsTextItem(QString(portName.c_str()));
        int r = 7 * i;
        int x = rectX + 66 + pos.x();
        int y = interval * i - r - 3 + pos.y();
        text->setPos(x, y);
        addToGroup(text);

        RTSPortExt2GItemPtr outPortGItem = new RTSPortExt2GItem(outPort, this);
        if (outPort->isServicePort) {
            outPortGItem->create(rectX + pos.x(), y + 3, RTSPortExt2GItem::SERVICEPORT);
        } else {
            outPortGItem->create(rectX + pos.x(), y + 3, RTSPortExt2GItem::OUTPORT);
        }
        addToGroup(outPortGItem);
        outPorts[outPort->name] = outPortGItem;
        impl->rtsPortMap[outPort] = outPortGItem.get();
    }

    checkRTCState();
}
//////////
RTSDiagramExt2ViewImpl::RTSDiagramExt2ViewImpl(RTSDiagramExt2View* self)
    :self(self), sourcePort(0), pollingPeriod(1000), activated(false)
{
    self->setDefaultLayoutArea(View::CENTER);

    QVBoxLayout* vbox = new QVBoxLayout();
    setScene(&scene);
    vbox->addWidget(this);
    self->setLayout(vbox);
    setAcceptDrops(false);
    setBackgroundBrush(QBrush(Qt::gray));
    connect(&scene, SIGNAL(selectionChanged()), self, SLOT(onRTSCompSelectionChange()));

    RTSNameServerView* nsView = RTSNameServerView::instance();
    if (nsView) {
        nsViewSelections = nsView->getSelection();
        nsViewSelectionChangedConnection.reset(
            nsView->sigSelectionChanged().connect(
                std::bind(&RTSDiagramExt2ViewImpl::onnsViewItemSelectionChanged, this, _1)));
    }

    self->sigActivated().connect(std::bind(&RTSDiagramExt2ViewImpl::onActivated, this, true));
    self->sigDeactivated().connect(std::bind(&RTSDiagramExt2ViewImpl::onActivated, this, false));

    itemTreeViewSelectionChangedConnection.reset(
        ItemTreeView::mainInstance()->sigSelectionChanged().connect(
            std::bind(&RTSDiagramExt2ViewImpl::onItemTreeViewSelectionChanged, this, _1)));

    QPen pen(Qt::DashDotLine);
    pen.setWidth(2);
    dragPortLine = new QGraphicsLineItem();
    dragPortLine->setPen(pen);
    dragPortLine->setZValue(100);
    dragPortLine->setVisible(false);
    scene.addItem(dragPortLine);

    targetMarker = 0;

    interval_ = 25;
    QFont f = font();
    QFontMetrics fm(f);
    int pixelsHigh = fm.height();
    interval_ = pixelsHigh * 2.0;
}


RTSDiagramExt2ViewImpl::~RTSDiagramExt2ViewImpl()
{
    rtsComps.clear();
    rtsConnections.clear();
    disconnect(&scene, SIGNAL(selectionChanged()), self, SLOT(onRTSCompSelectionChange()));
}


void RTSDiagramExt2ViewImpl::dragEnterEvent(QDragEnterEvent *event)
{
#if QT_VERSION >= QT_VERSION_CHECK(5, 0, 0)
    const RTSNameTreeWidget* nameServerItem =
        qobject_cast<const RTSNameTreeWidget*>(event->source()); //event->mimeData());
    if (nameServerItem) {
#else
    if (event->mimeData()->hasFormat("application/RTSNameServerItem")) {
#endif
        event->acceptProposedAction();
    }
    }


void RTSDiagramExt2ViewImpl::dragMoveEvent(QDragMoveEvent *event)
{
#if QT_VERSION >= QT_VERSION_CHECK(5, 0, 0)
    const RTSNameTreeWidget* nameServerItem =
        qobject_cast<const RTSNameTreeWidget*>(event->source());//event->mimeData());
    if (nameServerItem) {
#else
    if (event->mimeData()->hasFormat("application/RTSNameServerItem")) {
#endif
        event->acceptProposedAction();
    }
    }


void RTSDiagramExt2ViewImpl::dragLeaveEvent(QDragLeaveEvent *event)
{
    MessageView::instance()->putln(_("Drag and drop has been canceled. Please be operation again."));
}


void RTSDiagramExt2ViewImpl::dropEvent(QDropEvent *event)
{
    DDEBUG("RTSystemItem::dropEvent");

    for (list<NamingContextHelper::ObjectInfo>::iterator it = nsViewSelections.begin(); it != nsViewSelections.end(); it++) {
        NamingContextHelper::ObjectInfo& info = *it;
        if (!info.isAlive_) {
            MessageView::instance()->putln((boost::format(_("%1% is not alive")) % info.id_).str());
        } else {
            addRTSComp(info, mapToScene(event->pos()));
            DDEBUG_V("%s", info.getFullPath().c_str());
        }
    }
}


void RTSDiagramExt2ViewImpl::keyPressEvent(QKeyEvent *event)
{
    if (event->key() == Qt::Key_Delete) {
        deleteSelectedRTSItem();
    }
}


void RTSDiagramExt2ViewImpl::mousePressEvent(QMouseEvent* event)
{

    DDEBUG("RTSDiagramViewImpl::mousePressEvent");

    QPointF pos = mapToScene(event->pos());

    if (event->button() == Qt::LeftButton) {
        sourcePort = findTargetRTSPort(pos);
        if (sourcePort) {
            dragPortLine->setLine(pos.x(), pos.y(), pos.x(), pos.y());
            dragPortLine->setVisible(true);
            return;
        }

        QGraphicsItem* gItem = scene.itemAt(pos.x(), pos.y(), transform());
        if (gItem) {
            targetMarker = findConnectionMarker(gItem);
            if (targetMarker) {
                return;
            }
        } else {
            for (QGraphicsItem* item : scene.items()) {
                item->setSelected(false);
            }
            setDragMode(DragMode::ScrollHandDrag);
        }
        QGraphicsView::mousePressEvent(event);
    }

    if (event->button() == Qt::RightButton) {
        if (!selectionRTCs.empty()) {
            if (selectionRTCs.size() == 1) {
                if (!selectionRTCs.front()->rtsComp->rtc_) return;
                menuManager.setNewPopupMenu(this);
                menuManager.addItem("Activate")
                    ->sigTriggered().connect(std::bind(&RTSDiagramExt2ViewImpl::activateComponent, this));
                menuManager.addItem("Deactivate")
                    ->sigTriggered().connect(std::bind(&RTSDiagramExt2ViewImpl::deactivateComponent, this));
                menuManager.addItem("Reset")
                    ->sigTriggered().connect(std::bind(&RTSDiagramExt2ViewImpl::resetComponent, this));
                if (isManagedRTC(selectionRTCs.front()->rtsComp->rtc_) == false) {
                    menuManager.addItem("Exit")
                        ->sigTriggered().connect(std::bind(&RTSDiagramExt2ViewImpl::finalizeComponent, this));
                }
                menuManager.addSeparator();
                menuManager.addItem("Start")
                    ->sigTriggered().connect(std::bind(&RTSDiagramExt2ViewImpl::startExecutionContext, this));
                menuManager.addItem("Stop")
                    ->sigTriggered().connect(std::bind(&RTSDiagramExt2ViewImpl::stopExecutionContext, this));
                menuManager.addSeparator();
            }
            menuManager.addItem("Remove")
                ->sigTriggered().connect(std::bind(&RTSDiagramExt2ViewImpl::deleteSelectedRTSItem, this));

            menuManager.popupMenu()->popup(event->globalPos());

        } else if (!selectionRTSConnections.empty()) {
            menuManager.setNewPopupMenu(this);
            menuManager.addItem("Delete")
                ->sigTriggered().connect(std::bind(&RTSDiagramExt2ViewImpl::deleteSelectedRTSItem, this));

            menuManager.popupMenu()->popup(event->globalPos());

        } else {
            if (currentRTSItem->stateCheck() == 1) {
                menuManager.setNewPopupMenu(this);
                menuManager.addItem(_("Update"))
                    ->sigTriggered().connect(std::bind(&RTSDiagramExt2ViewImpl::updateStatus, this));

                menuManager.popupMenu()->popup(event->globalPos());
            }
        }
    }
}


void RTSDiagramExt2ViewImpl::mouseMoveEvent(QMouseEvent* event)
{
    QPointF pos = mapToScene(event->pos());

    if (sourcePort) {
        QPointF sp = sourcePort->pos;
        dragPortLine->setLine(sp.x(), sp.y(), pos.x(), pos.y());
        RTSPortExt2GItem* port = findTargetRTSPort(pos);
        if (port && !sourcePort->rtsPort->checkConnectablePort(port->rtsPort)) {
            setCursor(Qt::ForbiddenCursor);
        } else {
            setCursor(Qt::PointingHandCursor);
        }

        for (map<string, RTSCompExt2GItemPtr>::iterator it = rtsComps.begin(); it != rtsComps.end(); it++) {
            it->second->checkCandidate(sourcePort);
        }

    } else if (targetMarker) {
        targetMarker->move(pos.x(), pos.y());

    } else {
        RTSPortExt2GItem* port = findTargetRTSPort(pos);
        if (port) {
            setCursor(Qt::PointingHandCursor);
        } else {
            QGraphicsItem* gItem = scene.itemAt(pos.x(), pos.y(), transform());
            if (gItem) {
                if (findConnectionMarker(gItem)) {
                    setCursor(Qt::ClosedHandCursor);
                } else {
                    setCursor(Qt::ArrowCursor);
                }
            } else {
                setCursor(Qt::ArrowCursor);
            }
        }
    }
    QGraphicsView::mouseMoveEvent(event);
}


void RTSDiagramExt2ViewImpl::mouseReleaseEvent(QMouseEvent *event)
{
    QPointF pos = mapToScene(event->pos());

    if (sourcePort) {
        for (map<string, RTSCompExt2GItemPtr>::iterator it = rtsComps.begin(); it != rtsComps.end(); it++) {
            it->second->clearCandidate();
        }

        dragPortLine->setVisible(false);
        RTSPortExt2GItem* targetPort = findTargetRTSPort(pos);

        if (targetPort && sourcePort->rtsPort->checkConnectablePort(targetPort->rtsPort)) {
            if (sourcePort->rtsPort->isConnectedWith(targetPort->rtsPort)) {
                MessageView::instance()->putln(_("These are already connected."));

            } else {
                bool isAccepted = false;
                string name;
                vector<NamedValueExt2Ptr> propList;
                if (sourcePort->rtsPort->isServicePort && targetPort->rtsPort->isServicePort) {
                    DDEBUG("ServicePort Connect");
                    ServicePortConnectionDialogExt2 dialog;
                    dialog.setDisp(sourcePort->rtsPort, targetPort->rtsPort);
                    dialog.exec();
                    isAccepted = dialog.isAccepted;
                    name = dialog.nameLineEdit->text().toStdString();
                    propList = dialog.propList;

                } else {
                    DDEBUG("DataPort Connect");
                    DataPortConnectionDialogExt2 dialog;
                    dialog.setDisp(sourcePort->rtsPort, targetPort->rtsPort);
                    dialog.exec();
                    isAccepted = dialog.isAccepted;
                    name = dialog.nameLineEdit->text().toStdString();
                    propList = dialog.propList;
                }

                if (isAccepted) {
                    DDEBUG("RTSDiagramViewImpl::mouseReleaseEvent Accepted");
                    string id = "";
                    RTSConnectionExt2* rtsConnection =
                        currentRTSItem->addRTSConnection(
                            id, name, sourcePort->rtsPort, targetPort->rtsPort, propList, 0);
                    if (rtsConnection) {
                        createConnectionGItem(rtsConnection, sourcePort, targetPort);
                    }
                }
            }
        }
        sourcePort = 0;
    }
    targetMarker = 0;
    setDragMode(DragMode::NoDrag);
    setCursor(Qt::ArrowCursor);

    QGraphicsView::mouseReleaseEvent(event);
}


void RTSDiagramExt2ViewImpl::wheelEvent(QWheelEvent* event)
{
    double dSteps = (double)event->delta() / 120.0;
    double scaleVal = 1.0;
    scaleVal -= (dSteps / 20.0);
    scale(scaleVal, scaleVal);
    QGraphicsView::wheelEvent(event);
}


void RTSDiagramExt2ViewImpl::onnsViewItemSelectionChanged(const list<NamingContextHelper::ObjectInfo>& items)
{
    nsViewSelections = items;
}

void RTSDiagramExt2ViewImpl::onRTSCompSelectionChange()
{
    RTSCompExt2GItem* singleSelectedRTC = 0;
    if (selectionRTCs.size() == 1) {
        singleSelectedRTC = selectionRTCs.front();
    }

    RTSConnectionExt2GItem* singleSelectedConnection = 0;
    if (selectionRTSConnections.size() == 1) {
        singleSelectedConnection = selectionRTSConnections.front();
    }

    for (auto it = selectionRTSConnections.begin(); it != selectionRTSConnections.end(); it++) {
        (*it)->showMarker(false);
    }
    selectionRTCs.clear();
    selectionRTSConnections.clear();

    QList<QGraphicsItem *> items = scene.selectedItems();
    for (QList<QGraphicsItem*>::iterator it = items.begin(); it != items.end(); it++) {
        RTSCompExt2GItem* rtsComp = dynamic_cast<RTSCompExt2GItem*>(*it);
        if (rtsComp) {
            selectionRTCs.push_back(rtsComp);
        } else {
            RTSConnectionExt2GItem* rtsConnection = dynamic_cast<RTSConnectionExt2GItem*>(*it);
            if (rtsConnection) {
                selectionRTSConnections.push_back(rtsConnection);
                rtsConnection->showMarker(true);
            }
        }
    }

    if (selectionRTCs.size() == 1 && singleSelectedRTC != selectionRTCs.front()) {
        RTSNameServerView* nsView = RTSNameServerView::instance();
        if (nsView) {
            RTSCompExt2* selected = selectionRTCs.front()->rtsComp;
            NamingContextHelper::ObjectInfo nsInfo;
            nsInfo.hostAddress_ = selected->hostAddress;
            nsInfo.portNo_ = selected->portNo;
            nsInfo.isRegisteredInRtmDefaultNameServer_ = selected->isDefaultNS;
            nsView->setSelection(selected->name, selected->fullPath, nsInfo);
        }
    }

    if (selectionRTSConnections.size() == 1 && singleSelectedConnection != selectionRTSConnections.front()) {
        RTSNameServerView* nsView = RTSNameServerView::instance();
        if (nsView) {
            NamingContextHelper::ObjectInfo nsInfo;
            nsView->setSelection("", "", nsInfo);
        }
        RTSPropertiesView* propView = RTSPropertiesView::instance();
        if (propView) {
            RTSConnectionExt2* rtsConnection = selectionRTSConnections.front()->rtsConnection;
            propView->showConnectionProperties(rtsConnection->sourcePort->port, rtsConnection->id);
        }
    }
}


void RTSDiagramExt2ViewImpl::onRTSCompPositionChanged(const RTSCompExt2GItem* rtsCompGItem)
{
    list<RTSConnectionExt2*> rtsConnectionList;
    currentRTSItem->RTSCompToConnectionList(rtsCompGItem->rtsComp, rtsConnectionList, 1);

    for (auto it = rtsConnectionList.begin(); it != rtsConnectionList.end(); it++) {
        RTSConnectionExt2* rtsConnection = *it;
        RTSConnectionExt2GItem* gItem = rtsConnections.find(rtsConnection->id)->second.get();
        RTSPortExt2GItem* sourcePort = rtsPortMap.find(rtsConnection->sourcePort)->second;
        if (!gItem->changePortPos(true, sourcePort->pos)) {
            rtsConnections.erase(rtsConnection->id);
            RTSPortExt2GItem* targetPort = rtsPortMap.find(rtsConnection->targetPort)->second;
            rtsConnection->setPos = false;
            createConnectionGItem(rtsConnection, sourcePort, targetPort);
        }
    }

    rtsConnectionList.clear();
    currentRTSItem->RTSCompToConnectionList(rtsCompGItem->rtsComp, rtsConnectionList, 2);
    for (auto it = rtsConnectionList.begin(); it != rtsConnectionList.end(); it++) {
        RTSConnectionExt2* rtsConnection = *it;
        RTSConnectionExt2GItem* gItem = rtsConnections.find(rtsConnection->id)->second.get();
        RTSPortExt2GItem* targetPort = rtsPortMap.find(rtsConnection->targetPort)->second;
        if (!gItem->changePortPos(false, targetPort->pos)) {
            rtsConnections.erase(rtsConnection->id);
            RTSPortExt2GItem* sourcePort = rtsPortMap.find(rtsConnection->sourcePort)->second;
            rtsConnection->setPos = false;
            createConnectionGItem(rtsConnection, sourcePort, targetPort);
        }
    }
}


void RTSDiagramExt2ViewImpl::onActivated(bool on)
{
    DDEBUG_V("RTSDiagramExt2ViewImpl::onActivated : %d", on);
    activated = on;
    if( currentRTSItem ) {
        if(on) {
            updateStatusConnection.reset(
                    currentRTSItem->sigStatusUpdate().connect(
                        std::bind(&RTSDiagramExt2ViewImpl::onUpdateStatus, this, _1)));
        } else {
            updateStatusConnection.disconnect();
        }
        currentRTSItem->onActivated();
    }
}

void RTSDiagramExt2ViewImpl::onItemTreeViewSelectionChanged(const ItemList<RTSystemExt2Item>& items)
{
    RTSystemExt2Item* firstItem = items.toSingle();

    if (firstItem && firstItem != currentRTSItem) {
        setCurrentRTSItem(firstItem);
    }
}


void RTSDiagramExt2ViewImpl::onRTSystemItemDetachedFromRoot()
{
    currentRTSItem = nullptr;
    connectionOfRTSystemItemDetachedFromRoot.disconnect();
    updateView();
    setNewRTSItemDetector();
}


RTSPortExt2GItem* RTSDiagramExt2ViewImpl::findTargetRTSPort(QPointF& pos)
{
    DDEBUG("RTSDiagramExt2ViewImpl::findTargetRTSPort");
    for (map<string, RTSCompExt2GItemPtr>::iterator it = rtsComps.begin(); it != rtsComps.end(); it++) {
        map<string, RTSPortExt2GItemPtr>& inPorts = it->second->inPorts;
        for (map<string, RTSPortExt2GItemPtr>::iterator itr = inPorts.begin(); itr != inPorts.end(); itr++) {
            if (itr->second->sceneBoundingRect().contains(pos)) {
                return itr->second.get();
            }
        }
        map<string, RTSPortExt2GItemPtr>& outPorts = it->second->outPorts;
        for (map<string, RTSPortExt2GItemPtr>::iterator itr = outPorts.begin(); itr != outPorts.end(); itr++) {
            if (itr->second->sceneBoundingRect().contains(pos)) {
                return itr->second.get();
            }
        }
    }
    DDEBUG("RTSDiagramExt2ViewImpl::findTargetRTSPort NULL");
    return nullptr;
}

RTSCompExt2GItem* RTSDiagramExt2ViewImpl::findTargetRTC(RTSPortExt2GItem* port) 
{
    for (map<string, RTSCompExt2GItemPtr>::iterator it = rtsComps.begin(); it != rtsComps.end(); it++) {
        map<string, RTSPortExt2GItemPtr>& inPorts = it->second->inPorts;
        for (map<string, RTSPortExt2GItemPtr>::iterator itr = inPorts.begin(); itr != inPorts.end(); itr++) {
            if (itr->second == port) {
                return it->second.get();
            }
        }
        map<string, RTSPortExt2GItemPtr>& outPorts = it->second->outPorts;
        for (map<string, RTSPortExt2GItemPtr>::iterator itr = outPorts.begin(); itr != outPorts.end(); itr++) {
            if (itr->second == port) {
                return it->second.get();
            }
        }
    }
    return nullptr;
}


RTSConnectionMarkerExt2Item* RTSDiagramExt2ViewImpl::findConnectionMarker(QGraphicsItem* gItem)
{
    for (auto it = selectionRTSConnections.begin(); it != selectionRTSConnections.end(); it++) {
        RTSConnectionMarkerExt2Item* marker = (*it)->findMarker(gItem);
        if (marker) {
            return marker;
        }
    }
    return nullptr;
}


void RTSDiagramExt2ViewImpl::setNewRTSItemDetector()
{
    itemAddedConnection.disconnect();

    if (!currentRTSItem) {
        ItemList<RTSystemExt2Item> rtsItems;
        if (rtsItems.extractChildItems(RootItem::instance())) {
            setCurrentRTSItem(rtsItems[0]);
        } else {
            itemAddedConnection.reset(
                RootItem::instance()->sigItemAdded().connect(
                    [&](Item* item) {
                if (!currentRTSItem) {
                    auto rtsItem = dynamic_cast<RTSystemExt2Item*>(item);
                    if (rtsItem) {
                        setCurrentRTSItem(rtsItem);
                        itemAddedConnection.disconnect();
                    }
                }
            }));
        }
    }
}


void RTSDiagramExt2ViewImpl::addRTSComp(NamingContextHelper::ObjectInfo& info, const QPointF& pos)
{
    RTSCompExt2* rtsComp = currentRTSItem->addRTSComp(info, pos);
    if (rtsComp) {
        RTSCompExt2GItemPtr rtsCompGItem = new RTSCompExt2GItem(rtsComp, this, pos, interval_);
        rtsComps[info.getFullPath()] = rtsCompGItem;
        scene.addItem(rtsCompGItem);

        RTSystemExt2Item::RTSConnectionMap& connections = currentRTSItem->rtsConnections();
        for (RTSystemExt2Item::RTSConnectionMap::iterator itr = connections.begin(); itr != connections.end(); itr++) {
            if (rtsConnections.find(itr->second->id) == rtsConnections.end()) {
                RTSConnectionExt2* rtsConnection = itr->second.get();
                RTSPortExt2GItem* source = rtsPortMap.find(rtsConnection->sourcePort)->second;
                RTSPortExt2GItem* target = rtsPortMap.find(rtsConnection->targetPort)->second;
                createConnectionGItem(rtsConnection, source, target);
            }
        }
    }
    updateView();
}


void RTSDiagramExt2ViewImpl::addRTSComp(RTSCompExt2* rtsComp)
{
    RTSCompExt2GItemPtr rtsCompGItem = new RTSCompExt2GItem(rtsComp, this, rtsComp->pos(), interval_);
    rtsComps[rtsComp->fullPath] = rtsCompGItem;
    scene.addItem(rtsCompGItem);
}


void RTSDiagramExt2ViewImpl::deleteRTSComp(RTSCompExt2GItem* rtsCompGItem)
{
    list<RTSConnectionExt2*> rtsConnectionList;
    currentRTSItem->RTSCompToConnectionList(rtsCompGItem->rtsComp, rtsConnectionList);

    for (auto itr = rtsConnectionList.begin(); itr != rtsConnectionList.end(); itr++) {
        map<string, RTSConnectionExt2GItemPtr>::iterator it = rtsConnections.find((*itr)->id);
        if (it != rtsConnections.end()) {
            deleteRTSConnection(it->second);
        }
    }

    const string name = rtsCompGItem->rtsComp->fullPath;
    currentRTSItem->deleteRTSComp(name);
    rtsComps.erase(name);
}


void RTSDiagramExt2ViewImpl::deleteRTSConnection(RTSConnectionExt2GItem* rtsConnectionGItem)
{
    currentRTSItem->disconnectAndRemoveConnection(rtsConnectionGItem->rtsConnection);

    rtsConnections.erase(rtsConnectionGItem->rtsConnection->id);
}


void RTSDiagramExt2ViewImpl::deleteSelectedRTSItem()
{
    disconnect(&scene, SIGNAL(selectionChanged()), self, SLOT(onRTSCompSelectionChange()));
    for (auto it = selectionRTSConnections.begin(); it != selectionRTSConnections.end(); it++) {
        deleteRTSConnection(*it);
    }
    selectionRTSConnections.clear();
    for (auto it = selectionRTCs.begin(); it != selectionRTCs.end(); it++) {
        deleteRTSComp(*it);
    }
    selectionRTCs.clear();
    updateView();
    connect(&scene, SIGNAL(selectionChanged()), self, SLOT(onRTSCompSelectionChange()));
}


void RTSDiagramExt2ViewImpl::createConnectionGItem
(RTSConnectionExt2* rtsConnection, RTSPortExt2GItem* sourcePort, RTSPortExt2GItem* targetPort)
{
    DDEBUG("RTSDiagramViewImpl::createConnectionGItem");

    PortInfo sourceInfo;
    sourceInfo.isLeft = rtsConnection->sourcePort->isInPort;
    sourceInfo.portPos = sourcePort->pos;
    RTSCompExt2GItem* srcComp = findTargetRTC(sourcePort);
    sourceInfo.compPos = srcComp->rect->scenePos();
    sourceInfo.height = srcComp->rect->rect().bottom() - srcComp->rect->rect().top();
    sourceInfo.width = srcComp->rect->rect().right() - srcComp->rect->rect().left();

    PortInfo targetInfo;
    targetInfo.isLeft = rtsConnection->targetPort->isInPort;
    targetInfo.portPos = targetPort->pos;
    RTSCompExt2GItem* trgComp = findTargetRTC(targetPort);
    targetInfo.compPos = trgComp->rect->scenePos();
    targetInfo.height = trgComp->rect->rect().bottom() - trgComp->rect->rect().top();
    targetInfo.width = trgComp->rect->rect().right() - trgComp->rect->rect().left();

    RTSConnectionExt2GItemPtr gItem =
        new RTSConnectionExt2GItem(
            rtsConnection,
            sourceInfo,
            targetInfo);

    scene.addItem(gItem);
    rtsConnections[rtsConnection->id] = gItem;
    //DDEBUG("RTSDiagramViewImpl::createConnectionGItem End");
}


void RTSDiagramExt2ViewImpl::updateStatus()
{
    if (currentRTSItem) {
        currentRTSItem->checkStatus();
    }
}

void RTSDiagramExt2ViewImpl::setCurrentRTSItem(RTSystemExt2Item* item)
{
    DDEBUG("RTSDiagramExtViewImpl::setCurrentRTSItem");

    currentRTSItem = item;
    currentRTSItem->onActivated();

    updateStatusConnection.reset(
            currentRTSItem->sigStatusUpdate().connect(
                std::bind(&RTSDiagramExt2ViewImpl::onUpdateStatus, this, _1)));

    connectionOfRTSystemItemDetachedFromRoot.reset(
        item->sigDetachedFromRoot().connect(
            [&]() { onRTSystemItemDetachedFromRoot(); }));
    rtsLoadedConnection.reset(
            currentRTSItem->sigUpdated().connect(
                std::bind(&RTSDiagramExt2ViewImpl::onRTSystemLoaded, this)));

    updateView();
}


void RTSDiagramExt2ViewImpl::updateView()
{
    DDEBUG("RTSDiagramViewImpl::updateView");
    rtsComps.clear();
    rtsConnections.clear();
    rtsPortMap.clear();
    if (currentRTSItem) {
        setBackgroundBrush(QBrush(Qt::white));
        map<string, RTSCompExt2Ptr>& comps = currentRTSItem->rtsComps();
        for (map<string, RTSCompExt2Ptr>::iterator itr = comps.begin(); itr != comps.end(); itr++) {
            addRTSComp(itr->second.get());
        }
        RTSystemExt2Item::RTSConnectionMap& connections = currentRTSItem->rtsConnections();
        DDEBUG_V("con size : %d", currentRTSItem->rtsConnections().size());
        for (RTSystemExt2Item::RTSConnectionMap::iterator itr = connections.begin();
                itr != connections.end(); itr++) {
            DDEBUG("RTSDiagramViewImpl::updateView find connection");
            if(!itr->second) {
              DDEBUG("itr->second is NULL");
              continue;
            }
            
            if (rtsConnections.find(itr->second->id) == rtsConnections.end()) {
                DDEBUG("RTSDiagramViewImpl::updateView connection NOT FOUND");
                RTSConnectionExt2* rtsConnection = itr->second.get();
                if(!rtsConnection) {
                  DDEBUG("rtsConnection is NULL");
                  continue;
                }
                RTSPortExt2GItem* source = rtsPortMap.find(rtsConnection->sourcePort)->second;
                RTSPortExt2GItem* target = rtsPortMap.find(rtsConnection->targetPort)->second;
                createConnectionGItem(rtsConnection, source, target);
            }
        }
        setAcceptDrops(true);
    } else {
        setBackgroundBrush(QBrush(Qt::gray));
        setAcceptDrops(false);
    }
    DDEBUG("RTSDiagramViewImpl::updateView End");
}

void RTSDiagramExt2ViewImpl::updateRestoredView()
{
    DDEBUG("RTSDiagramViewImpl::updateRestoredView");
    //To redraw the connection line
    if (currentRTSItem) {
        for (auto it = rtsComps.begin(); it != rtsComps.end(); it++) {
            QPointF pos = it->second->pos();
            pos.setX(pos.x() + 1);
            it->second->setPos(pos);
            pos = it->second->pos();
            pos.setX(pos.x() - 1);
            it->second->setPos(pos);
        }
    }
}

void RTSDiagramExt2ViewImpl::activateComponent()
{
    RTSCompExt2GItem* target = selectionRTCs.front();
    if (target->rtsComp->activateComponent() == false) {
        QMessageBox::information(this, _("Activate"), _("Activation of target component FAILED."));
    }
}


void RTSDiagramExt2ViewImpl::deactivateComponent()
{
    RTSCompExt2GItem* target = selectionRTCs.front();
    if (target->rtsComp->deactivateComponent() == false) {
        QMessageBox::information(this, _("Deactivate"), _("Deactivation of target component FAILED."));
    }
}


void RTSDiagramExt2ViewImpl::resetComponent()
{
    RTSCompExt2GItem* target = selectionRTCs.front();
    if (target->rtsComp->resetComponent() == false) {
        QMessageBox::information(this, _("Reset"), _("FAILED to reset target component."));
    }
}


void RTSDiagramExt2ViewImpl::finalizeComponent()
{
    RTSCompExt2GItem* target = selectionRTCs.front();
    if (target->rtsComp->finalizeComponent() == false) {
        QMessageBox::information(this, _("Exit"), _("FAILED to exit target component."));
    }
    deleteSelectedRTSItem();
}


void RTSDiagramExt2ViewImpl::startExecutionContext()
{
    RTSCompExt2GItem* target = selectionRTCs.front();
    if (target->rtsComp->startExecutionContext() == false) {
        QMessageBox::information(this, _("Start"), _("FAILED to start ExecutionContext."));
    }
}


void RTSDiagramExt2ViewImpl::stopExecutionContext()
{
    RTSCompExt2GItem* target = selectionRTCs.front();
    if (target->rtsComp->stopExecutionContext() == false) {
        QMessageBox::information(this, _("Start"), _("FAILED to stop ExecutionContext."));
    }
}

void RTSDiagramExt2ViewImpl::onRTSystemLoaded()
{
    DDEBUG("RTSDiagramViewImpl::onRTSystemLoaded");
    updateView();
    if (currentRTSItem->isCheckAtLoading()) {
        if (currentRTSItem) {
            currentRTSItem->checkStatus();
        }
    }
    updateRestoredView();
}

void RTSDiagramExt2ViewImpl::onUpdateStatus(bool modified)
{
    DDEBUG_V("RTSDiagramExt2ViewImpl::onUpdateStatus:%d", modified);
    if (modified) {
        updateView();
        updateRestoredView();
    }
}
//////////
RTSDiagramExt2View::RTSDiagramExt2View()
{
    impl = new RTSDiagramExt2ViewImpl(this);
}

RTSDiagramExt2View::~RTSDiagramExt2View()
{
    delete impl;
}

RTSDiagramExt2View* RTSDiagramExt2View::instance()
{
    return ViewManager::findView<RTSDiagramExt2View>();
}

void RTSDiagramExt2View::onActivated()
{
    impl->setNewRTSItemDetector();
}

void RTSDiagramExt2View::onDeactivated()
{
    impl->itemAddedConnection.disconnect();
}

void RTSDiagramExt2View::onRTSCompSelectionChange()
{
    impl->onRTSCompSelectionChange();
}

bool RTSDiagramExt2View::storeState(Archive& archive)
{
    if (impl->currentRTSItem) {
        archive.writeItemId("currentRTSItem", impl->currentRTSItem);
    }
    return true;
}

bool RTSDiagramExt2View::restoreState(const Archive& archive)
{
    RTSystemExt2Item* item = archive.findItem<RTSystemExt2Item>("currentRTSItem");
    if (item) {
        impl->setCurrentRTSItem(item);
    }
    return true;
}

void RTSDiagramExt2View::initializeClass(ExtensionManager* ext)
{
    ext->viewManager().registerClass<RTSDiagramExt2View>(
        "RTSDiagramView", N_("RTC Diagram"), ViewManager::SINGLE_OPTIONAL);
}
