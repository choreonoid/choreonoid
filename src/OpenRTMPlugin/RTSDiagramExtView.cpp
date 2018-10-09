/*!
 * @author Shizuko Hattori
 * @file
 */
#include "RTSDiagramExtView.h"
#include "RTSNameServerView.h"
#include "RTSPropertiesView.h"
#include "RTSCommonUtil.h"
#include "OpenRTMUtil.h"
#include "PortConnectionDialogExt.h"
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

class RTSConnectionLineExtItem : public QGraphicsLineItem
{
public:
    RTSConnectionLineExtItem(qreal x1, qreal y1, qreal x2, qreal y2)
        : QGraphicsLineItem(x1, y1, x2, y2), x1(x1), x2(x2), y1(y1), y2(y2)
    {
        QPen pen;
        pen.setColor(QColor("black"));
        pen.setWidth(1);
        setPen(pen);
        cx = (x1 + x2) / 2.0;
        cy = (y1 + y2) / 2.0;
    }

    ~RTSConnectionLineExtItem()
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
class RTSConnectionMarkerExtItem : public QGraphicsRectItem
{
public:
    enum markerType { UNMOVABLE, HORIZONTAL, VERTIAL };
    markerType type;
    Signal<void(RTSConnectionMarkerExtItem*)> sigPositionChanged;
    static const int size = 6;

    RTSConnectionMarkerExtItem(qreal x, qreal y, markerType type_)
        : QGraphicsRectItem(x - size / 2, y - size / 2, size, size),
        type(type_)
    {
        setBrush(QBrush(QColor("black")));
        if (type != UNMOVABLE) {
            setFlags(QGraphicsItem::ItemIsMovable);
        }
    }

    ~RTSConnectionMarkerExtItem()
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

class RTSConnectionExtGItem : public QGraphicsItemGroup, public Referenced
{
public:
    enum lineType { THREEPARTS_TYPE, FIVEPARTS_TYPE };
    const qreal xxoffset = 5;

    RTSConnectionExtGItem(RTSConnectionExt* rtsConnection, PortInfo source, PortInfo target);
    ~RTSConnectionExtGItem();

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
    RTSConnectionMarkerExtItem* findMarker(QGraphicsItem* gItem);
    void lineMove(RTSConnectionMarkerExtItem* marker_);
    bool changePortPos(bool isSource, QPointF s);
    lineType getType(qreal sx, qreal tx);

    RTSConnectionExtPtr rtsConnection;

private:
    bool _sIsLeft;
    bool _tIsLeft;
    lineType type;
    RTSConnectionLineExtItem* line[5];
    RTSConnectionMarkerExtItem* marker[5];
    ConnectionSet signalConnections;
    QGraphicsOpacityEffect* effect;

    double calcLinePos(PortInfo source, PortInfo target);
};

typedef ref_ptr<RTSConnectionExtGItem> RTSConnectionExtGItemPtr;

RTSConnectionExtGItem::RTSConnectionExtGItem
(RTSConnectionExt* rtsConnection, PortInfo source, PortInfo target)
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
        line[0] = new RTSConnectionLineExtItem(p0e(0), p0e(1), p0s(0), p0s(1));
        addToGroup(line[0]);
        Vector2& p1s = rtsConnection->position[4];
        Vector2& p1e = rtsConnection->position[5];
        line[1] = new RTSConnectionLineExtItem(p1s(0), p1s(1), p1e(0), p1e(1));
        addToGroup(line[1]);
        for (int i = 2; i < 5; ++i) {
            Vector2& p0 = rtsConnection->position[i - 1];
            Vector2& p1 = rtsConnection->position[i];
            line[i] = new RTSConnectionLineExtItem(p0(0), p0(1), p1(0), p1(1));
            addToGroup(line[i]);
        }
        if (line[3]->x1 == line[3]->x2) {
            type = THREEPARTS_TYPE;
            marker[0] = new RTSConnectionMarkerExtItem(line[0]->x2, line[0]->y2, RTSConnectionMarkerExtItem::UNMOVABLE);
            marker[1] = new RTSConnectionMarkerExtItem(line[1]->x2, line[1]->y2, RTSConnectionMarkerExtItem::UNMOVABLE);
            marker[2] = new RTSConnectionMarkerExtItem(line[3]->cx, line[3]->cy, RTSConnectionMarkerExtItem::HORIZONTAL);
            signalConnections.add(
                marker[2]->sigPositionChanged.connect(
                    std::bind(&RTSConnectionExtGItem::lineMove, this, _1)));
            marker[3] = marker[4] = 0;
        } else {
            type = FIVEPARTS_TYPE;
            marker[0] = new RTSConnectionMarkerExtItem(line[0]->x2, line[0]->y2, RTSConnectionMarkerExtItem::UNMOVABLE);
            marker[1] = new RTSConnectionMarkerExtItem(line[1]->x2, line[1]->y2, RTSConnectionMarkerExtItem::UNMOVABLE);
            marker[2] = new RTSConnectionMarkerExtItem(line[2]->cx, line[2]->cy, RTSConnectionMarkerExtItem::HORIZONTAL);
            marker[3] = new RTSConnectionMarkerExtItem(line[3]->cx, line[3]->cy, RTSConnectionMarkerExtItem::VERTIAL);
            marker[4] = new RTSConnectionMarkerExtItem(line[4]->cx, line[4]->cy, RTSConnectionMarkerExtItem::HORIZONTAL);
            signalConnections.add(
                marker[2]->sigPositionChanged.connect(
                    std::bind(&RTSConnectionExtGItem::lineMove, this, _1)));
            signalConnections.add(
                marker[3]->sigPositionChanged.connect(
                    std::bind(&RTSConnectionExtGItem::lineMove, this, _1)));
            signalConnections.add(
                marker[4]->sigPositionChanged.connect(
                    std::bind(&RTSConnectionExtGItem::lineMove, this, _1)));
        }
    } else {
        qreal sx, tx;
        sx = firstLineX(source.portPos.x());
        tx = endLineX(target.portPos.x());
        type = getType(sx, tx);

        line[0] = new RTSConnectionLineExtItem(sx, source.portPos.y(), source.portPos.x(), source.portPos.y());
        addToGroup(line[0]);

        line[1] = new RTSConnectionLineExtItem(tx, target.portPos.y(), target.portPos.x(), target.portPos.y());
        addToGroup(line[1]);

        qreal centerX = (sx + tx) / 2.0;
        qreal centerY = (source.portPos.y() + target.portPos.y()) / 2.0;
        if (type == THREEPARTS_TYPE) {
            line[2] = new RTSConnectionLineExtItem(sx, source.portPos.y(), centerX, source.portPos.y());
            addToGroup(line[2]);
            line[3] = new RTSConnectionLineExtItem(centerX, source.portPos.y(), centerX, target.portPos.y());
            addToGroup(line[3]);
            line[4] = new RTSConnectionLineExtItem(centerX, target.portPos.y(), tx, target.portPos.y());
            addToGroup(line[4]);
            marker[0] = new RTSConnectionMarkerExtItem(line[0]->x2, line[0]->y2, RTSConnectionMarkerExtItem::UNMOVABLE);
            marker[1] = new RTSConnectionMarkerExtItem(line[1]->x2, line[1]->y2, RTSConnectionMarkerExtItem::UNMOVABLE);
            marker[2] = new RTSConnectionMarkerExtItem(line[3]->cx, line[3]->cy, RTSConnectionMarkerExtItem::HORIZONTAL);
            signalConnections.add(
                marker[2]->sigPositionChanged.connect(
                    std::bind(&RTSConnectionExtGItem::lineMove, this, _1)));
            marker[3] = marker[4] = 0;

        } else {
            centerY = calcLinePos(source, target);

            line[2] = new RTSConnectionLineExtItem(sx, source.portPos.y(), sx, centerY);
            addToGroup(line[2]);
            line[3] = new RTSConnectionLineExtItem(sx, centerY, tx, centerY);
            addToGroup(line[3]);
            line[4] = new RTSConnectionLineExtItem(tx, centerY, tx, target.portPos.y());
            addToGroup(line[4]);
            marker[0] = new RTSConnectionMarkerExtItem(line[0]->x2, line[0]->y2, RTSConnectionMarkerExtItem::UNMOVABLE);
            marker[1] = new RTSConnectionMarkerExtItem(line[1]->x2, line[1]->y2, RTSConnectionMarkerExtItem::UNMOVABLE);
            marker[2] = new RTSConnectionMarkerExtItem(line[2]->cx, line[2]->cy, RTSConnectionMarkerExtItem::HORIZONTAL);
            marker[3] = new RTSConnectionMarkerExtItem(line[3]->cx, line[3]->cy, RTSConnectionMarkerExtItem::VERTIAL);
            marker[4] = new RTSConnectionMarkerExtItem(line[4]->cx, line[4]->cy, RTSConnectionMarkerExtItem::HORIZONTAL);
            signalConnections.add(
                marker[2]->sigPositionChanged.connect(
                    std::bind(&RTSConnectionExtGItem::lineMove, this, _1)));
            signalConnections.add(
                marker[3]->sigPositionChanged.connect(
                    std::bind(&RTSConnectionExtGItem::lineMove, this, _1)));
            signalConnections.add(
                marker[4]->sigPositionChanged.connect(
                    std::bind(&RTSConnectionExtGItem::lineMove, this, _1)));
        }

        Vector2 pos[6];
        getLinePosition(pos);
        rtsConnection->setPosition(pos);
    }

    setFlags(QGraphicsItem::ItemIsSelectable);
}

RTSConnectionExtGItem::~RTSConnectionExtGItem()
{
    signalConnections.disconnect();

    for (int i = 0; i < 5; ++i) {
        if (marker[i]) {
            delete marker[i];
        }
    }
}

qreal RTSConnectionExtGItem::firstLineX(qreal x)
{
    if (_sIsLeft) {
        return x - xxoffset;
    } else {
        return x + xxoffset;
    }
}

qreal RTSConnectionExtGItem::endLineX(qreal x)
{
    if (_tIsLeft) {
        return x - xxoffset;
    } else {
        return x + xxoffset;
    }
}

int RTSConnectionExtGItem::markerIndex(QGraphicsItem* gItem)
{
    for (int i = 2; i < 5; ++i) {
        if (marker[i] == gItem) {
            return i;
        }
    }
    return -1;
}

RTSConnectionMarkerExtItem* RTSConnectionExtGItem::findMarker(QGraphicsItem* gItem)
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

void RTSConnectionExtGItem::lineMove(RTSConnectionMarkerExtItem* marker_)
{
    int i = markerIndex(marker_);
    if (i < 0) {
        return;
    }
    QPointF c = marker_->rect().center();

    if (type == RTSConnectionExtGItem::THREEPARTS_TYPE) {
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

RTSConnectionExtGItem::lineType RTSConnectionExtGItem::getType(qreal sx, qreal tx)
{
    if ((_sIsLeft && tx < sx) || (_tIsLeft &&  sx < tx) || (!_sIsLeft && sx < tx) || (!_tIsLeft && tx < sx)) {
        return THREEPARTS_TYPE;
    } else {
        return FIVEPARTS_TYPE;
    }
}

bool RTSConnectionExtGItem::changePortPos(bool isSource, QPointF s)
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

void RTSConnectionExtGItem::showMarker(bool on)
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

double RTSConnectionExtGItem::calcLinePos(PortInfo source, PortInfo target) {
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
    }

    return result;
}

//////////
class RTSCompExtGItem;

class RTSPortExtGItem : public QGraphicsItemGroup, public Referenced
{
public:
    enum portType { INPORT, OUTPORT, SERVICEPORT };
    RTSPortExt* rtsPort;
    QGraphicsPolygonItem* polygon;
    QPointF pos;
    RTSCompExtGItem* parent;

    RTSPortExtGItem(RTSPortExt* rtsPort, RTSCompExtGItem* rtc);
    void create(int rectX, const QPointF& pos, int i, portType type);
    void checkPortState();
    void setCandidate(bool isCand);

};

typedef ref_ptr<RTSPortExtGItem> RTSPortExtGItemPtr;

RTSPortExtGItem::RTSPortExtGItem(RTSPortExt* rtsPort, RTSCompExtGItem* rtc)
    : rtsPort(rtsPort), parent(rtc)
{

}

void RTSPortExtGItem::create(int rectX, const QPointF& pos, int i, portType type)
{
    int r = 7 * i;

    switch (type) {

        case INPORT:
            this->pos = QPointF(rectX + 5 + pos.x(), 5 + 7 + 7 + (25 * i) - r + pos.y());
            polygon = new QGraphicsPolygonItem(
                QPolygonF(QVector<QPointF>()
                    << QPointF(rectX + 0 + pos.x(), 0 + 7 + 7 + (25 * i) - r + pos.y())
                    << QPointF(rectX + 10 + pos.x(), 0 + 7 + 7 + (25 * i) - r + pos.y())
                    << QPointF(rectX + 10 + pos.x(), 10 + 7 + 7 + (25 * i) - r + pos.y())
                    << QPointF(rectX + 0 + pos.x(), 10 + 7 + 7 + (25 * i) - r + pos.y())
                    << QPointF(rectX + 5 + pos.x(), 5 + 7 + 7 + (25 * i) - r + pos.y())
                    << QPointF(rectX + 0 + pos.x(), 0 + 7 + 7 + (25 * i) - r + pos.y())));
            polygon->setPen(QPen(QColor("red")));
            checkPortState();
            addToGroup(polygon);
            break;

        case OUTPORT:
            polygon = new QGraphicsPolygonItem(
                QPolygonF(QVector<QPointF>()
                    << QPointF(rectX + 53 + pos.x(), 0 + 7 + 7 + (25 * i) - r + pos.y())
                    << QPointF(rectX + 60 + pos.x(), 0 + 7 + 7 + (25 * i) - r + pos.y())
                    << QPointF(rectX + 65 + pos.x(), 5 + 7 + 7 + (25 * i) - r + pos.y())
                    << QPointF(rectX + 60 + pos.x(), 10 + 7 + 7 + (25 * i) - r + pos.y())
                    << QPointF(rectX + 53 + pos.x(), 10 + 7 + 7 + (25 * i) - r + pos.y())
                    << QPointF(rectX + 53 + pos.x(), 0 + 7 + 7 + (25 * i) - r + pos.y())));
            this->pos = QPointF(rectX + 65 + pos.x(), 5 + 7 + 7 + (25 * i) - r + pos.y());
            polygon->setPen(QPen(QColor("red")));
            checkPortState();
            addToGroup(polygon);
            break;

        case SERVICEPORT:
            polygon = new QGraphicsPolygonItem(
                QPolygonF(QVector<QPointF>()
                    << QPointF(rectX + 53 + pos.x(), 0 + 7 + 7 + (25 * i) - r + pos.y())
                    << QPointF(rectX + 63 + pos.x(), 0 + 7 + 7 + (25 * i) - r + pos.y())
                    << QPointF(rectX + 63 + pos.x(), 5 + 7 + 7 + (25 * i) - r + pos.y())
                    << QPointF(rectX + 63 + pos.x(), 10 + 7 + 7 + (25 * i) - r + pos.y())
                    << QPointF(rectX + 53 + pos.x(), 10 + 7 + 7 + (25 * i) - r + pos.y())
                    << QPointF(rectX + 53 + pos.x(), 0 + 7 + 7 + (25 * i) - r + pos.y())));
            this->pos = QPointF(rectX + 63 + pos.x(), 5 + 7 + 7 + (25 * i) - r + pos.y());
            polygon->setPen(QPen(QColor("red")));
            checkPortState();
            addToGroup(polygon);
            break;
    }
}

void RTSPortExtGItem::checkPortState()
{
    polygon->setBrush(QBrush(QColor(rtsPort->isConnected_ ? "lightgreen" : (rtsPort->isServicePort ? "lightblue" : "blue"))));
}

void RTSPortExtGItem::setCandidate(bool isCand)
{
    if (isCand) {
        polygon->setPen(QPen(QColor("magenta"), 3));
    } else {
        polygon->setPen(QPen(QColor("red"), 1));
    }
}
//////////
class RTSCompExtGItem : public QGraphicsItemGroup, public Referenced
{
public:
    RTSCompExtGItem(RTSCompExt* rtsComp, RTSDiagramExtViewImpl* impl, const QPointF& pos);
    ~RTSCompExtGItem();
    QVariant itemChange(GraphicsItemChange change, const QVariant & value);
    void create(const QPointF& pos);
    void checkRTCState();
    void checkCandidate(RTSPortExtGItem* sourcePort);
    void clearCandidate();
    int correctTextY();

    RTSDiagramExtViewImpl* impl;
    RTSCompExt* rtsComp;
    map<string, RTSPortExtGItemPtr> inPorts;
    map<string, RTSPortExtGItemPtr> outPorts;
    QGraphicsRectItem* rect;
    QGraphicsOpacityEffect* effect;
    Signal<void(const RTSCompExtGItem*)> sigPositionChanged;
    Connection positionChangeConnection;
};

typedef ref_ptr<RTSCompExtGItem> RTSCompExtGItemPtr;

QVariant RTSCompExtGItem::itemChange(GraphicsItemChange change, const QVariant & value)
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

int RTSCompExtGItem::correctTextY()
{
    int numIn = rtsComp->inPorts.size();
    int numOut = rtsComp->outPorts.size();
    int numMax = (numIn < numOut ? numOut : numIn);
    if (!numMax) {
        numMax = 1;
    }
    return (25 * numMax) - 7 * (numMax - 1);
}

void RTSCompExtGItem::checkCandidate(RTSPortExtGItem* sourcePort)
{
    if (!sourcePort->rtsPort) {
        return;
    }
    if (sourcePort->rtsPort->isInPort) {
        for (map<string, RTSPortExtGItemPtr>::iterator it = outPorts.begin(); it != outPorts.end(); it++) {
            bool isCand = sourcePort->rtsPort->checkConnectablePort(it->second->rtsPort);
            it->second->setCandidate(isCand);
        }
    } else {
        for (map<string, RTSPortExtGItemPtr>::iterator it = inPorts.begin(); it != inPorts.end(); it++) {
            bool isCand = sourcePort->rtsPort->checkConnectablePort(it->second->rtsPort);
            it->second->setCandidate(isCand);
        }
    }
}

void RTSCompExtGItem::clearCandidate()
{
    for (map<string, RTSPortExtGItemPtr>::iterator it = inPorts.begin(); it != inPorts.end(); it++) {
        it->second->setCandidate(false);
    }

    for (map<string, RTSPortExtGItemPtr>::iterator it = outPorts.begin(); it != outPorts.end(); it++) {
        it->second->setCandidate(false);
    }
}

void RTSCompExtGItem::checkRTCState()
{
    RTC_STATUS status = rtsComp->rtc_status_;
    if (status == RTC_STATUS::RTC_INACTIVE) {
        rect->setBrush(QBrush(QColor("blue")));
    } else if (status == RTC_STATUS::RTC_ACTIVE) {
        rect->setBrush(QBrush(QColor("lightgreen")));
    } else if (status == RTC_STATUS::RTC_ERROR) {
        rect->setBrush(QBrush(QColor("red")));
    }

    for (map<string, RTSPortExtGItemPtr>::iterator it = inPorts.begin(); it != inPorts.end(); it++) {
        it->second->checkPortState();
    }

    for (map<string, RTSPortExtGItemPtr>::iterator it = outPorts.begin(); it != outPorts.end(); it++) {
        it->second->checkPortState();
    }
}

}
//////////
namespace cnoid {

class RTSDiagramExtViewImpl : public QGraphicsView
{
public:
    RTSystemItemExtPtr currentRTSItem;
    ScopedConnection itemAddedConnection;
    ScopedConnection itemTreeViewSelectionChangedConnection;
    ScopedConnection connectionOfRTSystemItemDetachedFromRoot;

    ScopedConnection rtsLoadedConnection;
    ScopedConnection updateStatusConnection;

    map<string, RTSCompExtGItemPtr> rtsComps;
    map<string, RTSConnectionExtGItemPtr> rtsConnections;
    map<RTSPortExt*, RTSPortExtGItem*> rtsPortMap;
    list<NamingContextHelper::ObjectInfo> nsViewSelections;
    list<RTSCompExtGItem*> selectionRTCs;
    list<RTSConnectionExtGItem*> selectionRTSConnections;
    MenuManager menuManager;
    RTSPortExtGItem* sourcePort;
    QGraphicsLineItem* dragPortLine;
    RTSConnectionMarkerExtItem* targetMarker;
    int pollingPeriod;

    RTSDiagramExtViewImpl(RTSDiagramExtView* self);
    ~RTSDiagramExtViewImpl();
    void setNewRTSItemDetector();
    void addRTSComp(NamingContextHelper::ObjectInfo& info, const QPointF& pos);
    void addRTSComp(RTSCompExt* rtsComp);
    void deleteRTSComp(RTSCompExtGItem* rtsComp);
    void deleteRTSConnection(RTSConnectionExtGItem* rtsConnection);
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
    RTSPortExtGItem* findTargetRTSPort(QPointF& pos);
    RTSCompExtGItem* findTargetRTC(RTSPortExtGItem* port);
    RTSConnectionMarkerExtItem* findConnectionMarker(QGraphicsItem* gItem);
    void createConnectionGItem(RTSConnectionExt* rtsConnection, RTSPortExtGItem* sourcePort, RTSPortExtGItem* targetPort);
    void onRTSCompSelectionChange();
    void onRTSCompPositionChanged(const RTSCompExtGItem*);
    void onActivated(bool on);
    void onItemTreeViewSelectionChanged(const ItemList<RTSystemExtItem>& items);
    void onRTSystemItemDetachedFromRoot();
    void setCurrentRTSItem(RTSystemExtItem* item);
    void updateView();
    void updateRestoredView();

    void activateComponent();
    void deactivateComponent();
    void resetComponent();
    void finalizeComponent();
    void startExecutionContext();
    void stopExecutionContext();

    void onRTSystemLoaded(bool isRestored);
    void onUpdateStatus(bool modified);

private:
    RTSDiagramExtView* self;
    QGraphicsScene  scene;

    ScopedConnection nsViewSelectionChangedConnection;

    bool activated;

    void updateStatus();
};

}
//////////
RTSCompExtGItem::RTSCompExtGItem(RTSCompExt* rtsComp, RTSDiagramExtViewImpl* impl, const QPointF& pos)
    : impl(impl), rtsComp(rtsComp)
{
    DDEBUG("RTSCompGItem::RTSCompGItem(RTSComp");
    effect = new QGraphicsOpacityEffect;
    effect->setOpacity(OPACITY);
    setGraphicsEffect(effect);
    if (!CORBA::is_nil(rtsComp->rtc_)) {
        rtsComp->isAlive_ = true;
        effect->setEnabled(false);
    } else {
        rtsComp->isAlive_ = false;
        effect->setEnabled(true);
    }

    create(pos);
    positionChangeConnection = sigPositionChanged.connect(
        std::bind(&RTSDiagramExtViewImpl::onRTSCompPositionChanged, impl, _1));
}

RTSCompExtGItem::~RTSCompExtGItem()
{
    positionChangeConnection.disconnect();
    for (auto it = inPorts.begin(); it != inPorts.end(); it++) {
        impl->rtsPortMap.erase(it->second->rtsPort);
    }
    for (auto it = outPorts.begin(); it != outPorts.end(); it++) {
        impl->rtsPortMap.erase(it->second->rtsPort);
    }
}

void RTSCompExtGItem::create(const QPointF& pos)
{
    QGraphicsTextItem * text = new QGraphicsTextItem(QString(rtsComp->name.c_str()));
    int height = correctTextY();
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
    for (vector<RTSPortExtPtr>::iterator it = rtsComp->inPorts.begin(); it != rtsComp->inPorts.end(); it++, i++) {
        RTSPortExt* inPort = *it;
        string portName = string(inPort->name);
        RTCCommonUtil::splitPortName(portName);
        text = new QGraphicsTextItem(QString(portName.c_str()));
        int x = rectX + pos.x() - text->boundingRect().width();
        int y = 25 * i - 7 * i - 3 + pos.y();
        text->setPos(x, y);
        addToGroup(text);

        RTSPortExtGItemPtr inPortGItem = new RTSPortExtGItem(inPort, this);
        inPortGItem->create(rectX, pos, i, RTSPortExtGItem::INPORT);
        addToGroup(inPortGItem);
        inPorts[inPort->name] = inPortGItem;
        impl->rtsPortMap[inPort] = inPortGItem.get();
    }

    i = 0;
    for (vector<RTSPortExtPtr>::iterator it = rtsComp->outPorts.begin(); it != rtsComp->outPorts.end(); it++, i++) {
        RTSPortExt* outPort = *it;
        string portName = string(outPort->name);
        RTCCommonUtil::splitPortName(portName);
        text = new QGraphicsTextItem(QString(portName.c_str()));
        int r = 7 * i;
        int x = rectX + 66 + pos.x();
        int y = 25 * i - r - 3 + pos.y();
        text->setPos(x, y);
        addToGroup(text);

        RTSPortExtGItemPtr outPortGItem = new RTSPortExtGItem(outPort, this);
        if (outPort->isServicePort) {
            outPortGItem->create(rectX, pos, i, RTSPortExtGItem::SERVICEPORT);
        } else {
            outPortGItem->create(rectX, pos, i, RTSPortExtGItem::OUTPORT);
        }
        addToGroup(outPortGItem);
        outPorts[outPort->name] = outPortGItem;
        impl->rtsPortMap[outPort] = outPortGItem.get();
    }

    checkRTCState();
}
//////////
RTSDiagramExtViewImpl::RTSDiagramExtViewImpl(RTSDiagramExtView* self)
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
                std::bind(&RTSDiagramExtViewImpl::onnsViewItemSelectionChanged, this, _1)));
    }

    self->sigActivated().connect(std::bind(&RTSDiagramExtViewImpl::onActivated, this, true));
    self->sigDeactivated().connect(std::bind(&RTSDiagramExtViewImpl::onActivated, this, false));

    itemTreeViewSelectionChangedConnection.reset(
        ItemTreeView::mainInstance()->sigSelectionChanged().connect(
            std::bind(&RTSDiagramExtViewImpl::onItemTreeViewSelectionChanged, this, _1)));

    QPen pen(Qt::DashDotLine);
    pen.setWidth(2);
    dragPortLine = new QGraphicsLineItem();
    dragPortLine->setPen(pen);
    dragPortLine->setZValue(100);
    dragPortLine->setVisible(false);
    scene.addItem(dragPortLine);

    targetMarker = 0;
}


RTSDiagramExtViewImpl::~RTSDiagramExtViewImpl()
{
    rtsComps.clear();
    rtsConnections.clear();
    disconnect(&scene, SIGNAL(selectionChanged()), self, SLOT(onRTSCompSelectionChange()));
}


void RTSDiagramExtViewImpl::dragEnterEvent(QDragEnterEvent *event)
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


void RTSDiagramExtViewImpl::dragMoveEvent(QDragMoveEvent *event)
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


void RTSDiagramExtViewImpl::dragLeaveEvent(QDragLeaveEvent *event)
{
    MessageView::instance()->putln(_("Drag and drop has been canceled. Please be operation again."));
}


void RTSDiagramExtViewImpl::dropEvent(QDropEvent *event)
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


void RTSDiagramExtViewImpl::keyPressEvent(QKeyEvent *event)
{
    if (event->key() == Qt::Key_Delete) {
        deleteSelectedRTSItem();
    }
}


void RTSDiagramExtViewImpl::mousePressEvent(QMouseEvent* event)
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
                    ->sigTriggered().connect(std::bind(&RTSDiagramExtViewImpl::activateComponent, this));
                menuManager.addItem("Deactivate")
                    ->sigTriggered().connect(std::bind(&RTSDiagramExtViewImpl::deactivateComponent, this));
                menuManager.addItem("Reset")
                    ->sigTriggered().connect(std::bind(&RTSDiagramExtViewImpl::resetComponent, this));
                if (isManagedRTC(selectionRTCs.front()->rtsComp->rtc_) == false) {
                    menuManager.addItem("Exit")
                        ->sigTriggered().connect(std::bind(&RTSDiagramExtViewImpl::finalizeComponent, this));
                }
                menuManager.addSeparator();
                menuManager.addItem("Start")
                    ->sigTriggered().connect(std::bind(&RTSDiagramExtViewImpl::startExecutionContext, this));
                menuManager.addItem("Stop")
                    ->sigTriggered().connect(std::bind(&RTSDiagramExtViewImpl::stopExecutionContext, this));
                menuManager.addSeparator();
            }
            menuManager.addItem("Remove")
                ->sigTriggered().connect(std::bind(&RTSDiagramExtViewImpl::deleteSelectedRTSItem, this));

            menuManager.popupMenu()->popup(event->globalPos());

        } else if (!selectionRTSConnections.empty()) {
            menuManager.setNewPopupMenu(this);
            menuManager.addItem("Delete")
                ->sigTriggered().connect(std::bind(&RTSDiagramExtViewImpl::deleteSelectedRTSItem, this));

            menuManager.popupMenu()->popup(event->globalPos());

        } else {
            if (currentRTSItem->stateCheck() == 1) {
                menuManager.setNewPopupMenu(this);
                menuManager.addItem(_("Update"))
                    ->sigTriggered().connect(std::bind(&RTSDiagramExtViewImpl::updateStatus, this));

                menuManager.popupMenu()->popup(event->globalPos());
            }
        }
    }
}


void RTSDiagramExtViewImpl::mouseMoveEvent(QMouseEvent* event)
{
    QPointF pos = mapToScene(event->pos());

    if (sourcePort) {
        QPointF sp = sourcePort->pos;
        dragPortLine->setLine(sp.x(), sp.y(), pos.x(), pos.y());
        RTSPortExtGItem* port = findTargetRTSPort(pos);
        if (port && !sourcePort->rtsPort->checkConnectablePort(port->rtsPort)) {
            setCursor(Qt::ForbiddenCursor);
        } else {
            setCursor(Qt::PointingHandCursor);
        }

        for (map<string, RTSCompExtGItemPtr>::iterator it = rtsComps.begin(); it != rtsComps.end(); it++) {
            it->second->checkCandidate(sourcePort);
        }

    } else if (targetMarker) {
        targetMarker->move(pos.x(), pos.y());

    } else {
        RTSPortExtGItem* port = findTargetRTSPort(pos);
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


void RTSDiagramExtViewImpl::mouseReleaseEvent(QMouseEvent *event)
{
    QPointF pos = mapToScene(event->pos());

    if (sourcePort) {
        for (map<string, RTSCompExtGItemPtr>::iterator it = rtsComps.begin(); it != rtsComps.end(); it++) {
            it->second->clearCandidate();
        }

        dragPortLine->setVisible(false);
        RTSPortExtGItem* targetPort = findTargetRTSPort(pos);

        if (targetPort && sourcePort->rtsPort->checkConnectablePort(targetPort->rtsPort)) {
            if (sourcePort->rtsPort->isConnectedWith(targetPort->rtsPort)) {
                MessageView::instance()->putln(_("These are already connected."));

            } else {
                bool isAccepted = false;
                string name;
                vector<NamedValueExtPtr> propList;
                if (sourcePort->rtsPort->isServicePort && targetPort->rtsPort->isServicePort) {
                    DDEBUG("ServicePort Connect");
                    ServicePortConnectionDialogExt dialog;
                    dialog.setDisp(sourcePort->rtsPort, targetPort->rtsPort);
                    dialog.exec();
                    isAccepted = dialog.isAccepted;
                    name = dialog.nameLineEdit->text().toStdString();
                    propList = dialog.propList;

                } else {
                    DDEBUG("DataPort Connect");
                    DataPortConnectionDialogExt dialog;
                    dialog.setDisp(sourcePort->rtsPort, targetPort->rtsPort);
                    dialog.exec();
                    isAccepted = dialog.isAccepted;
                    name = dialog.nameLineEdit->text().toStdString();
                    propList = dialog.propList;
                }

                if (isAccepted) {
                    DDEBUG("RTSDiagramViewImpl::mouseReleaseEvent Accepted");
                    string id = "";
                    RTSConnectionExt* rtsConnection =
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


void RTSDiagramExtViewImpl::wheelEvent(QWheelEvent* event)
{
    double dSteps = (double)event->delta() / 120.0;
    double scaleVal = 1.0;
    scaleVal -= (dSteps / 20.0);
    scale(scaleVal, scaleVal);
    QGraphicsView::wheelEvent(event);
}


void RTSDiagramExtViewImpl::onnsViewItemSelectionChanged(const list<NamingContextHelper::ObjectInfo>& items)
{
    nsViewSelections = items;
}

void RTSDiagramExtViewImpl::onRTSCompSelectionChange()
{
    RTSCompExtGItem* singleSelectedRTC = 0;
    if (selectionRTCs.size() == 1) {
        singleSelectedRTC = selectionRTCs.front();
    }

    RTSConnectionExtGItem* singleSelectedConnection = 0;
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
        RTSCompExtGItem* rtsComp = dynamic_cast<RTSCompExtGItem*>(*it);
        if (rtsComp) {
            selectionRTCs.push_back(rtsComp);
        } else {
            RTSConnectionExtGItem* rtsConnection = dynamic_cast<RTSConnectionExtGItem*>(*it);
            if (rtsConnection) {
                selectionRTSConnections.push_back(rtsConnection);
                rtsConnection->showMarker(true);
            }
        }
    }

    if (selectionRTCs.size() == 1 && singleSelectedRTC != selectionRTCs.front()) {
        RTSNameServerView* nsView = RTSNameServerView::instance();
        if (nsView) {
            RTSCompExt* selected = selectionRTCs.front()->rtsComp;
            QString hostInfo = QString::fromStdString(selected->hostAddress) + ":" + QString::number(selected->portNo);
            nsView->setSelection(selected->name, selected->fullPath, hostInfo.toStdString());
        }
    }

    if (selectionRTSConnections.size() == 1 && singleSelectedConnection != selectionRTSConnections.front()) {
        RTSNameServerView* nsView = RTSNameServerView::instance();
        if (nsView) {
            nsView->setSelection("", "", "");
        }
        RTSPropertiesView* propView = RTSPropertiesView::instance();
        if (propView) {
            RTSConnectionExt* rtsConnection = selectionRTSConnections.front()->rtsConnection;
            propView->showConnectionProperties(rtsConnection->sourcePort->port, rtsConnection->id);
        }
    }
}


void RTSDiagramExtViewImpl::onRTSCompPositionChanged(const RTSCompExtGItem* rtsCompGItem)
{
    list<RTSConnectionExt*> rtsConnectionList;
    currentRTSItem->RTSCompToConnectionList(rtsCompGItem->rtsComp, rtsConnectionList, 1);

    for (auto it = rtsConnectionList.begin(); it != rtsConnectionList.end(); it++) {
        RTSConnectionExt* rtsConnection = *it;
        RTSConnectionExtGItem* gItem = rtsConnections.find(rtsConnection->id)->second.get();
        RTSPortExtGItem* sourcePort = rtsPortMap.find(rtsConnection->sourcePort)->second;
        if (!gItem->changePortPos(true, sourcePort->pos)) {
            rtsConnections.erase(rtsConnection->id);
            RTSPortExtGItem* targetPort = rtsPortMap.find(rtsConnection->targetPort)->second;
            rtsConnection->setPos = false;
            createConnectionGItem(rtsConnection, sourcePort, targetPort);
        }
    }

    rtsConnectionList.clear();
    currentRTSItem->RTSCompToConnectionList(rtsCompGItem->rtsComp, rtsConnectionList, 2);
    for (auto it = rtsConnectionList.begin(); it != rtsConnectionList.end(); it++) {
        RTSConnectionExt* rtsConnection = *it;
        RTSConnectionExtGItem* gItem = rtsConnections.find(rtsConnection->id)->second.get();
        RTSPortExtGItem* targetPort = rtsPortMap.find(rtsConnection->targetPort)->second;
        if (!gItem->changePortPos(false, targetPort->pos)) {
            rtsConnections.erase(rtsConnection->id);
            RTSPortExtGItem* sourcePort = rtsPortMap.find(rtsConnection->sourcePort)->second;
            rtsConnection->setPos = false;
            createConnectionGItem(rtsConnection, sourcePort, targetPort);
        }
    }
}


void RTSDiagramExtViewImpl::onActivated(bool on)
{
    DDEBUG_V("RTSDiagramExtViewImpl::onActivated : %d", on);
    activated = on;
    if( currentRTSItem ) {
        if(on) {
            updateStatusConnection.reset(
                    currentRTSItem->sigStatusUpdate().connect(
                        std::bind(&RTSDiagramExtViewImpl::onUpdateStatus, this, _1)));
        } else {
            updateStatusConnection.disconnect();
        }
        currentRTSItem->onActivated();
    }
}

void RTSDiagramExtViewImpl::onItemTreeViewSelectionChanged(const ItemList<RTSystemExtItem>& items)
{
    RTSystemExtItem* firstItem = items.toSingle();

    if (firstItem && firstItem != currentRTSItem) {
        setCurrentRTSItem(firstItem);
    }
}


void RTSDiagramExtViewImpl::onRTSystemItemDetachedFromRoot()
{
    currentRTSItem = nullptr;
    connectionOfRTSystemItemDetachedFromRoot.disconnect();
    updateView();
    setNewRTSItemDetector();
}


RTSPortExtGItem* RTSDiagramExtViewImpl::findTargetRTSPort(QPointF& pos)
{
    for (map<string, RTSCompExtGItemPtr>::iterator it = rtsComps.begin(); it != rtsComps.end(); it++) {
        map<string, RTSPortExtGItemPtr>& inPorts = it->second->inPorts;
        for (map<string, RTSPortExtGItemPtr>::iterator itr = inPorts.begin(); itr != inPorts.end(); itr++) {
            if (itr->second->sceneBoundingRect().contains(pos)) {
                return itr->second.get();
            }
        }
        map<string, RTSPortExtGItemPtr>& outPorts = it->second->outPorts;
        for (map<string, RTSPortExtGItemPtr>::iterator itr = outPorts.begin(); itr != outPorts.end(); itr++) {
            if (itr->second->sceneBoundingRect().contains(pos)) {
                return itr->second.get();
            }
        }
    }
    return nullptr;
}

RTSCompExtGItem* RTSDiagramExtViewImpl::findTargetRTC(RTSPortExtGItem* port) 
{
    for (map<string, RTSCompExtGItemPtr>::iterator it = rtsComps.begin(); it != rtsComps.end(); it++) {
        map<string, RTSPortExtGItemPtr>& inPorts = it->second->inPorts;
        for (map<string, RTSPortExtGItemPtr>::iterator itr = inPorts.begin(); itr != inPorts.end(); itr++) {
            if (itr->second == port) {
                return it->second.get();
            }
        }
        map<string, RTSPortExtGItemPtr>& outPorts = it->second->outPorts;
        for (map<string, RTSPortExtGItemPtr>::iterator itr = outPorts.begin(); itr != outPorts.end(); itr++) {
            if (itr->second == port) {
                return it->second.get();
            }
        }
    }
    return nullptr;
}


RTSConnectionMarkerExtItem* RTSDiagramExtViewImpl::findConnectionMarker(QGraphicsItem* gItem)
{
    for (auto it = selectionRTSConnections.begin(); it != selectionRTSConnections.end(); it++) {
        RTSConnectionMarkerExtItem* marker = (*it)->findMarker(gItem);
        if (marker) {
            return marker;
        }
    }
    return nullptr;
}


void RTSDiagramExtViewImpl::setNewRTSItemDetector()
{
    itemAddedConnection.disconnect();

    if (!currentRTSItem) {
        ItemList<RTSystemExtItem> rtsItems;
        if (rtsItems.extractChildItems(RootItem::instance())) {
            setCurrentRTSItem(rtsItems[0]);
        } else {
            itemAddedConnection.reset(
                RootItem::instance()->sigItemAdded().connect(
                    [&](Item* item) {
                if (!currentRTSItem) {
                    auto rtsItem = dynamic_cast<RTSystemExtItem*>(item);
                    if (rtsItem) {
                        setCurrentRTSItem(rtsItem);
                        itemAddedConnection.disconnect();
                    }
                }
            }));
        }
    }
}


void RTSDiagramExtViewImpl::addRTSComp(NamingContextHelper::ObjectInfo& info, const QPointF& pos)
{
    RTSCompExt* rtsComp = currentRTSItem->addRTSComp(info, pos);
    if (rtsComp) {
        RTSCompExtGItemPtr rtsCompGItem = new RTSCompExtGItem(rtsComp, this, pos);
        rtsComps[info.getFullPath()] = rtsCompGItem;
        scene.addItem(rtsCompGItem);

        RTSystemExtItem::RTSConnectionMap& connections = currentRTSItem->rtsConnections();
        for (RTSystemExtItem::RTSConnectionMap::iterator itr = connections.begin(); itr != connections.end(); itr++) {
            if (rtsConnections.find(itr->second->id) == rtsConnections.end()) {
                RTSConnectionExt* rtsConnection = itr->second.get();
                RTSPortExtGItem* source = rtsPortMap.find(rtsConnection->sourcePort)->second;
                RTSPortExtGItem* target = rtsPortMap.find(rtsConnection->targetPort)->second;
                createConnectionGItem(rtsConnection, source, target);
            }
        }
    }
    updateView();
}


void RTSDiagramExtViewImpl::addRTSComp(RTSCompExt* rtsComp)
{
    RTSCompExtGItemPtr rtsCompGItem = new RTSCompExtGItem(rtsComp, this, rtsComp->pos());
    rtsComps[rtsComp->fullPath] = rtsCompGItem;
    scene.addItem(rtsCompGItem);
}


void RTSDiagramExtViewImpl::deleteRTSComp(RTSCompExtGItem* rtsCompGItem)
{
    list<RTSConnectionExt*> rtsConnectionList;
    currentRTSItem->RTSCompToConnectionList(rtsCompGItem->rtsComp, rtsConnectionList);

    for (auto itr = rtsConnectionList.begin(); itr != rtsConnectionList.end(); itr++) {
        map<string, RTSConnectionExtGItemPtr>::iterator it = rtsConnections.find((*itr)->id);
        if (it != rtsConnections.end()) {
            deleteRTSConnection(it->second);
        }
    }

    const string name = rtsCompGItem->rtsComp->fullPath;
    currentRTSItem->deleteRTSComp(name);
    rtsComps.erase(name);
}


void RTSDiagramExtViewImpl::deleteRTSConnection(RTSConnectionExtGItem* rtsConnectionGItem)
{
    currentRTSItem->disconnectAndRemoveConnection(rtsConnectionGItem->rtsConnection);

    rtsConnections.erase(rtsConnectionGItem->rtsConnection->id);
}


void RTSDiagramExtViewImpl::deleteSelectedRTSItem()
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


void RTSDiagramExtViewImpl::createConnectionGItem
(RTSConnectionExt* rtsConnection, RTSPortExtGItem* sourcePort, RTSPortExtGItem* targetPort)
{
    DDEBUG("RTSDiagramViewImpl::createConnectionGItem");

    PortInfo sourceInfo;
    sourceInfo.isLeft = rtsConnection->sourcePort->isInPort;
    sourceInfo.portPos = sourcePort->pos;
    RTSCompExtGItem* srcComp = findTargetRTC(sourcePort);
    sourceInfo.compPos = srcComp->rect->scenePos();
    sourceInfo.height = srcComp->rect->rect().bottom() - srcComp->rect->rect().top();
    sourceInfo.width = srcComp->rect->rect().right() - srcComp->rect->rect().left();

    PortInfo targetInfo;
    targetInfo.isLeft = rtsConnection->targetPort->isInPort;
    targetInfo.portPos = targetPort->pos;
    RTSCompExtGItem* trgComp = findTargetRTC(targetPort);
    targetInfo.compPos = trgComp->rect->scenePos();
    targetInfo.height = trgComp->rect->rect().bottom() - trgComp->rect->rect().top();
    targetInfo.width = trgComp->rect->rect().right() - trgComp->rect->rect().left();

    RTSConnectionExtGItemPtr gItem =
        new RTSConnectionExtGItem(
            rtsConnection,
            sourceInfo,
            targetInfo);

    DDEBUG_V("SrcPort x:%f, y:%f, TrgPort x:%f, y:%f",
        sourcePort->pos.x(), sourcePort->pos.y(),
        targetPort->pos.x(), targetPort->pos.y());


    DDEBUG_V("SrcComp x:%f, y:%f, TrgComp x:%f, y:%f",
        srcComp->rect->scenePos().x(), srcComp->rect->scenePos().y(),
        trgComp->rect->scenePos().x(), trgComp->rect->scenePos().y());
    DDEBUG_V("SrcComp Height x:%f, y:%f, TrgPort x:%f, y:%f",
        srcComp->rect->rect().top() - srcComp->rect->rect().bottom(), srcComp->rect->rect().right() - srcComp->rect->rect().left(),
        trgComp->rect->rect().top() - trgComp->rect->rect().bottom(), trgComp->rect->rect().right() - trgComp->rect->rect().left());

    

    scene.addItem(gItem);
    rtsConnections[rtsConnection->id] = gItem;
    DDEBUG("RTSDiagramViewImpl::createConnectionGItem End");
}


void RTSDiagramExtViewImpl::updateStatus()
{
    if (currentRTSItem) {
        currentRTSItem->checkStatus();
    }
}

//void RTSDiagramExtViewImpl::onTime()
//{
//    //DDEBUG("RTSDiagramViewImpl::onTime");
//    if (!currentRTSItem) {
//        return;
//    }
//
//    bool doConnectionCheck = true;
//
//    /**
//       This is a temporary code to avoid a crach.
//       The crach may be caused by the accesses to non-thread-safe objects
//       of omniORB or OpenRTM from the main thread and simulation threads.
//    */
//    if (SimulatorItem::findActiveSimulatorItemFor(currentRTSItem)) {
//        doConnectionCheck = false;
//    }
//
//    if (doConnectionCheck) {
//        if (currentRTSItem->checkStatus()) {
//            updateView();
//            updateRestoredView();
//        }
//    }
//
//    checkStatus();
//}


void RTSDiagramExtViewImpl::setCurrentRTSItem(RTSystemExtItem* item)
{
    DDEBUG("RTSDiagramExtViewImpl::setCurrentRTSItem");

    currentRTSItem = item;
    currentRTSItem->onActivated();

    updateStatusConnection.reset(
            currentRTSItem->sigStatusUpdate().connect(
                std::bind(&RTSDiagramExtViewImpl::onUpdateStatus, this, _1)));

    connectionOfRTSystemItemDetachedFromRoot.reset(
        item->sigDetachedFromRoot().connect(
            [&]() { onRTSystemItemDetachedFromRoot(); }));
    rtsLoadedConnection.reset(
            currentRTSItem->sigLoaded().connect(
                std::bind(&RTSDiagramExtViewImpl::onRTSystemLoaded, this, _1)));
    updateStatusConnection.reset(
            currentRTSItem->sigStatusUpdate().connect(
                std::bind(&RTSDiagramExtViewImpl::onUpdateStatus, this, _1)));

    updateView();
}


void RTSDiagramExtViewImpl::updateView()
{
    DDEBUG("RTSDiagramViewImpl::updateView");
    rtsComps.clear();
    rtsConnections.clear();
    rtsPortMap.clear();
    if (currentRTSItem) {
        setBackgroundBrush(QBrush(Qt::white));
        map<string, RTSCompExtPtr>& comps = currentRTSItem->rtsComps();
        for (map<string, RTSCompExtPtr>::iterator itr = comps.begin(); itr != comps.end(); itr++) {
            addRTSComp(itr->second.get());
        }
        RTSystemExtItem::RTSConnectionMap& connections = currentRTSItem->rtsConnections();
        for (RTSystemExtItem::RTSConnectionMap::iterator itr = connections.begin();
                itr != connections.end(); itr++) {
            DDEBUG("RTSDiagramViewImpl::updateView find connection");
            if (rtsConnections.find(itr->second->id) == rtsConnections.end()) {
                RTSConnectionExt* rtsConnection = itr->second.get();
                RTSPortExtGItem* source = rtsPortMap.find(rtsConnection->sourcePort)->second;
                RTSPortExtGItem* target = rtsPortMap.find(rtsConnection->targetPort)->second;
                createConnectionGItem(rtsConnection, source, target);
            }
        }
        setAcceptDrops(true);
    } else {
        setBackgroundBrush(QBrush(Qt::gray));
        setAcceptDrops(false);
    }
}

void RTSDiagramExtViewImpl::updateRestoredView()
{
    DDEBUG("RTSDiagramViewImpl::updateRestoredView");
    //To redraw the connection line
    if (currentRTSItem) {
        for (auto it = rtsComps.begin(); it != rtsComps.end(); it++) {
            QPointF pos = it->second->pos();
            pos.setX(pos.x() + 1);
            it->second->setPos(pos);
        }
    }
}

void RTSDiagramExtViewImpl::activateComponent()
{
    RTSCompExtGItem* target = selectionRTCs.front();
    if (target->rtsComp->activateComponent() == false) {
        QMessageBox::information(this, _("Activate"), _("Activation of target component FAILED."));
    }
}


void RTSDiagramExtViewImpl::deactivateComponent()
{
    RTSCompExtGItem* target = selectionRTCs.front();
    if (target->rtsComp->deactivateComponent() == false) {
        QMessageBox::information(this, _("Deactivate"), _("Deactivation of target component FAILED."));
    }
}


void RTSDiagramExtViewImpl::resetComponent()
{
    RTSCompExtGItem* target = selectionRTCs.front();
    if (target->rtsComp->resetComponent() == false) {
        QMessageBox::information(this, _("Reset"), _("FAILED to reset target component."));
    }
}


void RTSDiagramExtViewImpl::finalizeComponent()
{
    RTSCompExtGItem* target = selectionRTCs.front();
    if (target->rtsComp->finalizeComponent() == false) {
        QMessageBox::information(this, _("Exit"), _("FAILED to exit target component."));
    }
    deleteSelectedRTSItem();
}


void RTSDiagramExtViewImpl::startExecutionContext()
{
    RTSCompExtGItem* target = selectionRTCs.front();
    if (target->rtsComp->startExecutionContext() == false) {
        QMessageBox::information(this, _("Start"), _("FAILED to start ExecutionContext."));
    }
}


void RTSDiagramExtViewImpl::stopExecutionContext()
{
    RTSCompExtGItem* target = selectionRTCs.front();
    if (target->rtsComp->stopExecutionContext() == false) {
        QMessageBox::information(this, _("Start"), _("FAILED to stop ExecutionContext."));
    }
}

void RTSDiagramExtViewImpl::onRTSystemLoaded(bool value)
{
    DDEBUG_V("RTSDiagramViewImpl::onRTSystemLoaded : %d", value);
    updateView();
    if (value) {
        if (currentRTSItem) {
            currentRTSItem->checkStatus();
        }
    }
    updateRestoredView();
}

void RTSDiagramExtViewImpl::onUpdateStatus(bool modified)
{
    DDEBUG("RTSDiagramExtViewImpl::onUpdateStatus");
    if (modified) {
        updateView();
        updateRestoredView();
    }
}
//////////
RTSDiagramExtView::RTSDiagramExtView()
{
    impl = new RTSDiagramExtViewImpl(this);
}

RTSDiagramExtView::~RTSDiagramExtView()
{
    delete impl;
}

RTSDiagramExtView* RTSDiagramExtView::instance()
{
    return ViewManager::findView<RTSDiagramExtView>();
}

void RTSDiagramExtView::onActivated()
{
    impl->setNewRTSItemDetector();
}

void RTSDiagramExtView::onDeactivated()
{
    impl->itemAddedConnection.disconnect();
}

void RTSDiagramExtView::onRTSCompSelectionChange()
{
    impl->onRTSCompSelectionChange();
}

bool RTSDiagramExtView::storeState(Archive& archive)
{
    if (impl->currentRTSItem) {
        archive.writeItemId("currentRTSItem", impl->currentRTSItem);
    }
    return true;
}

bool RTSDiagramExtView::restoreState(const Archive& archive)
{
    RTSystemExtItem* item = archive.findItem<RTSystemExtItem>("currentRTSItem");
    if (item) {
        impl->setCurrentRTSItem(item);
    }
    return true;
}

void RTSDiagramExtView::initializeClass(ExtensionManager* ext)
{
    ext->viewManager().registerClass<RTSDiagramExtView>(
        "RTSDiagramView", N_("RTC Diagram"), ViewManager::SINGLE_OPTIONAL);
}
