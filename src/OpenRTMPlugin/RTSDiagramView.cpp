/*!
 * @author Shizuko Hattori
 * @file
 */
#include "RTSDiagramView.h"
#include "RTSystemItem.h"
#include "RTSNameServerView.h"
#include "RTSPropertiesView.h"
#include "OpenRTMUtil.h"
#include "PortConnectionDialog.h"
#include "LoggerUtil.h"
#include <cnoid/ViewManager>
#include <cnoid/MessageView>
#include <cnoid/MenuManager>
#include <cnoid/ConnectionSet>
#include <cnoid/Timer>
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
#include <fmt/format.h>
#include "gettext.h"

using namespace cnoid;
using namespace std;
using namespace RTC;

#define STATE_CHECK_TIME 1000  //msec
#define OPACITY 0.3

namespace {

class RTSConnectionLineItem : public QGraphicsLineItem
{
public:
    RTSConnectionLineItem(qreal x1, qreal y1, qreal x2, qreal y2)
        : QGraphicsLineItem(x1, y1, x2, y2), x1(x1), x2(x2), y1(y1), y2(y2)
    {
        QPen pen;
        pen.setColor(QColor("black"));
        pen.setWidth(1);
        setPen(pen);
        cx = (x1 + x2) / 2.0;
        cy = (y1 + y2) / 2.0;
    }

    ~RTSConnectionLineItem()
    {
        // cout << "delete LineItem" <<endl;
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

class RTSConnectionMarkerItem : public QGraphicsRectItem
{
public:
    enum markerType { UNMOVABLE, HORIZONTAL, VERTIAL };
    markerType type;
    Signal<void(RTSConnectionMarkerItem*)> sigPositionChanged;
    static const int size = 6;

    RTSConnectionMarkerItem(qreal x, qreal y, markerType type_)
        : QGraphicsRectItem(x - size / 2, y - size / 2, size, size),
        type(type_)
    {
        setBrush(QBrush(QColor("black")));
        if (type != UNMOVABLE) {
            setFlags(QGraphicsItem::ItemIsMovable);
        }
    }

    ~RTSConnectionMarkerItem()
    {
        //cout << "delete MarkerItem" <<endl;
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

struct PortInfo {
    bool isLeft;
    QPointF portPos;
    QPointF compPos;
    double height;
    double width;
};

class RTSConnectionGItem : public QGraphicsItemGroup, public Referenced
{
public:
    enum lineType { THREEPARTS_TYPE, FIVEPARTS_TYPE };
    const qreal xxoffset = 5;

    RTSConnectionGItem(RTSConnection* rtsConnection, PortInfo source, PortInfo target);
    ~RTSConnectionGItem();

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
    RTSConnectionMarkerItem* findMarker(QGraphicsItem* gItem);
    void lineMove(RTSConnectionMarkerItem* marker_);
    bool changePortPos(bool isSource, QPointF s);
    lineType getType(qreal sx, qreal tx);

    RTSConnectionPtr rtsConnection;

private:
    bool _sIsLeft;
    bool _tIsLeft;
    lineType type;
    RTSConnectionLineItem* line[5];
    RTSConnectionMarkerItem* marker[5];
    ConnectionSet signalConnections;
    QGraphicsOpacityEffect* effect;

    double calcLinePos(PortInfo source, PortInfo target);
};

typedef ref_ptr<RTSConnectionGItem> RTSConnectionGItemPtr;

class RTSPortGItem : public QGraphicsItemGroup, public Referenced
{
public:
    enum portType { INPORT, OUTPORT, SERVICEPORT };
    RTSPort* rtsPort;
    QGraphicsPolygonItem* polygon;
    QPointF pos;

    RTSPortGItem(RTSPort* rtsPort);
    void create(const int posX, const int posY, portType type);
    void stateCheck();
    void setCandidate(bool isCand);

};

typedef ref_ptr<RTSPortGItem> RTSPortGItemPtr;

class RTSCompGItem : public QGraphicsItemGroup, public Referenced
{
public:
    RTSCompGItem(RTSComp* rtsComp, RTSDiagramViewImpl* impl, const QPointF& pos, const int interval);
    ~RTSCompGItem();
    QVariant itemChange(GraphicsItemChange change, const QVariant & value);
    void create(const QPointF& pos, const int interval);
    void stateCheck();
    void checkCandidate(RTSPortGItem* sourcePort);
    void clearCandidate();
    int correctTextY(int interval);

    RTSDiagramViewImpl* impl;
    RTSComp* rtsComp;
    map<string, RTSPortGItemPtr> inPorts;
    map<string, RTSPortGItemPtr> outPorts;
    QGraphicsRectItem* rect;
    QGraphicsOpacityEffect* effect;
    Signal<void(const RTSCompGItem*)> sigPositionChanged;
    Connection positionChangeConnection;
};

typedef ref_ptr<RTSCompGItem> RTSCompGItemPtr;

}

namespace cnoid {

class RTSDiagramViewImpl : public QGraphicsView
{
public:
    RTSDiagramView* self;
    RTSystemItemPtr currentRTSItem;
    QGraphicsScene  scene;
    ScopedConnection nsViewSelectionChangedConnection;
    ScopedConnection itemAddedConnection;
    ScopedConnection itemTreeViewSelectionChangedConnection;
    ScopedConnection connectionOfRTSystemItemDetachedFromRoot;
    ScopedConnection timeOutConnection;

    ScopedConnection timerPeriodChangedConnection;
    ScopedConnection timerChangedConnection;
    ScopedConnection rtsLoadedConnection;

    map<string, RTSCompGItemPtr> rtsComps;
    map<string, RTSConnectionGItemPtr> rtsConnections;
    map<RTSPort*, RTSPortGItem*> rtsPortMap;
    list<NamingContextHelper::ObjectInfo> nsViewSelections;
    list<RTSCompGItem*> selectionRTCs;
    list<RTSConnectionGItem*> selectionRTSConnections;
    MenuManager menuManager;
    Timer timer;
    RTSPortGItem* sourcePort;
    QGraphicsLineItem* dragPortLine;
    RTSConnectionMarkerItem* targetMarker;
    int pollingPeriod;

    RTSDiagramViewImpl(RTSDiagramView* self);
    ~RTSDiagramViewImpl();
    void setNewRTSItemDetector();
    void addRTSComp(NamingContextHelper::ObjectInfo& info, const QPointF& pos);
    void addRTSComp(RTSComp* rtsComp);
    void deleteRTSComp(RTSCompGItem* rtsComp);
    void deleteRTSConnection(RTSConnectionGItem* rtsConnection);
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
    RTSPortGItem* findTargetRTSPort(QPointF& pos);
    RTSCompGItem* findTargetRTC(RTSPortGItem* port);
    RTSConnectionMarkerItem* findConnectionMarker(QGraphicsItem* gItem);
    void createConnectionGItem(RTSConnection* rtsConnection, RTSPortGItem* sourcePort, RTSPortGItem* targetPort);
    void onRTSCompSelectionChange();
    void onRTSCompPositionChanged(const RTSCompGItem*);
    void onTime();
    void onActivated(bool on);
    void timerPeriodUpdate(int value);
    void onItemTreeViewSelectionChanged(const ItemList<RTSystemItem>& items);
    void onRTSystemItemDetachedFromRoot();
    void setCurrentRTSItem(RTSystemItem* item);
    void updateView();
    void checkStatus();
    void updateSetting();
    void updateRestoredView();
    void activateComponent();
    void deactivateComponent();
    void resetComponent();
    void finalizeComponent();
    void startExecutionContext();
    void stopExecutionContext();

    void onRTSystemLoaded();

private:
    int interval_;
};

}


RTSConnectionGItem::RTSConnectionGItem
(RTSConnection* rtsConnection, PortInfo source, PortInfo target)
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
        line[0] = new RTSConnectionLineItem(p0e(0), p0e(1), p0s(0), p0s(1));
        addToGroup(line[0]);
        Vector2& p1s = rtsConnection->position[4];
        Vector2& p1e = rtsConnection->position[5];
        line[1] = new RTSConnectionLineItem(p1s(0), p1s(1), p1e(0), p1e(1));
        addToGroup(line[1]);
        for (int i = 2; i < 5; ++i) {
            Vector2& p0 = rtsConnection->position[i - 1];
            Vector2& p1 = rtsConnection->position[i];
            line[i] = new RTSConnectionLineItem(p0(0), p0(1), p1(0), p1(1));
            addToGroup(line[i]);
        }
        if (line[3]->x1 == line[3]->x2) {
            type = THREEPARTS_TYPE;
            marker[0] = new RTSConnectionMarkerItem(line[0]->x2, line[0]->y2, RTSConnectionMarkerItem::UNMOVABLE);
            marker[1] = new RTSConnectionMarkerItem(line[1]->x2, line[1]->y2, RTSConnectionMarkerItem::UNMOVABLE);
            marker[2] = new RTSConnectionMarkerItem(line[3]->cx, line[3]->cy, RTSConnectionMarkerItem::HORIZONTAL);
            signalConnections.add(
                marker[2]->sigPositionChanged.connect(
                    [&](RTSConnectionMarkerItem* marker){ lineMove(marker); }));
            marker[3] = marker[4] = 0;
        } else {
            type = FIVEPARTS_TYPE;
            marker[0] = new RTSConnectionMarkerItem(line[0]->x2, line[0]->y2, RTSConnectionMarkerItem::UNMOVABLE);
            marker[1] = new RTSConnectionMarkerItem(line[1]->x2, line[1]->y2, RTSConnectionMarkerItem::UNMOVABLE);
            marker[2] = new RTSConnectionMarkerItem(line[2]->cx, line[2]->cy, RTSConnectionMarkerItem::HORIZONTAL);
            marker[3] = new RTSConnectionMarkerItem(line[3]->cx, line[3]->cy, RTSConnectionMarkerItem::VERTIAL);
            marker[4] = new RTSConnectionMarkerItem(line[4]->cx, line[4]->cy, RTSConnectionMarkerItem::HORIZONTAL);
            signalConnections.add(
                marker[2]->sigPositionChanged.connect(
                    [&](RTSConnectionMarkerItem* marker){ lineMove(marker); }));
            signalConnections.add(
                marker[3]->sigPositionChanged.connect(
                    [&](RTSConnectionMarkerItem* marker){ lineMove(marker); }));
            signalConnections.add(
                marker[4]->sigPositionChanged.connect(
                    [&](RTSConnectionMarkerItem* marker){ lineMove(marker); }));
        }
    } else {
        qreal sx, tx;
        sx = firstLineX(source.portPos.x());
        tx = endLineX(target.portPos.x());
        type = getType(sx, tx);

        line[0] = new RTSConnectionLineItem(sx, source.portPos.y(), source.portPos.x(), source.portPos.y());
        addToGroup(line[0]);

        line[1] = new RTSConnectionLineItem(tx, target.portPos.y(), target.portPos.x(), target.portPos.y());
        addToGroup(line[1]);

        qreal centerX = (sx + tx) / 2.0;
        qreal centerY = (source.portPos.y() + target.portPos.y()) / 2.0;
        if (type == THREEPARTS_TYPE) {
            line[2] = new RTSConnectionLineItem(sx, source.portPos.y(), centerX, source.portPos.y());
            addToGroup(line[2]);
            line[3] = new RTSConnectionLineItem(centerX, source.portPos.y(), centerX, target.portPos.y());
            addToGroup(line[3]);
            line[4] = new RTSConnectionLineItem(centerX, target.portPos.y(), tx, target.portPos.y());
            addToGroup(line[4]);
            marker[0] = new RTSConnectionMarkerItem(line[0]->x2, line[0]->y2, RTSConnectionMarkerItem::UNMOVABLE);
            marker[1] = new RTSConnectionMarkerItem(line[1]->x2, line[1]->y2, RTSConnectionMarkerItem::UNMOVABLE);
            marker[2] = new RTSConnectionMarkerItem(line[3]->cx, line[3]->cy, RTSConnectionMarkerItem::HORIZONTAL);
            signalConnections.add(
                marker[2]->sigPositionChanged.connect(
                    [&](RTSConnectionMarkerItem* marker){ lineMove(marker); }));
            marker[3] = marker[4] = 0;

        } else {
            centerY = calcLinePos(source, target);

            line[2] = new RTSConnectionLineItem(sx, source.portPos.y(), sx, centerY);
            addToGroup(line[2]);
            line[3] = new RTSConnectionLineItem(sx, centerY, tx, centerY);
            addToGroup(line[3]);
            line[4] = new RTSConnectionLineItem(tx, centerY, tx, target.portPos.y());
            addToGroup(line[4]);
            marker[0] = new RTSConnectionMarkerItem(line[0]->x2, line[0]->y2, RTSConnectionMarkerItem::UNMOVABLE);
            marker[1] = new RTSConnectionMarkerItem(line[1]->x2, line[1]->y2, RTSConnectionMarkerItem::UNMOVABLE);
            marker[2] = new RTSConnectionMarkerItem(line[2]->cx, line[2]->cy, RTSConnectionMarkerItem::HORIZONTAL);
            marker[3] = new RTSConnectionMarkerItem(line[3]->cx, line[3]->cy, RTSConnectionMarkerItem::VERTIAL);
            marker[4] = new RTSConnectionMarkerItem(line[4]->cx, line[4]->cy, RTSConnectionMarkerItem::HORIZONTAL);
            signalConnections.add(
                marker[2]->sigPositionChanged.connect(
                    [&](RTSConnectionMarkerItem* marker){ lineMove(marker); }));
            signalConnections.add(
                marker[3]->sigPositionChanged.connect(
                    [&](RTSConnectionMarkerItem* marker){ lineMove(marker); }));
            signalConnections.add(
                marker[4]->sigPositionChanged.connect(
                    [&](RTSConnectionMarkerItem* marker){ lineMove(marker); }));
        }

        Vector2 pos[6];
        getLinePosition(pos);
        rtsConnection->setPosition(pos);
    }

    setFlags(QGraphicsItem::ItemIsSelectable);
}


RTSConnectionGItem::~RTSConnectionGItem()
{
    signalConnections.disconnect();

    for (int i = 0; i < 5; ++i) {
        if (marker[i]) {
            delete marker[i];
        }
    }
}


qreal RTSConnectionGItem::firstLineX(qreal x)
{
    if (_sIsLeft) {
        return x - xxoffset;
    } else {
        return x + xxoffset;
    }
}


qreal RTSConnectionGItem::endLineX(qreal x)
{
    if (_tIsLeft) {
        return x - xxoffset;
    } else {
        return x + xxoffset;
    }
}


int RTSConnectionGItem::markerIndex(QGraphicsItem* gItem)
{
    for (int i = 2; i < 5; ++i) {
        if (marker[i] == gItem) {
            return i;
        }
    }
    return -1;
}


RTSConnectionMarkerItem* RTSConnectionGItem::findMarker(QGraphicsItem* gItem)
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


void RTSConnectionGItem::lineMove(RTSConnectionMarkerItem* marker_)
{
    int i = markerIndex(marker_);
    if (i < 0) {
        return;
    }
    QPointF c = marker_->rect().center();

    if (type == RTSConnectionGItem::THREEPARTS_TYPE) {
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


RTSConnectionGItem::lineType RTSConnectionGItem::getType(qreal sx, qreal tx)
{
    if ((_sIsLeft && tx < sx) || (_tIsLeft &&  sx < tx) || (!_sIsLeft && sx < tx) || (!_tIsLeft && tx < sx)) {
        return THREEPARTS_TYPE;
    } else {
        return FIVEPARTS_TYPE;
    }
}


bool RTSConnectionGItem::changePortPos(bool isSource, QPointF s)
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


void RTSConnectionGItem::showMarker(bool on)
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

double RTSConnectionGItem::calcLinePos(PortInfo source, PortInfo target) {
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

RTSPortGItem::RTSPortGItem(RTSPort* rtsPort)
    : rtsPort(rtsPort)
{

}

void RTSPortGItem::create(const int posX, const int posY, portType type)
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
            stateCheck();
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
            stateCheck();
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
            stateCheck();
            addToGroup(polygon);
            break;
    }
}


void RTSPortGItem::stateCheck()
{
    polygon->setBrush(QBrush(QColor(rtsPort->isConnected() ? "lightgreen" : (rtsPort->isServicePort ? "lightblue" : "blue"))));
}


void RTSPortGItem::setCandidate(bool isCand)
{
    if (isCand) {
        polygon->setPen(QPen(QColor("magenta"), 3));
    } else {
        polygon->setPen(QPen(QColor("red"), 1));
    }
}


QVariant RTSCompGItem::itemChange(GraphicsItemChange change, const QVariant & value)
{
    if (change == ItemPositionChange) {
        rtsComp->setPos(rtsComp->pos() + value.value<QPointF>() - pos());
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


int RTSCompGItem::correctTextY(int interval)
{
    int numIn = rtsComp->inPorts.size();
    int numOut = rtsComp->outPorts.size();
    int numMax = (numIn < numOut ? numOut : numIn);
    if (!numMax) {
        numMax = 1;
    }
    return (interval * numMax) - 7 * (numMax - 1);
}


void RTSCompGItem::stateCheck()
{
    RTC_STATUS status = rtsComp->getRTCState();
    if (status == RTC_STATUS::RTC_INACTIVE) {
        rect->setBrush(QBrush(QColor("blue")));
    } else if (status == RTC_STATUS::RTC_ACTIVE) {
        rect->setBrush(QBrush(QColor("lightgreen")));
    } else if (status == RTC_STATUS::RTC_ERROR) {
        rect->setBrush(QBrush(QColor("red")));
    }

    for (map<string, RTSPortGItemPtr>::iterator it = inPorts.begin(); it != inPorts.end(); it++) {
        it->second->stateCheck();
    }

    for (map<string, RTSPortGItemPtr>::iterator it = outPorts.begin(); it != outPorts.end(); it++) {
        it->second->stateCheck();
    }
}


void RTSCompGItem::checkCandidate(RTSPortGItem* sourcePort)
{
    if (!sourcePort->rtsPort) {
        return;
    }
    if (sourcePort->rtsPort->isInPort) {
        for (map<string, RTSPortGItemPtr>::iterator it = outPorts.begin(); it != outPorts.end(); it++) {
            bool isCand = sourcePort->rtsPort->checkConnectablePort(it->second->rtsPort);
            it->second->setCandidate(isCand);
        }
    } else {
        for (map<string, RTSPortGItemPtr>::iterator it = inPorts.begin(); it != inPorts.end(); it++) {
            bool isCand = sourcePort->rtsPort->checkConnectablePort(it->second->rtsPort);
            it->second->setCandidate(isCand);
        }
    }
}


void RTSCompGItem::clearCandidate()
{
    for (map<string, RTSPortGItemPtr>::iterator it = inPorts.begin(); it != inPorts.end(); it++) {
        it->second->setCandidate(false);
    }

    for (map<string, RTSPortGItemPtr>::iterator it = outPorts.begin(); it != outPorts.end(); it++) {
        it->second->setCandidate(false);
    }
}


RTSDiagramViewImpl::RTSDiagramViewImpl(RTSDiagramView* self)
    :self(self), sourcePort(0), pollingPeriod(1000)
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
                [&](const list<NamingContextHelper::ObjectInfo>& items){
                    onnsViewItemSelectionChanged(items); }));
    }

    timer.setSingleShot(false);
    timeOutConnection.reset(
        timer.sigTimeout().connect([&](){ onTime(); }));
    self->sigActivated().connect([&](){ onActivated(true); });
    self->sigDeactivated().connect([&](){ onActivated(false); });

    itemTreeViewSelectionChangedConnection.reset(
        ItemTreeView::mainInstance()->sigSelectionChanged().connect(
            [&](const ItemList<>& items){ onItemTreeViewSelectionChanged(items); }));

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
    DDEBUG_V("Font size:%d", pixelsHigh);
    interval_ = pixelsHigh * 2.0;
}


RTSDiagramViewImpl::~RTSDiagramViewImpl()
{
    rtsComps.clear();
    rtsConnections.clear();
    disconnect(&scene, SIGNAL(selectionChanged()), self, SLOT(onRTSCompSelectionChange()));
}


void RTSDiagramViewImpl::dragEnterEvent(QDragEnterEvent *event)
{
    const RTSNameTreeWidget* nameServerItem =
        qobject_cast<const RTSNameTreeWidget*>(event->source()); //event->mimeData());
    if (nameServerItem) {
        event->acceptProposedAction();
    }
}


void RTSDiagramViewImpl::dragMoveEvent(QDragMoveEvent *event)
{
    const RTSNameTreeWidget* nameServerItem =
        qobject_cast<const RTSNameTreeWidget*>(event->source());//event->mimeData());
    if (nameServerItem) {
        event->acceptProposedAction();
    }
}


void RTSDiagramViewImpl::dragLeaveEvent(QDragLeaveEvent *event)
{
    MessageView::instance()->putln(_("Drag and drop has been canceled. Please be operation again."));
}


void RTSDiagramViewImpl::dropEvent(QDropEvent *event)
{
    DDEBUG("RTSystemItem::dropEvent");

    for (list<NamingContextHelper::ObjectInfo>::iterator it = nsViewSelections.begin(); it != nsViewSelections.end(); it++) {
        NamingContextHelper::ObjectInfo& info = *it;
        if (!info.isAlive_) {
            MessageView::instance()->putln(fmt::format(_("{} is not alive"), info.id_));
        } else {
            addRTSComp(info, mapToScene(event->pos()));
            DDEBUG_V("%s", info.getFullPath().c_str());
        }
    }
}


void RTSDiagramViewImpl::keyPressEvent(QKeyEvent *event)
{
    if (event->key() == Qt::Key_Delete) {
        deleteSelectedRTSItem();
    }
}


void RTSDiagramViewImpl::mousePressEvent(QMouseEvent* event)
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
                    ->sigTriggered().connect([&](){ activateComponent(); });
                menuManager.addItem("Deactivate")
                    ->sigTriggered().connect([&](){ deactivateComponent(); });
                menuManager.addItem("Reset")
                    ->sigTriggered().connect([&](){ resetComponent(); });
                if (!isManagedRTC(selectionRTCs.front()->rtsComp->rtc_)) {
                    menuManager.addItem("Exit")
                        ->sigTriggered().connect([&](){ finalizeComponent(); });
                }
                menuManager.addSeparator();
                menuManager.addItem("Start")
                    ->sigTriggered().connect([&](){ startExecutionContext(); });
                menuManager.addItem("Stop")
                    ->sigTriggered().connect([&](){ stopExecutionContext(); });
                menuManager.addSeparator();
            }
            menuManager.addItem("Remove")
                ->sigTriggered().connect([&](){ deleteSelectedRTSItem(); });

            menuManager.popupMenu()->popup(event->globalPos());

        } else if (!selectionRTSConnections.empty()) {
            menuManager.setNewPopupMenu(this);
            menuManager.addItem("Delete")
                ->sigTriggered().connect([&](){ deleteSelectedRTSItem(); });

            menuManager.popupMenu()->popup(event->globalPos());

        } else {
            if (currentRTSItem->stateCheck() == 1) {
                menuManager.setNewPopupMenu(this);
                menuManager.addItem(_("Update"))
                    ->sigTriggered().connect([&](){ onTime(); });

                menuManager.popupMenu()->popup(event->globalPos());
            }
        }
    }
}


void RTSDiagramViewImpl::mouseMoveEvent(QMouseEvent* event)
{
    QPointF pos = mapToScene(event->pos());

    if (sourcePort) {
        QPointF sp = sourcePort->pos;
        dragPortLine->setLine(sp.x(), sp.y(), pos.x(), pos.y());
        RTSPortGItem* port = findTargetRTSPort(pos);
        if (port && !sourcePort->rtsPort->checkConnectablePort(port->rtsPort)) {
            setCursor(Qt::ForbiddenCursor);
        } else {
            setCursor(Qt::PointingHandCursor);
        }

        for (map<string, RTSCompGItemPtr>::iterator it = rtsComps.begin(); it != rtsComps.end(); it++) {
            it->second->checkCandidate(sourcePort);
        }

    } else if (targetMarker) {
        targetMarker->move(pos.x(), pos.y());

    } else {
        RTSPortGItem* port = findTargetRTSPort(pos);
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


void RTSDiagramViewImpl::mouseReleaseEvent(QMouseEvent *event)
{
    QPointF pos = mapToScene(event->pos());

    if (sourcePort) {
        for (map<string, RTSCompGItemPtr>::iterator it = rtsComps.begin(); it != rtsComps.end(); it++) {
            it->second->clearCandidate();
        }

        dragPortLine->setVisible(false);
        RTSPortGItem* targetPort = findTargetRTSPort(pos);

        if (targetPort && sourcePort->rtsPort->checkConnectablePort(targetPort->rtsPort)) {
            if (sourcePort->rtsPort->isConnectedWith(targetPort->rtsPort)) {
                MessageView::instance()->putln(_("These are already connected."));

            } else {
                bool isAccepted = false;
                string name;
                vector<NamedValuePtr> propList;
                if (sourcePort->rtsPort->isServicePort && targetPort->rtsPort->isServicePort) {
                    DDEBUG("ServicePort Connect");
                    ServicePortConnectionDialog dialog;
                    dialog.setDisp(sourcePort->rtsPort, targetPort->rtsPort);
                    dialog.exec();
                    isAccepted = dialog.isAccepted;
                    name = dialog.nameLineEdit->text().toStdString();
                    propList = dialog.propList;

                } else {
                    DDEBUG("DataPort Connect");
                    DataPortConnectionDialog dialog;
                    dialog.setDisp(sourcePort->rtsPort, targetPort->rtsPort);
                    dialog.exec();
                    isAccepted = dialog.isAccepted;
                    name = dialog.nameLineEdit->text().toStdString();
                    propList = dialog.propList;
                }

                if (isAccepted) {
                    DDEBUG("RTSDiagramViewImpl::mouseReleaseEvent Accepted");
                    string id = "";
                    timeOutConnection.block();
                    RTSConnection* rtsConnection =
                        currentRTSItem->addRTSConnection(
                            id, name, sourcePort->rtsPort, targetPort->rtsPort, propList, 0);
                    if (rtsConnection) {
                        createConnectionGItem(rtsConnection, sourcePort, targetPort);
                    }
                    timeOutConnection.unblock();
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


void RTSDiagramViewImpl::wheelEvent(QWheelEvent* event)
{
    double dSteps = (double)event->delta() / 120.0;
    double scaleVal = 1.0;
    scaleVal -= (dSteps / 20.0);
    scale(scaleVal, scaleVal);
    QGraphicsView::wheelEvent(event);
}


void RTSDiagramViewImpl::onnsViewItemSelectionChanged(const list<NamingContextHelper::ObjectInfo>& items)
{
    nsViewSelections = items;
}

void RTSDiagramViewImpl::onRTSCompSelectionChange()
{
    RTSCompGItem* singleSelectedRTC = 0;
    if (selectionRTCs.size() == 1) {
        singleSelectedRTC = selectionRTCs.front();
    }

    RTSConnectionGItem* singleSelectedConnection = 0;
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
        RTSCompGItem* rtsComp = dynamic_cast<RTSCompGItem*>(*it);
        if (rtsComp) {
            selectionRTCs.push_back(rtsComp);
        } else {
            RTSConnectionGItem* rtsConnection = dynamic_cast<RTSConnectionGItem*>(*it);
            if (rtsConnection) {
                selectionRTSConnections.push_back(rtsConnection);
                rtsConnection->showMarker(true);
            }
        }
    }

    if (selectionRTCs.size() == 1 && singleSelectedRTC != selectionRTCs.front()) {
        RTSNameServerView* nsView = RTSNameServerView::instance();
        if (nsView) {
            RTSComp* selected = selectionRTCs.front()->rtsComp;
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
            RTSConnection* rtsConnection = selectionRTSConnections.front()->rtsConnection;
            propView->showConnectionProperties(rtsConnection->sourcePort->port, rtsConnection->id);
        }
    }
}


void RTSDiagramViewImpl::onRTSCompPositionChanged(const RTSCompGItem* rtsCompGItem)
{
    list<RTSConnection*> rtsConnectionList;
    currentRTSItem->RTSCompToConnectionList(rtsCompGItem->rtsComp, rtsConnectionList, 1);

    for (auto it = rtsConnectionList.begin(); it != rtsConnectionList.end(); it++) {
        RTSConnection* rtsConnection = *it;
        RTSConnectionGItem* gItem = rtsConnections.find(rtsConnection->id)->second.get();
        RTSPortGItem* sourcePort = rtsPortMap.find(rtsConnection->sourcePort)->second;
        if (!gItem->changePortPos(true, sourcePort->pos)) {
            rtsConnections.erase(rtsConnection->id);
            RTSPortGItem* targetPort = rtsPortMap.find(rtsConnection->targetPort)->second;
            rtsConnection->setPos = false;
            createConnectionGItem(rtsConnection, sourcePort, targetPort);
        }
    }

    rtsConnectionList.clear();
    currentRTSItem->RTSCompToConnectionList(rtsCompGItem->rtsComp, rtsConnectionList, 2);
    for (auto it = rtsConnectionList.begin(); it != rtsConnectionList.end(); it++) {
        RTSConnection* rtsConnection = *it;
        RTSConnectionGItem* gItem = rtsConnections.find(rtsConnection->id)->second.get();
        RTSPortGItem* targetPort = rtsPortMap.find(rtsConnection->targetPort)->second;
        if (!gItem->changePortPos(false, targetPort->pos)) {
            rtsConnections.erase(rtsConnection->id);
            RTSPortGItem* sourcePort = rtsPortMap.find(rtsConnection->sourcePort)->second;
            rtsConnection->setPos = false;
            createConnectionGItem(rtsConnection, sourcePort, targetPort);
        }
    }
}


void RTSDiagramViewImpl::onActivated(bool on)
{
    DDEBUG_V("RTSDiagramViewImpl::onActivated : %d", on);
    if (on) {
        DDEBUG_V("Timer Start");
        timer.start();
    } else {
        DDEBUG_V("Timer Stop");
        timer.stop();
    }
}

void RTSDiagramViewImpl::timerPeriodUpdate(int value)
{
    DDEBUG_V("RTSDiagramViewImpl::timerPeriodUpdate : %d", value);
    if (pollingPeriod != value) {
        pollingPeriod = value;

        bool isStarted = timer.isActive();
        if (isStarted) {
            DDEBUG_V("Timer Stop");
            timer.stop();
        }
        timer.setInterval(value);
        if (isStarted) {
            DDEBUG_V("Timer Start");
            timer.start();
        }
    }
}

void RTSDiagramViewImpl::onItemTreeViewSelectionChanged(const ItemList<RTSystemItem>& items)
{
    RTSystemItem* firstItem = items.toSingle();

    if (firstItem && firstItem != currentRTSItem) {
        setCurrentRTSItem(firstItem);
    }
}


void RTSDiagramViewImpl::onRTSystemItemDetachedFromRoot()
{
    currentRTSItem = nullptr;
    connectionOfRTSystemItemDetachedFromRoot.disconnect();
    updateView();
    setNewRTSItemDetector();
}


RTSPortGItem* RTSDiagramViewImpl::findTargetRTSPort(QPointF& pos)
{
    for (map<string, RTSCompGItemPtr>::iterator it = rtsComps.begin(); it != rtsComps.end(); it++) {
        map<string, RTSPortGItemPtr>& inPorts = it->second->inPorts;
        for (map<string, RTSPortGItemPtr>::iterator itr = inPorts.begin(); itr != inPorts.end(); itr++) {
            if (itr->second->sceneBoundingRect().contains(pos)) {
                return itr->second.get();
            }
        }
        map<string, RTSPortGItemPtr>& outPorts = it->second->outPorts;
        for (map<string, RTSPortGItemPtr>::iterator itr = outPorts.begin(); itr != outPorts.end(); itr++) {
            if (itr->second->sceneBoundingRect().contains(pos)) {
                return itr->second.get();
            }
        }
    }
    return nullptr;
}

RTSCompGItem* RTSDiagramViewImpl::findTargetRTC(RTSPortGItem* port) 
{
    for (map<string, RTSCompGItemPtr>::iterator it = rtsComps.begin(); it != rtsComps.end(); it++) {
        map<string, RTSPortGItemPtr>& inPorts = it->second->inPorts;
        for (map<string, RTSPortGItemPtr>::iterator itr = inPorts.begin(); itr != inPorts.end(); itr++) {
            if (itr->second == port) {
                return it->second.get();
            }
        }
        map<string, RTSPortGItemPtr>& outPorts = it->second->outPorts;
        for (map<string, RTSPortGItemPtr>::iterator itr = outPorts.begin(); itr != outPorts.end(); itr++) {
            if (itr->second == port) {
                return it->second.get();
            }
        }
    }
    return nullptr;
}

RTSConnectionMarkerItem* RTSDiagramViewImpl::findConnectionMarker(QGraphicsItem* gItem)
{
    for (auto it = selectionRTSConnections.begin(); it != selectionRTSConnections.end(); it++) {
        RTSConnectionMarkerItem* marker = (*it)->findMarker(gItem);
        if (marker) {
            return marker;
        }
    }
    return nullptr;
}


void RTSDiagramViewImpl::setNewRTSItemDetector()
{
    itemAddedConnection.disconnect();

    if (!currentRTSItem) {
        ItemList<RTSystemItem> rtsItems;
        if (rtsItems.extractChildItems(RootItem::instance())) {
            setCurrentRTSItem(rtsItems[0]);
        } else {
            itemAddedConnection.reset(
                RootItem::instance()->sigItemAdded().connect(
                    [&](Item* item) {
                if (!currentRTSItem) {
                    auto rtsItem = dynamic_cast<RTSystemItem*>(item);
                    if (rtsItem) {
                        setCurrentRTSItem(rtsItem);
                        itemAddedConnection.disconnect();
                    }
                }
            }));
        }
    }
}


void RTSDiagramViewImpl::addRTSComp(NamingContextHelper::ObjectInfo& info, const QPointF& pos)
{
    timeOutConnection.block();
    RTSComp* rtsComp = currentRTSItem->addRTSComp(info, pos);
    if (rtsComp) {
        RTSCompGItemPtr rtsCompGItem = new RTSCompGItem(rtsComp, this, pos, interval_);
        rtsComps[info.getFullPath()] = rtsCompGItem;
        scene.addItem(rtsCompGItem);

        RTSystemItem::RTSConnectionMap& connections = currentRTSItem->rtsConnections();
        for (RTSystemItem::RTSConnectionMap::iterator itr = connections.begin(); itr != connections.end(); itr++) {
            if (rtsConnections.find(itr->second->id) == rtsConnections.end()) {
                RTSConnection* rtsConnection = itr->second.get();
                RTSPortGItem* source = rtsPortMap.find(rtsConnection->sourcePort)->second;
                RTSPortGItem* target = rtsPortMap.find(rtsConnection->targetPort)->second;
                createConnectionGItem(rtsConnection, source, target);
            }
        }
    }
    updateView();
    timeOutConnection.unblock();
}


void RTSDiagramViewImpl::addRTSComp(RTSComp* rtsComp)
{
    timeOutConnection.block();
    RTSCompGItemPtr rtsCompGItem = new RTSCompGItem(rtsComp, this, rtsComp->pos(), interval_);
    rtsComps[rtsComp->fullPath] = rtsCompGItem;
    scene.addItem(rtsCompGItem);
    timeOutConnection.unblock();
}


void RTSDiagramViewImpl::deleteRTSComp(RTSCompGItem* rtsCompGItem)
{
    timeOutConnection.block();

    list<RTSConnection*> rtsConnectionList;
    currentRTSItem->RTSCompToConnectionList(rtsCompGItem->rtsComp, rtsConnectionList);

    for (auto itr = rtsConnectionList.begin(); itr != rtsConnectionList.end(); itr++) {
        map<string, RTSConnectionGItemPtr>::iterator it = rtsConnections.find((*itr)->id);
        if (it != rtsConnections.end()) {
            deleteRTSConnection(it->second);
        }
    }

    const string name = rtsCompGItem->rtsComp->fullPath;
    currentRTSItem->deleteRTSComp(name);
    rtsComps.erase(name);

    timeOutConnection.unblock();
}


void RTSDiagramViewImpl::deleteRTSConnection(RTSConnectionGItem* rtsConnectionGItem)
{
    timeOutConnection.block();

    currentRTSItem->disconnectAndRemoveConnection(rtsConnectionGItem->rtsConnection);

    rtsConnections.erase(rtsConnectionGItem->rtsConnection->id);
    timeOutConnection.unblock();
}


void RTSDiagramViewImpl::deleteSelectedRTSItem()
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


void RTSDiagramViewImpl::createConnectionGItem
(RTSConnection* rtsConnection, RTSPortGItem* sourcePort, RTSPortGItem* targetPort)
{
    DDEBUG("RTSDiagramViewImpl::createConnectionGItem");

    PortInfo sourceInfo;
    sourceInfo.isLeft = rtsConnection->sourcePort->isInPort;
    sourceInfo.portPos = sourcePort->pos;
    RTSCompGItem* srcComp = findTargetRTC(sourcePort);
    sourceInfo.compPos = srcComp->rect->scenePos();
    sourceInfo.height = srcComp->rect->rect().bottom() - srcComp->rect->rect().top();
    sourceInfo.width = srcComp->rect->rect().right() - srcComp->rect->rect().left();

    PortInfo targetInfo;
    targetInfo.isLeft = rtsConnection->targetPort->isInPort;
    targetInfo.portPos = targetPort->pos;
    RTSCompGItem* trgComp = findTargetRTC(targetPort);
    targetInfo.compPos = trgComp->rect->scenePos();
    targetInfo.height = trgComp->rect->rect().bottom() - trgComp->rect->rect().top();
    targetInfo.width = trgComp->rect->rect().right() - trgComp->rect->rect().left();

    RTSConnectionGItemPtr gItem =
        new RTSConnectionGItem(
            rtsConnection,
            sourceInfo,
            targetInfo);


    scene.addItem(gItem);
    rtsConnections[rtsConnection->id] = gItem;
    DDEBUG("RTSDiagramViewImpl::createConnectionGItem End");
}


void RTSDiagramViewImpl::onTime()
{
    //DDEBUG("RTSDiagramViewImpl::onTime");
    if (!currentRTSItem) {
        return;
    }

    bool doConnectionCheck = true;

    /**
       This is a temporary code to avoid a crach.
       The crach may be caused by the accesses to non-thread-safe objects
       of omniORB or OpenRTM from the main thread and simulation threads.
    */
    if (SimulatorItem::findActiveSimulatorItemFor(currentRTSItem)) {
        doConnectionCheck = false;
    }

    if (doConnectionCheck) {
        if (currentRTSItem->checkStatus()) {
            updateView();
            updateRestoredView();
        }
    }

    checkStatus();
}


void RTSDiagramViewImpl::setCurrentRTSItem(RTSystemItem* item)
{
    DDEBUG("RTSDiagramViewImpl::setCurrentRTSItem");
    timer.setInterval(item->pollingCycle());

    currentRTSItem = item;
    connectionOfRTSystemItemDetachedFromRoot.reset(
        item->sigDetachedFromRoot().connect(
            [&]() { onRTSystemItemDetachedFromRoot(); }));
    timerPeriodChangedConnection.reset(
            currentRTSItem->sigTimerPeriodChanged().connect(
                [&](int value){ timerPeriodUpdate(value); }));
    timerChangedConnection.reset(
            currentRTSItem->sigTimerChanged().connect(
                [&](bool on){ onActivated(on); }));
    rtsLoadedConnection.reset(
            currentRTSItem->sigUpdated().connect(
                [&](){ onRTSystemLoaded(); }));

    updateView();
}


void RTSDiagramViewImpl::checkStatus()
{
    for (auto it = rtsComps.begin(); it != rtsComps.end(); it++) {
        it->second->stateCheck();
    }
}

void RTSDiagramViewImpl::updateView()
{
    DDEBUG("RTSDiagramViewImpl::updateView");
    timeOutConnection.block();
    rtsComps.clear();
    rtsConnections.clear();
    rtsPortMap.clear();
    if (currentRTSItem) {
        setBackgroundBrush(QBrush(Qt::white));
        map<string, RTSCompPtr>& comps = currentRTSItem->rtsComps();
        for (map<string, RTSCompPtr>::iterator itr = comps.begin(); itr != comps.end(); itr++) {
            addRTSComp(itr->second.get());
        }
        RTSystemItem::RTSConnectionMap& connections = currentRTSItem->rtsConnections();
        DDEBUG_V("con size : %d", currentRTSItem->rtsConnections().size());
        for (RTSystemItem::RTSConnectionMap::iterator itr = connections.begin();
                itr != connections.end(); itr++) {
            DDEBUG("RTSDiagramViewImpl::updateView find connection");
            if (rtsConnections.find(itr->second->id) == rtsConnections.end()) {
                RTSConnection* rtsConnection = itr->second.get();
                RTSPortGItem* source = rtsPortMap.find(rtsConnection->sourcePort)->second;
                RTSPortGItem* target = rtsPortMap.find(rtsConnection->targetPort)->second;
                createConnectionGItem(rtsConnection, source, target);
            }
        }
        setAcceptDrops(true);
    } else {
        setBackgroundBrush(QBrush(Qt::gray));
        setAcceptDrops(false);
    }
    timeOutConnection.unblock();
    DDEBUG("RTSDiagramViewImpl::updateView End");
}

void RTSDiagramViewImpl::updateRestoredView()
{
    DDEBUG("RTSDiagramViewImpl::updateRestoredView");
    //To redraw the connection line
    if (currentRTSItem) {
        bool isUpdated = currentRTSItem->isConsistentWithFile();
        for (auto it = rtsComps.begin(); it != rtsComps.end(); it++) {
            QPointF pos = it->second->pos();
            pos.setX(pos.x() + 1);
            it->second->setPos(pos);
            pos.setX(pos.x() - 1);
            it->second->setPos(pos);
        }
        currentRTSItem->setConsistentWithFile(isUpdated);
    }
}

void RTSDiagramViewImpl::activateComponent()
{
    RTSCompGItem* target = selectionRTCs.front();
    if (!target->rtsComp->activateComponent()) {
        QMessageBox::information(this, _("Activate"), _("Activation of target component FAILED."));
    }
}


void RTSDiagramViewImpl::deactivateComponent()
{
    RTSCompGItem* target = selectionRTCs.front();
    if (!target->rtsComp->deactivateComponent()) {
        QMessageBox::information(this, _("Deactivate"), _("Deactivation of target component FAILED."));
    }
}


void RTSDiagramViewImpl::resetComponent()
{
    RTSCompGItem* target = selectionRTCs.front();
    if (!target->rtsComp->resetComponent()) {
        QMessageBox::information(this, _("Reset"), _("FAILED to reset target component."));
    }
}


void RTSDiagramViewImpl::finalizeComponent()
{
    RTSCompGItem* target = selectionRTCs.front();
    if (target->rtsComp->finalizeComponent() == false) {
        QMessageBox::information(this, _("Exit"), _("FAILED to exit target component."));
    }
    deleteSelectedRTSItem();
}


void RTSDiagramViewImpl::startExecutionContext()
{
    RTSCompGItem* target = selectionRTCs.front();
    if (!target->rtsComp->startExecutionContext()) {
        QMessageBox::information(this, _("Start"), _("FAILED to start ExecutionContext."));
    }
}


void RTSDiagramViewImpl::stopExecutionContext()
{
    RTSCompGItem* target = selectionRTCs.front();
    if (!target->rtsComp->stopExecutionContext()) {
        QMessageBox::information(this, _("Start"), _("FAILED to stop ExecutionContext."));
    }
}


RTSCompGItem::RTSCompGItem(RTSComp* rtsComp, RTSDiagramViewImpl* impl, const QPointF& pos, const int interval)
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

    create(pos, interval);
    positionChangeConnection = sigPositionChanged.connect(
        [impl](const RTSCompGItem* item){
            impl->onRTSCompPositionChanged(item); });
}


RTSCompGItem::~RTSCompGItem()
{
    positionChangeConnection.disconnect();
    for (auto it = inPorts.begin(); it != inPorts.end(); it++) {
        impl->rtsPortMap.erase(it->second->rtsPort);
    }
    for (auto it = outPorts.begin(); it != outPorts.end(); it++) {
        impl->rtsPortMap.erase(it->second->rtsPort);
    }
}


void RTSCompGItem::create(const QPointF& pos, const int interval)
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
    for (vector<RTSPortPtr>::iterator it = rtsComp->inPorts.begin(); it != rtsComp->inPorts.end(); it++, i++) {
        RTSPort* inPort = *it;
        string portName = string(inPort->name);
        RTCCommonUtil::splitPortName(portName);
        text = new QGraphicsTextItem(QString(portName.c_str()));
        int x = rectX + pos.x() - text->boundingRect().width();
        int y = interval * i - 7 * i - 3 + pos.y();

        text->setPos(x, y);
        addToGroup(text);

        RTSPortGItemPtr inPortGItem = new RTSPortGItem(inPort);
        inPortGItem->create(rectX + pos.x(), y + 3, RTSPortGItem::INPORT);
        addToGroup(inPortGItem);
        inPorts[inPort->name] = inPortGItem;
        impl->rtsPortMap[inPort] = inPortGItem.get();
    }

    i = 0;
    for (vector<RTSPortPtr>::iterator it = rtsComp->outPorts.begin(); it != rtsComp->outPorts.end(); it++, i++) {
        RTSPort* outPort = *it;
        string portName = string(outPort->name);
        RTCCommonUtil::splitPortName(portName);
        text = new QGraphicsTextItem(QString(portName.c_str()));
        int r = 7 * i;
        int x = rectX + 66 + pos.x();
        int y = interval * i - r - 3 + pos.y();

        text->setPos(x, y);
        addToGroup(text);

        RTSPortGItemPtr outPortGItem = new RTSPortGItem(outPort);
        if (outPort->isServicePort) {
            outPortGItem->create(rectX + pos.x(), y + 3, RTSPortGItem::SERVICEPORT);
        } else {
            outPortGItem->create(rectX + pos.x(), y + 3, RTSPortGItem::OUTPORT);
        }
        addToGroup(outPortGItem);
        outPorts[outPort->name] = outPortGItem;
        impl->rtsPortMap[outPort] = outPortGItem.get();
    }

    stateCheck();
}


RTSDiagramView::RTSDiagramView()
{
    impl = new RTSDiagramViewImpl(this);
}


RTSDiagramView::~RTSDiagramView()
{
    delete impl;
}


RTSDiagramView* RTSDiagramView::instance()
{
    return ViewManager::findView<RTSDiagramView>();
}


void RTSDiagramView::onActivated()
{
    impl->setNewRTSItemDetector();
}


void RTSDiagramView::onDeactivated()
{
    impl->itemAddedConnection.disconnect();
}


void RTSDiagramView::onRTSCompSelectionChange()
{
    impl->onRTSCompSelectionChange();
}


bool RTSDiagramView::storeState(Archive& archive)
{
    if (impl->currentRTSItem) {
        archive.writeItemId("currentRTSItem", impl->currentRTSItem);
    }
    return true;
}


bool RTSDiagramView::restoreState(const Archive& archive)
{
    RTSystemItem* item = archive.findItem<RTSystemItem>("currentRTSItem");
    if (item) {
        impl->setCurrentRTSItem(item);
    }
    return true;
}


void RTSDiagramView::initializeClass(ExtensionManager* ext)
{
    ext->viewManager().registerClass<RTSDiagramView>(
        "RTSDiagramView", N_("RTC Diagram"), ViewManager::SINGLE_OPTIONAL);
}

void RTSDiagramViewImpl::onRTSystemLoaded()
{
    DDEBUG("RTSDiagramViewImpl::onRTSystemLoaded");

    updateView();
    if (currentRTSItem->isCheckAtLoading()) {
        checkStatus();
    }
    updateRestoredView();
}
