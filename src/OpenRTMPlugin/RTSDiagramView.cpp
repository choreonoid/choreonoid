/*!
 * @author Shizuko Hattori
 * @file
 */
#include "RTSDiagramView.h"

#include <QtWidgets>

#include "RTSNameServerView.h"
#include "RTSPropertiesView.h"
#include "RTSCommonUtil.h"
#include "OpenRTMUtil.h"
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

#include "ConnectorCreaterDialog.h"

#include "LoggerUtil.h"

#include "gettext.h"

#if QT_VERSION >= QT_VERSION_CHECK(5, 0, 0)
#define USE_QT5
#endif

using namespace cnoid;
using namespace std;
using namespace std::placeholders;
using namespace RTC;

namespace cnoid {

class RTSConnectionLineItem : public QGraphicsLineItem {
public:
  RTSConnectionLineItem(qreal x1, qreal y1, qreal x2, qreal y2) :
    QGraphicsLineItem(x1, y1, x2, y2), x1(x1), x2(x2), y1(y1), y2(y2) {
    QPen pen;
    pen.setColor(QColor("black"));
    pen.setWidth(1);
    setPen(pen);
    cx = (x1 + x2) / 2.0;
    cy = (y1 + y2) / 2.0;
  }

  ~RTSConnectionLineItem() {
    // cout << "delete LineItem" <<endl;
  }

  void update() {
    setLine(x1, y1, x2, y2);
    cx = (x1 + x2) / 2.0;
    cy = (y1 + y2) / 2.0;
  }

  void setPos(qreal x1_, qreal y1_, qreal x2_, qreal y2_) {
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

class RTSConnectionMarkerItem : public QGraphicsRectItem {
public:
  enum markerType { UNMOVABLE, HORIZONTAL, VERTIAL };
  markerType type;
  Signal<void(RTSConnectionMarkerItem*)> sigPositionChanged;
  static const int size = 6;

  RTSConnectionMarkerItem(qreal x, qreal y, markerType type_) :
    QGraphicsRectItem(x - size / 2, y - size / 2, size, size),
    type(type_) {
    setBrush(QBrush(QColor("black")));
    if (type != UNMOVABLE)
      setFlags(QGraphicsItem::ItemIsMovable);
  }

  ~RTSConnectionMarkerItem() {
    //cout << "delete MarkerItem" <<endl;
  }

  void setPos(qreal x, qreal y) { setRect(x - size / 2, y - size / 2, size, size); }
  void move(qreal x, qreal y) {
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
////////////////////
class RTSConnectionGItem : public QGraphicsItemGroup, public Referenced {
public:
  enum lineType { THREEPARTS_TYPE, FIVEPARTS_TYPE };
  const qreal xxoffset = 5;

  RTSConnectionGItem(RTSConnection* rtsConnection, bool sIsLeft, QPointF s, bool tIsLeft, QPointF t);
  ~RTSConnectionGItem();

  void paint(QPainter *painter, const QStyleOptionGraphicsItem *option, QWidget *widget) {
    QStyleOptionGraphicsItem myoption(*option);
    myoption.state &= !QStyle::State_Selected;
    QGraphicsItemGroup::paint(painter, &myoption, widget);
  };
  QPainterPath shape() const {
    QPainterPath ret;
    for (int i = 0; i < 5; i++) {
      if (line[i]) {
        QGraphicsLineItem atari(line[i]->line());
        atari.setPen(QPen(QBrush(Qt::black), 10));
        ret.addPath(atari.shape());
      }
    }
    return ret;
  };
  QRectF boundingRect() const {
    QRectF rect;
    for (int i = 0; i < 5; i++) {
      if (line[i]) {
        QGraphicsLineItem atari(line[i]->line());
        atari.setPen(QPen(QBrush(Qt::black), 10));
        rect |= atari.boundingRect();
      }
    }
    return rect;
  };

  void getLinePosition(Vector2 pos[]) {
    pos[0](0) = line[0]->x2;
    pos[0](1) = line[0]->y2;
    for (int i = 2; i < 5; i++) {
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
};
typedef ref_ptr<RTSConnectionGItem> RTSConnectionGItemPtr;

RTSConnectionGItem::RTSConnectionGItem(RTSConnection* rtsConnection, bool sIsLeft, QPointF s,
 	bool tIsLeft, QPointF t) :
	rtsConnection(rtsConnection), _sIsLeft(sIsLeft), _tIsLeft(tIsLeft) {
	effect = new QGraphicsOpacityEffect;
	effect->setOpacity(0.3);
	setGraphicsEffect(effect);
	if (rtsConnection->isAlive())
		effect->setEnabled(false);
	else
		effect->setEnabled(true);

	if (rtsConnection->setPos) {
		Vector2& p0s = rtsConnection->position[0];
		Vector2& p0e = rtsConnection->position[1];
		line[0] = new RTSConnectionLineItem(p0e(0), p0e(1), p0s(0), p0s(1));
		addToGroup(line[0]);
		Vector2& p1s = rtsConnection->position[4];
		Vector2& p1e = rtsConnection->position[5];
		line[1] = new RTSConnectionLineItem(p1s(0), p1s(1), p1e(0), p1e(1));
		addToGroup(line[1]);
		for (int i = 2; i<5; i++) {
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
			signalConnections.add(marker[2]->sigPositionChanged.connect(
				std::bind(&RTSConnectionGItem::lineMove, this, _1)));
			marker[3] = marker[4] = 0;
		} else {
			type = FIVEPARTS_TYPE;
			marker[0] = new RTSConnectionMarkerItem(line[0]->x2, line[0]->y2, RTSConnectionMarkerItem::UNMOVABLE);
			marker[1] = new RTSConnectionMarkerItem(line[1]->x2, line[1]->y2, RTSConnectionMarkerItem::UNMOVABLE);
			marker[2] = new RTSConnectionMarkerItem(line[2]->cx, line[2]->cy, RTSConnectionMarkerItem::HORIZONTAL);
			marker[3] = new RTSConnectionMarkerItem(line[3]->cx, line[3]->cy, RTSConnectionMarkerItem::VERTIAL);
			marker[4] = new RTSConnectionMarkerItem(line[4]->cx, line[4]->cy, RTSConnectionMarkerItem::HORIZONTAL);
			signalConnections.add(marker[2]->sigPositionChanged.connect(
				std::bind(&RTSConnectionGItem::lineMove, this, _1)));
			signalConnections.add(marker[3]->sigPositionChanged.connect(
				std::bind(&RTSConnectionGItem::lineMove, this, _1)));
			signalConnections.add(marker[4]->sigPositionChanged.connect(
				std::bind(&RTSConnectionGItem::lineMove, this, _1)));
		}
	} else {
		qreal sx, tx;
		sx = firstLineX(s.x());
		tx = endLineX(t.x());
		type = getType(sx, tx);

		line[0] = new RTSConnectionLineItem(sx, s.y(), s.x(), s.y());
		addToGroup(line[0]);

		line[1] = new RTSConnectionLineItem(tx, t.y(), t.x(), t.y());
		addToGroup(line[1]);

		qreal centerX = (sx + tx) / 2.0;
		qreal centerY = (s.y() + t.y()) / 2.0;
		if (type == THREEPARTS_TYPE) {
			line[2] = new RTSConnectionLineItem(sx, s.y(), centerX, s.y());
			addToGroup(line[2]);
			line[3] = new RTSConnectionLineItem(centerX, s.y(), centerX, t.y());
			addToGroup(line[3]);
			line[4] = new RTSConnectionLineItem(centerX, t.y(), tx, t.y());
			addToGroup(line[4]);
			marker[0] = new RTSConnectionMarkerItem(line[0]->x2, line[0]->y2, RTSConnectionMarkerItem::UNMOVABLE);
			marker[1] = new RTSConnectionMarkerItem(line[1]->x2, line[1]->y2, RTSConnectionMarkerItem::UNMOVABLE);
			marker[2] = new RTSConnectionMarkerItem(line[3]->cx, line[3]->cy, RTSConnectionMarkerItem::HORIZONTAL);
			signalConnections.add(marker[2]->sigPositionChanged.connect(
				std::bind(&RTSConnectionGItem::lineMove, this, _1)));
			marker[3] = marker[4] = 0;
		} else {
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
				std::bind(&RTSConnectionGItem::lineMove, this, _1)));
			signalConnections.add(marker[3]->sigPositionChanged.connect(
				std::bind(&RTSConnectionGItem::lineMove, this, _1)));
			signalConnections.add(marker[4]->sigPositionChanged.connect(
				std::bind(&RTSConnectionGItem::lineMove, this, _1)));
		}

		Vector2 pos[6];
		getLinePosition(pos);
		rtsConnection->setPosition(pos);
	}

	setFlags(QGraphicsItem::ItemIsSelectable);
}

RTSConnectionGItem::~RTSConnectionGItem() {
  signalConnections.disconnect();

  for (int i = 0; i<5; i++) {
    if (marker[i]) {
      delete marker[i];
    }
  }
}

qreal RTSConnectionGItem::firstLineX(qreal x) {
  if (_sIsLeft)
    return x - xxoffset;
  else
    return x + xxoffset;
}

qreal RTSConnectionGItem::endLineX(qreal x) {
  if (_tIsLeft)
    return x - xxoffset;
  else
    return x + xxoffset;
}

int RTSConnectionGItem::markerIndex(QGraphicsItem* gItem) {
  for (int i = 2; i<5; i++)
    if (marker[i] == gItem)
      return i;
  return -1;
}

RTSConnectionMarkerItem* RTSConnectionGItem::findMarker(QGraphicsItem* gItem) {
  if (this == gItem || gItem == 0)
    return 0;
  int i = markerIndex(gItem);
  if (i<0)
    return 0;
  return marker[i];
}

void RTSConnectionGItem::lineMove(RTSConnectionMarkerItem* marker_) {
  int i = markerIndex(marker_);
  if (i<0)
    return;
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

RTSConnectionGItem::lineType RTSConnectionGItem::getType(qreal sx, qreal tx) {
  if ((_sIsLeft && tx<sx) || (_tIsLeft &&  sx<tx) ||
    (!_sIsLeft && sx<tx) || (!_tIsLeft && tx<sx))
    return THREEPARTS_TYPE;
  else
    return FIVEPARTS_TYPE;
}

bool RTSConnectionGItem::changePortPos(bool isSource, QPointF s) {
  if (isSource) {
    qreal sx = firstLineX(s.x());
    lineType type_ = getType(sx, line[1]->x1);
    if (type != type_)
      return false;
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
    if (type != type_)
      return false;
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

void RTSConnectionGItem::showMarker(bool on) {
  if (on) {
    for (int i = 0; i<5; i++) {
      if (marker[i] && scene()) {
        scene()->addItem(marker[i]);
      }
    }
  } else {
    for (int i = 0; i<5; i++) {
      if (marker[i] && scene()) {
        scene()->removeItem(marker[i]);
      }
    }
  }
}
////////////////////
class RTSPortGItem : public QGraphicsItemGroup, public Referenced {
public:
  enum portType { INPORT, OUTPORT, SERVICEPORT };
  RTSPortGItem(RTSPort* rtsPort);

  RTSPort* rtsPort;
  QGraphicsPolygonItem* polygon;
  QPointF pos;

  void create(int rectX, const QPointF& pos, int i, portType type);
  void stateCheck();
  void setCandidate(bool isCand);

};
typedef ref_ptr<RTSPortGItem> RTSPortGItemPtr;

RTSPortGItem::RTSPortGItem(RTSPort* rtsPort)
  : rtsPort(rtsPort) {
}

void RTSPortGItem::create(int rectX, const QPointF& pos, int i, portType type) {
  int r = 7 * i;
  switch (type) {
    case INPORT:
      this->pos = QPointF(rectX + 5 + pos.x(), 5 + 7 + 7 + (25 * i) - r + pos.y());
      polygon = new QGraphicsPolygonItem(QPolygonF(QVector<QPointF>()
        << QPointF(rectX + 0 + pos.x(), 0 + 7 + 7 + (25 * i) - r + pos.y())
        << QPointF(rectX + 10 + pos.x(), 0 + 7 + 7 + (25 * i) - r + pos.y())
        << QPointF(rectX + 10 + pos.x(), 10 + 7 + 7 + (25 * i) - r + pos.y())
        << QPointF(rectX + 0 + pos.x(), 10 + 7 + 7 + (25 * i) - r + pos.y())
        << QPointF(rectX + 5 + pos.x(), 5 + 7 + 7 + (25 * i) - r + pos.y())
        << QPointF(rectX + 0 + pos.x(), 0 + 7 + 7 + (25 * i) - r + pos.y())));
      polygon->setPen(QPen(QColor("red")));
      stateCheck();
      addToGroup(polygon);
      break;
    case OUTPORT:
      polygon = new QGraphicsPolygonItem(QPolygonF(QVector<QPointF>()
        << QPointF(rectX + 53 + pos.x(), 0 + 7 + 7 + (25 * i) - r + pos.y())
        << QPointF(rectX + 60 + pos.x(), 0 + 7 + 7 + (25 * i) - r + pos.y())
        << QPointF(rectX + 65 + pos.x(), 5 + 7 + 7 + (25 * i) - r + pos.y())
        << QPointF(rectX + 60 + pos.x(), 10 + 7 + 7 + (25 * i) - r + pos.y())
        << QPointF(rectX + 53 + pos.x(), 10 + 7 + 7 + (25 * i) - r + pos.y())
        << QPointF(rectX + 53 + pos.x(), 0 + 7 + 7 + (25 * i) - r + pos.y())));
      this->pos = QPointF(rectX + 65 + pos.x(), 5 + 7 + 7 + (25 * i) - r + pos.y());
      polygon->setPen(QPen(QColor("red")));
      stateCheck();
      addToGroup(polygon);
      break;
    case SERVICEPORT:
      polygon = new QGraphicsPolygonItem(QPolygonF(QVector<QPointF>()
        << QPointF(rectX + 53 + pos.x(), 0 + 7 + 7 + (25 * i) - r + pos.y())
        << QPointF(rectX + 63 + pos.x(), 0 + 7 + 7 + (25 * i) - r + pos.y())
        << QPointF(rectX + 63 + pos.x(), 5 + 7 + 7 + (25 * i) - r + pos.y())
        << QPointF(rectX + 63 + pos.x(), 10 + 7 + 7 + (25 * i) - r + pos.y())
        << QPointF(rectX + 53 + pos.x(), 10 + 7 + 7 + (25 * i) - r + pos.y())
        << QPointF(rectX + 53 + pos.x(), 0 + 7 + 7 + (25 * i) - r + pos.y())));
      this->pos = QPointF(rectX + 63 + pos.x(), 5 + 7 + 7 + (25 * i) - r + pos.y());
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

void RTSPortGItem::setCandidate(bool isCand) {
  if (isCand) {
    polygon->setPen(QPen(QColor("magenta"), 3));
  } else {
    polygon->setPen(QPen(QColor("red"), 1));
  }
}

////////////////////
class RTSCompGItem : public QGraphicsItemGroup, public Referenced {
public:
  RTSCompGItem(RTSComp* rtsComp, RTSDiagramViewImpl* impl, const QPointF& pos);
  ~RTSCompGItem();
  QVariant itemChange(GraphicsItemChange change, const QVariant & value);
  void create(const QPointF& pos);
  void stateCheck();
  void setStatus(RTC_STATUS status);
  void checkCandidate(RTSPortGItem* sourcePort);
  void clearCandidate();

  RTSDiagramViewImpl* impl;
  RTSComp* rtsComp;
  map<string, RTSPortGItemPtr> inPorts;
  map<string, RTSPortGItemPtr> outPorts;
  QGraphicsRectItem* rect;

  QGraphicsOpacityEffect* effect;
  Signal<void(const RTSCompGItem*)> sigPositionChanged;
  Connection positionChangeConnection;
private:
  int correctTextY();
};
typedef ref_ptr<RTSCompGItem> RTSCompGItemPtr;

QVariant RTSCompGItem::itemChange(GraphicsItemChange change, const QVariant & value)
{
    if(change == ItemPositionChange){
        rtsComp->setPos(rtsComp->pos() + value.value<QPointF>() - pos());
        for(auto it = inPorts.begin(); it != inPorts.end(); ++it){
            it->second->pos += value.value<QPointF>() - pos();
        }
        for(auto it = outPorts.begin(); it != outPorts.end(); ++it){
            it->second->pos += value.value<QPointF>() - pos();
        }
        sigPositionChanged(this);
    }
    return QGraphicsItem::itemChange(change, value);
}

int RTSCompGItem::correctTextY() {
  int numIn = rtsComp->inPorts.size();
  int numOut = rtsComp->outPorts.size();
  int numMax = (numIn < numOut ? numOut : numIn);
  if (!numMax)
    numMax = 1;
  return (25 * numMax) - 7 * (numMax - 1);
}

void RTSCompGItem::stateCheck() {
  setStatus(rtsComp->getRTCState());

  for (map<string, RTSPortGItemPtr>::iterator it = inPorts.begin(); it != inPorts.end(); it++) {
    it->second->stateCheck();
  }

  for (map<string, RTSPortGItemPtr>::iterator it = outPorts.begin(); it != outPorts.end(); it++) {
    it->second->stateCheck();
  }
}

void RTSCompGItem::setStatus(RTC_STATUS status) {
  if (status == RTC_STATUS::RTC_INACTIVE) {
    rect->setBrush(QBrush(QColor("blue")));
  } else if (status == RTC_STATUS::RTC_ACTIVE) {
    rect->setBrush(QBrush(QColor("lightgreen")));
  } else if (status == RTC_STATUS::RTC_ERROR) {
    rect->setBrush(QBrush(QColor("red")));
  }
}

////////////////////
#define STATE_CHECK_TIME 1000  //msec

void RTSCompGItem::checkCandidate(RTSPortGItem* sourcePort) {
  if (!sourcePort->rtsPort) return;
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

void RTSCompGItem::clearCandidate() {
  for (map<string, RTSPortGItemPtr>::iterator it = inPorts.begin(); it != inPorts.end(); it++) {
    it->second->setCandidate(false);
  }

  for (map<string, RTSPortGItemPtr>::iterator it = outPorts.begin(); it != outPorts.end(); it++) {
    it->second->setCandidate(false);
  }
}
////////////////////
class RTSDiagramViewImpl : public QGraphicsView {

public:
  RTSDiagramView* self;
  RTSystemItemPtr currentRTSItem;
  QGraphicsScene  scene;
  ScopedConnection nsViewSelectionChangedConnection;
  ScopedConnection itemAddedConnection;
  ScopedConnection itemTreeViewSelectionChangedConnection;
  ScopedConnection connectionOfRTSystemItemDetachedFromRoot;
  ScopedConnection timeOutConnection;
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

  RTSDiagramViewImpl(RTSDiagramView* self);
  ~RTSDiagramViewImpl();
  void setNewRTSItemDetector();
  void addRTSComp(NamingContextHelper::ObjectInfo& info, const QPointF& pos);
  void addRTSComp(RTSComp* rtsComp);
  void deleteRTSComp(RTSCompGItem* rtsComp);
  void deleteRTSConnection(RTSConnectionGItem* rtsConnection);
  void deleteSelectedRTSItem();
  void dragEnterEvent(QDragEnterEvent* event);
  void dragMoveEvent(QDragMoveEvent*  event);
  void dragLeaveEvent(QDragLeaveEvent* event);
  void dropEvent(QDropEvent*      event);
  void mouseMoveEvent(QMouseEvent*     event);
  void mousePressEvent(QMouseEvent*     event);
  void mouseReleaseEvent(QMouseEvent*     event);
  void wheelEvent(QWheelEvent*			event);
  void keyPressEvent(QKeyEvent*       event);
  void onnsViewItemSelectionChanged(const list<NamingContextHelper::ObjectInfo>& items);
  RTSPortGItem* findTargetRTSPort(QPointF& pos);
  RTSConnectionMarkerItem* findConnectionMarker(QGraphicsItem* gItem);
  void createConnectionGItem(RTSConnection* rtsConnection, RTSPortGItem* sourcePort, RTSPortGItem* targetPort);
  void onRTSCompSelectionChange();
  void onRTSCompPositionChanged(const RTSCompGItem*);
  void onTime();
  void onActivated(bool on);
  void onItemTreeViewSelectionChanged(const ItemList<RTSystemItem>& items);
  void onRTSystemItemDetachedFromRoot();
  void setCurrentRTSItem(RTSystemItem* item);
  void updateView();
  void updateSetting();

private:
  void activateComponent();
  void deactivateComponent();
  void resetComponent();
  void finalizeComponent();
  void startExecutionContext();
  void stopExecutionContext();

  int pollingPeriod;
};

RTSDiagramViewImpl::RTSDiagramViewImpl(RTSDiagramView* self)
  :self(self), sourcePort(0), pollingPeriod(500) {
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
        std::bind(&RTSDiagramViewImpl::onnsViewItemSelectionChanged, this, _1)));
  }

  timer.setSingleShot(false);
  MappingPtr appVars = AppConfig::archive()->openMapping("OpenRTM");
  pollingPeriod = appVars->get("pollingCycle", 500);
  timer.setInterval(pollingPeriod);
  timeOutConnection.reset(
    timer.sigTimeout().connect(
      std::bind(&RTSDiagramViewImpl::onTime, this)));
  self->sigActivated().connect(std::bind(&RTSDiagramViewImpl::onActivated, this, true));
  self->sigDeactivated().connect(std::bind(&RTSDiagramViewImpl::onActivated, this, false));

  itemTreeViewSelectionChangedConnection.reset(
    ItemTreeView::mainInstance()->sigSelectionChanged().connect(
      std::bind(&RTSDiagramViewImpl::onItemTreeViewSelectionChanged, this, _1)));

  QPen pen(Qt::DashDotLine);
  pen.setWidth(2);
  dragPortLine = new QGraphicsLineItem();
  dragPortLine->setPen(pen);
  dragPortLine->setZValue(100);
  dragPortLine->setVisible(false);
  scene.addItem(dragPortLine);

  targetMarker = 0;
}

RTSDiagramViewImpl::~RTSDiagramViewImpl() {
  rtsComps.clear();
  rtsConnections.clear();
  disconnect(&scene, SIGNAL(selectionChanged()), self, SLOT(onRTSCompSelectionChange()));
}

void RTSDiagramViewImpl::dragEnterEvent(QDragEnterEvent *event) {
#ifdef USE_QT5
    const RTSNameTreeWidget* nameServerItem =
            qobject_cast<const RTSNameTreeWidget*>(event->source());//event->mimeData());
    if (nameServerItem) {
#else
  if (event->mimeData()->hasFormat("application/RTSNameServerItem")) {
#endif
    event->acceptProposedAction();
  }
}

void RTSDiagramViewImpl::dragMoveEvent(QDragMoveEvent *event) {
#ifdef USE_QT5
  const RTSNameTreeWidget* nameServerItem =
    qobject_cast<const RTSNameTreeWidget*>(event->source());//event->mimeData());
  if (nameServerItem) {
#else
  if (event->mimeData()->hasFormat("application/RTSNameServerItem")) {
#endif
    event->acceptProposedAction();
  }
}

void RTSDiagramViewImpl::dragLeaveEvent(QDragLeaveEvent *event) {
  MessageView::instance()->putln(_("Drag and drop has been canceled. Please be operation again."));
}

void RTSDiagramViewImpl::dropEvent(QDropEvent *event) {
  DDEBUG("RTSystemItem::dropEvent");

  for (list<NamingContextHelper::ObjectInfo>::iterator it = nsViewSelections.begin(); it != nsViewSelections.end(); it++) {
    NamingContextHelper::ObjectInfo& info = *it;
    if (!info.isAlive) {
      MessageView::instance()->putln((boost::format(_("%1% is not alive")) % info.id).str());
    } else {
      addRTSComp(info, mapToScene(event->pos()));
      DDEBUG_V("%s", info.getFullPath().c_str());
    }
  }
}

void RTSDiagramViewImpl::keyPressEvent(QKeyEvent *event) {
  if (event->key() == Qt::Key_Delete) {
    deleteSelectedRTSItem();
  }
}

void RTSDiagramViewImpl::mousePressEvent(QMouseEvent* event) {
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
      setDragMode(DragMode::ScrollHandDrag);
    }
    QGraphicsView::mousePressEvent(event);
  }

  if (event->button() == Qt::RightButton) {
    if (!selectionRTCs.empty()) {
      if (selectionRTCs.size() == 1) {
        menuManager.setNewPopupMenu(this);
        menuManager.addItem("Activate")
          ->sigTriggered().connect(std::bind(&RTSDiagramViewImpl::activateComponent, this));
        menuManager.addItem("Deactivate")
          ->sigTriggered().connect(std::bind(&RTSDiagramViewImpl::deactivateComponent, this));
        menuManager.addItem("Reset")
          ->sigTriggered().connect(std::bind(&RTSDiagramViewImpl::resetComponent, this));
        if (isManagedRTC(selectionRTCs.front()->rtsComp->rtc_) == false) {
          menuManager.addItem("Exit")
            ->sigTriggered().connect(std::bind(&RTSDiagramViewImpl::finalizeComponent, this));
        }
        menuManager.addSeparator();
        menuManager.addItem("Start")
          ->sigTriggered().connect(std::bind(&RTSDiagramViewImpl::startExecutionContext, this));
        menuManager.addItem("Stop")
          ->sigTriggered().connect(std::bind(&RTSDiagramViewImpl::stopExecutionContext, this));
        menuManager.addSeparator();
      }
      menuManager.addItem("Remove")
        ->sigTriggered().connect(std::bind(&RTSDiagramViewImpl::deleteSelectedRTSItem, this));

      menuManager.popupMenu()->popup(event->globalPos());

    } else if (!selectionRTSConnections.empty()) {
      menuManager.setNewPopupMenu(this);
      menuManager.addItem("Delete")
        ->sigTriggered().connect(std::bind(&RTSDiagramViewImpl::deleteSelectedRTSItem, this));

      menuManager.popupMenu()->popup(event->globalPos());
    }
  }
}

void RTSDiagramViewImpl::mouseMoveEvent(QMouseEvent* event) {
  QPointF pos = mapToScene(event->pos());

  if (sourcePort) {
    QPointF sp = sourcePort->pos;
    dragPortLine->setLine(sp.x(), sp.y(), pos.x(), pos.y());
    RTSPortGItem* port = findTargetRTSPort(pos);
    if (port && !sourcePort->rtsPort->checkConnectablePort(port->rtsPort))
      setCursor(Qt::ForbiddenCursor);
    else
      setCursor(Qt::PointingHandCursor);

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

void RTSDiagramViewImpl::mouseReleaseEvent(QMouseEvent *event) {
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
          ServiceConnectorCreaterDialog createDialog;
          createDialog.setDisp(sourcePort->rtsPort, targetPort->rtsPort);
          createDialog.exec();
          isAccepted = createDialog.isAccepted;
          name = createDialog.nameLineEdit->text().toStdString();
          propList = createDialog.propList;

        } else {
          DDEBUG("DataPort Connect");
          DataConnectorCreaterDialog createDialog;
          createDialog.setDisp(sourcePort->rtsPort, targetPort->rtsPort);
          createDialog.exec();
          isAccepted = createDialog.isAccepted;
          name = createDialog.nameLineEdit->text().toStdString();
          propList = createDialog.propList;
        }

        if (isAccepted) {
          DDEBUG("RTSDiagramViewImpl::mouseReleaseEvent Accepted");
          string id = "";
          timeOutConnection.block();
          RTSConnection* rtsConnection = currentRTSItem->addRTSConnection(id, name,
            sourcePort->rtsPort, targetPort->rtsPort, propList);
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

void RTSDiagramViewImpl::wheelEvent(QWheelEvent* event) {
  double dSteps = (double)event->delta() / 120.0;
  double scaleVal = 1.0;
  scaleVal -= (dSteps / 20.0);
  scale(scaleVal, scaleVal);
  QGraphicsView::wheelEvent(event);
}
/////
void RTSDiagramViewImpl::onnsViewItemSelectionChanged(const list<NamingContextHelper::ObjectInfo>& items) {
  nsViewSelections = items;
}

void RTSDiagramViewImpl::onRTSCompSelectionChange() {
  RTSCompGItem* singleSelectedRTC = 0;
  if (selectionRTCs.size() == 1)
    singleSelectedRTC = selectionRTCs.front();

  RTSConnectionGItem* singleSelectedConnection = 0;
  if (selectionRTSConnections.size() == 1)
    singleSelectedConnection = selectionRTSConnections.front();

  for (list<RTSConnectionGItem*>::iterator it = selectionRTSConnections.begin();
    it != selectionRTSConnections.end(); it++) {
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
      nsView->setSelection(selectionRTCs.front()->rtsComp->name, selectionRTCs.front()->rtsComp->fullPath);
    }
  }

  if (selectionRTSConnections.size() == 1 && singleSelectedConnection != selectionRTSConnections.front()) {
    RTSNameServerView* nsView = RTSNameServerView::instance();
    if (nsView) {
      nsView->setSelection("", "");
    }
    RTSPropertiesView* propView = RTSPropertiesView::instance();
    if (propView) {
      RTSConnection* rtsConnection = selectionRTSConnections.front()->rtsConnection;
      propView->showConnectionProperties(rtsConnection->sourcePort->port, rtsConnection->id);
    }
  }
}

void RTSDiagramViewImpl::onRTSCompPositionChanged(const RTSCompGItem* rtsCompGItem) {
  list<RTSConnection*> rtsConnectionList;
  currentRTSItem->RTSCompToConnectionList(rtsCompGItem->rtsComp, rtsConnectionList, 1);

  for (list<RTSConnection*>::iterator it = rtsConnectionList.begin();
    it != rtsConnectionList.end(); it++) {
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
  for (list<RTSConnection*>::iterator it = rtsConnectionList.begin();
    it != rtsConnectionList.end(); it++) {
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

void RTSDiagramViewImpl::onActivated(bool on) {
  if (on) {
    timer.start();
  } else {
    timer.stop();
  }
}


void RTSDiagramViewImpl::onItemTreeViewSelectionChanged(const ItemList<RTSystemItem>& items) {
  RTSystemItem* firstItem = items.toSingle();

  if (firstItem && firstItem != currentRTSItem) {
    setCurrentRTSItem(firstItem);
  }
}

void RTSDiagramViewImpl::onRTSystemItemDetachedFromRoot() {
  currentRTSItem = nullptr;
  connectionOfRTSystemItemDetachedFromRoot.disconnect();
  updateView();
  setNewRTSItemDetector();
}
/////
RTSPortGItem* RTSDiagramViewImpl::findTargetRTSPort(QPointF& pos) {
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
  return 0;
}

RTSConnectionMarkerItem* RTSDiagramViewImpl::findConnectionMarker(QGraphicsItem* gItem) {
  for (list<RTSConnectionGItem*>::iterator it = selectionRTSConnections.begin(); it != selectionRTSConnections.end(); it++) {
    RTSConnectionMarkerItem* marker = (*it)->findMarker(gItem);
    if (marker) {
      return marker;
    }
  }
  return 0;
}

void RTSDiagramViewImpl::setNewRTSItemDetector() {
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

void RTSDiagramViewImpl::addRTSComp(NamingContextHelper::ObjectInfo& info, const QPointF& pos) {
  timeOutConnection.block();
 RTSComp* rtsComp = currentRTSItem->addRTSComp(info, pos);
  if (rtsComp) {
    RTSCompGItemPtr rtsCompGItem = new RTSCompGItem(rtsComp, this, pos);
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

void RTSDiagramViewImpl::addRTSComp(RTSComp* rtsComp) {
  timeOutConnection.block();
  RTSCompGItemPtr rtsCompGItem = new RTSCompGItem(rtsComp, this, rtsComp->pos());
  rtsComps[rtsComp->fullPath] = rtsCompGItem;
  scene.addItem(rtsCompGItem);
  timeOutConnection.unblock();
}

void RTSDiagramViewImpl::deleteRTSComp(RTSCompGItem* rtsCompGItem) {
  timeOutConnection.block();

  list<RTSConnection*> rtsConnectionList;
  currentRTSItem->RTSCompToConnectionList(rtsCompGItem->rtsComp, rtsConnectionList);

  for (list<RTSConnection*>::iterator itr = rtsConnectionList.begin();
    itr != rtsConnectionList.end(); itr++) {
    map<string, RTSConnectionGItemPtr>::iterator it = rtsConnections.find((*itr)->id);
    if (it != rtsConnections.end())
      deleteRTSConnection(it->second);
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


void RTSDiagramViewImpl::deleteSelectedRTSItem() {
  disconnect(&scene, SIGNAL(selectionChanged()), self, SLOT(onRTSCompSelectionChange()));
  for (list<RTSConnectionGItem*>::iterator it = selectionRTSConnections.begin();
    it != selectionRTSConnections.end(); it++) {
    deleteRTSConnection(*it);
  }
  selectionRTSConnections.clear();
  for (list<RTSCompGItem*>::iterator it = selectionRTCs.begin();
    it != selectionRTCs.end(); it++) {
    deleteRTSComp(*it);
  }
  selectionRTCs.clear();
  updateView();
  connect(&scene, SIGNAL(selectionChanged()), self, SLOT(onRTSCompSelectionChange()));
}

void RTSDiagramViewImpl::createConnectionGItem(RTSConnection* rtsConnection,
  RTSPortGItem* sourcePort, RTSPortGItem* targetPort) {
  DDEBUG("RTSDiagramViewImpl::createConnectionGItem");
  RTSConnectionGItemPtr gItem = new RTSConnectionGItem(rtsConnection,
    rtsConnection->sourcePort->isInPort,
    sourcePort->pos,
    rtsConnection->targetPort->isInPort,
    targetPort->pos);

  scene.addItem(gItem);
  rtsConnections[rtsConnection->id] = gItem;
}

void RTSDiagramViewImpl::onTime() {
  if (!currentRTSItem)
    return;

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
    bool modified = false;
    for (map<string, RTSCompGItemPtr>::iterator it = rtsComps.begin();
      it != rtsComps.end(); it++) {
      if (!currentRTSItem->compIsAlive(it->second->rtsComp)) {
        if (!it->second->effect->isEnabled()) {
          // it->second->effect->setEnabled(true);
          modified = true;
        } else {
          deleteRTSComp(it->second);
        }
      } else {
        if (it->second->effect->isEnabled()) {
          //  it->second->effect->setEnabled(false);
          modified = true;
        }
      }
    }

    if (currentRTSItem->connectionCheck())
      modified = true;

    if (modified)
      updateView();
  }

  for (map<string, RTSCompGItemPtr>::iterator it = rtsComps.begin();
    it != rtsComps.end(); it++) {
    it->second->stateCheck();
  }
}

void RTSDiagramViewImpl::setCurrentRTSItem(RTSystemItem* item)
{
    timer.setInterval(item->pollingCycle());

  currentRTSItem = item;
  connectionOfRTSystemItemDetachedFromRoot.reset(
    item->sigDetachedFromRoot().connect(
      [&]() { onRTSystemItemDetachedFromRoot(); }));
  updateView();
}

void RTSDiagramViewImpl::updateView() {
    DDEBUG("RTSDiagramViewImpl::updateView");
    timeOutConnection.block();
    rtsComps.clear();
    rtsConnections.clear();
    rtsPortMap.clear();
    if(currentRTSItem){
        setBackgroundBrush(QBrush(Qt::white));
        map<string, RTSCompPtr>& comps = currentRTSItem->rtsComps();
        for(map<string, RTSCompPtr>::iterator itr = comps.begin(); itr != comps.end(); itr++){
            addRTSComp(itr->second.get());
        }
        RTSystemItem::RTSConnectionMap& connections = currentRTSItem->rtsConnections();
        for(RTSystemItem::RTSConnectionMap::iterator itr = connections.begin();
                itr != connections.end(); itr++){
            DDEBUG("RTSDiagramViewImpl::updateView find connection")
            if(rtsConnections.find(itr->second->id)==rtsConnections.end()){
                RTSConnection* rtsConnection = itr->second.get();
                RTSPortGItem* source = rtsPortMap.find(rtsConnection->sourcePort)->second;
                RTSPortGItem* target = rtsPortMap.find(rtsConnection->targetPort)->second;
                createConnectionGItem(rtsConnection, source, target);
            }
        }
        setAcceptDrops(true);
    }else{
        setBackgroundBrush(QBrush(Qt::gray));
        setAcceptDrops(false);
    }
    timeOutConnection.unblock();
}

void RTSDiagramViewImpl::updateSetting() {
  DDEBUG("RTSDiagramViewImpl::updateSetting");

  MappingPtr appVars = AppConfig::archive()->openMapping("OpenRTM");
  int pPeriod = appVars->get("pollingCycle", 500);
  if (pollingPeriod != pPeriod) {
    timer.stop();
    timer.setInterval(pPeriod);
    pPeriod = pollingPeriod;
    timer.start();
  }
}
/////
void RTSDiagramViewImpl::activateComponent() {
  RTSCompGItem* target = selectionRTCs.front();
  if (target->rtsComp->activateComponent() == false) {
    QMessageBox::information(this, _("Activate"), _("Activation of target component FAILED."));
  }
}

void RTSDiagramViewImpl::deactivateComponent() {
  RTSCompGItem* target = selectionRTCs.front();
  if (target->rtsComp->deactivateComponent() == false) {
    QMessageBox::information(this, _("Deactivate"), _("Deactivation of target component FAILED."));
  }
}

void RTSDiagramViewImpl::resetComponent() {
  RTSCompGItem* target = selectionRTCs.front();
  if (target->rtsComp->resetComponent() == false) {
    QMessageBox::information(this, _("Reset"), _("FAILED to reset target component."));
  }
}

void RTSDiagramViewImpl::finalizeComponent() {
  RTSCompGItem* target = selectionRTCs.front();
  if (target->rtsComp->finalizeComponent() == false) {
    QMessageBox::information(this, _("Exit"), _("FAILED to exit target component."));
  }
  deleteSelectedRTSItem();
}

void RTSDiagramViewImpl::startExecutionContext() {
  RTSCompGItem* target = selectionRTCs.front();
  if (target->rtsComp->startExecutionContext() == false) {
    QMessageBox::information(this, _("Start"), _("FAILED to start ExecutionContext."));
  }
}

void RTSDiagramViewImpl::stopExecutionContext() {
  RTSCompGItem* target = selectionRTCs.front();
  if (target->rtsComp->stopExecutionContext() == false) {
    QMessageBox::information(this, _("Start"), _("FAILED to stop ExecutionContext."));
  }
}
//////////
RTSCompGItem::RTSCompGItem(RTSComp* rtsComp, RTSDiagramViewImpl* impl, const QPointF& pos) :
  impl(impl), rtsComp(rtsComp) {
  effect = new QGraphicsOpacityEffect;
  effect->setOpacity(0.3);
  setGraphicsEffect(effect);
  if (rtsComp->rtc_)
    effect->setEnabled(false);
  else
    effect->setEnabled(true);

  create(pos);
  positionChangeConnection = sigPositionChanged.connect(
    std::bind(&RTSDiagramViewImpl::onRTSCompPositionChanged, impl, _1));
}

RTSCompGItem::~RTSCompGItem() {
  positionChangeConnection.disconnect();
  for (map<string, RTSPortGItemPtr>::iterator it = inPorts.begin();
    it != inPorts.end(); it++) {
    impl->rtsPortMap.erase(it->second->rtsPort);
  }
  for (map<string, RTSPortGItemPtr>::iterator it = outPorts.begin();
    it != outPorts.end(); it++) {
    impl->rtsPortMap.erase(it->second->rtsPort);
  }
}

void RTSCompGItem::create(const QPointF& pos) {
  QGraphicsTextItem * text = new QGraphicsTextItem(QString(rtsComp->name.c_str()));
  int height = correctTextY();
  text->setPos(0 + pos.x(), height + 5 + pos.y());
  addToGroup(text);

  rect = new QGraphicsRectItem(7, 7, 50, height);
  int rectX = (text->boundingRect().width() / 2 - 50 / 2) - 5;
  rect->setPos(rectX + pos.x(), pos.y());
  rect->setPen(QPen(QColor("darkgray")));
  addToGroup(rect);
  setFlags(QGraphicsItem::ItemIsMovable | QGraphicsItem::ItemIsSelectable | QGraphicsItem::ItemSendsScenePositionChanges);

  int i = 0;
  for (vector<RTSPortPtr>::iterator it = rtsComp->inPorts.begin(); it != rtsComp->inPorts.end(); it++, i++) {
    RTSPort* inPort = *it;
    string portName = string(inPort->name);
    RTCCommonUtil::splitPortName(portName);
    text = new QGraphicsTextItem(QString(portName.c_str()));
    int x = rectX + pos.x() - text->boundingRect().width();
    int y = 25 * i - 7 * i - 3 + pos.y();
    text->setPos(x, y);
    addToGroup(text);

    RTSPortGItemPtr inPortGItem = new RTSPortGItem(inPort);
    inPortGItem->create(rectX, pos, i, RTSPortGItem::INPORT);
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
    int y = 25 * i - r - 3 + pos.y();
    text->setPos(x, y);
    addToGroup(text);

    RTSPortGItemPtr outPortGItem = new RTSPortGItem(outPort);
    if (outPort->isServicePort) {
      outPortGItem->create(rectX, pos, i, RTSPortGItem::SERVICEPORT);
    } else {
      outPortGItem->create(rectX, pos, i, RTSPortGItem::OUTPORT);
    }
    addToGroup(outPortGItem);
    outPorts[outPort->name] = outPortGItem;
    impl->rtsPortMap[outPort] = outPortGItem.get();
  }

  stateCheck();
}
//////////
RTSDiagramView::RTSDiagramView() {
  impl = new RTSDiagramViewImpl(this);
}

RTSDiagramView::~RTSDiagramView() {
  delete impl;
}

RTSDiagramView* RTSDiagramView::instance() {
  return ViewManager::findView<RTSDiagramView>();
}

void RTSDiagramView::onActivated() {
  impl->setNewRTSItemDetector();
}

void RTSDiagramView::onDeactivated() {
  impl->itemAddedConnection.disconnect();
}

void RTSDiagramView::onRTSCompSelectionChange() {
  impl->onRTSCompSelectionChange();
}

void RTSDiagramView::updateView() {
  impl->updateView();
}

void RTSDiagramView::updateSetting() {
  impl->updateSetting();
}

bool RTSDiagramView::storeState(Archive& archive) {
  if (impl->currentRTSItem) {
    archive.writeItemId("currentRTSItem", impl->currentRTSItem);
  }
  return true;
}

bool RTSDiagramView::restoreState(const Archive& archive) {
  RTSystemItem* item = archive.findItem<RTSystemItem>("currentRTSItem");
  if (item) {
    impl->setCurrentRTSItem(item);
  }
  return true;
}

void RTSDiagramView::initializeClass(ExtensionManager* ext) {
  ext->viewManager().registerClass<RTSDiagramView>(
    "RTSDiagramView", N_("RTC Diagram"), ViewManager::SINGLE_OPTIONAL);
}

////////////////////
SettingDialog::SettingDialog() {
  chkLog = new QCheckBox(_("Log Output"));
  QLabel* lblLevel = new QLabel(_("Log Level:"));
  cmbLogLevel = new QComboBox();
  cmbLogLevel->addItem("SILENT");
  cmbLogLevel->addItem("FATAL");
  cmbLogLevel->addItem("ERROR");
  cmbLogLevel->addItem("WARN");
  cmbLogLevel->addItem("INFO");
  cmbLogLevel->addItem("DEBUG");
  cmbLogLevel->addItem("TRACE");
  cmbLogLevel->addItem("VERBOSE");
  cmbLogLevel->addItem("PARANOID");

  QLabel* lblName = new QLabel(_("VendorName:"));
  leName = new QLineEdit;
  QLabel* lblVersion = new QLabel(_("Version:"));
  leVersion = new QLineEdit;
  QLabel* lblPolling = new QLabel(_("Polling Cycle:"));
  lePoling = new QLineEdit;
  QLabel* lblUnit = new QLabel("ms");
#ifndef OPENRTM_VERSION11
  QLabel* lblHeartBeat = new QLabel(_("Heartbeat Period:"));
  leHeartBeat = new QLineEdit;
  QLabel* lblUnitHb = new QLabel("ms");
#endif

  QFrame* frmDetail = new QFrame;
  QGridLayout* gridSubLayout = new QGridLayout(frmDetail);
  gridSubLayout->addWidget(chkLog, 0, 0, 1, 1);
  gridSubLayout->addWidget(lblLevel, 0, 1, 1, 1);
  gridSubLayout->addWidget(cmbLogLevel, 0, 2, 1, 1);
  gridSubLayout->addWidget(lblName, 1, 0, 1, 1);
  gridSubLayout->addWidget(leName, 1, 1, 1, 2);
  gridSubLayout->addWidget(lblVersion, 2, 0, 1, 1);
  gridSubLayout->addWidget(leVersion, 2, 1, 1, 2);
  gridSubLayout->addWidget(lblPolling, 3, 0, 1, 1);
  gridSubLayout->addWidget(lePoling, 3, 1, 1, 2);
  gridSubLayout->addWidget(lblUnit, 3, 3, 1, 1);
#ifndef OPENRTM_VERSION11
  gridSubLayout->addWidget(lblHeartBeat, 4, 0, 1, 1);
  gridSubLayout->addWidget(leHeartBeat, 4, 1, 1, 2);
  gridSubLayout->addWidget(lblUnitHb, 4, 3, 1, 1);
#endif
  //
  QFrame* frmButton = new QFrame;
  QPushButton* okButton = new QPushButton(_("&OK"));
  okButton->setDefault(true);
  QPushButton* cancelButton = new QPushButton(_("&Cancel"));
  QHBoxLayout* buttonBotLayout = new QHBoxLayout(frmButton);
  buttonBotLayout->addWidget(cancelButton);
  buttonBotLayout->addStretch();
  buttonBotLayout->addWidget(okButton);

  QVBoxLayout* mainLayout = new QVBoxLayout;
  mainLayout->addWidget(frmDetail);
  mainLayout->addWidget(frmButton);
  setLayout(mainLayout);

  connect(okButton, SIGNAL(clicked()), this, SLOT(oKClicked()));
  connect(cancelButton, SIGNAL(clicked()), this, SLOT(rejected()));
  connect(this, SIGNAL(rejected()), this, SLOT(rejected()));
  connect(chkLog, SIGNAL(clicked(bool)), this, SLOT(logChanged(bool)));

  setWindowTitle(_("OpenRTM Preferences"));
  /////
  MappingPtr appVars = AppConfig::archive()->openMapping("OpenRTM");
  leName->setText(QString::fromStdString(appVars->get("defaultVendor", "AIST")));
  leVersion->setText(QString::fromStdString(appVars->get("defaultVersion", "1.0.0")));
  lePoling->setText(QString::number(appVars->get("pollingCycle", 500)));
#ifndef OPENRTM_VERSION11
  leHeartBeat->setText(QString::number(appVars->get("heartBeatPeriod", 500)));
#endif
  chkLog->setChecked(appVars->get("outputLog", false));
  cmbLogLevel->setCurrentText(QString::fromStdString(appVars->get("logLevel", "INFO")));
  cmbLogLevel->setEnabled(chkLog->isChecked());
}

void SettingDialog::logChanged(bool state) {
  cmbLogLevel->setEnabled(chkLog->isChecked());
}

void SettingDialog::oKClicked() {
  AppConfig::archive()->openMapping("OpenRTM")->write("defaultVendor", leName->text().toStdString(), DOUBLE_QUOTED);
  AppConfig::archive()->openMapping("OpenRTM")->write("defaultVersion", leVersion->text().toStdString(), DOUBLE_QUOTED);
  AppConfig::archive()->openMapping("OpenRTM")->write("pollingCycle", lePoling->text().toInt());
#ifndef OPENRTM_VERSION11
  AppConfig::archive()->openMapping("OpenRTM")->write("heartBeatPeriod", leHeartBeat->text().toInt());
#endif
  AppConfig::archive()->openMapping("OpenRTM")->write("outputLog", chkLog->isChecked());
  AppConfig::archive()->openMapping("OpenRTM")->write("logLevel", cmbLogLevel->currentText().toStdString());
  //
  RTSDiagramView::instance()->updateSetting();
  close();
}

void SettingDialog::rejected() {
  close();
}

}
