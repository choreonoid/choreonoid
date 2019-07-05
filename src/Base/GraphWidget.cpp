/**
   @author Shin'ichiro Nakaoka
*/

#include "GraphWidget.h"
#include "GraphBar.h"
#include "ToolBar.h"
#include "TimeBar.h"
#include "ToolBarArea.h"
#include "ScrollBar.h"
#include <cnoid/View>
#include <cnoid/MainWindow>
#include <cnoid/ConnectionSet>
#include <QGridLayout>
#include <QPainter>
#include <QFocusEvent>
#include <fmt/format.h>
#include <cmath>
#include <deque>
#include <limits>
#include <algorithm>

#ifdef _MSC_VER
#include <float.h>
inline int isfinite(double x) { return _finite(x); }
#endif

#include <iostream>

#include "gettext.h"

using namespace std;
using namespace cnoid;

namespace {

const bool TRACE_FUNCTIONS = false;
        
const size_t MAX_NUM_HISTORIES = 10;

struct EditHistory
{
    EditHistory() { frame = 0; }
    int frame;
    vector<double> orgValues;
    vector<double> newValues;
};
typedef std::shared_ptr<EditHistory> EditHistoryPtr;
typedef deque<EditHistoryPtr> EditHistoryList;

}

namespace cnoid {

class GraphDataHandlerImpl
{
public:
    GraphDataHandlerImpl();
        
    Signal<void()> sigDataUpdated;

    /**
       The size of this array is the actual data size + 2 and the actual data is
       stored from the second elements. The prepended / appended elements are
       same as the initial / last elements of the actual data, and the elements
       are used for calculating velocities.
    */
    vector<double> values;
    int numFrames; // the actual number of frames (values.size() - 2)
        
    int prevNumValues;
    double offset;
    double stepRatio;
    double lowerValueLimit;
    double upperValueLimit;
    double lowerVelocityLimit;
    double upperVelocityLimit;

    int id;
    std::string label;
    float r, g, b;

    EditHistoryList editHistories;
    int currentHistory;

    vector<bool> controlPointMask;
    bool isControlPointUpdateNeeded;

    GraphDataHandler::DataRequestCallback dataRequestCallback;
    GraphDataHandler::DataModifiedCallback dataModifiedCallback;
};

class GraphWidgetImpl
{
public:

    GraphWidget* self;
    View* parentView;
    QWidget* screen;
    DoubleScrollBar* hScrollbar;
    DoubleScrollBar* vScrollbar;

    QPen pen;

    QVector<qreal> cursorDashes;
    QVector<qreal> limitValueDashes;
    QPolygonF polyline;

    GraphWidget::Mode mode;
    GraphWidget::EditMode editMode;

    std::vector<GraphDataHandlerPtr> handlers;
    ConnectionSet connections;

    bool isOrgValueVisible;
    bool isVelocityVisible;
    bool isAccelerationVisible;

    bool isReconfigurationNeeded;
    bool isDomainUpdateNeeded;

    double domainLowerX;
    double domainUpperX;

    double rangeLowerY;
    double rangeUpperY;

    double cursorX;
    double screenCursorX;

    double currentScreenX;
    double currentScreenY;

    double leftX;
    double centerY;
    double scaleX;
    double scaleY;

    double visibleWidth;
    double visibleHeight;
    double visibleHeightHalf;
    double bottomY;
    double topY;

    double screenMarginX;
    double screenCenterY;

    double screenWidth;
    double screenHeight;

    double lineWidth;

    bool isLimitVisible;
    bool isGridOn;
    double gridWidth;
    double gridHeight;

    GraphDataHandlerImpl* editTarget;
    bool isControlPointsHighlighted;
    int controlPointStep;
    int controlPointOffset;

    enum { DRAG_NONE, DRAG_CURSOR, DRAG_EDIT, DRAG_ZOOM, DRAG_MOVE } dragState;

    double pressedScreenX;
    double pressedScreenY;
    double dragPrevScreenX;
    double dragPrevScreenY;

    double dragOrgScaleX;
    double dragOrgScaleY;
    double dragOrgLeftX;
    double dragOrgCenterY;

    bool isEditBufferedUpdateMode;
    int editedFrameBegin;
    int editedFrameEnd;

    GraphWidget::ScrollMode autoScrollMode;

    TimeBar* timeBar;
    bool timeBarSyncMode;
    Connection timeChangedConnetion;

    QLabel statusLabel;

    GraphWidgetImpl(GraphWidget* self, View* parentView);
    ~GraphWidgetImpl();

    void onActivated();
    void onDeactivated();

    void addDataHandler(GraphDataHandlerPtr handler);
    void clearDataHandlers();
    void updateData(GraphDataHandlerPtr handler);
    bool setCursorPosition(double x, bool enableTimeBarSync, bool forceTimeChange);
    bool setCursorPositionWithScreenX(double screenX, bool enableTimeBarSync, bool forceTimeChange);

    void setTimeBarSyncMode(bool on);
    void enableTimeBarSync(bool on);
    void setAutoScrollMode(GraphWidget::ScrollMode mode);
    void setVerticalValueRange(double lower, double upper);
    void showLimits(bool show);
    void showRulers(bool show);
    void showGrid(bool show);
    void setGridSize(double width, double height);
    void setControlPointStep(int step, int offset);
    void highlightControlPoints(bool on);
    void changeMode(GraphWidget::Mode mode);
    void changeEditMode(GraphWidget::EditMode mode);
        
    void setupConfiguration();
    void updateDomain();

    bool onFocusInEvent(QFocusEvent* event);
    bool onScreenResizeEvent(QResizeEvent* event);
    bool onScreenMouseButtonPressEvent(QMouseEvent* event);
    bool onScreenMouseButtonReleaseEvent(QMouseEvent* event);
    bool onScreenMouseMoveEvent(QMouseEvent* event);
    bool onScreenKeyPressEvent(QKeyEvent* event);
    bool onScreenKeyReleaseEvent(QKeyEvent* event);
    void zoomScreen(int screenX, int screenY);
    void moveScreen(double screenX, double screenY);
    void startTrajectoryEdit();
    void finishTrajectoryEdit();
    void doTrajectoryEdit(int screenX, int screenY, Qt::KeyboardModifiers modifiers);
    void storeOrgValuesToHistory(int frameBegin, int frameEnd);
    void undoTrajectoryEdit();
    void redoTrajectoryEdit();
    void selectEditTargetByClicking(double screenX, double screenY);
    bool onScreenPaintEvent(QPaintEvent* event);
    void drawTrajectory(QPainter& painter, const QRect& rect, GraphDataHandlerImpl* data);
    void drawLimits(QPainter& painter, GraphDataHandlerImpl* data);
    void updateControlPoints(GraphDataHandlerImpl* data);
    void drawGrid(QPainter& painter);
    void onHScrollbarChanged(double value);
    void onVScrollbarChanged(double value);
    bool storeState(Archive& archive);
    bool restoreState(const Archive& archive);        
};

}


GraphDataHandler::GraphDataHandler()
{
    id = -1;
    impl = new GraphDataHandlerImpl();
}

GraphDataHandlerImpl::GraphDataHandlerImpl()
{
    offset = 0.0;
    stepRatio = 1.0;

    r = 0.0f;
    g = 0.0f;
    b = 0.8f;

    lowerValueLimit = -std::numeric_limits<double>::max();
    upperValueLimit =  std::numeric_limits<double>::max();
    lowerVelocityLimit = -std::numeric_limits<double>::max();
    upperVelocityLimit =  std::numeric_limits<double>::max();

    isControlPointUpdateNeeded = true;

    currentHistory = 0;
}


GraphDataHandler::~GraphDataHandler()
{
    delete impl;
}


void GraphDataHandler::setColor(float r, float g, float b)
{
    impl->r = r;
    impl->g = g;
    impl->b = b;
}


void GraphDataHandler::setLabel(const std::string& label)
{
    impl->label = label;
}


void GraphDataHandler::setFrameProperties(int numFrames, double frameRate, double offset)
{
    impl->values.resize(numFrames + 2);
    impl->numFrames = numFrames;
    impl->stepRatio = 1.0 / frameRate;
    impl->offset = offset;
    impl->isControlPointUpdateNeeded = true;
}


void GraphDataHandler::setValueLimits(double lower, double upper)
{
    impl->lowerValueLimit = lower;
    impl->upperValueLimit = upper;
}


void GraphDataHandler::setVelocityLimits(double lower, double upper)
{
    impl->lowerVelocityLimit = lower;
    impl->upperVelocityLimit = upper;
}


void GraphDataHandler::addVerticalLine(double /* x */, const std::string& /* label */)
{

}


void GraphDataHandler::addHorizontalLine(double /* y */, const std::string& /* label */)
{

}


void GraphDataHandler::clearLines()
{

}

        
void GraphDataHandler::update()
{
    if(TRACE_FUNCTIONS){
        cout << "GraphDataHandler::update()" << endl;
    }
    impl->sigDataUpdated();
}


void GraphDataHandler::setDataRequestCallback(DataRequestCallback callback)
{
    impl->dataRequestCallback = callback;
}


void GraphDataHandler::setDataModifiedCallback(DataModifiedCallback callback)
{
    impl->dataModifiedCallback = callback;
}


GraphWidget::GraphWidget(View* parentView)
{
    impl = new GraphWidgetImpl(this, parentView);

    impl->screen->setBackgroundRole(QPalette::Base);
    impl->screen->setAutoFillBackground(true);
}


GraphWidgetImpl::GraphWidgetImpl(GraphWidget* self, View* parentView)
    : self(self),
      parentView(parentView)
{
    cursorDashes.push_back(8.0);
    cursorDashes.push_back(2.0);
    limitValueDashes.push_back(4.0);
    limitValueDashes.push_back(4.0);

    screen = new QWidget(self);
    screen->setMouseTracking(true);
    screen->installEventFilter(self);

    hScrollbar = new DoubleScrollBar(Qt::Horizontal, self);
    hScrollbar->sigValueChanged().connect(
        [&](double value){ onHScrollbarChanged(value); });
    
    vScrollbar = new DoubleScrollBar(Qt::Vertical, self);
    vScrollbar->sigValueChanged().connect(
        [&](double value){ onVScrollbarChanged(value); });

    QGridLayout* grid = new QGridLayout(self);
    grid->setSpacing(0);
    grid->setContentsMargins(0, 0, 0, 0);
    grid->addWidget(screen, 0, 0);
    grid->addWidget(hScrollbar, 1, 0);
    grid->addWidget(vScrollbar, 0, 1);
    grid->setColumnStretch(0, 1);
    grid->setRowStretch(0, 1);
    grid->setSizeConstraint(QLayout::SetNoConstraint);
    self->setLayout(grid);
    
    isOrgValueVisible = true;
    isVelocityVisible = false;
    isAccelerationVisible = false;
    
    mode = GraphWidget::VIEW_MODE;
    editMode = GraphWidget::FREE_LINE_MODE;
    dragState = DRAG_NONE;
    
    screenMarginX = 3.0;
    scaleX = 200.0;
    scaleY = 100.0;
    leftX = 0.0;
    centerY = 0.0;

    currentScreenX = 0.0;
    currentScreenY = 0.0;

    rangeUpperY = 10.0;
    rangeLowerY = -rangeUpperY;

    lineWidth = 1.0;
    isLimitVisible = true;
    isGridOn = true;
    gridWidth = 0.2;
    gridHeight = 0.2;

    editTarget = 0;
    isControlPointsHighlighted = false;
    controlPointStep = 1;
    controlPointOffset = 0;

    isReconfigurationNeeded = false;

    isDomainUpdateNeeded = true;

    autoScrollMode = GraphWidget::CONTINUOUS;

    timeBar = TimeBar::instance();
    timeBarSyncMode = true;

    parentView->sigActivated().connect([&](){ onActivated(); });
    parentView->sigDeactivated().connect([&](){ onDeactivated(); });

    statusLabel.setAlignment(Qt::AlignLeft);
    QFont font = statusLabel.font();
    font.setFixedPitch(true);
    statusLabel.setFont(font);

    cursorX = -1.0;
}


GraphWidget::~GraphWidget()
{
    delete impl;
}


GraphWidgetImpl::~GraphWidgetImpl()
{
    connections.disconnect();

    if(auto bar = GraphBar::instance()){
        bar->releaseFocus(self);
    }
}


void GraphWidgetImpl::onActivated()
{
    enableTimeBarSync(timeBarSyncMode);
}


void GraphWidgetImpl::onDeactivated()
{
    enableTimeBarSync(false);
}


void GraphWidget::addDataHandler(GraphDataHandlerPtr handler)
{
    impl->addDataHandler(handler);
}


void GraphWidgetImpl::addDataHandler(GraphDataHandlerPtr handler)
{
    GraphDataHandlerImpl* data = handler->impl;
    
    handlers.push_back(handler);
    connections.add(
        handler->impl->sigDataUpdated.connect(
            [this,handler](){ updateData(handler); }));

    isReconfigurationNeeded = true;
    isDomainUpdateNeeded = true;

    updateData(handler);

    if(!editTarget){
        if(data->dataModifiedCallback){
            editTarget = data;
        }
    }
}


void GraphWidget::clearDataHandlers()
{
    impl->clearDataHandlers();
}


void GraphWidgetImpl::clearDataHandlers()
{
    connections.disconnect();
    handlers.clear();

    editTarget = 0;

    isReconfigurationNeeded = true;
    isDomainUpdateNeeded = true;

    screen->update();
}


void GraphWidgetImpl::updateData(GraphDataHandlerPtr handler)
{
    if(TRACE_FUNCTIONS){
        cout << "GraphWidgetImpl::updateData()" << endl;
    }
    
    GraphDataHandlerImpl* data = handler->impl;

    if(data->prevNumValues != data->numFrames){
        isReconfigurationNeeded = true;
        isDomainUpdateNeeded = true;
    }
    
    if(data->dataRequestCallback){
        vector<double>& values = data->values;
        data->dataRequestCallback(0, data->numFrames, &(values[1]));
    }
    screen->update();
}


void GraphWidget::setRenderingTypes
(bool showOriginalValues, bool showVelocities, bool showAccelerations)
{
    impl->isOrgValueVisible = showOriginalValues;
    impl->isVelocityVisible = showVelocities;
    impl->isAccelerationVisible = showAccelerations;
    impl->screen->update();
}


void GraphWidget::getRenderingTypes
(bool& showOriginalValues, bool& showVelocities, bool& showAccelerations)
{
    showOriginalValues = impl->isOrgValueVisible;
    showVelocities = impl->isVelocityVisible;
    showAccelerations = impl->isAccelerationVisible;
}


/**
   @return true if posistion is within the range of the data
*/
bool GraphWidget::setCursorPosition(double x)
{
    return impl->setCursorPosition(x, false, false);
}


bool GraphWidgetImpl::setCursorPosition(double x, bool isTimeBarSyncEnabled, bool forceTimeChange)
{
    if(TRACE_FUNCTIONS){
        cout << "GraphWidgetImpl::setCursorPosition()" << endl;
    }
    
    bool isWithinDomain = true;
    
    if(x < domainLowerX){
        x = domainLowerX;
        isWithinDomain = false;
    } else if(x > domainUpperX){
        x = domainUpperX;
        isWithinDomain = false;
    }

    if(x != cursorX){

        bool doScroll = false;
        const double rightX = leftX + visibleWidth;
        
        if(autoScrollMode == GraphWidget::CONTINUOUS){
            if(x < leftX){
                if(leftX - x < visibleWidth / 3.0){
                    leftX = x;
                } else {
                    leftX = x - visibleWidth / 2.0;
                }
                doScroll = true;
            } else if(x >= rightX){
                if(x - rightX < visibleWidth / 3.0){
                    leftX = x - visibleWidth;
                } else {
                    leftX = x - visibleWidth / 2.0;
                }
                doScroll = true;
            }
        } else if(autoScrollMode == GraphWidget::PAGE){
            if(x < leftX){
                if(leftX - x < visibleWidth / 3.0){
                    leftX -= visibleWidth;
                } else {
                    leftX = x - visibleWidth / 2.0;
                }
                doScroll = true;
            } else if(x >= rightX){
                if(x - rightX < visibleWidth / 3.0){
                    leftX += visibleWidth;
                } else {
                    leftX = x - visibleWidth / 2.0;
                }
                doScroll = true;
            }
        }

        if(doScroll){
            leftX = std::max(domainLowerX, std::min(domainUpperX - visibleWidth, leftX));
            screenCursorX = screenMarginX + (x - leftX) * scaleX;
            isReconfigurationNeeded = true;
            screen->update();
        } else {
            double oldScreenCursorX = screenCursorX;
            screenCursorX = screenMarginX + (x - leftX) * scaleX;
            screen->update((int)oldScreenCursorX - 2, 0, 4, (int)screenHeight);
            screen->update((int)screenCursorX - 2, 0, 4, (int)screenHeight);
        }
        
        cursorX = x;
    }

    if(isTimeBarSyncEnabled && timeBarSyncMode){
        double time = cursorX;
        if(timeBar->time() != time || forceTimeChange){
            timeBar->setTime(time);
        }
    }

    return isWithinDomain;
}


bool GraphWidgetImpl::setCursorPositionWithScreenX(double screenX, bool isTimeBarSyncEnabled, bool forceTimeChange)
{
    return setCursorPosition(leftX + (screenX - screenMarginX) / scaleX, isTimeBarSyncEnabled, forceTimeChange);
}


void GraphWidget::setTimeBarSyncMode(bool on)
{
    impl->setTimeBarSyncMode(on);
}


void GraphWidgetImpl::setTimeBarSyncMode(bool on)
{
    timeBarSyncMode = on;
    enableTimeBarSync(on);
}


void GraphWidgetImpl::enableTimeBarSync(bool on)
{
    if(TRACE_FUNCTIONS){
        cout << "GraphWidgetImpl::enableTimeBarSync()" << endl;
    }
    
    if(timeChangedConnetion.connected()){
        if(!on){
            timeChangedConnetion.disconnect();
        }
    } else {
        if(on && parentView->isActive()){
            timeChangedConnetion =
                timeBar->sigTimeChanged().connect(
                    [&](double time){ return setCursorPosition(time, false, false); });
            setCursorPosition(timeBar->time(), false, false);
        }
    }
}


bool GraphWidget::isTimeBarSyncMode()
{
    return impl->timeBarSyncMode;
}


void GraphWidget::setAutoScrollMode(ScrollMode mode)
{
    impl->setAutoScrollMode(mode);
}


void GraphWidgetImpl::setAutoScrollMode(GraphWidget::ScrollMode mode)
{
    autoScrollMode = mode;
}


GraphWidget::ScrollMode GraphWidget::autoScrollMode()
{
    return impl->autoScrollMode;
}


void GraphWidget::setVerticalValueRange(double lower, double upper)
{
    impl->setVerticalValueRange(lower, upper);
}


void GraphWidgetImpl::setVerticalValueRange(double lower, double upper)
{
    rangeLowerY = lower;
    rangeUpperY = upper;
    isReconfigurationNeeded = true;
    screen->update();
}


void GraphWidget::getVerticalValueRange(double& lower, double& upper)
{
    lower = impl->rangeLowerY;
    upper = impl->rangeUpperY;
}


void GraphWidget::setLineWidth(double width)
{
    impl->lineWidth = width;
    impl->screen->update();
}


double GraphWidget::getLineWidth()
{
    return impl->lineWidth;
}


void GraphWidget::showRulers(bool show)
{
    impl->showRulers(show);
}


void GraphWidgetImpl::showRulers(bool show)
{

}


bool GraphWidget::showsRulers()
{
    return false;
}


void GraphWidget::showLimits(bool show)
{
    impl->showLimits(show);
}


void GraphWidgetImpl::showLimits(bool show)
{
    isLimitVisible = show;
    screen->update();
}


bool GraphWidget::showsLimits()
{
    return impl->isLimitVisible;
}


void GraphWidget::showGrid(bool show)
{
    impl->showGrid(show);
}


void GraphWidgetImpl::showGrid(bool show)
{
    isGridOn = show;
    isReconfigurationNeeded = true;
    screen->update();
}


bool GraphWidget::showsGrid()
{
    return impl->isGridOn;
}


void GraphWidget::setGridSize(double width, double height)
{
    impl->setGridSize(width, height);
}


void GraphWidgetImpl::setGridSize(double width, double height)
{
    gridWidth = width;
    gridHeight = height;
    if(isGridOn){
        isReconfigurationNeeded = true;
        screen->update();
    }
}


void GraphWidget::getGridSize(double& width, double& height)
{
    width = impl->gridWidth;
    height = impl->gridHeight;
}


void GraphWidget::setControlPointStep(int step, int offset)
{
    impl->setControlPointStep(step, offset);
}


void GraphWidgetImpl::setControlPointStep(int step, int offset)
{
    bool changed = false;
    
    if(step != controlPointStep){
        controlPointStep = step;
        changed = true;
    }
    if(offset != controlPointOffset){
        controlPointOffset = offset;
        changed = true;
    }

    if(changed){
        for(size_t i=0; i < handlers.size(); ++i){
            handlers[i]->impl->isControlPointUpdateNeeded = true;
        }
        screen->update();
    }
}


void GraphWidget::getControlPointStep(int& step, int& offset)
{
    step = impl->controlPointStep;
    offset = impl->controlPointOffset;
}


void GraphWidget::highlightControlPoints(bool on)
{
    impl->highlightControlPoints(on);
}


void GraphWidgetImpl::highlightControlPoints(bool on)
{
    isControlPointsHighlighted = on;
    screen->update();
}


bool GraphWidget::highlightsControlPoints()
{
    return impl->isControlPointsHighlighted;
}


void GraphWidget::changeMode(Mode mode)
{
    impl->changeMode(mode);
}


void GraphWidgetImpl::changeMode(GraphWidget::Mode mode)
{
    if(mode != this->mode){
    
        if(mode == GraphWidget::EDIT_MODE){
            for(size_t i=0; i < handlers.size(); ++i){
                updateControlPoints(handlers[i]->impl);
            }
        }

        screen->update();
    }
}


GraphWidget::Mode GraphWidget::mode()
{
    return impl->mode;
}



void GraphWidget::changeEditMode(EditMode mode)
{
    impl->changeEditMode(mode);
}


void GraphWidgetImpl::changeEditMode(GraphWidget::EditMode mode)
{
    editMode = mode;
}


GraphWidget::EditMode GraphWidget::editMode()
{
    return impl->editMode;
}


QLabel& GraphWidget::statusLabel()
{
    return impl->statusLabel;
}


void GraphWidgetImpl::setupConfiguration()
{
    if(isDomainUpdateNeeded){
        updateDomain();
    }
    
    visibleWidth = (screenWidth - screenMarginX * 2) / scaleX;

    double rightX = leftX + visibleWidth;
    if(rightX > domainUpperX){
        leftX -= (rightX - domainUpperX);
    }

    if(leftX < domainLowerX){
        leftX = domainLowerX;
    }

    screenCenterY = screenHeight / 2.0;

    visibleHeight = screenHeight / scaleY;
    visibleHeightHalf = visibleHeight / 2.0;
    topY = centerY - visibleHeightHalf;
    bottomY = centerY + visibleHeightHalf;

    hScrollbar->blockSignals(true);

    if(domainUpperX == domainLowerX){
        hScrollbar->setRange(domainLowerX, domainLowerX + 1.0);
        hScrollbar->setPageStep(1.0);
    } else {
        hScrollbar->setRange(domainLowerX, domainUpperX);
        hScrollbar->setPageStep(visibleWidth);
    }
    hScrollbar->setSingleStep(visibleWidth / 100.0);
    
    hScrollbar->setValue(leftX);
    hScrollbar->blockSignals(false);

    vScrollbar->blockSignals(true);
    vScrollbar->setRange(rangeLowerY, rangeUpperY);
    vScrollbar->setPageStep(visibleHeight);
    vScrollbar->setSingleStep(visibleHeight / 100.0);
    vScrollbar->setValue(topY);
    vScrollbar->blockSignals(false);

    //hRuler.setRange(leftX, rightX, cursorX, std::numeric_limits<double>::max());
    //vRuler.setRange(-topY, -bottomY, 0, std::numeric_limits<double>::max());

    isReconfigurationNeeded = false;
}


void GraphWidgetImpl::updateDomain()
{
    double tmpDomainLowerX =  std::numeric_limits<double>::max();
    double tmpDomainUpperX = -std::numeric_limits<double>::max();

    for(size_t i=0; i < handlers.size(); ++i){
        GraphDataHandlerImpl* data = handlers[i]->impl;

        if(data->offset < tmpDomainLowerX){
            tmpDomainLowerX = data->offset;
        }

        double upper = data->offset + data->numFrames * data->stepRatio;
        if(upper > tmpDomainUpperX){
            tmpDomainUpperX = upper;
        }
    }
    
    if(tmpDomainLowerX == std::numeric_limits<double>::max()){
        domainLowerX = 0.0;
//        domainUpperX = 10.0;
        domainUpperX = 0.0;
    } else {
        domainLowerX = tmpDomainLowerX;
        domainUpperX = tmpDomainUpperX;
    }

    isDomainUpdateNeeded = false;
}


bool GraphWidget::eventFilter(QObject* obj, QEvent* event)
{
    if(obj == impl->screen){
        switch(event->type()){
        case QEvent::FocusIn:
            return impl->onFocusInEvent(static_cast<QFocusEvent*>(event));
        case QEvent::Paint:
            return impl->onScreenPaintEvent(static_cast<QPaintEvent*>(event));
        case QEvent::Resize:
            return impl->onScreenResizeEvent(static_cast<QResizeEvent*>(event));
        case QEvent::MouseMove:
            return impl->onScreenMouseMoveEvent(static_cast<QMouseEvent*>(event));
        case QEvent::MouseButtonPress:
            return impl->onScreenMouseButtonPressEvent(static_cast<QMouseEvent*>(event));
        case QEvent::MouseButtonRelease:
            return impl->onScreenMouseButtonReleaseEvent(static_cast<QMouseEvent*>(event));
        case QEvent::KeyPress:
            return impl->onScreenKeyPressEvent(static_cast<QKeyEvent*>(event));
        case QEvent::KeyRelease:
            return impl->onScreenKeyReleaseEvent(static_cast<QKeyEvent*>(event));
        default:
            return false;
        }
    }
    return QWidget::eventFilter(obj, event);
}


bool GraphWidgetImpl::onFocusInEvent(QFocusEvent*)
{
    if(auto bar = GraphBar::instance()){
        bar->focus(self);
    }
    return false;
}


bool GraphWidgetImpl::onScreenResizeEvent(QResizeEvent* event)
{
    screenWidth = event->size().width();
    screenHeight = event->size().height();
    setupConfiguration();
    return false;
}


bool GraphWidgetImpl::onScreenMouseButtonPressEvent(QMouseEvent* event)
{
    screen->setFocus(Qt::MouseFocusReason);
    
    dragState = DRAG_NONE;
    
    pressedScreenX = event->x();
    pressedScreenY = event->y();
    dragPrevScreenX = event->x();
    dragPrevScreenY = event->y();

    dragOrgLeftX = leftX;
    dragOrgCenterY = centerY;

    if(event->button() == Qt::LeftButton){
        if(event->modifiers() & Qt::ControlModifier){
            selectEditTargetByClicking(pressedScreenX, pressedScreenY);
        } else if(mode == GraphWidget::EDIT_MODE && editTarget) {
            startTrajectoryEdit();
            doTrajectoryEdit((int)pressedScreenX, (int)pressedScreenY, event->modifiers());
        } else {
            dragState = DRAG_CURSOR;
            setCursorPositionWithScreenX(pressedScreenX, true, false);
        }
            
    } else if(event->button() == Qt::MidButton){
        dragState = DRAG_ZOOM;
        dragOrgScaleX = scaleX;
        dragOrgScaleY = scaleY;
    } else if(event->button() == Qt::RightButton){
        dragState = DRAG_MOVE;
    }
    
    return true;
}


bool GraphWidgetImpl::onScreenMouseButtonReleaseEvent(QMouseEvent*)
{
    if(dragState == DRAG_EDIT){
        finishTrajectoryEdit();
    }
    
    dragState = DRAG_NONE;

    return true;
}


bool GraphWidgetImpl::onScreenMouseMoveEvent(QMouseEvent* event)
{
    currentScreenX = event->x();
    currentScreenY = event->y();

    switch(dragState){

    case DRAG_CURSOR:
        setCursorPositionWithScreenX(currentScreenX, true, false);
        break;

    case DRAG_EDIT:
        doTrajectoryEdit(currentScreenX, currentScreenY, event->modifiers());
        break;

    case DRAG_ZOOM:
        zoomScreen(currentScreenX, currentScreenY);
        break;

    case DRAG_MOVE:
        moveScreen(currentScreenX, currentScreenY);
        break;

    default:
        break;
    }

    double x = leftX + (currentScreenX - screenMarginX) / scaleX;
    double y = - centerY - (currentScreenY - screenCenterY) / scaleY;

    //hRuler.property_position() = x;
    //vRuler.property_position() = y;

    statusLabel.setText(
        fmt::format(_("Graph: Position = ({0:.5f}, {1:.5f})"), x, y).c_str());

    dragPrevScreenX = currentScreenX;
    dragPrevScreenY = currentScreenY;
    
    return true;
}


bool GraphWidgetImpl::onScreenKeyPressEvent(QKeyEvent* event)
{
    bool unprocessed = false;
    
    if(dragState == DRAG_EDIT){

        switch(event->key()){

        case Qt::Key_Shift:
            doTrajectoryEdit(dragPrevScreenX, dragPrevScreenY, event->modifiers() & Qt::ShiftModifier);
            break;

        default:
            unprocessed = true;
            break;
        }

    } else if(dragState == DRAG_NONE) {
        switch(event->key()){

        case Qt::Key_Z:
            if(event->modifiers() & Qt::ControlModifier){
                if(event->modifiers() & Qt::ShiftModifier){
                    redoTrajectoryEdit();
                } else {
                    undoTrajectoryEdit();
                }
            }
            break;
            
        case Qt::Key_F:
            changeEditMode(GraphWidget::FREE_LINE_MODE);
            break;
            
        case Qt::Key_L:
            changeEditMode(GraphWidget::LINE_MODE);
            break;

        case Qt::Key_Space:
        {
            QMouseEvent mouseEvent(QEvent::MouseButtonPress,
                                   QPoint(currentScreenX, currentScreenY),
                                   Qt::MidButton, Qt::MidButton,
                                   event->modifiers());
            unprocessed = !onScreenMouseButtonPressEvent(&mouseEvent);
        }
        break;
        
        default:
            unprocessed = true;
            break;
        }
    }
    
    return !unprocessed;
}


bool GraphWidgetImpl::onScreenKeyReleaseEvent(QKeyEvent* event)
{
    bool unprocessed = false;
    
    switch(event->key()){

    case Qt::Key_Shift:
        if(dragState == DRAG_EDIT){
            doTrajectoryEdit(dragPrevScreenX, dragPrevScreenY, event->modifiers() & Qt::ShiftModifier);
        }
        break;

    case Qt::Key_Space:
    {
        QMouseEvent mouseEvent(QEvent::MouseButtonPress,
                               QPoint(currentScreenX, currentScreenY),
                               Qt::MidButton, Qt::MidButton,
                               event->modifiers());
        unprocessed = !onScreenMouseButtonReleaseEvent(&mouseEvent);
    }
    break;

    default:
        unprocessed = true;
        break;
    }
    
    return unprocessed;
}


void GraphWidgetImpl::zoomScreen(int screenX, int screenY)
{
    double zoomRatioX = pow(1.01, screenX - pressedScreenX);
    double zoomRatioY = pow(1.01, pressedScreenY - screenY);
    
    scaleX = dragOrgScaleX * zoomRatioX;
    scaleY = dragOrgScaleY * zoomRatioY;
    
    double dpx = (pressedScreenX - screenMarginX) / dragOrgScaleX;
    double dx = dpx * (zoomRatioX - 1.0) / zoomRatioX;
    
    leftX = dragOrgLeftX + dx;
    if(leftX < domainLowerX){
        leftX = domainLowerX;
    }
    
    double dpy = (pressedScreenY - screenCenterY) / dragOrgScaleY;
    double dy = dpy * (zoomRatioY - 1.0) / zoomRatioY;
    centerY = dragOrgCenterY + dy;
    
    isReconfigurationNeeded = true;
    screen->update();
}


void GraphWidgetImpl::moveScreen(double screenX, double screenY)
{
    leftX = dragOrgLeftX + (pressedScreenX - screenX) / scaleX;
    centerY = dragOrgCenterY + (pressedScreenY - screenY) / scaleY;
    isReconfigurationNeeded = true;
    screen->update();
}


void GraphWidgetImpl::startTrajectoryEdit()
{
    if((controlPointStep == 1) || editMode != GraphWidget::FREE_LINE_MODE){
        isEditBufferedUpdateMode = timeBar->isDoingPlayback();
        editedFrameBegin = std::numeric_limits<int>::max();
        editedFrameEnd = std::numeric_limits<int>::min();
        editTarget->editHistories.push_back(EditHistoryPtr(new EditHistory()));
        editTarget->currentHistory = editTarget->editHistories.size();
        dragState = DRAG_EDIT;
    }
}


void GraphWidgetImpl::finishTrajectoryEdit()
{
    EditHistoryList& histories = editTarget->editHistories;
    //vector<double>& values = editTarget->values;
    double* values = &editTarget->values[1];

    if(!(editedFrameBegin < editedFrameEnd)){
        histories.pop_back();
    } else {
        if(isEditBufferedUpdateMode){
            editTarget->dataModifiedCallback
                (editedFrameBegin, editedFrameEnd - editedFrameBegin, &values[editedFrameBegin]);
        }

        // set new values (for redo) to history
        vector<double>& newValues = histories.back()->newValues;
        newValues.resize(editedFrameEnd - editedFrameBegin);
        std::copy(&values[editedFrameBegin], &values[editedFrameEnd], newValues.begin());
        while(histories.size() > MAX_NUM_HISTORIES){
            histories.pop_front();
            editTarget->currentHistory = histories.size();
        }
    }
}


void GraphWidgetImpl::doTrajectoryEdit(int screenX, int screenY, Qt::KeyboardModifiers modifiers)
{
    static const int FROM = 0;
    static const int TO   = 1;
    
    double x[2];
    double y[2];
    double frameAtX[2];
    double flooredFrameAtX[2];
    int frameToEdit[2];

    int orgScreenX, orgScreenY;
    if(editMode == GraphWidget::FREE_LINE_MODE){
        orgScreenX = dragPrevScreenX;
        orgScreenY = dragPrevScreenY;
    } else {
        orgScreenX = pressedScreenX;
        orgScreenY = pressedScreenY;
    }

    //x[FROM] = leftX + (orgScreenX - screenMarginX) / scaleX;
    //x[TO]   = leftX + (screenX - screenMarginX) / scaleX;
    x[FROM] = dragOrgLeftX + (orgScreenX - screenMarginX) / scaleX;
    x[TO]   = dragOrgLeftX + (screenX - screenMarginX) / scaleX;
    y[FROM] = - centerY - (orgScreenY - screenCenterY) / scaleY;
    y[TO]   = - centerY - (screenY - screenCenterY) / scaleY;

    for(int i=0; i < 2; ++i){
        frameAtX[i] = x[i] / editTarget->stepRatio;
        flooredFrameAtX[i] = floor(frameAtX[i]);
        frameToEdit[i] = static_cast<int>(flooredFrameAtX[i]) + 1;
    }

    bool draggedLeftToRight = (x[FROM] <= x[TO]);
    int LEFT = draggedLeftToRight ? 0 : 1;
    int RIGHT = draggedLeftToRight ? 1 : 0;
    
    int& frameBegin = frameToEdit[LEFT];
    int& frameEnd = frameToEdit[RIGHT];

    bool inAdjacentFrames = (frameBegin == frameEnd);
    if(inAdjacentFrames){
        double odd = frameAtX[TO] - flooredFrameAtX[TO];
        if(odd >= 0.5){
            frameEnd++;
        } else if(odd < 0.5){
            frameBegin--;
        }
    }

    if(editMode == GraphWidget::LINE_MODE && !inAdjacentFrames){
        if(draggedLeftToRight){
            if(frameAtX[LEFT] - flooredFrameAtX[LEFT] >= 0.5){
                frameBegin++;
            }
            if(frameAtX[RIGHT] - flooredFrameAtX[RIGHT] >= 0.5){
                frameEnd++;
            }
        } else {
            if(frameAtX[LEFT] - flooredFrameAtX[LEFT] < 0.5){
                frameBegin--;
            }
            if(frameAtX[RIGHT] - flooredFrameAtX[RIGHT] < 0.5){
                frameEnd--;
            }
        }
    }
    
    for(int i=0; i < 2; ++i){
        if(frameToEdit[i] < 0){
            frameToEdit[i] = 0;
        }
    }
    //vector<double>& values = editTarget->values;
    double* values = &editTarget->values[1];
    const int numFrames = editTarget->numFrames;
    
    if(frameEnd > numFrames){
        frameEnd = numFrames;
    }

    if(editMode == GraphWidget::LINE_MODE){
        EditHistoryPtr& history = editTarget->editHistories.back();
        std::copy(history->orgValues.begin(), history->orgValues.end(), &values[history->frame]);
    }

    if(frameBegin < frameEnd){

        storeOrgValuesToHistory(frameBegin, frameEnd);

        if(inAdjacentFrames){
            values[frameBegin] = min(max(y[TO], editTarget->lowerValueLimit), editTarget->upperValueLimit);

        } else if(editMode == GraphWidget::FREE_LINE_MODE){
            double k = (y[RIGHT] - y[LEFT]) / (frameAtX[RIGHT] - frameAtX[LEFT]);
            for(int frame = frameBegin; frame < frameEnd; ++frame){
                double v = k * (frame - frameAtX[LEFT]) + y[LEFT];
                values[frame] = min(max(v, editTarget->lowerValueLimit), editTarget->upperValueLimit);
            }

        } else if(editMode == GraphWidget::LINE_MODE){
            double k;
            double y0;
            int frame0;
            if(modifiers & Qt::ShiftModifier){
                k = 0.0;
                y0 = y[TO];
                frame0 = frameBegin;
            } else {
                if(draggedLeftToRight){
                    frame0 = frameBegin - 1;
                } else {
                    frame0 = frameBegin;
                }
                k = (y[RIGHT] - y[LEFT]) / (frameEnd - frameBegin);
                y0 = y[LEFT];
            }
            
            for(int frame = frameBegin; frame < frameEnd; ++frame){
                double v = k * (frame - frame0) + y0;
                values[frame] = min(max(v, editTarget->lowerValueLimit), editTarget->upperValueLimit);
            }
        }

        editedFrameBegin = std::min(editedFrameBegin, frameBegin);
        editedFrameEnd = std::max(editedFrameEnd, frameEnd);

        if(!isEditBufferedUpdateMode){
            editTarget->dataModifiedCallback(frameBegin, frameEnd - frameBegin, &values[frameBegin]);
        }
    }

    screen->update();


    double cursorX;
    if(TO==RIGHT && frameToEdit[LEFT] < frameToEdit[RIGHT]){
        cursorX = (frameToEdit[TO] - 1.0 + 0.000001) * editTarget->stepRatio;
    } else {
        cursorX = (frameToEdit[TO] + 0.000001) * editTarget->stepRatio;
    }        

    setCursorPosition(cursorX, true, true);
}


void GraphWidgetImpl::storeOrgValuesToHistory(int frameBegin, int frameEnd)
{
    //vector<double>& values = editTarget->values;
    double* values = &editTarget->values[1];
    EditHistoryPtr& history = editTarget->editHistories.back();
    vector<double>& storedValues = history->orgValues;
    int nStoredValues = storedValues.size();
    if(!nStoredValues){
        history->frame = frameBegin;
    }

    int leftPartBegin = std::min(frameBegin, history->frame);
    int leftPartEnd = std::min(frameEnd, history->frame);
    int rightPartBegin = std::max(frameBegin, history->frame + nStoredValues);
    int rightPartEnd = std::max(frameEnd, history->frame + nStoredValues);

    history->frame = leftPartBegin;
    storedValues.resize(rightPartEnd - leftPartBegin);

    if(leftPartBegin < leftPartEnd){
        std::copy(&values[leftPartBegin], &values[leftPartEnd], storedValues.begin());
    }
    if(rightPartBegin < rightPartEnd){
        std::copy(&values[rightPartBegin], &values[rightPartEnd],
                  storedValues.begin() + (rightPartBegin - history->frame));
    }
}


void GraphWidgetImpl::undoTrajectoryEdit()
{
    if(dragState == DRAG_NONE && editTarget){
        int& currentHistory = editTarget->currentHistory;
        if(currentHistory > 0){
            currentHistory--;
            EditHistoryPtr history = editTarget->editHistories[currentHistory];
            std::copy(history->orgValues.begin(), history->orgValues.end(),
                      editTarget->values.begin() + history->frame + 1);
            editTarget->dataModifiedCallback(history->frame, history->orgValues.size(), &history->orgValues[0]);
            screen->update();
        }
    }
}


void GraphWidgetImpl::redoTrajectoryEdit()
{
    if(dragState == DRAG_NONE && editTarget){
        EditHistoryList& histories = editTarget->editHistories;
        int& currentHistory = editTarget->currentHistory;
        if(currentHistory < static_cast<int>(histories.size())){
            EditHistoryPtr history = editTarget->editHistories[currentHistory];
            std::copy(history->newValues.begin(), history->newValues.end(),
                      editTarget->values.begin() + history->frame + 1);
            editTarget->dataModifiedCallback(history->frame, history->newValues.size(), &history->newValues[0]);
            currentHistory++;
            screen->update();
        }
    }
}


void GraphWidgetImpl::selectEditTargetByClicking(double screenX, double screenY)
{
    static const double pixelMargin = 6.0;
    
    GraphDataHandlerImpl* newTarget = 0;
    
    double pointerX = leftX + (screenX - screenMarginX) / scaleX;
    double pointerY = - centerY - (screenY - screenCenterY) / scaleY;
    double yMargin = pixelMargin / scaleY;

    for(size_t i=0; i < handlers.size(); ++i){
        GraphDataHandlerImpl* data = handlers[i]->impl;
        double* values = &data->values[1];
        const int size = data->numFrames;
        int frameAtPointer = (int)floor(pointerX / data->stepRatio);
        int frameMargin = (int)std::max(pixelMargin / scaleX / data->stepRatio, 1.0);
        int frameBegin = std::max(frameAtPointer - frameMargin, 0);
        int frameEnd = std::min(frameAtPointer + frameMargin, size);
        double yMin =  std::numeric_limits<double>::max();
        double yMax = -std::numeric_limits<double>::max();
        for(int j=frameBegin; j < frameEnd; ++j){
            double y = values[j];
            yMin = std::min(yMin, y - yMargin);
            yMax = std::max(yMax, y + yMargin);
        }

        if((pointerY >= yMin) && (pointerY <= yMax)){
            newTarget = data;
            break;
        }
    }

    // unselect operation
    if(newTarget == editTarget){
        newTarget = 0;
    }
    GraphWidget::Mode newMode = newTarget ? GraphWidget::EDIT_MODE : GraphWidget::VIEW_MODE;
    
    if(newMode != mode){
        editTarget = newTarget;
        changeMode(newMode);
    } else {
        if(newTarget != editTarget){
            screen->update();
        }
        editTarget = newTarget;
    }
}


bool GraphWidgetImpl::onScreenPaintEvent(QPaintEvent* event)
{
    if(isReconfigurationNeeded){
        setupConfiguration();
    }

    QPainter painter(screen);

    //painter.setRenderHint(QPainter::Antialiasing);
    painter.setClipRegion(event->region());
    painter.setClipping(true);

    pen.setStyle(Qt::SolidLine);
    pen.setWidthF(1.0);
    painter.setPen(pen);

    // draw the grid
    if(isGridOn){
        drawGrid(painter);
    }

    // draw x-axis
    pen.setColor(QColor(0, 0, 0));
    painter.setPen(pen);
    double ay = screenCenterY - centerY * scaleY;
    painter.drawLine(QPointF(0.0, ay), QPointF(screenWidth, ay));

    if(isLimitVisible){
        for(size_t i=0; i < handlers.size(); ++i){
            drawLimits(painter, handlers[i]->impl);
        }
    }

    pen.setWidthF(lineWidth);
    painter.setPen(pen);
    
    for(size_t i=0; i < handlers.size(); ++i){
        GraphDataHandlerImpl* data = handlers[i]->impl;
        drawTrajectory(painter, event->rect(), data);
    }

    pen.setWidthF(1.0);
    painter.setPen(pen);
    
    // draw the cursor
    pen.setColor(QColor(25, 25, 25, 230));
    pen.setDashPattern(cursorDashes);
    painter.setPen(pen);
    painter.drawLine(QPointF(screenCursorX, 0.0), QPointF(screenCursorX, screenHeight));

    //return true;
    return false;
}

#include <iomanip>


static inline double calcVelocity(int frame, double* values, double stepRatio2)
{
    return (values[frame + 1] - values[frame - 1]) / stepRatio2;
}


void GraphWidgetImpl::drawTrajectory
(QPainter& painter, const QRect& rect, GraphDataHandlerImpl* data)
{
    double areaLeft = rect.x();
    double areaRight = rect.x() + rect.width();
    if(areaLeft < screenMarginX){
        areaLeft = screenMarginX;
    }
    if(areaRight > screenWidth - screenMarginX){
        areaRight = screenWidth - screenMarginX;
    }

    double clippedLeftX = leftX + (areaLeft - screenMarginX) / scaleX;
    int frame_begin = (int)floor(clippedLeftX / data->stepRatio);

    const double xratio = data->stepRatio * scaleX;

    double interFrameOffset = fmod(clippedLeftX / data->stepRatio, 1.0) * xratio;
    double screenOffsetX = areaLeft - interFrameOffset;

    double clippedRightX = leftX + (areaRight - screenMarginX) / scaleX;
    int frame_end = (int)floor(clippedRightX / data->stepRatio) + 2;

    //vector<double>& values = data->values;
    double* values = &data->values[1];
    const int numFrames = data->numFrames;

    if(frame_end > numFrames){
        frame_end = numFrames;
    }

    if(frame_begin < frame_end){

        QColor color;
        
        if(isVelocityVisible){

            values[-1] = values[0];
            values[numFrames] = values[numFrames - 1];
            const double stepRatio2 = 2.0 * data->stepRatio;

            int frame = frame_begin;
            if(frame < 0){
                frame = 0;
            }
            color.setRgbF((data->g + 0.5) / 2.0, (data->b + 0.2) / 2.0, data->r);
            pen.setColor(color);
            painter.setPen(pen);

            if(xratio > 0.2){
                const int n = frame_end - frame;
                polyline.resize(n);
                for(int i=0; i < n; ++i){ 
                    const double px = screenOffsetX + (frame - frame_begin) * xratio;
                    const double v = calcVelocity(frame, values, stepRatio2);
                    const double py = screenCenterY - (v + centerY) * scaleY;
                    polyline[i] = QPointF(px, py);
                    ++frame;
                }
            } else {
                const int m = (int)(0.5 / xratio);
                const int n = ceil(double(frame_end - frame) / m);
                polyline.resize(n * 2);
                for(int i=0; i < n; ++i){
                    double px_min = screenOffsetX + (frame - frame_begin) * xratio;
                    double px_max = px_min;
                    const int next = std::min(frame + m, frame_end);
                    double min = calcVelocity(frame++, values, stepRatio2);
                    double max = min;
                    while(frame < next){
                        const double v = calcVelocity(frame, values, stepRatio2);
                        if(v > max){
                            max = v;
                            px_max = screenOffsetX + (frame - frame_begin) * xratio;
                        } else if(v < min){
                            min = v;
                            px_min = screenOffsetX + (frame - frame_begin) * xratio;
                        }
                        frame++;
                    }
                    const double upper = screenCenterY - (max + centerY) * scaleY;
                    const double lower = screenCenterY - (min + centerY) * scaleY;

                    if(px_min <= px_max){
                        polyline[i*2] = QPointF(px_min, lower);
                        polyline[i*2+1] = QPointF(px_max, upper);
                    } else {
                        polyline[i*2] = QPointF(px_max, upper);
                        polyline[i*2+1] = QPointF(px_min, lower);
                    }
                }
            }

            painter.drawPolyline(polyline);
        }

        if(isOrgValueVisible){

            int frame = frame_begin;
            if(frame < 0){
                frame = 0;
            }
            color.setRgbF(data->r, data->g, data->b);
            pen.setColor(color);
            painter.setPen(pen);
            
            if(xratio > 0.2){
                const int n = frame_end - frame;
                polyline.resize(n);
                for(int i=0; i < n; ++i){
                    const double px = screenOffsetX + (frame - frame_begin) * xratio;
                    const double py = screenCenterY - (values[frame] + centerY) * scaleY;
                    polyline[i] = QPointF(px, py);
                    ++frame;
                }
            } else {
                const int m = (int)(0.5 / xratio);
                const int n = ceil(double(frame_end - frame) / m);
                polyline.resize(n * 2);
                for(int i=0; i < n; ++i){
                    double px_min = screenOffsetX + (frame - frame_begin) * xratio;
                    double px_max = px_min;
                    const int next = std::min(frame + m, frame_end);
                    double min = values[frame++];
                    double max = min;
                    while(frame < next){
                        const double v = values[frame++];
                        if(v > max){
                            max = v;
                            px_max = screenOffsetX + (frame - frame_begin) * xratio;
                        } else if(v < min){
                            min = v;
                            px_min = screenOffsetX + (frame - frame_begin) * xratio;
                        }
                    }
                    const double upper = screenCenterY - (max + centerY) * scaleY;
                    const double lower = screenCenterY - (min + centerY) * scaleY;
                    if(px_min <= px_max){
                        polyline[i*2] = QPointF(px_min, lower);
                        polyline[i*2+1] = QPointF(px_max, upper);
                    } else {
                        polyline[i*2] = QPointF(px_max, upper);
                        polyline[i*2+1] = QPointF(px_min, lower);
                    }
                }
            }

            painter.drawPolyline(polyline);

            if(isControlPointsHighlighted || (mode == GraphWidget::EDIT_MODE && data == editTarget)){
                
                updateControlPoints(data);
                
                int frame = frame_begin;
                if(frame < 0){
                    frame = 0;
                }
                const auto& mask = data->controlPointMask;
                while(frame < frame_end){
                    if(mask[frame]){
                        double px = screenOffsetX + (frame - frame_begin) * xratio;
                        double py = screenCenterY - (values[frame] + centerY) * scaleY;
                        painter.fillRect(QRectF(px - 2, py - 2, 4, 4), color);
                    }
                    ++frame;
                }
            }
        }
    }
}


void GraphWidgetImpl::updateControlPoints(GraphDataHandlerImpl* data)
{
    if(data->isControlPointUpdateNeeded){
        data->isControlPointUpdateNeeded = false;
        const int numFrames = data->numFrames;
        data->controlPointMask.resize(numFrames);
        for(int i=0; i < numFrames; ++i){
            data->controlPointMask[i] = (i % controlPointStep == controlPointOffset);
        }
    }
}


void GraphWidgetImpl::drawLimits(QPainter& painter, GraphDataHandlerImpl* data)
{
    pen.setDashPattern(limitValueDashes);

    QColor color;

    if(isVelocityVisible){
        color.setRgbF((data->g + 0.5) / 2.0, (data->b + 0.2) / 2.0, data->r);
        pen.setColor(color);
        painter.setPen(pen);
        
        double lower = screenCenterY - (data->lowerVelocityLimit + centerY) * scaleY;
        if(isfinite(lower)){
            painter.drawLine(QPointF(screenMarginX, lower), QPointF(screenWidth - screenMarginX, lower));
        }
        double upper = screenCenterY - (data->upperVelocityLimit + centerY) * scaleY;
        if(isfinite(upper)){
            painter.drawLine(QPointF(screenMarginX, upper), QPointF(screenWidth - screenMarginX, upper));
        }
    }
    
    if(isOrgValueVisible){

        color.setRgbF(data->r, data->g, data->b);
        pen.setColor(color);
        painter.setPen(pen);
        
        double lower = screenCenterY - (data->lowerValueLimit + centerY) * scaleY;
        if(isfinite(lower)){
            painter.drawLine(QPointF(screenMarginX, lower), QPointF(screenWidth - screenMarginX, lower));
        }
        double upper = screenCenterY - (data->upperValueLimit + centerY) * scaleY;
        if(isfinite(upper)){
            painter.drawLine(QPointF(screenMarginX, upper), QPointF(screenWidth - screenMarginX, upper));
        }
    }

    pen.setStyle(Qt::SolidLine);
}


        
void GraphWidgetImpl::drawGrid(QPainter& painter)
{
    QColor color;
    color.setRgbF(0.8, 0.8, 0.8);
    pen.setColor(color);
    painter.setPen(pen);

    double x = leftX - fmod(leftX, gridWidth);
    while(x <= leftX + visibleWidth){
        double px = (x - leftX) * scaleX + screenMarginX;
        painter.drawLine(QPointF(px, 0.0), QPointF(px, screenHeight));
        x += gridWidth;
    }

    double y = topY - fmod(topY, gridHeight);
    while(y <= bottomY){
        double py = (y - topY) * scaleY;
        painter.drawLine(QPointF(screenMarginX, py), QPointF(screenWidth - screenMarginX, py));
        y += gridHeight;
    }
}


void GraphWidgetImpl::onHScrollbarChanged(double value)
{
    if(value != leftX){
        leftX = value;
        isReconfigurationNeeded = true;
        screen->update();
    }
}


void GraphWidgetImpl::onVScrollbarChanged(double value)
{
    double y = value + visibleHeightHalf;
    if(y != centerY){
        centerY = y;
        isReconfigurationNeeded = true;
        screen->update();
    }
}


bool GraphWidget::saveImage(const std::string& filename)
{
    QPixmap pixmap(impl->screen->size());
    impl->screen->render(&pixmap);
    pixmap.save(filename.c_str());
    return true;
}


bool GraphWidget::storeState(Archive& archive)
{
    return impl->storeState(archive);
}


bool GraphWidgetImpl::storeState(Archive& archive)
{
    static const char* modeLabel[] = { "view", "edit" };
    archive.write("mode", modeLabel[mode]);

    static const char* editModeLabel[] = { "freeLine", "line" };
    archive.write("editMode", editModeLabel[editMode]);
    
    archive.write("original", isOrgValueVisible);
    archive.write("velocity", isVelocityVisible);
    archive.write("acceleration", isAccelerationVisible);
    archive.write("limits", isLimitVisible);
    archive.write("grid", isGridOn);
    archive.write("gridWidth", gridWidth);
    archive.write("gridHeight", gridHeight);
    archive.write("lineWidth", lineWidth);
    archive.write("rulers", self->showsRulers());
    archive.write("sync", timeBarSyncMode);
    archive.write("controlPointStep", controlPointStep);
    archive.write("controlPointOffset", controlPointOffset);
    archive.write("controlPointHeighlight", isControlPointsHighlighted);

    static const char* scrollModeLabel[] = { "off", "continuous", "page" };
    archive.write("scrollMode", scrollModeLabel[autoScrollMode]);
    
    archive.write("lower", rangeLowerY);
    archive.write("upper", rangeUpperY);

    return true;
}


bool GraphWidget::restoreState(const Archive& archive)
{
    return impl->restoreState(archive);
}


bool GraphWidgetImpl::restoreState(const Archive& archive)
{
    string modeLabel;
    if(archive.read("mode", modeLabel)){
        changeMode((modeLabel == "edit") ? GraphWidget::EDIT_MODE : GraphWidget::VIEW_MODE);
    }
    string editModeLabel;
    if(archive.read("editMode", editModeLabel)){
        if(editModeLabel == "freeLine"){
            editMode = GraphWidget::FREE_LINE_MODE;
        } else if(editModeLabel == "line"){
            editMode = GraphWidget::LINE_MODE;
        }
    }
        
    archive.read("original", isOrgValueVisible);
    archive.read("velocity", isVelocityVisible);
    archive.read("acceleration", isAccelerationVisible);
    archive.read("limits", isLimitVisible);
    archive.read("grid", isGridOn);
    archive.read("gridWidth", gridWidth);
    archive.read("gridHeight", gridHeight);
    archive.read("lineWidth", lineWidth);
    self->showRulers(archive.get("rulers", self->showsRulers()));
    setTimeBarSyncMode(archive.get("sync", timeBarSyncMode));

    int step = archive.get("controlPointStep", controlPointStep);
    int offset = archive.get("controlPointOffset", controlPointOffset);
    setControlPointStep(step, offset);
    archive.read("controlPointHeighlight", isControlPointsHighlighted);

    string scrollModeLabel;
    if(archive.read("scrollMoe", scrollModeLabel)){
        if(scrollModeLabel == "off"){
            setAutoScrollMode(GraphWidget::OFF);
        } else if(scrollModeLabel == "continuous"){
            setAutoScrollMode(GraphWidget::CONTINUOUS);
        } else if(scrollModeLabel == "page"){
            setAutoScrollMode(GraphWidget::PAGE);
        }
    }
    
    archive.read("lower", rangeLowerY);
    archive.read("upper", rangeUpperY);

    isReconfigurationNeeded = true;
    screen->update();

    if(GraphBar::instance()->focusedGraphWidget() == self){
        GraphBar::instance()->focus(self, true);
    }

    return true;
}
