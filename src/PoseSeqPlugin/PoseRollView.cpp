/**
   \file
   \author Shin'ichiro Nakaoka
*/

#include "PoseRollView.h"
#include "PoseSeqViewBase.h"
#include "PronunSymbol.h"
#include <cnoid/RootItem>
#include <cnoid/ScrollBar>
#include <cnoid/CheckBox>
#include <cnoid/Separator>
#include <cnoid/EigenUtil>
#include <cnoid/ViewManager>
#include <QGridLayout>
#include <QHeaderView>
#include <QPaintEvent>
#include <QPainter>
#include "gettext.h"

#include <iostream>

using namespace std;
using namespace std::placeholders;
using namespace cnoid;

namespace {

const bool TRACE_FUNCTIONS = false;
const bool FAST_DRAW_MODE = false;

const double leftMargin = 0.2;

class ScrollBarEx : public ScrollBar
{
public:
    ScrollBarEx(Qt::Orientation orientation) : ScrollBar(orientation) { }
    virtual void sliderChange(SliderChange change){
        ScrollBar::sliderChange(change);
        if(change == QAbstractSlider::SliderStepsChange){
            if((maximum() - minimum()) == 0){
                hide();
            } else {
                show();
            }
        }
    }
};
}

namespace cnoid {

class PoseRollViewImpl : public PoseSeqViewBase
{
public:
    PoseRollViewImpl(PoseRollView* self);
    ~PoseRollViewImpl();

    PoseRollView* self;
    QGridLayout* gridLayout;
    ScrollBarEx* treeWidgetVerticalScrollBar;
    QWidget* screen;
    QVector<qreal> dash;
    QPainter painter;
    QPen markerPen;
    QPen pronumSymbolPen;
    QPen selectedMarkerPen;
    QPen gridPen;
    QPen cursorPen;

    ToolButton commandMenuButton;
    Menu commandMenu;
    MenuManager commandMenuManager;
    Action* lipSyncCheck;

    QLabel poseNameLabel;
    DoubleSpinBox currentTimeSpin;
    Connection currentTimeSpinConnection;
    DoubleSpinBox poseTimeSpin;
    Connection poseTimeSpinConnection;
    DoubleSpinBox poseTTimeSpin;
    Connection poseTTimeSpinConnection;

    DoubleScrollBar* hScrollBar;
    Connection hScrollBarChangedConnection;
    bool isTmpScrollBlocked;

    double left;
    double right;
    double treeTopOnScreen;
    double screenWidth;
    double screenHeight;
    double treeBottomOnScreen;
    double timeToScreenX;
    double barLength;

    double timeLength;
    DoubleSpinBox timeLengthSpin;
    DoubleSpinBox gridIntervalSpin;

    bool updateRowRectsNeeded;
            
    struct RowInfo {
        RowInfo() { }
        bool isVisible;
        int visibleRowIndex;
        int y;
        int height;
        int linkIndex;
        int jointId;
    };
    vector<RowInfo> itemIndexToRowInfoMap;
                
    vector<LinkTreeItem*> visibleRows;
    vector<LinkTreeItem*> linkIndexToVisibleRowAncestorMap;
    LinkTreeItem* visibleRowAncestorOfZmp;

    struct RowRenderInfo {
        bool rendered;
        double lastPoseTime;
    };
    vector<RowRenderInfo> rowRenderInfos;

    double pointerX;
    double pointerY;
    double pressedScreenX;
    double dragOrgLeft;
    double dragOrgScale;
    double pickDistance;
    PoseSeq::iterator pickedPoseIter;
    enum PickedPart { PICK_NONE, PICK_LEFT, PICK_BODY, PICK_RIGHT };
    PickedPart pickedPart;
    double pickedTime;

    enum DragMode { DRAG_NONE, DRAG_POSES, DRAG_TRANSITION_TIME, DRAG_TIME_CURSOR, DRAG_SCALING } dragMode;
    enum DragState { DS_INITIAL, DS_DRAGGED } dragState;

    double draggingPosesOrgTime;
            
    // for key pose marker rendering
    PoseSeq::iterator markerPoseIter;
    double markerTime0;
    double markerX0;
    double markerX1;
    double markerY0;
    double markerY1;
    bool isMarkerPronunSymbol;
    bool isMarkerSelected;

    void initialize();
    QHBoxLayout* layoutOperationParts();
    void setupScreen();
            
    virtual void setCurrentPoseSeqItem(PoseSeqItemPtr poseSeqItem);
    void requestRowRectsUpdate();
    void onTimeLengthChanged(double value);
    void onPoseTimeSpinChanged(double value);
    void onPoseTTimeSpinChanged(double value);
    virtual void onInsertPoseButtonClicked();
    virtual void onPoseInserted(PoseSeq::iterator it, bool isMoving);
    virtual void onPoseRemoving(PoseSeq::iterator it, bool isMoving);
    virtual void onPoseModified(PoseSeq::iterator it);
    void onCurrentTimeSpinChanged(double value);
    void setCurrentTime(double time, bool blockScroll = false);
    virtual bool onTimeChanged(double time);
    void setTimeOfScreenLeft(double time, bool changeScrollBar = true, bool forceChange = false);
    void onHScrollbarChanged(double value);

    void onGridResolutionChanged(double value);

    virtual void onLinkTreeUpdateRequest(bool isInitialCreation);
    void updateRowRects();
    void updateRowRectsSub(QTreeWidgetItem* treeWidgetItem);
    LinkTreeItem* getFirstVisibleAncestor(const LinkTreeItem* item);
    bool checkIfPoseHasRow(const PosePtr& pose, const LinkTreeItem* item);
    double searchLastPoseTime(const LinkTreeItem* item);
    void processKeyPoseMarkersSub(LinkTreeItem* item, std::function<void()> func);
    void processKeyPoseMarkers(std::function<void()> func);
            
    bool onScreenPaintEvent(QPaintEvent* event);        
    void drawBackground();
    void drawTimeCursor();
    void drawKeyPoseMarker();

    virtual void onTimeScaleChanged();
            
    virtual void onSelectedPosesModified();
    void pickPose();
    void pickPoseSub();
            
    void onScreenFontChanged();
    bool onScreenResizeEvent(QResizeEvent* event);
    bool onScreenMouseButtonPressEvent(QMouseEvent* event);
    void pickPoseOnButtonPress(bool isAdding);
    bool onScreenMouseButtonReleaseEvent(QMouseEvent* event);
    bool onScreenMouseMoveEvent(QMouseEvent* event);
    void pickPoseOnMotionNotify();
    void dragSelectedPoses();
    void dragTransitionTime();
    void dragScaling();
    bool onScreenKeyPressEvent(QKeyEvent* event);
    bool onScreenKeyReleaseEvent(QKeyEvent* event);
    void selectPrevPose(bool isAdding);
    void selectNextPose(bool isAdding);
    void onMenuButtonClicked();

    bool storeState(Archive& archive);
    bool restoreState(const Archive& archive);
};
}


void PoseRollView::initializeClass(ExtensionManager* ext)
{
    ext->viewManager().registerClass<PoseRollView>(
        "PoseRollView", N_("Pose Roll"), ViewManager::SINGLE_OPTIONAL);
}


PoseRollView::PoseRollView()
{
    setDefaultLayoutArea(View::BOTTOM);
    setSizePolicy(QSizePolicy::Ignored, QSizePolicy::Ignored);

    impl = new PoseRollViewImpl(this);
    impl->initialize();
}


PoseRollViewImpl::PoseRollViewImpl(PoseRollView* self)
    : PoseSeqViewBase(self),
      self(self),
      commandMenuManager(&commandMenu)
{

}


void PoseRollViewImpl::initialize()
{
    timeLength = 10.0;
    updateRowRectsNeeded = true;
    isTmpScrollBlocked = false;

    QVBoxLayout* vbox = new QVBoxLayout();
    vbox->setSpacing(0);
    
    vbox->addLayout(layoutOperationParts());
    
    setupScreen();

    QHBoxLayout* treeBox = new QHBoxLayout();

    linkTreeWidget->installEventFilter(self);
    linkTreeWidget->setAlternatingRowColors(true);

    treeWidgetVerticalScrollBar = new ScrollBarEx(Qt::Vertical);
    treeBox->addWidget(treeWidgetVerticalScrollBar);
    linkTreeWidget->setVerticalScrollBar(treeWidgetVerticalScrollBar);
    linkTreeWidget->setVerticalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
    treeWidgetVerticalScrollBar->sigValueChanged().connect(
        std::bind(&PoseRollViewImpl::requestRowRectsUpdate, this));

    // setup the link tree view model
    linkTreeWidget->sigItemExpanded().connect(
        std::bind(&PoseRollViewImpl::requestRowRectsUpdate, this));
    linkTreeWidget->sigItemCollapsed().connect(
        std::bind(&PoseRollViewImpl::requestRowRectsUpdate, this));
    treeBox->addWidget(linkTreeWidget, 1);

    hScrollBar = new DoubleScrollBar(Qt::Horizontal);
    hScrollBar->setSingleStep(0.1);
    hScrollBar->setRange(-leftMargin, timeLength + (2.0 / timeToScreenX));
    hScrollBarChangedConnection = hScrollBar->sigValueChanged().connect(
        std::bind(&PoseRollViewImpl::onHScrollbarChanged, this, _1));

    QHBoxLayout* hbox = new QHBoxLayout();
    hbox->setSpacing(8);
    hbox->addSpacing(16);
    hbox->addWidget(&currentItemLabel);
    hbox->addWidget(new QLabel(":"));
    hbox->addWidget(&poseNameLabel);

    gridLayout = new QGridLayout();
    gridLayout->setSpacing(0);
    gridLayout->setContentsMargins(0, 0, 0, 0);
    gridLayout->addLayout(treeBox, 0, 0);
    gridLayout->addWidget(screen, 0, 1);
    gridLayout->addLayout(hbox, 1, 0);
    gridLayout->addWidget(hScrollBar, 1, 1);
    gridLayout->setColumnStretch(1, 1);

    vbox->addLayout(gridLayout, 1);
    self->setLayout(vbox);

    commandMenuManager.addItem(_("Select specified key poses"))->sigTriggered().connect(
        std::bind(&PoseRollViewImpl::onSelectSpecifiedKeyPosesActivated, this));
    commandMenuManager.addItem(_("Adjust step positions"))->sigTriggered().connect(
        std::bind(&PoseRollViewImpl::onAdjustStepPositionsActivated, this));
    commandMenuManager.addItem(_("Adjust waist positions of selected key poses"))->sigTriggered().connect(
        std::bind(&PoseRollViewImpl::onAdjustWaistPositionActivated, this));
    commandMenuManager.addItem(_("Rotate yaw orientations"))->sigTriggered().connect(
        std::bind(&PoseRollViewImpl::onRotateYawOrientationsActivated, this));
    commandMenuManager.addItem(_("Update key poses with balanced trajectories"))->sigTriggered().connect(
        std::bind(&PoseRollViewImpl::onUpdateKeyposesWithBalancedTrajectoriesActivated, this));
    commandMenuManager.addItem(_("Flip poses against the x-z plane"))->sigTriggered().connect(
        std::bind(&PoseRollViewImpl::onFlipPosesActivated, this));

    lipSyncCheck = commandMenuManager.addCheckItem(_("Show lip-sync elements"));
    lipSyncCheck->sigToggled().connect(
        std::bind(static_cast<void(QWidget::*)()>(&QWidget::update), screen));

    commandMenuButton.setText(_("Menu"));
    commandMenuButton.setSizePolicy(QSizePolicy::Preferred, QSizePolicy::Expanding);
    commandMenuButton.sigClicked().connect(std::bind(&PoseRollViewImpl::onMenuButtonClicked, this));

    onSelectedItemsChanged(RootItem::instance()->selectedItems());
}


PoseRollView::~PoseRollView()
{
    delete impl;
}


PoseRollViewImpl::~PoseRollViewImpl()
{

}


QHBoxLayout* PoseRollViewImpl::layoutOperationParts()
{
    QHBoxLayout* hbox = new QHBoxLayout();
    hbox->setSpacing(0);

    hbox->addWidget(&commandMenuButton);
    
    updateButton.setEnabled(false);
    deleteButton.setEnabled(false);

    timeSyncCheck.setText(_("Sync"));

    hbox->addWidget(new QLabel(_("T:")));
    currentTimeSpin.setToolTip(_("Current time"));
    currentTimeSpin.setAlignment(Qt::AlignCenter);
    currentTimeSpin.setDecimals(3);
    currentTimeSpin.setRange(0.0, 999.999);
    currentTimeSpin.setSingleStep(0.005);
    currentTimeSpinConnection = currentTimeSpin.sigValueChanged().connect(
        std::bind(&PoseRollViewImpl::onCurrentTimeSpinChanged, this, _1));
    hbox->addWidget(&currentTimeSpin);

    hbox->addWidget(new QLabel(" / "));
    timeLengthSpin.setToolTip(_("Time length for editing"));
    timeLengthSpin.setAlignment(Qt::AlignCenter);
    timeLengthSpin.setDecimals(0);
    timeLengthSpin.setRange(1.0, 999.0);
    timeLengthSpin.setSingleStep(1.0);
    timeLengthSpin.setValue(timeLength);
    timeLengthSpin.sigValueChanged().connect(
        std::bind(&PoseRollViewImpl::onTimeLengthChanged, this, _1));
    hbox->addWidget(&timeLengthSpin);

    hbox->addWidget(&timeSyncCheck);
    hbox->addWidget(&insertPoseButton);
    hbox->addWidget(new QLabel(_("TT:")));
    hbox->addWidget(&transitionTimeSpin);
    hbox->addWidget(&updateButton);
    hbox->addWidget(&updateAllToggle);
    hbox->addWidget(&autoUpdateModeCheck);

    hbox->addWidget(new QLabel(_("T:")));
    poseTimeSpin.setToolTip(_("Time of the selected pose"));
    poseTimeSpin.setAlignment(Qt::AlignCenter);
    poseTimeSpin.setEnabled(false);
    poseTimeSpin.setDecimals(3);
    poseTimeSpin.setRange(0.0, 999.999);
    poseTimeSpin.setSingleStep(0.005);
    poseTimeSpinConnection = poseTimeSpin.sigValueChanged().connect(
        std::bind(&PoseRollViewImpl::onPoseTimeSpinChanged, this, _1));
    hbox->addWidget(&poseTimeSpin);

    hbox->addWidget(new QLabel(_("TT:")));
    poseTTimeSpin.setToolTip(_("Transition time of the selected pose"));
    poseTTimeSpin.setAlignment(Qt::AlignCenter);
    poseTTimeSpin.setEnabled(false);
    poseTTimeSpin.setDecimals(3);
    poseTTimeSpin.setRange(0.0, 9.999);
    poseTTimeSpin.setSingleStep(0.005);
    poseTTimeSpinConnection = poseTTimeSpin.sigValueChanged().connect(
        std::bind(&PoseRollViewImpl::onPoseTTimeSpinChanged, this, _1));
    hbox->addWidget(&poseTTimeSpin);

    hbox->addWidget(&deleteButton);

    hbox->addWidget(new QLabel(_("Grid:")));
    gridIntervalSpin.setAlignment(Qt::AlignCenter);
    gridIntervalSpin.setDecimals(0);
    gridIntervalSpin.setRange(1.0, 10.0);
    gridIntervalSpin.setSingleStep(1.0);
    gridIntervalSpin.sigValueChanged().connect(
        std::bind(&PoseRollViewImpl::onGridResolutionChanged, this, _1));
    hbox->addWidget(&gridIntervalSpin);

    hbox->addStretch();

    return hbox;
}


void PoseRollViewImpl::setupScreen()
{
    screen = new QWidget();
    screen->setMouseTracking(true);
    screen->installEventFilter(self);
    screen->setBackgroundRole(QPalette::Base);
    screen->setAutoFillBackground(true);

    dragMode = DRAG_NONE;
    dragState = DS_INITIAL;
    
    left = -leftMargin;
    right = 0.0;
    screenWidth = 0.0;
    screenHeight = 0.0;
    timeToScreenX = 120.0;
    barLength = 1.0;

    dash.push_back(2.0);
    dash.push_back(2.0);

    markerPen.setWidth(2);
    markerPen.setColor(Qt::black);
    pronumSymbolPen.setWidth(2);
    pronumSymbolPen.setColor(Qt::darkGreen);
    selectedMarkerPen.setWidth(3);
    selectedMarkerPen.setColor(Qt::red);
    
    gridPen.setWidth(1);
    gridPen.setDashPattern(dash);
    gridPen.setColor(QColor(50, 50, 50));

    cursorPen.setWidth(1);
    cursorPen.setColor(Qt::white);
}


bool PoseRollView::eventFilter(QObject* obj, QEvent* event)
{
    if(obj == impl->linkTreeWidget){
        switch(event->type()){
        case QEvent::FontChange:
        case QEvent::StyleChange:
        case QEvent::LanguageChange:
        case QEvent::LocaleChange:
            impl->requestRowRectsUpdate();
            return false;
            break;
        default:
            break;
        }
    } else if(obj == impl->screen){
        switch(event->type()){
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
        
    return View::eventFilter(obj, event);
}


PoseSeqItem* PoseRollView::currentPoseSeqItem()
{
    return impl->currentPoseSeqItem.get();
}


void PoseRollViewImpl::setCurrentPoseSeqItem(PoseSeqItemPtr poseSeqItem)
{
    BodyPtr prevBody = body;
    
    PoseSeqViewBase::setCurrentPoseSeqItem(poseSeqItem);

    if(poseSeqItem){
        double lower, upper;
        poseSeqItem->poseSeq()->getDomain(lower, upper);
        if(timeLengthSpin.value() < upper){
            timeLengthSpin.setValue(upper);
        }
        barLength = poseSeqItem->barLength();
    }

    if(body != prevBody){
        updateRowRectsNeeded = true;
    }

    screen->update();
}


void PoseRollViewImpl::requestRowRectsUpdate()
{
    updateRowRectsNeeded = true;
    screen->update();
}


void PoseRollViewImpl::onTimeLengthChanged(double value)
{
    timeLength = value;
    hScrollBar->setRange(-leftMargin, timeLength + (2.0 / timeToScreenX));

    if(currentTime > timeLength){
        setCurrentTime(timeLength);
    } else {
        screen->update();
    }
}


void PoseRollViewImpl::onPoseTimeSpinChanged(double value)
{
    if(!selectedPoseIters.empty()){
        double scaledTime = value;
        double time = scaledTime / timeScale;
        
        if(time != (*selectedPoseIters.begin())->time()){
            currentPoseSeqItem->beginEditing();
            if(currentPoseSeqItem->endEditing(moveSelectedPoses(time))){
                doAutomaticInterpolationUpdate();
            }
            setCurrentTime(scaledTime);
        }
    }
}


void PoseRollViewImpl::onPoseTTimeSpinChanged(double value)
{
    if(!selectedPoseIters.empty()){
        double scaledTTime = value;
        double ttime = scaledTTime / timeScale;
        currentPoseSeqItem->beginEditing();
        if(currentPoseSeqItem->endEditing(
               modifyTransitionTimeOfSelectedPoses(ttime))){
            doAutomaticInterpolationUpdate();
        }
    }
}


void PoseRollViewImpl::onInsertPoseButtonClicked()
{
    if(currentPoseSeqItem){
        currentPoseSeqItem->beginEditing();
        PoseSeq::iterator poseIter = insertPose();
        currentPoseSeqItem->endEditing(poseIter != seq->end());
    }
}


void PoseRollViewImpl::onPoseInserted(PoseSeq::iterator it, bool isMoving)
{
    if(TRACE_FUNCTIONS){
        cout << "PoseRollViewImpl::onPoseInserted()" << endl;
    }

    PoseSeqViewBase::onPoseInserted(it, isMoving);

    screen->update();
}


void PoseRollViewImpl::onPoseRemoving(PoseSeq::iterator it, bool isMoving)
{
    if(TRACE_FUNCTIONS){
        cout << "PoseRollViewImpl::onPoseRemoving()" << endl;
    }

    PoseSeqViewBase::onPoseRemoving(it, isMoving);

    if(!isMoving){
        screen->update();
    }
}


void PoseRollViewImpl::onPoseModified(PoseSeq::iterator it)
{
    PoseSeqViewBase::onPoseModified(it);
    screen->update();
}


void PoseRollViewImpl::onCurrentTimeSpinChanged(double value)
{
    if(value > timeLength){
        timeLengthSpin.setValue(value);
    }
    
    setCurrentTime(value);
}


void PoseRollViewImpl::setCurrentTime(double time, bool blockScroll)
{
    time = std::max(0.0, time);

    isTmpScrollBlocked = blockScroll;
    onTimeChanged(time);
    isTmpScrollBlocked = false;
    
    if(timeSyncCheck.isChecked()){
        connectionOfTimeChanged.block();
        timeBar->setTime(time);
        connectionOfTimeChanged.unblock();
    }
}


bool PoseRollViewImpl::onTimeChanged(double time)
{
    if(TRACE_FUNCTIONS){
        cout << "PoseRollViewImpl::onTimeChanged(" << time << ")" << endl;
    }

    const double width = screenWidth / timeToScreenX;
    bool newTimeInCurrentVisibleRange = (time >= left && time < right);

    // Scroll sync
    if(!newTimeInCurrentVisibleRange && !isTmpScrollBlocked){
        if(time < left){
            if(left - time < width / 3.0){
                setTimeOfScreenLeft(left - width * 0.9);
            } else {
                setTimeOfScreenLeft(time - width / 2.0);
            }
        } else {
            if(time - right < width / 3.0){
                setTimeOfScreenLeft(left + width * 0.9);
            } else {
                setTimeOfScreenLeft(time - width / 2.0);
            }
        }
    } else if(newTimeInCurrentVisibleRange || (currentTime >= left && currentTime < right)){
        screen->update();
    }
            
    if(time != currentTime){
        currentTime = time;
        currentTimeSpinConnection.block();
        currentTimeSpin.setValue(time);
        currentTimeSpinConnection.unblock();
    }

    return (seq && (time < timeScale * seq->endingTime()));
}


void PoseRollViewImpl::setTimeOfScreenLeft(double time, bool changeScrollBar, bool forceChange)
{
    time = std::max(-leftMargin, std::min(timeLength, time));
    
    if(time != left || forceChange){

        left = time;
        right = left + screenWidth / timeToScreenX;

        if(changeScrollBar){
            hScrollBarChangedConnection.block();
            hScrollBar->setValue(left);
            hScrollBarChangedConnection.unblock();
        }
        
        screen->update();
    }
}


void PoseRollViewImpl::onHScrollbarChanged(double value)
{
    setTimeOfScreenLeft(value, false);
}


void PoseRollViewImpl::onGridResolutionChanged(double value)
{
    screen->update();
}


void PoseRollViewImpl::onLinkTreeUpdateRequest(bool isInitialCreation)
{
    PoseSeqViewBase::onLinkTreeUpdateRequest(isInitialCreation);

    itemIndexToRowInfoMap.resize(linkTreeWidget->numLinkTreeItems());
    
    updateRowRectsNeeded = true;
}


void PoseRollViewImpl::updateRowRects()
{
    if(TRACE_FUNCTIONS){
        cout << "PoseRollViewImpl::updateRowRects(): updateRowRectsNeeded = " << updateRowRectsNeeded << endl;
    }
    
    if(updateRowRectsNeeded){

        linkIndexToVisibleRowAncestorMap.clear();
        visibleRows.clear();
        
        if(body){
            linkIndexToVisibleRowAncestorMap.resize(body->numLinks());
            updateRowRectsSub(linkTreeWidget->invisibleRootItem());
            rowRenderInfos.resize(visibleRows.size());
        }

        treeTopOnScreen = linkTreeWidget->header()->geometry().bottom();
        treeBottomOnScreen = treeTopOnScreen;
        if(!visibleRows.empty()){
            RowInfo& bottomRow = itemIndexToRowInfoMap[visibleRows.back()->rowIndex()];
            treeBottomOnScreen += bottomRow.y + bottomRow.height;
        }
        updateRowRectsNeeded = false;
    }
}


void PoseRollViewImpl::updateRowRectsSub(QTreeWidgetItem* treeWidgetItem)
{
    LinkTreeItem* item = dynamic_cast<LinkTreeItem*>(treeWidgetItem);
    if(item){
        QRect rect = linkTreeWidget->visualItemRect(item);
        RowInfo& rowInfo = itemIndexToRowInfoMap[item->rowIndex()];
        if(rect.isEmpty()){
            rowInfo.isVisible = false;
            rowInfo.visibleRowIndex = -1;
            rowInfo.y = 0;
            rowInfo.height = 0;
        } else {
            rowInfo.isVisible = true;
            rowInfo.visibleRowIndex = visibleRows.size();
            rowInfo.y = rect.y();
            rowInfo.height = rect.height();
            visibleRows.push_back(item);
        }
        Link* link = item->link();
        if(!link){
            rowInfo.linkIndex = -1;
            rowInfo.jointId = -1;
            if(item == zmpRow){
                visibleRowAncestorOfZmp = rowInfo.isVisible ? item : getFirstVisibleAncestor(item);
            }
        } else {
            rowInfo.linkIndex = link->index();
            rowInfo.jointId = link->jointId();
            linkIndexToVisibleRowAncestorMap[link->index()] = rowInfo.isVisible ? item : getFirstVisibleAncestor(item);
        }
    }

    int n = treeWidgetItem->childCount();
    for(int i=0; i < n; ++i){
        updateRowRectsSub(treeWidgetItem->child(i));
    }
}


LinkTreeItem* PoseRollViewImpl::getFirstVisibleAncestor(const LinkTreeItem* item)
{
    for(QTreeWidgetItem* parent = item->parent(); parent; parent = parent->parent()){
        LinkTreeItem* row = dynamic_cast<LinkTreeItem*>(parent);
        if(row){
            const RowInfo& info = itemIndexToRowInfoMap[row->rowIndex()];
            if(info.isVisible){
                return row;
            }
        }
    }
    return 0;
}


bool PoseRollViewImpl::checkIfPoseHasRow(const PosePtr& pose, const LinkTreeItem* item)
{
    if(item == zmpRow && pose->isZmpValid()){
        return true;
    }
    const RowInfo& info = itemIndexToRowInfoMap[item->rowIndex()];
    if(info.jointId >= 0 && pose->isJointValid(info.jointId)){
        return true;
    }
    int n = item->childCount();
    for(int i=0; i < n; ++i){
        LinkTreeItem* childItem = dynamic_cast<LinkTreeItem*>(item->child(i));
        if(childItem && checkIfPoseHasRow(pose, childItem)){
            return true;
        }
    }
    return false;
}


double PoseRollViewImpl::searchLastPoseTime(const LinkTreeItem* item)
{
    PoseSeq::iterator last = markerPoseIter;

    while(last != seq->begin()){
        last--;
        PosePtr pose = last->get<Pose>();
        if(pose){
            if(checkIfPoseHasRow(pose, item)){
                break;
            }
        }
    }
    return timeScale * last->time();
}


void PoseRollViewImpl::processKeyPoseMarkersSub(LinkTreeItem* item, std::function<void()> func)
{
    while(item){
        const RowInfo& rowInfo = itemIndexToRowInfoMap[item->rowIndex()];
        RowRenderInfo& renderInfo = rowRenderInfos[rowInfo.visibleRowIndex];
        if(renderInfo.rendered){
            break;
        } else {
            renderInfo.rendered = true;

            if(renderInfo.lastPoseTime == -std::numeric_limits<double>::max()){
                renderInfo.lastPoseTime = searchLastPoseTime(item);
            }
            
            double t0 = std::max(markerTime0, renderInfo.lastPoseTime);
            markerX0 = floor(timeToScreenX * (t0 - left));
            markerY0 = rowInfo.y + treeTopOnScreen;
            markerY1 = markerY0 + rowInfo.height;
            
            func();
            
            if(!isMarkerPronunSymbol){
                renderInfo.lastPoseTime = timeScale * markerPoseIter->time();
            }
        }
        item = dynamic_cast<LinkTreeItem*>(item->parent());
    }
}


void PoseRollViewImpl::processKeyPoseMarkers(std::function<void()> func)
{
    for(size_t i=0; i < rowRenderInfos.size(); ++i){
        RowRenderInfo& info = rowRenderInfos[i];
        if(FAST_DRAW_MODE){
            info.lastPoseTime = left;
        } else {
            info.lastPoseTime = -std::numeric_limits<double>::max();
        }
    }
    
    markerPoseIter = seq->seek(seq->begin(), left / timeScale);

    const std::vector<int>& lipSyncLinkIndices = currentPoseSeqItem->interpolator()->lipSyncLinkIndices();

    while(markerPoseIter != seq->end()){

        if(FAST_DRAW_MODE){
            if(timeScale * markerPoseIter->time() > right){
                break;
            }
        }

        double time = timeScale * markerPoseIter->time();
        double ttime = timeScale * markerPoseIter->maxTransitionTime();
        if(ttime == 0.0){
            markerTime0 = -std::numeric_limits<double>::max();
        } else {
            markerTime0 = time - ttime;
        }

        for(size_t i=0; i < rowRenderInfos.size(); ++i){
            RowRenderInfo& renderInfo = rowRenderInfos[i];
            renderInfo.rendered = false;
        }
        isMarkerSelected = (findPoseIterInSelected(markerPoseIter) != selectedPoseIters.end());

        markerX1 = floor(timeToScreenX * (time - left));

        PosePtr pose = markerPoseIter->get<Pose>();
        if(pose){
            isMarkerPronunSymbol = false;
            int n = std::min(body->numJoints(), pose->numJoints());
            for(int i=0; i < n; ++i){
                Link* joint = body->joint(i);
                if(pose->isJointValid(i) && joint->isValid()){
                    processKeyPoseMarkersSub(linkIndexToVisibleRowAncestorMap[joint->index()], func);
                }
            }
            for(Pose::LinkInfoMap::iterator it = pose->ikLinkBegin(); it != pose->ikLinkEnd(); ++it){
                processKeyPoseMarkersSub(linkIndexToVisibleRowAncestorMap[it->first], func);
            }
            
            if(pose->isZmpValid()){
                processKeyPoseMarkersSub(visibleRowAncestorOfZmp, func);
            }
        } else if(lipSyncCheck->isChecked()) {
            PronunSymbolPtr pronun = markerPoseIter->get<PronunSymbol>();
            if(pronun){
                isMarkerPronunSymbol = true;
                for(size_t i=0; i < lipSyncLinkIndices.size(); ++i){
                    processKeyPoseMarkersSub(linkIndexToVisibleRowAncestorMap[lipSyncLinkIndices[i]], func);
                }
            }
        }   

        ++markerPoseIter;
    }
}


bool PoseRollViewImpl::onScreenPaintEvent(QPaintEvent* event)
{
    if(TRACE_FUNCTIONS){
        cout << "PoseRollViewImpl::onScreenPaintEvent()" << endl;
    }
    
    updateRowRects();

    painter.begin(screen);

    drawBackground();
    
    painter.setClipRect(0.0, treeTopOnScreen, screenWidth, screenHeight - treeTopOnScreen);
    painter.setClipping(true);

    if(seq){
        processKeyPoseMarkers(std::bind(&PoseRollViewImpl::drawKeyPoseMarker, this));
    }

    painter.setClipping(false);
    drawTimeCursor();

    painter.end();

    return false;
}


void PoseRollViewImpl::drawBackground()
{
    double bodyHeight = screenHeight - treeTopOnScreen;
    double rowBottom = 0.0;
    double offset = treeTopOnScreen;

    painter.setPen(gridPen);
    
    // draw row lines
    if(!visibleRows.empty()){

        double x = timeToScreenX * (floor(left) - left);

        for(size_t i=0; i < visibleRows.size(); ++i){
            LinkTreeItem* item = visibleRows[i];
            const RowInfo& r = itemIndexToRowInfoMap[item->rowIndex()];
            if(r.y < 0.0){
                continue;
            }
            if(r.y > bodyHeight){
                break;
            }
            double y = r.y + offset;
            painter.drawLine(x, y, screenWidth, y);
        }

        const RowInfo& r = itemIndexToRowInfoMap[visibleRows.back()->rowIndex()];
        double y = r.y + r.height + offset;
        rowBottom = std::min(y, (double)screenHeight);
        painter.drawLine(x, y, screenWidth, y);
    }

    // draw time grid lines
    //double t = floor(left + 0.9999); // for avoiding -0.0

    double step = barLength / gridIntervalSpin.value();
    double t = floor(left / step) * step;
    if(t != left) t += step;
    
    while(true){

        double x = timeToScreenX * (t - left);
        if(x > screenWidth){
            break;
        }
        x = floor(x) /* + 0.5 */;

        QString timeText = QString::number(t, 'f', 1);
        QFontMetrics metrics(painter.fontMetrics());
        QRect r = metrics.boundingRect(timeText);
        double tx = x - r.width() / 2.0;
        double ty = treeTopOnScreen / 2.0 - (r.height() / 2.0 - metrics.ascent());
        painter.drawText(tx, ty, timeText);

        painter.drawLine(x, treeTopOnScreen, x, rowBottom);

        t += step;
    }
}


void PoseRollViewImpl::drawKeyPoseMarker()
{
    QPen* pen;
    if(isMarkerSelected){
        if(FAST_DRAW_MODE){
            painter.setBrush(QBrush(QColor(255, 200, 200)));
        } else {
            QLinearGradient gradient(markerX0, 0.0, markerX1, 0.0);
            gradient.setColorAt(0.0, QColor(255, 240, 240));
            gradient.setColorAt(1.0, QColor(255, 150, 150));
            painter.setBrush(gradient);
        }
        pen = &selectedMarkerPen;
    } else {
        if(FAST_DRAW_MODE){
            painter.setBrush(QBrush(QColor(180, 180, 180)));
        } else {
            QLinearGradient gradient(markerX0, 0.0, markerX1, 0.0);
            gradient.setColorAt(0.0, QColor(245, 245, 245));
            gradient.setColorAt(1.0, QColor(15, 15, 15));
            painter.setBrush(gradient);
        }
        pen = isMarkerSelected ? &pronumSymbolPen : &markerPen;
    }
    QPointF points[3];
    points[0] = QPointF(markerX0, markerY1);
    points[1] = QPointF(markerX1, markerY0);
    points[2] = QPointF(markerX1, markerY1);
    painter.setPen(Qt::NoPen);
    painter.drawConvexPolygon(points, 3);
    painter.setPen(*pen);
    painter.setBrush(Qt::NoBrush);
    painter.drawLine(points[1], points[2]);
}


void PoseRollViewImpl::drawTimeCursor()
{
    double x = floor(timeToScreenX * (currentTime - left)) /* + 0.5 */;

    if(x >= 0.0 && x < screenWidth){
        painter.setPen(cursorPen);
        painter.setCompositionMode(QPainter::RasterOp_SourceXorDestination);
        painter.drawLine(x, 0, x, screenHeight);
        painter.setCompositionMode(QPainter::CompositionMode_SourceOver);
    };
}


void PoseRollViewImpl::onTimeScaleChanged()
{
    PoseSeqViewBase::onTimeScaleChanged();
    screen->update();
}
    

void PoseRollViewImpl::onSelectedPosesModified()
{
    PoseSeqViewBase::onSelectedPosesModified();

    poseTimeSpinConnection.block();
    poseTTimeSpinConnection.block();
    
    if(selectedPoseIters.empty()){
        poseNameLabel.setText("");
        poseTimeSpin.setEnabled(false);
        poseTimeSpin.setValue(0.0);
        poseTTimeSpin.setEnabled(false);
        poseTTimeSpin.setValue(0.0);
    } else {
        PoseSeq::iterator poseIter = *selectedPoseIters.begin();
        poseNameLabel.setText(poseIter->name().c_str());
        poseTimeSpin.setEnabled(true);
        poseTimeSpin.setValue(timeScale * poseIter->time());
        poseTTimeSpin.setEnabled(true);
        poseTTimeSpin.setValue(timeScale * poseIter->maxTransitionTime());
    }

    poseTTimeSpinConnection.unblock();
    poseTimeSpinConnection.unblock();
    
    screen->update();
}
    
    
void PoseRollViewImpl::pickPose()
{
    if(seq){
        pickedPoseIter = seq->end();
        pickDistance = std::numeric_limits<double>::max();
        pickedPart = PICK_NONE;
        processKeyPoseMarkers(std::bind(&PoseRollViewImpl::pickPoseSub, this));
    }
}


void PoseRollViewImpl::pickPoseSub()
{
    if(markerY0 <= pointerY && pointerY < markerY1){
        if(!isMarkerPronunSymbol){
            const double margin = 2.0;
            if((markerX0 - margin) <= pointerX && pointerX <= (markerX1 + margin)){
                double dLeft = pointerX - markerX0;
                if(dLeft < 0.0 || dLeft >= 6.0){
                    dLeft = std::numeric_limits<double>::max();
                }
                double dRight = fabs(markerX1 - pointerX);
                double d;
                PickedPart part;
                if(dLeft < dRight){
                    d = dLeft;
                    part = PICK_LEFT;
                    pickedTime = (markerX0 / timeToScreenX) + left;
                } else {
                    d = dRight;
                    pickedTime = (markerX1 / timeToScreenX) + left;
                    if(d <= margin) {
                        part = PICK_RIGHT;
                    } else {
                        part = PICK_BODY;
                    }
                }
                if(d < pickDistance){
                    pickDistance = d;
                    pickedPoseIter = markerPoseIter;
                    pickedPart = part;
                }
            }
        }
    }
}
                    
                
bool PoseRollViewImpl::onScreenResizeEvent(QResizeEvent* event)
{
    screenWidth = event->size().width();
    screenHeight = event->size().height();
    hScrollBar->setPageStep(screenWidth / timeToScreenX);
    right = left + screenWidth / timeToScreenX;
    screen->update();
    return false;
}


bool PoseRollViewImpl::onScreenMouseButtonPressEvent(QMouseEvent* event)
{
    screen->setFocus(Qt::MouseFocusReason);

    pressedScreenX = event->x();
    pointerX = event->x();
    pointerY = event->y();
    dragOrgLeft = left;

    dragMode = DRAG_NONE;
    dragState = DS_INITIAL;
    
    if(event->type() == QEvent::MouseButtonPress){
        if(event->button() == Qt::LeftButton){
            if(pointerY < treeTopOnScreen || pointerY > treeBottomOnScreen){
                setCurrentTime(pointerX / timeToScreenX + left);
                dragMode = DRAG_TIME_CURSOR;
            } else {
                pickPoseOnButtonPress(event->modifiers() & Qt::ControlModifier);
            }
        } else if(event->button() == Qt::MidButton){
            dragMode = DRAG_SCALING;
            dragOrgScale = timeToScreenX;
        } else if(event->button() == Qt::RightButton){
            popupContextMenu(event);
        }
    }
    
    return true;
}


void PoseRollViewImpl::pickPoseOnButtonPress(bool isAdding)
{
    if(!seq){
        return;
    }

    pickPose();

    toggleSelection(pickedPoseIter, isAdding, true);
    
    if(pickedPoseIter != seq->end()){
        if(pickedPart == PICK_RIGHT){
            dragMode = DRAG_POSES;
            draggingPosesOrgTime = timeScale * (*selectedPoseIters.begin())->time();
            screen->setCursor(Qt::ClosedHandCursor);
        } else if(pickedPart == PICK_LEFT){
            dragMode = DRAG_TRANSITION_TIME;
            screen->setCursor(Qt::SplitHCursor);
        }
    }
}
    
 
bool PoseRollViewImpl::onScreenMouseMoveEvent(QMouseEvent* event)
{
    if(TRACE_FUNCTIONS){
        cout << "PoseRollViewImpl::onScreenMotionNotifyEvent()" << endl;
    }
    
    pointerX = event->x();
    pointerY = event->y();

    switch(dragMode){

    case DRAG_NONE:
        screen->setCursor(Qt::ArrowCursor);
        pickPoseOnMotionNotify();
        break;

    case DRAG_POSES:
        dragSelectedPoses();
        break;

    case DRAG_TRANSITION_TIME:
        dragTransitionTime();
        break;

    case DRAG_TIME_CURSOR:
        setCurrentTime(pointerX / timeToScreenX + left, true);
        break;

    case DRAG_SCALING:
        dragScaling();
        break;

    default:
        break;
    }
        
    return true;
}


bool PoseRollViewImpl::onScreenMouseButtonReleaseEvent(QMouseEvent* event)
{
    switch(dragMode){

    case DRAG_POSES:
    case DRAG_TRANSITION_TIME:
        if(dragState == DS_DRAGGED){
            currentPoseSeqItem->endEditing();
            doAutomaticInterpolationUpdate();
        }
        break;

    default:
        break;
    }
    
    dragMode = DRAG_NONE;
    screen->setCursor(Qt::ArrowCursor);
    
    return true;
}


void PoseRollViewImpl::pickPoseOnMotionNotify()
{
    if(seq && !updateRowRectsNeeded){
        pickPose();
        if(pickedPoseIter != seq->end()){
            if(pickedPart == PICK_LEFT){
                screen->setCursor(Qt::SplitHCursor);
            } else if(pickedPart == PICK_RIGHT){
                screen->setCursor(Qt::OpenHandCursor);
            }
        }
    }
}


void PoseRollViewImpl::dragSelectedPoses()
{
    if(dragState == DS_INITIAL){
        currentPoseSeqItem->beginEditing();
        dragState = DS_DRAGGED;
    }
    double scaledTime = draggingPosesOrgTime + (pointerX - pressedScreenX) / timeToScreenX;
    moveSelectedPoses(scaledTime / timeScale);
}


void PoseRollViewImpl::dragTransitionTime()
{
    if(dragState == DS_INITIAL){
        currentPoseSeqItem->beginEditing();
        dragState = DS_DRAGGED;
    }
    seq->beginPoseModification(pickedPoseIter);
    double scaledTime = pickedTime + (pointerX - pressedScreenX) / timeToScreenX;
    double ttime = pickedPoseIter->time() - scaledTime / timeScale;
    pickedPoseIter->setMaxTransitionTime(std::max(ttime, 0.0));
    seq->endPoseModification(pickedPoseIter);
}


void PoseRollViewImpl::dragScaling()
{
    double zoomRatio = pow(1.01, pointerX - pressedScreenX);
    
    timeToScreenX = dragOrgScale * zoomRatio;
    
    double dpx = pressedScreenX / dragOrgScale;
    double dx = dpx * (zoomRatio - 1.0) / zoomRatio;
    
    hScrollBarChangedConnection.block();
    hScrollBar->setPageStep(screenWidth / timeToScreenX);
    hScrollBarChangedConnection.unblock();

    setTimeOfScreenLeft(dragOrgLeft + dx, true, true);
}


bool PoseRollViewImpl::onScreenKeyPressEvent(QKeyEvent* event)
{
    bool handled = false;

    if(event->key() == Qt::Key_Space){
        QMouseEvent mouseEvent(QEvent::MouseButtonPress,
                               QPoint(pointerX, pointerY),
                               Qt::MidButton, Qt::MidButton,
                               event->modifiers());
        return onScreenMouseButtonPressEvent(&mouseEvent);
    }

    bool isCtrlActive = (event->modifiers() & Qt::ControlModifier);

    if(isCtrlActive){

        handled = true;
        
        switch(event->key()){
        case Qt::Key_A:
            selectAllPoses();
            break;
        case Qt::Key_X:
            cutSelectedPoses();
            break;
        case Qt::Key_C:
            copySelectedPoses();
            break;
        case Qt::Key_V:
            pasteCopiedPoses(currentTime / timeScale);
            break;
        case Qt::Key_Z:
            if(currentPoseSeqItem){
                if(event->modifiers() & Qt::ShiftModifier){
                    currentPoseSeqItem->redo();
                } else {
                    currentPoseSeqItem->undo();
                }
            }
            break;
        default:
            handled = false;
            break;
        }
    }

    if(!handled){

        handled = true;
        
        switch(event->key()){
        
        case Qt::Key_Left:
            selectPrevPose(isCtrlActive);
            break;
        
        case Qt::Key_Right:
            selectNextPose(isCtrlActive);
            break;

        default:
            handled = false;
            break;
        }
    }
    
    return handled;
}


bool PoseRollViewImpl::onScreenKeyReleaseEvent(QKeyEvent* event)
{
    if(event->key() == Qt::Key_Space){
        QMouseEvent mouseEvent(QEvent::MouseButtonPress,
                               QPoint(pointerX, pointerY),
                               Qt::MidButton, Qt::MidButton,
                               event->modifiers());
        return onScreenMouseButtonReleaseEvent(&mouseEvent);
    }
    
    return false;
}


void PoseRollViewImpl::selectPrevPose(bool isAdding)
{
    if(!selectedPoseIters.empty()){
        PoseSeq::iterator it = *selectedPoseIters.begin();
        if(it != seq->begin()){
            it--;
        }
        while(true){
            if(!lipSyncCheck->isChecked() && !it->get<Pose>()){
                if(it != seq->begin()){
                    it--;
                    continue;
                } else {
                    it = seq->end();
                }
            }
            break;
        }
        if(it != seq->end()){
            toggleSelection(it, isAdding, true);
        }
    }
}

        
void PoseRollViewImpl::selectNextPose(bool isAdding)
{
    if(!selectedPoseIters.empty()){
        PoseSeq::iterator it = *(--selectedPoseIters.end());
        ++it;
        if(!lipSyncCheck->isChecked()){
            while(it != seq->end() && !it->get<Pose>()){
                ++it;
            }
        }
        if(it != seq->end()){
            toggleSelection(it, isAdding, true);
            
        }
    }
}


void PoseRollViewImpl::onMenuButtonClicked()
{
    commandMenu.exec(commandMenuButton.mapToGlobal(QPoint(0,0)));
}


bool PoseRollView::storeState(Archive& archive)
{
    return impl->storeState(archive);
}


bool PoseRollViewImpl::storeState(Archive& archive)
{
    if(PoseSeqViewBase::storeState(archive)){
        if(!timeSyncCheck.isChecked()){
            archive.write("time", currentTime);
        }
        archive.write("timeLength", timeLength);
        archive.write("showLipSync", lipSyncCheck->isChecked());
        archive.write("gridInterval", gridIntervalSpin.value());
        return true;
    }
    return false;
}


bool PoseRollView::restoreState(const Archive& archive)
{
    return impl->restoreState(archive);
}


bool PoseRollViewImpl::restoreState(const Archive& archive)
{
    updateRowRectsNeeded = true;
    
    timeLengthSpin.setValue(archive.get("timeLength", timeLengthSpin.value()));
    lipSyncCheck->setChecked(archive.get("showLipSync", lipSyncCheck->isChecked()));
    gridIntervalSpin.setValue(archive.get("gridInterval", gridIntervalSpin.value()));
        
    PoseSeqViewBase::restoreState(archive);
        
    if(!timeSyncCheck.isChecked()){
        double time;
        if(archive.read("time", time)){
            currentTimeSpin.setValue(time);
        }
    }
    return true;
}
