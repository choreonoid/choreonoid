/**
   @author Shin'ichiro Nakaoka
*/

#include "ViewArea.h"
#include "View.h"
#include "ViewManager.h"
#include "MenuManager.h"
#include "MessageView.h"
#include "MainWindow.h"
#include "Timer.h"
#include <QApplication>
#include <QBoxLayout>
#include <QSplitter>
#include <QTabWidget>
#include <QTabBar>
#include <QRubberBand>
#include <QMouseEvent>
#include <QDesktopWidget>
#include <QLabel>
#include <memory>
#include <bitset>
#include "gettext.h"

using namespace std;
using namespace cnoid;

namespace {

enum DropArea { OVER = -1, LEFT = 0, TOP, RIGHT, BOTTOM, NUM_DROP_AREAS };
const int SPLIT_DISTANCE_THRESHOLD = 35;

vector<ViewArea*> viewAreas;
bool isBeforeDoingInitialLayout = true;

class TabWidget : public QTabWidget
{
public:
    TabWidget(QWidget* parent) : QTabWidget(parent) { }
    QTabBar* tabBar() { return QTabWidget::tabBar(); }
};
    

class ViewPane : public QWidget
{
public:
    ViewPane(ViewArea::Impl* viewAreaImpl, QWidget* parent = nullptr);
    int addView(View* view);
    void onViewTitleChanged(const QString &title);
    void removeView(View* view);
    void setTabVisible(bool on);
    void makeDirect();

    QTabBar* tabBar() { return tabWidget->tabBar(); }
    const QTabBar* tabBar() const { return tabWidget->tabBar(); }
    int currentIndex() const {
        return directView ? 0 : tabWidget->currentIndex();
    }
    void setCurrentIndex(int index) {
        if(!directView){
            tabWidget->setCurrentIndex(index);
        }
    }
    View* currentView() const {
        return directView ? directView : static_cast<View*>(tabWidget->currentWidget());
    }
    int count() const {
        return directView ? 1 : tabWidget->count();
    }
    View* view(int index) const {
        if(directView){
            if(index == 0){
                return directView;
            }
            return nullptr;
        } else {
            return static_cast<View*>(tabWidget->widget(index));
        }
    }
    
    virtual bool eventFilter(QObject* object, QEvent* event);

    virtual QSize minimumSizeHint () const {
        QSize s = QWidget::minimumSizeHint();
        if(!tabBar()->isVisible()){
            s.rheight() -= tabBar()->minimumSizeHint().height();
        }
        return s;
    }

    TabWidget* tabWidget;
    QVBoxLayout* vbox;
    View* directView;
    ViewArea::Impl* viewAreaImpl;
};

}

namespace cnoid {

class ViewArea::Impl
{
public:
    Impl(ViewArea* self);
    ~Impl();

    ViewArea* self;
    QVBoxLayout* vbox;
    QSplitter* topSplitter;

    int numViews;
    bool viewTabsVisible;
    bool isMaximizedBeforeFullScreen;
    bool needToUpdateDefaultPaneAreas;
    std::unique_ptr<vector<View*>> defaultViewsToShow;

    ViewPane* areaToPane[View::NUM_AREAS];

    struct AreaDetectionInfo {
        AreaDetectionInfo() {
            for(int i=0; i < View::NUM_AREAS; ++i){
                scores[i] = 0;
            }
        }
        ViewPane* pane;
        int scores[View::NUM_AREAS];
    };

    typedef bitset<NUM_DROP_AREAS> EdgeContactState;

    QPoint tabDragStartPosition;
    bool isViewDragging;
    View* draggedView;
    QSize draggedViewWindowSize;
    ViewArea* dragDestViewArea;
    ViewPane* dragSrcPane;
    ViewPane* dragDestPane;
    bool isViewDraggingOnOuterEdge;
    int dropEdge;
    QRubberBand* rubberBand;

    vector<QLabel*> viewSizeLabels;
    Timer viewSizeLabelTimer;
    
    MenuManager viewMenuManager;

    void setSingleView(View* view);
    void createDefaultPanes();

    void detectExistingPaneAreas();
    ViewPane* updateAreaDetectionInfos(QSplitter* splitter, const EdgeContactState& edge, vector<AreaDetectionInfo>& infos);
    void setBestAreaMatchPane(vector<AreaDetectionInfo>& infos, View::LayoutArea area, ViewPane* firstPane);

    void setViewTabsVisible(QSplitter* splitter, bool on);
    void setFullScreen(bool on);
    bool addView(View* view);
    void addView(ViewPane* pane, View* view, bool makeCurrent);
    bool removeView(View* view);
    bool removeView(ViewPane* pane, View* view, bool isMovingInViewArea);
    View* findFirstView(QSplitter* splitter);

    void getVisibleViews(vector<View*>& out_views, QSplitter* splitter = nullptr);
    void getVisibleViewsIter(QSplitter* splitter, vector<View*>& out_views);
    void showViewSizeLabels(QSplitter* splitter);
    void hideViewSizeLabels();
    
    bool viewTabMousePressEvent(ViewPane* pane, QMouseEvent* event);
    bool viewTabMouseMoveEvent(ViewPane* pane, QMouseEvent* event);
    bool viewTabMouseReleaseEvent(QMouseEvent *event);

    void showRectangle(QRect r);
    void dragView(QMouseEvent* event);
    void dragViewInsidePane(const QPoint& posInDestPane);
    void dropViewInsidePane(ViewPane* pane, View* view, int dropEdge);
    void dragViewOnOuterEdge();
    void dropViewToOuterEdge(View* view);
    void dragViewOutside(const QPoint& pos);
    void dropViewOutside(const QPoint& pos);
    void separateView(View* view);
    void clearAllPanes();
    void clearAllPanesSub(QSplitter* splitter);
    void getAllViews(vector<View*>& out_views);
    void getAllViewsSub(QSplitter* splitter, vector<View*>& out_views);

    void storeLayout(Archive* archive);
    MappingPtr storeSplitterState(QSplitter* splitter, Archive* archive);
    MappingPtr storePaneState(ViewPane* pane, Archive* archive);
    void restoreLayout(Archive* archive);
    QWidget* restoreViewContainer(const Mapping& state, Archive* archive);
    QWidget* restoreSplitter(const Mapping& state, Archive* archive);
    ViewPane* restorePane(const Mapping& state, Archive* archive);
    void resetLayout();

    void removePaneIfEmpty(ViewPane* pane);
    void removePaneSub(QWidget* widgetToRemove, QWidget* widgetToRaise);
    void clearEmptyPanes();
    QWidget* clearEmptyPanesSub(QSplitter* splitter);
};

}


namespace {

class CustomSplitter : public QSplitter
{
public:
    ViewArea::Impl* viewAreaImpl;
    bool defaultOpaqueResize;
    
    CustomSplitter(ViewArea::Impl* viewAreaImpl, QWidget* parent = nullptr)
        : QSplitter(parent),
          viewAreaImpl(viewAreaImpl) {
        defaultOpaqueResize = opaqueResize();
    }

    CustomSplitter(ViewArea::Impl* viewAreaImpl, Qt::Orientation orientation, QWidget* parent = nullptr)
        : QSplitter(orientation, parent),
          viewAreaImpl(viewAreaImpl) {
        defaultOpaqueResize = opaqueResize();
    }

    bool moveSplitterPosition(int d){
        QList<int> s = sizes();
        if(s.size() >= 2){
            s[0] += d;
            s[1] -= d;
        }
        if(s[0] >= 0 && s[1] >= 0){
            setSizes(s);
        }
        return true;
    }

    QSplitterHandle* createHandle();
};


class CustomSplitterHandle : public QSplitterHandle
{
    CustomSplitter* splitter;
    bool isDragging;

public:
    CustomSplitterHandle(CustomSplitter* splitter)
        : QSplitterHandle(splitter->orientation(), splitter),
          splitter(splitter) {
        setFocusPolicy(Qt::WheelFocus);
        isDragging = false;
    }

    virtual void mousePressEvent(QMouseEvent* event){
        if(event->button() == Qt::LeftButton){
            isDragging = true;
            splitter->viewAreaImpl->showViewSizeLabels(splitter);
            if(event->modifiers() & Qt::ShiftModifier){
                splitter->setOpaqueResize(!splitter->defaultOpaqueResize);
            } else {
                splitter->setOpaqueResize(splitter->defaultOpaqueResize);
            }
        }
        QSplitterHandle::mouseMoveEvent(event);
    }

    virtual void mouseMoveEvent(QMouseEvent* event){
        QSplitterHandle::mouseMoveEvent(event);
        if(isDragging){
            splitter->viewAreaImpl->showViewSizeLabels(splitter);
        }
    }
    
    virtual void mouseReleaseEvent(QMouseEvent* event) {
        QSplitterHandle::mouseReleaseEvent(event);
        isDragging = false;
        splitter->setOpaqueResize(splitter->defaultOpaqueResize);
    }

    virtual void keyPressEvent(QKeyEvent* event) {

        bool processed = false;
        int r = 1;
        if(event->modifiers() & Qt::ShiftModifier){
            r = 10;
        }
        int key = event->key();
        if(orientation() == Qt::Horizontal){
            if(key == Qt::Key_Left){
                processed = splitter->moveSplitterPosition(-r);
            } else if(key == Qt::Key_Right){
                processed = splitter->moveSplitterPosition( r);
            }
        } else {
            if(key == Qt::Key_Up){
                processed = splitter->moveSplitterPosition(-r);
            } else if(key == Qt::Key_Down){
                processed = splitter->moveSplitterPosition( r);
            }
        }
        if(processed){
            splitter->viewAreaImpl->showViewSizeLabels(splitter);
        } else {
            QSplitterHandle::keyPressEvent(event);
        }
    }
};


QSplitterHandle* CustomSplitter::createHandle()
{
    return new CustomSplitterHandle(this);
}


ViewPane::ViewPane(ViewArea::Impl* viewAreaImpl, QWidget* parent)
    : QWidget(parent),
      viewAreaImpl(viewAreaImpl)
{
    vbox = new QVBoxLayout(this);
    vbox->setSpacing(0);
    vbox->setContentsMargins(0, 0, 0, 0);

    tabWidget = new TabWidget(this);
    tabWidget->setMovable(true);
    tabWidget->setUsesScrollButtons(true);
    tabWidget->tabBar()->installEventFilter(this);

    if(!viewAreaImpl->viewTabsVisible){
        tabWidget->tabBar()->hide();
    }
    
    vbox->addWidget(tabWidget);

    directView = nullptr;
}


int ViewPane::addView(View* view)
{
    int index = 0;
    if(viewAreaImpl->viewTabsVisible){
        index = tabWidget->addTab(view, view->windowTitle());
    } else {
        if(viewAreaImpl->self->isWindow() && tabWidget->count() == 0 && !directView){
            tabWidget->hide();
            directView = view;
            directView->setParent(this);
            vbox->addWidget(directView);
            directView->show();
        } else {
            tabWidget->show();
            if(directView){
                tabWidget->addTab(directView, directView->windowTitle());
                directView = nullptr;
            }
            index = tabWidget->addTab(view, view->windowTitle());
        }
    }
    connect(view, &QWidget::windowTitleChanged, this, &ViewPane::onViewTitleChanged);
    return index;
}


/**
   \todo Implement this in View.cpp. See the View::bringToFront function.
*/
void ViewPane::onViewTitleChanged(const QString &title)
{
    if(auto view = dynamic_cast<View*>(sender())){
        int index = tabWidget->indexOf(view);
        tabWidget->setTabText(index, view->windowTitle());
    }
}


void ViewPane::removeView(View* view)
{
    if(view == directView){
        vbox->removeWidget(directView);
        directView = nullptr;
    } else {
        tabWidget->removeTab(tabWidget->indexOf(view));
        if(viewAreaImpl->self->isWindow() && tabWidget->count() == 1 && !viewAreaImpl->viewTabsVisible){
            makeDirect();
        }
    }
    disconnect(view, &QWidget::windowTitleChanged, this, &ViewPane::onViewTitleChanged);
}


void ViewPane::setTabVisible(bool on)
{
    if(on){
        if(directView){
            tabWidget->addTab(directView, directView->windowTitle());
            directView = nullptr;
            tabWidget->show();
        }
        tabBar()->show();
    } else {
        if(!directView){
            if(viewAreaImpl->self->isWindow() && tabWidget->count() == 1){
                makeDirect();
            } else {
                tabBar()->hide();
            }
        }
    }
}


void ViewPane::makeDirect()
{
    if(!directView && tabWidget->count() == 1){
        directView = static_cast<View*>(tabWidget->widget(0));
        tabWidget->removeTab(0);
        directView->setParent(this);
        directView->show();
        vbox->addWidget(directView);
        tabWidget->hide();
    }
}


bool ViewPane::eventFilter(QObject* object, QEvent* event)
{
    if(object == tabWidget->tabBar()){
        switch(event->type()){
        case QEvent::MouseButtonPress:
            return viewAreaImpl->viewTabMousePressEvent(this, static_cast<QMouseEvent*>(event));
        case QEvent::MouseButtonDblClick:
            break;
        case QEvent::MouseButtonRelease:
            return viewAreaImpl->viewTabMouseReleaseEvent(static_cast<QMouseEvent*>(event));
        case QEvent::MouseMove:
            return viewAreaImpl->viewTabMouseMoveEvent(this, static_cast<QMouseEvent*>(event));
        default:
            break;
        }
    }
    return false;
}

}


ViewArea::ViewArea(QWidget* parent)
    : QWidget(parent)
{
    if(!parent){
        //setParent(MainWindow::instance());
        //setWindowFlags(Qt::Tool);
        setAttribute(Qt::WA_DeleteOnClose);
    }
    
    impl = new Impl(this);
}


ViewArea::Impl::Impl(ViewArea* self)
    : self(self)
{
    numViews = 0;
    viewTabsVisible = true;
    isMaximizedBeforeFullScreen = false;
    isViewDragging = false;
    draggedView = nullptr;
    dragSrcPane = nullptr;
    dragDestViewArea = nullptr;
    dragDestPane = nullptr;

    vbox = new QVBoxLayout(self);
    vbox->setSpacing(0);
    vbox->setContentsMargins(0, 0, 0, 0);

    rubberBand = new QRubberBand(QRubberBand::Rectangle, self);
    rubberBand->hide();

    viewSizeLabelTimer.setSingleShot(true);
    viewSizeLabelTimer.setInterval(1000);
    viewSizeLabelTimer.sigTimeout().connect([&](){ hideViewSizeLabels(); });

    topSplitter = nullptr;
    needToUpdateDefaultPaneAreas = true;

    viewAreas.push_back(self);
}


ViewArea::~ViewArea()
{
    viewAreas.erase(std::find(viewAreas.begin(), viewAreas.end(), this));
    delete impl;
}


ViewArea::Impl::~Impl()
{
    clearAllPanes();
    delete rubberBand;
}


void ViewArea::setSingleView(View* view)
{
    impl->setSingleView(view);
}


void ViewArea::Impl::setSingleView(View* view)
{
    clearAllPanes();

    viewTabsVisible = false;
    
    topSplitter = new CustomSplitter(this, self);
    vbox->addWidget(topSplitter);

    ViewPane* pane = new ViewPane(this, topSplitter);
    topSplitter->addWidget(pane);
    for(int i=0; i < View::NUM_AREAS; ++i){
        areaToPane[i] = pane;
    }
    needToUpdateDefaultPaneAreas = false;

    addView(pane, view, true);

    defaultViewsToShow.reset();
}


void ViewArea::createDefaultPanes()
{
    impl->createDefaultPanes();
}


void ViewArea::Impl::createDefaultPanes()
{
    clearAllPanes();

    topSplitter = new CustomSplitter(this, self);
    vbox->addWidget(topSplitter);
    topSplitter->setOrientation(Qt::Horizontal);
    
    QSplitter* vSplitter0 = new CustomSplitter(this, Qt::Vertical, topSplitter);
    topSplitter->addWidget(vSplitter0);
    
    areaToPane[View::LEFT_TOP] = new ViewPane(this, vSplitter0);
    vSplitter0->addWidget(areaToPane[View::LEFT_TOP]);
    
    areaToPane[View::LEFT_BOTTOM] = new ViewPane(this, vSplitter0);
    vSplitter0->addWidget(areaToPane[View::LEFT_BOTTOM]);
    
    QSplitter* vSplitter1 = new CustomSplitter(this, Qt::Vertical, topSplitter);
    topSplitter->addWidget(vSplitter1);

    QSplitter* hSplitter1 = new CustomSplitter(this, Qt::Horizontal, vSplitter1);
    vSplitter1->addWidget(hSplitter1);
    
    areaToPane[View::BOTTOM] = new ViewPane(this, vSplitter1);
    vSplitter1->addWidget(areaToPane[View::BOTTOM]);
    
    areaToPane[View::CENTER] = new ViewPane(this, hSplitter1);
    hSplitter1->addWidget(areaToPane[View::CENTER]);
    
    areaToPane[View::RIGHT]  = new ViewPane(this, hSplitter1);
    hSplitter1->addWidget(areaToPane[View::RIGHT]);

    QList<int> sizes;
    sizes << 100 << 600;
    topSplitter->setSizes(sizes);
    sizes.last() = 100;
    vSplitter0->setSizes(sizes);
    sizes.last() = 40;
    vSplitter1->setSizes(sizes);
    sizes.last() = 160;
    hSplitter1->setSizes(sizes);

    needToUpdateDefaultPaneAreas = false;
}


void ViewArea::Impl::detectExistingPaneAreas()
{
    for(int i=0; i < View::NUM_AREAS; ++i){
        areaToPane[i] = nullptr;
    }

    vector<AreaDetectionInfo> infos;
    EdgeContactState edge;
    edge.set();

    ViewPane* firstPane = updateAreaDetectionInfos(topSplitter, edge, infos);

    if(infos.empty()){
        createDefaultPanes();

    } else {
        setBestAreaMatchPane(infos, View::CENTER, firstPane);
        setBestAreaMatchPane(infos, View::LEFT_TOP, firstPane);
        setBestAreaMatchPane(infos, View::LEFT_BOTTOM, firstPane);
        setBestAreaMatchPane(infos, View::RIGHT, firstPane);
        setBestAreaMatchPane(infos, View::BOTTOM, firstPane);
    }

    needToUpdateDefaultPaneAreas = false;
}


/**
   @return The first found pane
*/
ViewPane* ViewArea::Impl::updateAreaDetectionInfos
(QSplitter* splitter, const EdgeContactState& edge, vector<AreaDetectionInfo>& infos)
{
    ViewPane* firstPane = nullptr;
    
    QWidget* childWidgets[2];
    childWidgets[0] = (splitter->count() > 0) ? splitter->widget(0) : 0;
    childWidgets[1] = (splitter->count() > 1) ? splitter->widget(1) : 0;
    bool isSingle = !(childWidgets[0] && childWidgets[1]);

    for(int i=0; i < 2; ++i){
        EdgeContactState currentEdge(edge);
        if(!isSingle){
            if(splitter->orientation() == Qt::Vertical){
                currentEdge.reset((i == 0) ? BOTTOM : TOP);
            } else {
                currentEdge.reset((i == 0) ? RIGHT : LEFT);
            }
        }
        if(childWidgets[i]){
            QSplitter* childSplitter = dynamic_cast<QSplitter*>(childWidgets[i]);
            if(childSplitter){
                ViewPane* pane = updateAreaDetectionInfos(childSplitter, currentEdge, infos);
                if(!firstPane){
                    firstPane = pane;
                }
            } else {
                ViewPane* pane = dynamic_cast<ViewPane*>(childWidgets[i]);
                if(pane){
                    AreaDetectionInfo info;
                    info.pane = pane;

                    // calculate scores for area matching
                    static const int offset = 100000000;
                    int width = pane->width();
                    int height = pane->height();

                    info.scores[View::CENTER] = (4 - currentEdge.count()) * offset + width * height;

                    if(currentEdge.test(LEFT) && !currentEdge.test(RIGHT)){
                        info.scores[View::LEFT] = offset + height;
                        info.scores[View::LEFT_TOP] = offset + height;
                        info.scores[View::LEFT_BOTTOM] = offset + height;
                        if(currentEdge.test(TOP) && !currentEdge.test(BOTTOM)){
                            info.scores[View::LEFT_TOP] += 100;
                        } else if(currentEdge.test(BOTTOM) && !currentEdge.test(TOP)){
                            info.scores[View::LEFT_BOTTOM] += 100;
                        }
                    }
                    if(currentEdge.test(RIGHT) && !currentEdge.test(LEFT)){
                        info.scores[View::RIGHT] = offset + height;
                    }
                    if(currentEdge.test(BOTTOM) && !currentEdge.test(TOP)){
                        info.scores[View::BOTTOM] = offset + width;
                    }
                    
                    infos.push_back(info);

                    if(!firstPane){
                        firstPane = pane;
                    }
                }
            }
        }
    }
    return firstPane;
}


void ViewArea::Impl::setBestAreaMatchPane(vector<AreaDetectionInfo>& infos, View::LayoutArea area, ViewPane* firstPane)
{
    int topScore = 0;
    int topIndex = -1;

    for(int i=0; i < (signed)infos.size(); ++i){
        int s = infos[i].scores[area];
        if(s > topScore){
            topScore = s;
            topIndex = i;
        }
    }

    if(topIndex >= 0){
        areaToPane[area] = infos[topIndex].pane;
    } else {
        areaToPane[area] = firstPane;
    }
}


bool ViewArea::viewTabsVisible() const
{
    return impl->viewTabsVisible;
}


void ViewArea::setViewTabsVisible(bool on)
{
    if(impl->viewTabsVisible != on){
        if(impl->topSplitter){
            impl->setViewTabsVisible(impl->topSplitter, on);
        }
        impl->viewTabsVisible = on;
    }
}


void ViewArea::Impl::setViewTabsVisible(QSplitter* splitter, bool on)
{
    for(int i=0; i < splitter->count(); ++i){
        QSplitter* childSplitter = dynamic_cast<QSplitter*>(splitter->widget(i));
        if(childSplitter){
            setViewTabsVisible(childSplitter, on);
        } else {
            ViewPane* pane = dynamic_cast<ViewPane*>(splitter->widget(i));
            if(pane){
                pane->setTabVisible(on);
            }
        }
    }
}


void ViewArea::Impl::setFullScreen(bool on)
{
    if(self->isWindow()){
        if(on){
            if(!self->isFullScreen()){
                isMaximizedBeforeFullScreen = self->windowState() & Qt::WindowMaximized;
                self->showFullScreen();
            }
        } else if(self->isFullScreen()){
            self->showNormal();
            if(isMaximizedBeforeFullScreen){
                self->showMaximized();
            }
        }
    }
}

    
bool ViewArea::addView(View* view)
{
    return impl->addView(view);
}


bool ViewArea::Impl::addView(View* view)
{
    if(view->viewArea() == self){
        return false;
    }
    if(isBeforeDoingInitialLayout){
        if(!defaultViewsToShow){
            defaultViewsToShow.reset(new vector<View*>());
        }
        defaultViewsToShow->push_back(view);
    } else {
        if(needToUpdateDefaultPaneAreas){
            detectExistingPaneAreas();
        }
        addView(areaToPane[view->defaultLayoutArea()], view, false);
    }

    return true;
}


void ViewArea::Impl::addView(ViewPane* pane, View* view, bool makeCurrent)
{
    if(view->viewArea()){
        view->viewArea()->impl->removeView(view);
    }

    int index = pane->addView(view);
    if(makeCurrent){
        pane->setCurrentIndex(index);
    }
    view->setViewArea(self);

    ++numViews;

    if(numViews == 1){
        self->setWindowTitle(view->windowTitle());
    } else if(numViews == 2){
        self->setWindowTitle("Views");
        self->setViewTabsVisible(true);
    }
}


bool ViewArea::removeView(View* view)
{
    return impl->removeView(view);
}


bool ViewArea::Impl::removeView(View* view)
{
    bool removed = false;

    if(view->viewArea() == self){
        ViewPane* pane = nullptr;
        for(QWidget* widget = view->parentWidget(); widget; widget = widget->parentWidget()){
            pane = dynamic_cast<ViewPane*>(widget);
            if(pane){
                break;
            }
        }
        if(pane){
            removed = removeView(pane, view, false);
        }
    }

    return removed;
}


bool ViewArea::Impl::removeView(ViewPane* pane, View* view, bool isMovingInViewArea)
{
    bool removed = false;

    //! \todo check if the owner view area is same as this
    if(view->viewArea() == self){
        pane->removeView(view);
        if(pane->count() > 0){
            pane->setCurrentIndex(0);
        }
        --numViews;
        removed = true;

        if(!isMovingInViewArea){
            view->hide();
            view->setParent(0);
            removePaneIfEmpty(pane);
            if(self->isWindow()){
                if(numViews == 1){
                    View* existingView = findFirstView(topSplitter);
                    if(existingView){
                        self->setWindowTitle(existingView->windowTitle());
                    }
                } else if(numViews == 0){
                    self->deleteLater();
                }
            }
        }
        view->setViewArea(0);
    }
    return removed;
}


View* ViewArea::Impl::findFirstView(QSplitter* splitter)
{
    View* view = nullptr;
    for(int i=0; i < splitter->count(); ++i){
        QSplitter* childSplitter = dynamic_cast<QSplitter*>(splitter->widget(i));
        if(childSplitter){
            view = findFirstView(childSplitter);
        } else {
            ViewPane* pane = dynamic_cast<ViewPane*>(splitter->widget(i));
            if(pane && pane->count() > 0){
                view = pane->view(0);
            }
        }
        if(view){
            break;
        }
    }
    return view;
}


int ViewArea::numViews() const
{
    return impl->numViews;
}


void ViewArea::Impl::getVisibleViews(vector<View*>& out_views, QSplitter* splitter)
{
    getVisibleViewsIter(splitter ? splitter : topSplitter, out_views);
}


void ViewArea::Impl::getVisibleViewsIter(QSplitter* splitter, vector<View*>& out_views)
{
    QList<int> sizes = splitter->sizes();
    for(int i=0; i < splitter->count(); ++i){
        QSplitter* childSplitter = dynamic_cast<QSplitter*>(splitter->widget(i));
        if(childSplitter){
            getVisibleViewsIter(childSplitter, out_views);
        } else {
            if(sizes[i] > 0){
                ViewPane* pane = dynamic_cast<ViewPane*>(splitter->widget(i));
                if(pane){
                    View* view = pane->currentView();
                    if(view){
                        out_views.push_back(view);
                    }
                }
            }
        }
    }
}


void ViewArea::Impl::showViewSizeLabels(QSplitter* splitter)
{
    vector<View*> views;
    getVisibleViews(views, splitter);
    
    for(size_t i=0; i < views.size(); ++i){
        View* view = views[i];
        QLabel* label = nullptr;
        if(i < viewSizeLabels.size()){
            label = viewSizeLabels[i];
        } else {
            label = new QLabel(self);
            label->setFrameStyle(QFrame::Box | QFrame::Raised);
            label->setLineWidth(0);
            label->setMidLineWidth(1);
            label->setAutoFillBackground(true);
            label->setAlignment(Qt::AlignCenter);
            viewSizeLabels.push_back(label);
        }
        label->setText(QString("%1 x %2").arg(view->width()).arg(view->height()));
        QPoint p = view->viewAreaPos();
        QSize s = label->size();
        int x = p.x() + view->width() / 2 - s.width() / 2;
        int y = p.y() + view->height() / 2 - s.height() / 2;
        label->move(x, y);
        label->show();
    }

    viewSizeLabelTimer.start();
}


void ViewArea::Impl::hideViewSizeLabels()
{
    for(size_t i=0; i < viewSizeLabels.size(); ++i){
        viewSizeLabels[i]->hide();
    }
}


void ViewArea::keyPressEvent(QKeyEvent* event)
{
    event->ignore();
    
    switch(event->key()){

    case Qt::Key_F12:
        setViewTabsVisible(!impl->viewTabsVisible);
        event->accept();
        break;
    }

    if(isWindow()){
        switch(event->key()){
        case Qt::Key_F11:
            impl->setFullScreen(!isFullScreen());
            event->accept();
            break;
        }
    }
}


void ViewArea::restoreAllViewAreaLayouts(ArchivePtr archive)
{
    Impl* mainViewAreaImpl = MainWindow::instance()->viewArea()->impl;
    
    if(archive){
        Listing& layouts = *archive->findListing("viewAreas");
        if(layouts.isValid()){
            QDesktopWidget* desktop = QApplication::desktop();
            const int numScreens = desktop->screenCount();
            
            for(int i=0; i < layouts.size(); ++i){
                Mapping& layout = *layouts[i].toMapping();
                Archive* contents = dynamic_cast<Archive*>(layout.get("contents").toMapping());
                if(contents){
                    contents->inheritSharedInfoFrom(*archive);
                    const string type = layout.get("type").toString();

                    if(type == "embedded"){
                        mainViewAreaImpl->restoreLayout(contents);
                        mainViewAreaImpl->self->setViewTabsVisible(layout.get("tabs", mainViewAreaImpl->viewTabsVisible));

                    } else if(type == "independent"){
                        ViewArea* viewWindow = new ViewArea;
                        viewWindow->impl->viewTabsVisible = layout.get("tabs", true);
                        viewWindow->impl->restoreLayout(contents);

                        if(viewWindow->impl->numViews == 0){
                            delete viewWindow;
                        } else {
                            const Listing& geo = *layout.findListing("geometry");
                            if(geo.isValid() && geo.size() == 4){
                                // -1 means the primary screen
                                int screen = -1;
                                if(layout.read("screen", screen)){
                                    if(screen >= numScreens){
                                        screen = -1;
                                    }
                                }
                                const QRect s = desktop->screenGeometry(screen);
                                const QRect r(geo[0].toInt(), geo[1].toInt(), geo[2].toInt(), geo[3].toInt());
                                viewWindow->setGeometry(r.translated(s.x(), s.y()));
                            }
                            if(layout.get("fullScreen", false)){
                                layout.read("maximized", viewWindow->impl->isMaximizedBeforeFullScreen);
                                viewWindow->showFullScreen();
                            } else {
                                if(layout.get("maximized"), false){
                                    viewWindow->showMaximized();
                                } else {
                                    viewWindow->show();
                                }
                            }
                        }
                    }
                }
            }
        }
    }
    if(isBeforeDoingInitialLayout){
        mainViewAreaImpl->resetLayout();
    }
}


void ViewArea::restoreLayout(ArchivePtr archive)
{
    impl->restoreLayout(archive.get());
}


void ViewArea::Impl::restoreLayout(Archive* archive)
{
    clearAllPanes();

    QWidget* topWidget = restoreViewContainer(*archive, archive);

    topSplitter = dynamic_cast<QSplitter*>(topWidget);
    
    if(!topSplitter){
        topSplitter = new CustomSplitter(this);
        ViewPane* topPane = dynamic_cast<ViewPane*>(topWidget);
        if(!topPane){
            topPane = new ViewPane(this);
        }
        topSplitter->addWidget(topPane);
    }
    vbox->addWidget(topSplitter);
    
    needToUpdateDefaultPaneAreas = true;

    isBeforeDoingInitialLayout = false;
    defaultViewsToShow.reset();
}


void ViewArea::Impl::clearAllPanes()
{
    if(topSplitter){
        clearAllPanesSub(topSplitter);
        delete topSplitter;
        topSplitter = nullptr;
    }
    numViews = 0;
    needToUpdateDefaultPaneAreas = true;
}


void ViewArea::Impl::clearAllPanesSub(QSplitter* splitter)
{
    for(int i=0; i < splitter->count(); ++i){
        QSplitter* childSplitter = dynamic_cast<QSplitter*>(splitter->widget(i));
        if(childSplitter){
            clearAllPanesSub(childSplitter);
        } else {
            ViewPane* pane = dynamic_cast<ViewPane*>(splitter->widget(i));
            if(pane){
                while(pane->count() > 0){
                    int index = pane->count() - 1;
                    View* view = pane->view(index);
                    if(view){
                        pane->removeView(view);
                        view->hide();
                        view->setParent(0);
                        view->setViewArea(0);
                    }
                }
            }
        }
    }
}


void ViewArea::Impl::getAllViews(vector<View*>& out_views)
{
    getAllViewsSub(topSplitter, out_views);
}


void ViewArea::Impl::getAllViewsSub(QSplitter* splitter, vector<View*>& out_views)
{
    for(int i=0; i < splitter->count(); ++i){
        QSplitter* childSplitter = dynamic_cast<QSplitter*>(splitter->widget(i));
        if(childSplitter){
            getAllViewsSub(childSplitter, out_views);
        } else {
            ViewPane* pane = dynamic_cast<ViewPane*>(splitter->widget(i));
            if(pane){
                for(int i=0; i < pane->count(); ++i){
                    View* view = pane->view(i);
                    if(view){
                        out_views.push_back(view);
                    }
                }
            }
        }
    }
}


QWidget* ViewArea::Impl::restoreViewContainer(const Mapping& state, Archive* archive)
{
    QWidget* widget = nullptr;
    string type;
    if(state.read("type", type)){
        if(type == "splitter"){
            widget = restoreSplitter(state, archive);
        } else if(type == "pane"){
            widget = restorePane(state, archive);
        }
    }
    return widget;
}


QWidget* ViewArea::Impl::restoreSplitter(const Mapping& state, Archive* archive)
{
    QWidget* widget = nullptr;
    QWidget* childWidgets[2] = { 0, 0 };
    
    const Listing& children = *state.findListing("children");
    if(children.isValid()){
        int numChildren = std::min(children.size(), 2);
        for(int i=0; i < numChildren; ++i){
            if(children[i].isMapping()){
                const Mapping& childState = *children[i].toMapping();
                childWidgets[i] = restoreViewContainer(childState, archive);
            }
        }
        if(!childWidgets[0] || !childWidgets[1]){
            for(int i=0; i < 2; ++i){
                if(childWidgets[i]){
                    widget = childWidgets[i];
                    break;
                }
            }
        } else {
            QSplitter* splitter = new CustomSplitter(this);
            string orientation;
            if(state.read("orientation", orientation)){
                splitter->setOrientation((orientation == "vertical") ? Qt::Vertical : Qt::Horizontal);
            }
            splitter->addWidget(childWidgets[0]);
            splitter->addWidget(childWidgets[1]);

            const Listing& sizes = *state.findListing("sizes");
            if(sizes.isValid() && sizes.size() == 2){
                QList<int> s;
                int size;
                for(int i=0; i < 2; ++i){
                    if(sizes[i].read(size)){
                        s.push_back(size);
                    }
                }
                splitter->setSizes(s);
            }
            widget = splitter;
        }
    }
    return widget;
}


ViewPane* ViewArea::Impl::restorePane(const Mapping& state, Archive* archive)
{
    ViewPane* pane = nullptr;
    const Listing& views = *state.findListing("views");
    
    if(views.isValid() && !views.empty()){
        pane = new ViewPane(this);
        int currentId = 0;
        state.read("current", currentId);
        for(int i=0; i < views.size(); ++i){
            const ValueNode& node = views[i];
            View* view = nullptr;
            int id;
            bool isCurrent = false;
            if(node.read(id)){
                view = archive->findView(id);
                if(view){
                    isCurrent = (id == currentId);
                }
            }
            if(view){
                addView(pane, view, isCurrent);
            }
        }
        if(pane->count() == 0){
            delete pane;
            pane = nullptr;
        }
    }
    return pane;
}


void ViewArea::storeAllViewAreaLayouts(ArchivePtr archive)
{
    ListingPtr layouts = new Listing;
    for(size_t i=0; i < viewAreas.size(); ++i){
        ArchivePtr layout = new Archive;
        layout->inheritSharedInfoFrom(*archive);
        viewAreas[i]->storeLayout(layout);
        layouts->append(layout);
    }
    if(!layouts->empty()){
        archive->insert("viewAreas", layouts);
    }
}


void ViewArea::storeLayout(ArchivePtr archive)
{
    impl->storeLayout(archive.get());
}


void ViewArea::Impl::storeLayout(Archive* archive)
{
    QDesktopWidget* desktop = QApplication::desktop();
    const int primaryScreen = desktop->primaryScreen();
    
    try {
        MappingPtr state = storeSplitterState(topSplitter, archive);
        if(state){
            if(!self->isWindow()){
                archive->write("type", "embedded");
            } else {
                archive->write("type", "independent");
                const int screen = desktop->screenNumber(self->pos());
                if(screen != primaryScreen){
                    archive->write("screen", screen);
                }
                const QRect s = desktop->screenGeometry(screen);
                const QRect r = self->geometry().translated(-s.x(), -s.y());
                Listing* geometry = archive->createFlowStyleListing("geometry");
                geometry->append(r.x());
                geometry->append(r.y());
                geometry->append(r.width());
                geometry->append(r.height());
                if(self->isFullScreen()){
                    archive->write("fullScreen", true);
                    archive->write("maximized", isMaximizedBeforeFullScreen);
                } else {
                    archive->write("fullScreen", false);
                    archive->write("maximized", self->isMaximized());
                }
            }
            archive->write("tabs", viewTabsVisible);
            archive->insert("contents", state);
        }
    }
    catch(const ValueNode::Exception& ex){
        MessageView::instance()->putln(ex.message());
    }
}


MappingPtr ViewArea::Impl::storeSplitterState(QSplitter* splitter, Archive* archive)
{
    MappingPtr state = new Mapping;

    ListingPtr children = new Listing;

    for(int i=0; i < splitter->count(); ++i){
        QSplitter* childSplitter = dynamic_cast<QSplitter*>(splitter->widget(i));
        if(childSplitter){
            MappingPtr childState = storeSplitterState(childSplitter, archive);
            if(childState){
                children->append(childState);
            }
        } else {
            ViewPane* pane = dynamic_cast<ViewPane*>(splitter->widget(i));
            if(pane && pane->count() > 0){
                MappingPtr childState = storePaneState(pane, archive);
                if(childState){
                    children->append(childState);
                }
            }
        }
    }

    const int numChildren = children->size();
    if(numChildren == 0){
        state.reset();
    } else if(numChildren == 1){
        state = children->at(0)->toMapping();
    } else if(numChildren == 2){
        state->write("type", "splitter");
        state->write("orientation", (splitter->orientation() == Qt::Vertical) ? "vertical" : "horizontal");
        Listing* sizeSeq = state->createFlowStyleListing("sizes");
        QList<int> sizes = splitter->sizes();
        for(int i=0; i < sizes.size(); ++i){
            sizeSeq->append(sizes[i]);
        }
        state->insert("children", children);
    }

    return state;
}


MappingPtr ViewArea::Impl::storePaneState(ViewPane* pane, Archive* archive)
{
    MappingPtr state = new Mapping;
    
    state->write("type", "pane");
    
    Listing* views = state->createFlowStyleListing("views");
    const int n = pane->count();
    for(int i=0; i < n; ++i){
        View* view = pane->view(i);
        int id = archive->getViewId(view);
        if(id >= 0){
            views->append(id);
            if(n >= 2 && (i == pane->currentIndex())){
                state->write("current", id);
            }
        }
    }
    if(views->empty()){
        state.reset();
    }
    return state;
}


void ViewArea::resetAllViewAreaLayouts()
{
    ViewArea* mainViewArea = MainWindow::instance()->viewArea();
    mainViewArea->impl->resetLayout();
    
    //! \todo close all the independent view windows
}


void ViewArea::resetLayout()
{
    impl->resetLayout();
}


void ViewArea::Impl::resetLayout()
{
    if(isBeforeDoingInitialLayout){
        isBeforeDoingInitialLayout = false;
        createDefaultPanes();
        if(defaultViewsToShow){
            for(size_t i=0; i < defaultViewsToShow->size(); ++i){
                addView((*defaultViewsToShow)[i]);
            }
            defaultViewsToShow.reset();
        }
    } else {
        vector<View*> views;
        getAllViews(views);
        clearAllPanes();
        createDefaultPanes();
        for(size_t i=0; i < views.size(); ++i){
            addView(views[i]);
        }
    }
}


bool ViewArea::Impl::viewTabMousePressEvent(ViewPane* pane, QMouseEvent* event)
{
    if(event->button() == Qt::LeftButton){
        tabDragStartPosition = event->pos();
    } else if(event->button() == Qt::RightButton){
        if(View* view = pane->currentView()){
            viewMenuManager.setNewPopupMenu(self);

            view->onAttachedMenuRequest(viewMenuManager);

            if(viewMenuManager.numItems() > 0){
                viewMenuManager.addSeparator();
            }
            viewMenuManager.addItem(_("Separate the view"))
                ->sigTriggered().connect([this, view](){ separateView(view); });
                
            viewMenuManager.popupMenu()->popup(event->globalPos());
        }
    }
    return false;
}


bool ViewArea::Impl::viewTabMouseMoveEvent(ViewPane* pane, QMouseEvent* event)
{
    if(!isViewDragging){
        if(event->buttons() & Qt::LeftButton){
            if((event->pos() - tabDragStartPosition).manhattanLength() > QApplication::startDragDistance()){
                if(!pane->tabBar()->geometry().contains(event->pos())){
                    draggedView = pane->currentView();
                    if(draggedView){
                        isViewDragging = true;
                        dragSrcPane = pane;

                        QWidget* toplevel = pane;
                        while(toplevel->parentWidget()){
                            toplevel = toplevel->parentWidget();
                        }
                        QSize s = toplevel->frameGeometry().size();
                        draggedViewWindowSize.setWidth(draggedView->width() + s.width() - toplevel->width());
                        draggedViewWindowSize.setHeight(draggedView->height() + s.height() - toplevel->height());
                        
                        QApplication::setOverrideCursor(Qt::ClosedHandCursor);
                    }
                }
            }
        }
    } else {
        ViewArea* prevViewArea = dragDestViewArea;
        dragDestViewArea = nullptr;
        dragDestPane = nullptr;

        // find a window other than the rubber band
        QPoint p = event->globalPos();
        QWidget* window = nullptr;
        for(int i=0; i < 3; ++i){
            window = QApplication::topLevelAt(p);
            if(window && !dynamic_cast<QRubberBand*>(window)){
                break;
            }
            p += QPoint(-1, -1);
        }
        if(window){
            dragDestViewArea = dynamic_cast<ViewArea*>(window);
            if(!dragDestViewArea){
                if(MainWindow* mainWindow = dynamic_cast<MainWindow*>(window)){
                    ViewArea* viewArea = mainWindow->viewArea();
                    if(viewArea->rect().contains(viewArea->mapFromGlobal(event->globalPos()))){
                        dragDestViewArea = viewArea;
                    }
                }
            }
        }
        if(dragDestViewArea){
            QWidget* pointed = dragDestViewArea->childAt(dragDestViewArea->mapFromGlobal(event->globalPos()));
            while(pointed){
                dragDestPane = dynamic_cast<ViewPane*>(pointed);
                if(dragDestPane){
                    dragView(event);
                    break;
                }
                pointed = pointed->parentWidget();
            }
        } else {
            dragViewOutside(event->globalPos());
        }
        if(prevViewArea != dragDestViewArea){
            if(prevViewArea){
                prevViewArea->impl->rubberBand->hide();
            } else {
                rubberBand->hide();
            }
        }
    }
    return false;
}


bool ViewArea::Impl::viewTabMouseReleaseEvent(QMouseEvent *event)
{
    if(isViewDragging){
        bool isMovingInViewArea = (self == dragDestViewArea);
        removeView(dragSrcPane, draggedView, isMovingInViewArea);
        if(!dragDestPane){
            rubberBand->hide();
            dropViewOutside(event->globalPos());
        } else {
            Impl* destImpl = dragDestViewArea->impl;
            destImpl->rubberBand->hide();
            destImpl->needToUpdateDefaultPaneAreas = true;
            if(isViewDraggingOnOuterEdge){
                destImpl->dropViewToOuterEdge(draggedView);
            } else {
                destImpl->dropViewInsidePane(dragDestPane, draggedView, dropEdge);
            }
        }
        if(isMovingInViewArea){
            removePaneIfEmpty(dragSrcPane);
        }
        
        QApplication::restoreOverrideCursor();
    }
    isViewDragging = false;
    draggedView = nullptr;
    dragDestViewArea = nullptr;
    dragDestPane = nullptr;
    
    return false;
}


void ViewArea::Impl::showRectangle(QRect r)
{
    rubberBand->setParent(self);
    rubberBand->setGeometry(r);
    rubberBand->show();
}


void ViewArea::Impl::dragView(QMouseEvent* event)
{
    dropEdge = LEFT;
        
    QPoint p = dragDestViewArea->mapFromGlobal(event->globalPos());
    const int w = dragDestViewArea->width();
    const int h = dragDestViewArea->height();
    
    int distance[4];
    distance[LEFT] = p.x();
    distance[TOP] = p.y();
    distance[RIGHT] = w - p.x();
    distance[BOTTOM] = h - p.y();
        
    for(int i=TOP; i <= BOTTOM; ++i){
        if(distance[dropEdge] > distance[i]){
            dropEdge = i;
        }
    }

    if(distance[dropEdge] < 8){
        isViewDraggingOnOuterEdge = true;
        dragViewOnOuterEdge();
    } else {
        isViewDraggingOnOuterEdge = false;
        dragViewInsidePane(dragDestPane->mapFromGlobal(event->globalPos()));
    }
}

    
void ViewArea::Impl::dragViewInsidePane(const QPoint& posInDestPane)
{
    dropEdge = LEFT;
        
    const int w = dragDestPane->width();
    const int h = dragDestPane->height();
    
    int distance[4];
    distance[LEFT] = posInDestPane.x();
    distance[TOP] = posInDestPane.y();
    distance[RIGHT] = w - posInDestPane.x();
    distance[BOTTOM] = h - posInDestPane.y();
        
    for(int i=TOP; i <= BOTTOM; ++i){
        if(distance[dropEdge] > distance[i]){
            dropEdge = i;
        }
    }

    QRect r;
    if(SPLIT_DISTANCE_THRESHOLD < distance[dropEdge]){
        r.setRect(0, 0, w, h);
        dropEdge = OVER;
    } else if(dropEdge == LEFT){
        r.setRect(0, 0, w / 2, h);
    } else if(dropEdge == TOP){
        r.setRect(0, 0, w, h /2);
    } else if(dropEdge == RIGHT){
        r.setRect(w / 2, 0, w / 2, h);
    } else if(dropEdge == BOTTOM){
        r.setRect(0, h / 2, w, h / 2);
    }

    r.translate(dragDestViewArea->mapFromGlobal(dragDestPane->mapToGlobal(QPoint(0, 0))));
    dragDestViewArea->impl->showRectangle(r);
}


void ViewArea::Impl::dropViewInsidePane(ViewPane* pane, View* view, int dropEdge)
{
    if(dropEdge == OVER){
        addView(pane, view, true);

    } else {
        QSize destSize = pane->size();

        QSplitter* parentSplitter = static_cast<QSplitter*>(pane->parentWidget());

        if(parentSplitter->count() >= 2){
            QList<int> sizes = parentSplitter->sizes();
            QSplitter* newSplitter = new CustomSplitter(this, parentSplitter);
            parentSplitter->insertWidget(parentSplitter->indexOf(pane), newSplitter);
            newSplitter->addWidget(pane);
            parentSplitter->setSizes(sizes);
            parentSplitter = newSplitter;
        }
        
        ViewPane* newViewPane = new ViewPane(this, parentSplitter);

        if(dropEdge == LEFT){
            parentSplitter->setOrientation(Qt::Horizontal);
            parentSplitter->insertWidget(0, newViewPane);
        } else if(dropEdge == RIGHT){
            parentSplitter->setOrientation(Qt::Horizontal);
            parentSplitter->insertWidget(1, newViewPane);
        } else if(dropEdge == TOP){
            parentSplitter->setOrientation(Qt::Vertical);
            parentSplitter->insertWidget(0, newViewPane);
        } else {
            parentSplitter->setOrientation(Qt::Vertical);
            parentSplitter->insertWidget(1, newViewPane);
        }
        addView(newViewPane, view, true);

        int half;
        if(parentSplitter->orientation() == Qt::Horizontal){
            half = destSize.height() / 2;
        } else {
            half = destSize.width() / 2;
        }
        QList<int> sizes;
        sizes << half << half;
        parentSplitter->setSizes(sizes);
    }
}


void ViewArea::Impl::dragViewOnOuterEdge()
{
    QRect r;
    int w = dragDestViewArea->width();
    int h = dragDestViewArea->height();
    if(dropEdge == LEFT){
        r.setRect(0, 0, w / 2, h);
    } else if(dropEdge == TOP){
        r.setRect(0, 0, w, h /2);
    } else if(dropEdge == RIGHT){
        r.setRect(w / 2, 0, w / 2, h);
    } else if(dropEdge == BOTTOM){
        r.setRect(0, h / 2, w, h / 2);
    }
    dragDestViewArea->impl->showRectangle(r);
}


void ViewArea::Impl::dropViewToOuterEdge(View* view)
{
    QSize size = topSplitter->size();

    if(topSplitter->count() >= 2){
        QSplitter* newTopSplitter = new CustomSplitter(this, self);
        newTopSplitter->addWidget(topSplitter);
        topSplitter = newTopSplitter;
        vbox->addWidget(topSplitter);
    }
    ViewPane* newViewPane = new ViewPane(this, topSplitter);

    if(dropEdge == LEFT){
        topSplitter->setOrientation(Qt::Horizontal);
        topSplitter->insertWidget(0, newViewPane);
    } else if(dropEdge == RIGHT){
        topSplitter->setOrientation(Qt::Horizontal);
        topSplitter->addWidget(newViewPane);
    } else if(dropEdge == TOP){
        topSplitter->setOrientation(Qt::Vertical);
        topSplitter->insertWidget(0, newViewPane);
    } else {
        topSplitter->setOrientation(Qt::Vertical);
        topSplitter->addWidget(newViewPane);
    }
    addView(newViewPane, view, true);

    int half;
    if(topSplitter->orientation() == Qt::Horizontal){
        half = size.height() / 2;
    } else {
        half = size.width() / 2;
    }
    QList<int> sizes;
    sizes << half << half;
    topSplitter->setSizes(sizes);
}


void ViewArea::Impl::dragViewOutside(const QPoint& pos)
{
    rubberBand->setParent(0);
    rubberBand->setGeometry(QRect(pos, draggedViewWindowSize));
    rubberBand->show();
}


void ViewArea::Impl::dropViewOutside(const QPoint& pos)
{
    ViewArea* viewWindow = new ViewArea;
    viewWindow->setSingleView(draggedView);
    viewWindow->move(pos);
    viewWindow->resize(draggedViewWindowSize);
    viewWindow->show();
}


void ViewArea::Impl::separateView(View* view)
{
    QPoint pos = view->mapToGlobal(QPoint(0, 0));
    removeView(view);
    ViewArea* viewWindow = new ViewArea;
    viewWindow->setSingleView(view);
    viewWindow->setGeometry(pos.x(), pos.y(), view->width(), view->height());
    viewWindow->show();
}    
    


void ViewArea::Impl::removePaneIfEmpty(ViewPane* pane)
{
    if(pane->count() == 0){
        removePaneSub(pane, nullptr);
        needToUpdateDefaultPaneAreas = true;
    }
}


void ViewArea::Impl::removePaneSub(QWidget* widgetToRemove, QWidget* widgetToRaise)
{
    if(QSplitter* splitter = dynamic_cast<QSplitter*>(widgetToRemove->parentWidget())){
        QList<int> sizes;
        if(widgetToRaise){
            sizes = splitter->sizes();
            splitter->insertWidget(splitter->indexOf(widgetToRemove), widgetToRaise);
        }
        widgetToRemove->hide();
        widgetToRemove->setParent(nullptr);
        widgetToRemove->deleteLater();

        if(splitter->count() >= 2){
            splitter->setSizes(sizes);
        } else if(splitter->count() == 1){
            removePaneSub(splitter, splitter->widget(0));
        } else {
            removePaneSub(splitter, 0);
        }
    }
}


void ViewArea::Impl::clearEmptyPanes()
{
    QWidget* widget = clearEmptyPanesSub(topSplitter);
    if(widget != topSplitter){
        if(widget){
            if(QSplitter* splitter = dynamic_cast<QSplitter*>(widget)){
                splitter->setParent(self);
                topSplitter->hide();
                topSplitter->deleteLater();
                vbox->addWidget(splitter);
                topSplitter = splitter;
            }
        }
    }
}


QWidget* ViewArea::Impl::clearEmptyPanesSub(QSplitter* splitter)
{
    QList<int> sizes = splitter->sizes();
    int i = 0;
    while(i < splitter->count()){
        if(QSplitter* childSplitter = dynamic_cast<QSplitter*>(splitter->widget(i))){
            QWidget* widget = clearEmptyPanesSub(childSplitter);
            if(widget == childSplitter){
                ++i;
            } else {
                if(widget){
                    splitter->insertWidget(i++, widget);
                }
                // "delete childSplitter" causes a crash
                childSplitter->hide();
                childSplitter->setParent(0);
                childSplitter->deleteLater();
            }
        } else if(ViewPane* pane = dynamic_cast<ViewPane*>(splitter->widget(i))){
            if(pane->count() > 0){
                ++i;
            } else {
                // "delete pane" causes a crash
                pane->hide();
                pane->setParent(0);
                pane->deleteLater();
                needToUpdateDefaultPaneAreas = true;
            }
        }
    }
    QWidget* validWidget = nullptr;
    if(splitter->count() >= 2){
        splitter->setSizes(sizes);
        validWidget = splitter;
    } else if(splitter->count() == 1){
        validWidget = splitter->widget(0);
        needToUpdateDefaultPaneAreas = true;
    }
    return validWidget;
}
