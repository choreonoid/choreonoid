/**
   @author Shin'ichiro Nakaoka
*/

#include "MainWindow.h"
#include "App.h"
#include "View.h"
#include "ToolBar.h"
#include "ToolBarArea.h"
#include "InfoBar.h"
#include "MenuManager.h"
#include "AppConfig.h"
#include "LazyCaller.h"
#include "Archive.h"
#include "ViewManager.h"
#include "TimeBar.h"
#include <QSplitter>
#include <QTabWidget>
#include <QTabBar>
#include <QMouseEvent>
#include <QApplication>
#include <QBoxLayout>
#include <QRubberBand>
#include <QFileDialog>
#include <boost/bind.hpp>
#include <boost/tuple/tuple.hpp>
#include <boost/filesystem.hpp>
#include <set>
#include <bitset>
#include <iostream>
#include "gettext.h"

using namespace std;
using namespace cnoid;

namespace {

const bool TRACE_FUNCTIONS = false;

MainWindow* mainWindow = 0;

enum DropArea { OVER = -1, LEFT = 0, TOP, RIGHT, BOTTOM, NUM_DROP_AREAS };
const int SPLIT_DISTANCE_THRESHOLD = 35;

class TabWidget : public QTabWidget
{
public:
    TabWidget(MainWindowImpl* mwi, QWidget* parent = 0);

    int addView(View* view) {
        return addTab(view, view->windowTitle());
    }
            
    QTabBar* tabBar() const { return QTabWidget::tabBar(); }
    virtual bool eventFilter(QObject* object, QEvent* event);

    // For the whole tab widget dragging
    //virtual void mousePressEvent(QMouseEvent *event);
    //virtual void mouseMoveEvent(QMouseEvent *event);
    //virtual void mouseReleaseEvent(QMouseEvent *event);

    MainWindowImpl* mwi;
};

}

namespace cnoid {
    
class MainWindowImpl
{
public:
    MainWindow* self;

    MainWindowImpl(MainWindow* self, const char* appName, ExtensionManager* ext);
    ~MainWindowImpl();

    std::vector<ToolBar*> toolBars;

    TabWidget* areaToPane[View::NUM_AREAS];

    struct AreaDetectionInfo {
        AreaDetectionInfo() {
            for(int i=0; i < View::NUM_AREAS; ++i){
                scores[i] = 0;
            }
        }
        TabWidget* pane;
        int scores[View::NUM_AREAS];
    };

    typedef bitset<NUM_DROP_AREAS> EdgeContactState;

    bool needToUpdateDefaultPaneAreas;

    ToolBarArea* toolBarArea;
    QWidget* centralWidget;
    QVBoxLayout* centralVBox;
    QSplitter* topSplitter;

    string appName;

    MappingPtr config;
    ArchivePtr initialLayoutArchive;
    Action* storeLastLayoutCheck;
        
    bool isBeforeShowing;
    bool isBeforeDoingInitialLayout;
    bool isMaximized;
    bool isMaximizedJustBeforeFullScreen;
    bool isFullScreen;
    QSize normalStateSize;
    QString currentLayoutFolder;
    Action* fullScreenCheck;

    QPoint tabDragStartPosition;
    bool isViewDragging;
    TabWidget* dragSrcPane;
    TabWidget* dragDestPane;
    bool isViewDraggingOutsidePane;
    int dropEdge;
    QRubberBand* rubberBand;

    MenuManager viewMenuManager;

    void setupMenus(ExtensionManager* ext);
    void createDefaultPanes();

    void detectExistingPaneAreas();
    TabWidget* updateAreaDetectionInfos(QSplitter* splitter, const EdgeContactState& edge, vector<AreaDetectionInfo>& infos);
    void setBestAreaMatchPane(vector<AreaDetectionInfo>& infos, View::LayoutArea area, TabWidget* firstPane);

    bool addView(View* view);
    bool addView(TabWidget* pane, View* view);
    void putViewOnDefaultArea(View* view);
    bool removeView(View* view);
    void removeView(TabWidget* pane, View* view);

    void showFirst();
    void onFullScreenToggled(bool on);
    void resizeEvent(QResizeEvent* event);
    void keyPressEvent(QKeyEvent* event);

    bool viewTabMousePressEvent(TabWidget* pane, QMouseEvent* event);
    bool viewTabMouseMoveEvent(TabWidget* pane, QMouseEvent* event);
    bool viewTabMouseReleaseEvent(TabWidget* pane, QMouseEvent *event);
        
    void startViewDrag(View* view);
    void dragView(QMouseEvent* event);
    void dragViewInsidePane(const QPoint& posInDestPane);
    void dropViewInsidePane();
    void dragViewOutsidePane();
    void dropViewOutsidePane();
    void removePaneIfEmpty(TabWidget* pane);
    void clearAllPanes();
    void clearAllPanesSub(QSplitter* splitter);
    void getAllViews(vector<View*>& out_views);
    void getAllViewsSub(QSplitter* splitter, vector<View*>& out_views);
        
    void storeLayout(ArchivePtr& archive);
    MappingPtr storeSplitterState(QSplitter* splitter, ArchivePtr& archive);
    MappingPtr storePaneState(TabWidget* pane, ArchivePtr& archive);
    void restoreLayout(ArchivePtr& archive);
    QWidget* restoreSplitterState(const Mapping& state, TabWidget*& out_firstPane, ArchivePtr& archive);
    TabWidget* restorePaneState(const Mapping& state, ArchivePtr& archive);
    void resetLayout();
    void resetViewLayout();
    void storeWindowStateConfig();
};
}


TabWidget::TabWidget(MainWindowImpl* mwi, QWidget* parent)
    : QTabWidget(parent),
      mwi(mwi)
{
    setMovable(true);
    setUsesScrollButtons(true);
    tabBar()->installEventFilter(this);
}


bool TabWidget::eventFilter(QObject* object, QEvent* event)
{
    if(object == tabBar()){
        switch(event->type()){
        case QEvent::MouseButtonPress:
            return mwi->viewTabMousePressEvent(this, static_cast<QMouseEvent*>(event));
        case QEvent::MouseButtonDblClick:
            break;
        case QEvent::MouseButtonRelease:
            return mwi->viewTabMouseReleaseEvent(this, static_cast<QMouseEvent*>(event));
        case QEvent::MouseMove:
            return mwi->viewTabMouseMoveEvent(this, static_cast<QMouseEvent*>(event));
        default:
            break;
        }
    }
    return false;
}


void MainWindow::initialize(const char* appName, ExtensionManager* ext)
{
    if(!mainWindow){
        new MainWindow(appName, ext);
    }
}


MainWindow* MainWindow::instance()
{
    return mainWindow;
}


MainWindow::MainWindow(const char* appName, ExtensionManager* ext)
{
    mainWindow = this;

    setWindowTitle(appName);
    setFocusPolicy(Qt::WheelFocus);

    impl = new MainWindowImpl(this, appName, ext);
}


MainWindowImpl::MainWindowImpl(MainWindow* self, const char* appName, ExtensionManager* ext)
    : self(self),
      appName(appName)
{
    isBeforeDoingInitialLayout = true;
    isMaximized = false;
    isViewDragging = false;
    
    config = AppConfig::archive()->openMapping("MainWindow");

    centralWidget = new QWidget(self);
    
    centralVBox = new QVBoxLayout(centralWidget);
    centralVBox->setSpacing(0);
    centralVBox->setContentsMargins(0, 0, 0, 0);

    toolBarArea = new ToolBarArea(centralWidget);
    centralVBox->addWidget(toolBarArea);

    self->setCentralWidget(centralWidget);

    setupMenus(ext);

    rubberBand = new QRubberBand(QRubberBand::Rectangle, centralWidget);
    rubberBand->hide();

    topSplitter = 0;
    createDefaultPanes();

    self->setStatusBar(InfoBar::instance());

    toolBarArea->setInitialLayout(config);

    isBeforeShowing = true;

    if(TRACE_FUNCTIONS){
        cout << "size = (" << self->width() << ", " << self->height() << ")" << endl;
    }

    normalStateSize.setWidth(config->get("width", self->width()));
    normalStateSize.setHeight(config->get("height", self->width()));
}


void MainWindow::setProjectTitle(const std::string& title)
{
    QString qtitle(_("%1 - %2"));
    setWindowTitle(qtitle.arg(title.c_str()).arg(impl->appName.c_str()));
}


MainWindow::~MainWindow()
{
    if(impl){
        delete impl;
        impl = 0;
    }
}


MainWindowImpl::~MainWindowImpl()
{

}


void MainWindowImpl::setupMenus(ExtensionManager* ext)
{
    MenuManager& mm = ext->menuManager();

    mm.setPath("/" N_("File")).setBackwardMode().addItem(_("Exit"))
        ->sigTriggered().connect(boost::bind(&MainWindow::close, self));

    mm.setPath("/" N_("Edit"));

    Menu* viewMenu = static_cast<Menu*>(mm.setPath("/" N_("View")).current());
    mm.setPath(N_("Show View"));
    mm.setCurrent(viewMenu).setPath(N_("Create View"));
    mm.setCurrent(viewMenu).setPath(N_("Delete View"));

    mm.setCurrent(viewMenu).addSeparator().setPath(N_("Show Toolbar"));
    Menu* showToolBarMenu = static_cast<Menu*>(mm.current());
    showToolBarMenu->sigAboutToShow().connect(boost::bind(&ToolBarArea::setVisibilityMenuItems, toolBarArea, showToolBarMenu));
    
    mm.setPath("/View");
    mm.addSeparator();
    fullScreenCheck = mm.addCheckItem(_("Full Screen"));
    fullScreenCheck->setChecked(config->get("fullScreen", false));
    fullScreenCheck->sigToggled().connect(boost::bind(&MainWindowImpl::onFullScreenToggled, this, _1));

    mm.setPath("/View");
    mm.setPath(N_("Layout"));
    
    storeLastLayoutCheck = mm.addCheckItem(_("Store Last Toolbar Layout"));
    storeLastLayoutCheck->setChecked(config->get("storeLastLayout", false));
    
    mm.addItem(_("Reset Layout"))->sigTriggered().connect(boost::bind(&MainWindowImpl::resetLayout, this));
    
    mm.setPath("/" N_("Tools"));
    mm.setPath("/" N_("Filters"));
    mm.setPath("/" N_("Options"));
    mm.setPath("/").setBackwardMode().setPath(N_("Help"));
}


void MainWindowImpl::createDefaultPanes()
{
    if(!topSplitter){
        topSplitter = new QSplitter(centralWidget);
        centralVBox->addWidget(topSplitter, 1);
    }
    topSplitter->setOrientation(Qt::Horizontal);
    
    QSplitter* vSplitter0 = new QSplitter(Qt::Vertical, topSplitter);
    topSplitter->addWidget(vSplitter0);
    
    areaToPane[View::LEFT_TOP] = new TabWidget(this, vSplitter0);
    vSplitter0->addWidget(areaToPane[View::LEFT_TOP]);
    
    areaToPane[View::LEFT_BOTTOM] = new TabWidget(this, vSplitter0);
    vSplitter0->addWidget(areaToPane[View::LEFT_BOTTOM]);
    
    QSplitter* vSplitter1 = new QSplitter(Qt::Vertical, topSplitter);
    topSplitter->addWidget(vSplitter1);

    QSplitter* hSplitter1 = new QSplitter(Qt::Horizontal, vSplitter1);
    vSplitter1->addWidget(hSplitter1);
    
    areaToPane[View::BOTTOM] = new TabWidget(this, vSplitter1);
    vSplitter1->addWidget(areaToPane[View::BOTTOM]);
    
    areaToPane[View::CENTER] = new TabWidget(this, hSplitter1);
    hSplitter1->addWidget(areaToPane[View::CENTER]);
    
    areaToPane[View::RIGHT]  = new TabWidget(this, hSplitter1);
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


void MainWindowImpl::detectExistingPaneAreas()
{
    for(int i=0; i < View::NUM_AREAS; ++i){
        areaToPane[i] = 0;
    }

    vector<AreaDetectionInfo> infos;
    EdgeContactState edge;
    edge.set();

    TabWidget* firstPane = updateAreaDetectionInfos(topSplitter, edge, infos);

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
TabWidget* MainWindowImpl::updateAreaDetectionInfos
(QSplitter* splitter, const EdgeContactState& edge, vector<AreaDetectionInfo>& infos)
{
    TabWidget* firstPane = 0;
    
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
                TabWidget* pane = updateAreaDetectionInfos(childSplitter, currentEdge, infos);
                if(!firstPane){
                    firstPane = pane;
                }
            } else {
                TabWidget* pane = dynamic_cast<TabWidget*>(childWidgets[i]);
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


void MainWindowImpl::setBestAreaMatchPane(vector<AreaDetectionInfo>& infos, View::LayoutArea area, TabWidget* firstPane)
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


bool MainWindow::addView(View* view)
{
    return impl->addView(view);
}


bool MainWindowImpl::addView(View* view)
{
    if(needToUpdateDefaultPaneAreas){
        detectExistingPaneAreas();
    }
    return addView(areaToPane[view->defaultLayoutArea()], view);
}


bool MainWindowImpl::addView(TabWidget* pane, View* view)
{
    if(view->isManagedByMainWindow_){
        return false;
    }
    pane->addView(view);
    view->isManagedByMainWindow_ = true;
    return true;
}


bool MainWindow::removeView(View* view)
{
    return impl->removeView(view);
}


bool MainWindowImpl::removeView(View* view)
{
    bool removed = false;

    if(view && view->isManagedByMainWindow_){

        TabWidget* pane = 0;
        for(QWidget* widget = view->parentWidget(); widget; widget = widget->parentWidget()){
            if(pane = dynamic_cast<TabWidget*>(widget)){
                break;
            }
        }
        removeView(pane, view);
        
        if(pane && pane->count() == 0){
            removePaneIfEmpty(pane);
        }
        removed = true;
    }

    return removed;
}


void MainWindowImpl::removeView(TabWidget* pane, View* view)
{
    if(pane){
        pane->removeTab(pane->indexOf(view));
        view->hide();
        view->setParent(0);
    }
    view->isManagedByMainWindow_ = false;
}


void MainWindow::addToolBar(ToolBar* toolbar)
{
    impl->toolBarArea->addToolBar(toolbar);
}


void MainWindow::getAllToolBars(std::vector<ToolBar*>& out_toolBars)
{
    impl->toolBarArea->getAllToolBars(out_toolBars);
}


void MainWindow::getVisibleToolBars(std::vector<ToolBar*>& out_toolBars)
{
    impl->toolBarArea->getVisibleToolBars(out_toolBars);
}


void MainWindow::setInitialLayout(ArchivePtr archive)
{
    if(TRACE_FUNCTIONS){
        cout << "MainWindow::setInitialLayout()" << endl;
    }
    if(impl->isBeforeDoingInitialLayout){
        impl->initialLayoutArchive = archive;
        impl->toolBarArea->setInitialLayout(archive);
    }
}


void MainWindow::changeEvent(QEvent* event)
{
    if(event->type() == QEvent::WindowStateChange){
        if(TRACE_FUNCTIONS){
            cout << "MainWindow::changeEvent() of WindowStateChange: " << windowState() << endl;
        }
        if(!(windowState() & Qt::WindowFullScreen)){
            impl->isMaximized = (windowState() & Qt::WindowMaximized);
        }
    }
}


void MainWindow::show()
{
    impl->showFirst();
}


void MainWindowImpl::showFirst()
{
    if(TRACE_FUNCTIONS){
        cout << "MainWindowImpl::showFirst()" << endl;
    }
    if(isBeforeShowing){

        self->resize(normalStateSize);
        
        if(config->get("fullScreen", false)){
            isMaximizedJustBeforeFullScreen = isMaximized;
            self->showFullScreen();
        } else if(config->get("maximized", true)){
            self->showMaximized();
        } else {
            self->QMainWindow::show();
        }
        isBeforeShowing = false;
    } else {
        self->QMainWindow::show();
    }
}


void MainWindowImpl::onFullScreenToggled(bool on)
{
    if(on){
        if(!self->isFullScreen()){
            isMaximizedJustBeforeFullScreen = isMaximized;
            self->showFullScreen();
        }
    } else {
        if(self->isFullScreen()){
            if(isMaximizedJustBeforeFullScreen){
                self->showMaximized();
            } else {
                self->showNormal();
            }
        }
    }
}


void MainWindow::resizeEvent(QResizeEvent* event)
{
    QMainWindow::resizeEvent(event);
    impl->resizeEvent(event);
}


void MainWindowImpl::resizeEvent(QResizeEvent* event)
{
    if(TRACE_FUNCTIONS){
        cout << "MainWindowImpl::resizeEvent(): size = (";
        cout << event->size().width() << ", " << event->size().height() << ")";
        cout << ", isMaximized = " << (self->windowState() & Qt::WindowMaximized) << ", " << isMaximized;
        cout << ", isVisible = " << self->isVisible() << endl;
    }
    
    if(isBeforeDoingInitialLayout){

        bool isMaximized = self->windowState() & (Qt::WindowMaximized | Qt::WindowFullScreen);

        if(!isMaximized || self->isVisible()){

            if(!isMaximized){
                /**
                   This is needed to do the default layout of toolbars correctly
                   because toolBarArea is resized later.
                */
                toolBarArea->resize(event->size().width(), toolBarArea->height());
            }

            if(TRACE_FUNCTIONS){
                cout << "MainWindowImpl::resizeEvent(): initializeLayout" << endl;
            }
            
            if(initialLayoutArchive){
                restoreLayout(initialLayoutArchive);
            } else {
                toolBarArea->doInitialLayout();
                resetViewLayout();
            }
            initialLayoutArchive = 0;
            isBeforeDoingInitialLayout = false;
        }

    } else {
        if(!(self->windowState() &
             (Qt::WindowMinimized | Qt::WindowMaximized | Qt::WindowFullScreen))){ // normal state ?
            normalStateSize = self->size();
        }
    }
}


void MainWindow::keyPressEvent(QKeyEvent* event)
{
    impl->keyPressEvent(event);
}


void MainWindowImpl::keyPressEvent(QKeyEvent* event)
{
    switch(event->key()){
        
    case Qt::Key_F11:
        fullScreenCheck->toggle();
        break;

    // TimeBar operations
    case Qt::Key_F5:
    case Qt::Key_F6:
    {
        TimeBar* timeBar = TimeBar::instance();
        if(timeBar->isDoingPlayback()){
            timeBar->stopPlayback(true);
        } else {
            if(event->key() == Qt::Key_F5){
                timeBar->setTime(0.0);
            }
            timeBar->startPlayback();
        }
        break;
    }

    default:
        break;
    }
}


void MainWindowImpl::resetLayout()
{
    toolBarArea->resetLayout(config);
    resetViewLayout();
}


void MainWindowImpl::resetViewLayout()
{
    vector<View*> views;
    getAllViews(views);
    clearAllPanes();
    createDefaultPanes();
    
    for(size_t i=0; i < views.size(); ++i){
        addView(views[i]);
    }
}


void MainWindow::restoreLayout(ArchivePtr archive)
{
    impl->restoreLayout(archive);
}


void MainWindowImpl::restoreLayout(ArchivePtr& archive)
{
    if(TRACE_FUNCTIONS){
        cout << "MainWindowImpl::restoreLayout()" << endl;
    }

    if(!isBeforeDoingInitialLayout){
        toolBarArea->restoreLayout(archive);
    } else {
        toolBarArea->doInitialLayout();
    }
    
    ArchivePtr layoutOfViews = archive->findSubArchive("layoutOfViews");

    if(layoutOfViews->isValid()){

        clearAllPanes();

        TabWidget* firstPane = 0;

        QWidget* restoredWidget = restoreSplitterState(*layoutOfViews, firstPane, archive);

        topSplitter = dynamic_cast<QSplitter*>(restoredWidget);
        
        if(!topSplitter){
            topSplitter = new QSplitter();
            firstPane = dynamic_cast<TabWidget*>(restoredWidget);
            if(!firstPane){
                firstPane = new TabWidget(this);
            }
            topSplitter->addWidget(firstPane);
        }
        centralVBox->addWidget(topSplitter, 1);

        needToUpdateDefaultPaneAreas = true;
    }
}


void MainWindowImpl::clearAllPanes()
{
    if(TRACE_FUNCTIONS){
        cout << "clearAllPanes" << endl;
    }
    clearAllPanesSub(topSplitter);
    delete topSplitter;
    topSplitter = 0;
    if(TRACE_FUNCTIONS){
        cout << "End of clearAllPanes" << endl;
    }

    needToUpdateDefaultPaneAreas = true;
}


void MainWindowImpl::clearAllPanesSub(QSplitter* splitter)
{
    if(TRACE_FUNCTIONS){
        cout << "clearAllPanesSub" << endl;
    }
    
    for(int i=0; i < splitter->count(); ++i){
        QSplitter* childSplitter = dynamic_cast<QSplitter*>(splitter->widget(i));
        if(childSplitter){
            clearAllPanesSub(childSplitter);
        } else {
            TabWidget* pane = dynamic_cast<TabWidget*>(splitter->widget(i));
            if(pane){
                while(pane->count() > 0){
                    int index = pane->count() - 1;
                    View* view = dynamic_cast<View*>(pane->widget(index));
                    if(view){
                        removeView(pane, view);
                    }
                }
            }
        }
    }
}


void MainWindowImpl::getAllViews(vector<View*>& out_views)
{
    getAllViewsSub(topSplitter, out_views);
}


void MainWindowImpl::getAllViewsSub(QSplitter* splitter, vector<View*>& out_views)
{
    for(int i=0; i < splitter->count(); ++i){
        QSplitter* childSplitter = dynamic_cast<QSplitter*>(splitter->widget(i));
        if(childSplitter){
            getAllViewsSub(childSplitter, out_views);
        } else {
            TabWidget* pane = dynamic_cast<TabWidget*>(splitter->widget(i));
            if(pane){
                for(int i=0; i < pane->count(); ++i){
                    View* view = dynamic_cast<View*>(pane->widget(i));
                    if(view){
                        out_views.push_back(view);
                    }
                }
            }
        }
    }
}


QWidget* MainWindowImpl::restoreSplitterState(const Mapping& state, TabWidget*& out_firstPane, ArchivePtr& archive)
{
    if(TRACE_FUNCTIONS){
        cout << "MainWindowImpl::restoreSplitterState" << endl;
    }
    
    QWidget* restoredWidget = 0;
    QWidget* childWidgets[2] = { 0, 0 };
    
    const Listing& children = *state.findListing("children");
    if(children.isValid()){
        int numChildren = std::min(children.size(), 2);
        for(int i=0; i < numChildren; ++i){
            if(children[i].isMapping()){
                const Mapping& childState = *children[i].toMapping();
                string type;
                if(childState.read("type", type)){
                    if(type == "splitter"){
                        childWidgets[i] = restoreSplitterState(childState, out_firstPane, archive);
                    } else if(type == "pane"){
                        TabWidget* pane = restorePaneState(childState, archive);
                        if(pane){
                            childWidgets[i] = pane;
                            if(!out_firstPane){
                                out_firstPane = pane;
                            }
                        }
                    }
                }
            }
        }

        if(childWidgets[0] && childWidgets[1]){

            QSplitter* splitter = new QSplitter();

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
            restoredWidget = splitter;
            
        } else {
            for(int i=0; i < 2; ++i){
                if(childWidgets[i]){
                    restoredWidget = childWidgets[i];
                    break;
                }
            }
        }
    }

    return restoredWidget;
}


TabWidget* MainWindowImpl::restorePaneState(const Mapping& state, ArchivePtr& archive)
{
    if(TRACE_FUNCTIONS){
        cout << "MainWindowImpl::restorePaneState" << endl;
    }
    
    TabWidget* pane = 0;
    const Listing& views = *state.findListing("views");
    
    if(views.isValid() && !views.empty()){
        pane = new TabWidget(this);

        int currentId = 0;
        string currentName;
        if(!state.read("current", currentId)){
            state.read("current", currentName);
        }
        
        for(int i=0; i < views.size(); ++i){
            const ValueNode& node = views[i];
            View* view = 0;
            int id;
            bool isCurrent = false;
            if(node.read(id)){
                view = archive->findView(id);
                if(view){
                    isCurrent = (id == currentId);
                }
            } else if(node.isString()){
                // for the Choreonoid version 1.4 or earlier
                view = ViewManager::getOrCreateViewOfDefaultName(node.toString());
                if(view){
                    if(view->name() == currentName){
                        isCurrent = true;
                    }
                }
            }
            if(view){
                addView(pane, view);
                if(isCurrent){
                    pane->setCurrentIndex(i);
                }
            }
        }
        if(pane->count() == 0){
            delete pane;
            pane = 0;
        }
    }

    return pane;
}


void MainWindow::storeLayout(ArchivePtr archive)
{
    impl->storeLayout(archive);
}


void MainWindowImpl::storeLayout(ArchivePtr& archive)
{
    try {
        MappingPtr state = storeSplitterState(topSplitter, archive);
        if(state){
            archive->insert("layoutOfViews", state);
        }
        toolBarArea->storeLayout(archive);
    }
    catch(const ValueNode::Exception& ex){
        cout << ex.message() << endl;
    }
}


MappingPtr MainWindowImpl::storeSplitterState(QSplitter* splitter, ArchivePtr& archive)
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
            TabWidget* pane = dynamic_cast<TabWidget*>(splitter->widget(i));
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


MappingPtr MainWindowImpl::storePaneState(TabWidget* pane, ArchivePtr& archive)
{
    MappingPtr state = new Mapping();
    
    state->write("type", "pane");
    
    Listing* views = state->createFlowStyleListing("views");
    const int n = pane->count();
    for(int i=0; i < n; ++i){
        View* view = dynamic_cast<View*>(pane->widget(i));
        int id = archive->getViewId(view);
        if(id >= 0){
            views->append(id);
            if(i == pane->currentIndex()){
                state->write("current", id);
            }
        }
    }
    if(views->empty()){
        state.reset();
    }
    return state;
}


bool MainWindowImpl::viewTabMousePressEvent(TabWidget* pane, QMouseEvent* event)
{
    if(event->button() == Qt::LeftButton){
        tabDragStartPosition = event->pos();
    } else if(event->button() == Qt::RightButton){
        if(View* view = dynamic_cast<View*>(pane->currentWidget())){
            viewMenuManager.setNewPopupMenu(self);
            view->onAttachedMenuRequest(viewMenuManager);
            if(viewMenuManager.numItems() > 0){
                viewMenuManager.popupMenu()->popup(event->globalPos());
            }
        }
    }
    return false;
}


bool MainWindowImpl::viewTabMouseMoveEvent(TabWidget* pane, QMouseEvent* event)
{
    if(!isViewDragging){
        if(event->buttons() & Qt::LeftButton){
            if((event->pos() - tabDragStartPosition).manhattanLength() > QApplication::startDragDistance()){
                if(!pane->tabBar()->geometry().contains(event->pos())){
                    View* view = dynamic_cast<View*>(pane->currentWidget());
                    if(view){
                        isViewDragging = true;
                        dragSrcPane = pane;
                        startViewDrag(view);
                    }
                }
            }
        }
    } else {
        dragDestPane = 0;
        QWidget* pointed = topSplitter->childAt(topSplitter->mapFromGlobal(event->globalPos()));
        while(pointed){
            dragDestPane = dynamic_cast<TabWidget*>(pointed);
            if(dragDestPane){
                dragView(event);
                break;
            }
            pointed = pointed->parentWidget();
        }
        if(!dragDestPane){
            rubberBand->hide();
        }
    }
    return false;
}


bool MainWindowImpl::viewTabMouseReleaseEvent(TabWidget* pane, QMouseEvent *event)
{
    if(isViewDragging){
        if(dragDestPane){
            if(isViewDraggingOutsidePane){
                dropViewOutsidePane();
            } else {
                dropViewInsidePane();
            }
        }
        QApplication::restoreOverrideCursor();
    }
    isViewDragging = false;
    return false;
}


void MainWindowImpl::startViewDrag(View* view)
{
    QApplication::setOverrideCursor(Qt::ClosedHandCursor);
}


void MainWindowImpl::dragView(QMouseEvent* event)
{
    dropEdge = LEFT;
        
    QPoint p = topSplitter->mapFromGlobal(event->globalPos());
    const int w = topSplitter->width();
    const int h = topSplitter->height();
    
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

    isViewDraggingOutsidePane = false;
    if(distance[dropEdge] < 8){
        isViewDraggingOutsidePane = true;
        dragViewOutsidePane();
    } else {
        dragViewInsidePane(dragDestPane->mapFromGlobal(event->globalPos()));
    }
}

    
void MainWindowImpl::dragViewInsidePane(const QPoint& posInDestPane)
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

    r.translate(centralWidget->mapFromGlobal(dragDestPane->mapToGlobal(QPoint(0, 0))));
    rubberBand->setGeometry(r);
    rubberBand->show();
}


void MainWindowImpl::dropViewInsidePane()
{
    View* view = static_cast<View*>(dragSrcPane->currentWidget());
    if(dropEdge == OVER){
        int index = dragDestPane->addView(view);
        dragDestPane->setCurrentIndex(index);
    } else {

        QSize destSize = dragDestPane->size();

        QSplitter* parentSplitter = static_cast<QSplitter*>(dragDestPane->parentWidget());
        QList<int> parentSizes = parentSplitter->sizes();
        QSplitter* newSplitter = new QSplitter(parentSplitter);
        parentSplitter->insertWidget(parentSplitter->indexOf(dragDestPane), newSplitter);
        TabWidget* newTabWidget = new TabWidget(this, newSplitter);

        if(dropEdge == LEFT){
            newSplitter->setOrientation(Qt::Horizontal);
            newSplitter->addWidget(newTabWidget);
            newSplitter->addWidget(dragDestPane);
        } else if(dropEdge == RIGHT){
            newSplitter->setOrientation(Qt::Horizontal);
            newSplitter->addWidget(dragDestPane);
            newSplitter->addWidget(newTabWidget);
        } else if(dropEdge == TOP){
            newSplitter->setOrientation(Qt::Vertical);
            newSplitter->addWidget(newTabWidget);
            newSplitter->addWidget(dragDestPane);
        } else {
            newSplitter->setOrientation(Qt::Vertical);
            newSplitter->addWidget(dragDestPane);
            newSplitter->addWidget(newTabWidget);
        }
        newTabWidget->addView(view);

        int half;
        if(newSplitter->orientation() == Qt::Horizontal){
            half = destSize.height() / 2;
        } else {
            half = destSize.width() / 2;
        }
        QList<int> sizes;
        sizes << half << half;
        newSplitter->setSizes(sizes);

        parentSplitter->setSizes(parentSizes);
    }
    if(dragSrcPane->count() > 0){
        dragSrcPane->setCurrentIndex(0);
    }
    removePaneIfEmpty(dragSrcPane);
    rubberBand->hide();

    needToUpdateDefaultPaneAreas = true;
}


void MainWindowImpl::dragViewOutsidePane()
{
    QRect r;
    int w = topSplitter->width();
    int h = topSplitter->height();
    if(dropEdge == LEFT){
        r.setRect(0, 0, w / 2, h);
    } else if(dropEdge == TOP){
        r.setRect(0, 0, w, h /2);
    } else if(dropEdge == RIGHT){
        r.setRect(w / 2, 0, w / 2, h);
    } else if(dropEdge == BOTTOM){
        r.setRect(0, h / 2, w, h / 2);
    }
    r.translate(topSplitter->pos());
    rubberBand->setGeometry(r);
    rubberBand->show();
}


void MainWindowImpl::dropViewOutsidePane()
{
    View* view = static_cast<View*>(dragSrcPane->currentWidget());

    QSize size = topSplitter->size();

    if(topSplitter->count() >= 2){
        QSplitter* newTopSplitter = new QSplitter(centralWidget);
        newTopSplitter->addWidget(topSplitter);
        topSplitter = newTopSplitter;
        centralVBox->addWidget(topSplitter, 1);
    }
    TabWidget* newTabWidget = new TabWidget(this, topSplitter);

    if(dropEdge == LEFT){
        topSplitter->setOrientation(Qt::Horizontal);
        topSplitter->insertWidget(0, newTabWidget);
    } else if(dropEdge == RIGHT){
        topSplitter->setOrientation(Qt::Horizontal);
        topSplitter->addWidget(newTabWidget);
    } else if(dropEdge == TOP){
        topSplitter->setOrientation(Qt::Vertical);
        topSplitter->insertWidget(0, newTabWidget);
    } else {
        topSplitter->setOrientation(Qt::Vertical);
        topSplitter->addWidget(newTabWidget);
    }
    newTabWidget->addView(view);

    int half;
    if(topSplitter->orientation() == Qt::Horizontal){
        half = size.height() / 2;
    } else {
        half = size.width() / 2;
    }
    QList<int> sizes;
    sizes << half << half;
    topSplitter->setSizes(sizes);
    
    removePaneIfEmpty(dragSrcPane);
    rubberBand->hide();

    needToUpdateDefaultPaneAreas = true;
}


void MainWindowImpl::removePaneIfEmpty(TabWidget* pane)
{
    if(pane->count() == 0){
        QSplitter* parentSplitter = dynamic_cast<QSplitter*>(pane->parentWidget());
        if(parentSplitter){
            if(parentSplitter != topSplitter || parentSplitter->count() >= 2){
                pane->hide();
                pane->deleteLater();
                if(parentSplitter->count() <= 1){
                    QSplitter* grandParentSplitter = dynamic_cast<QSplitter*>(parentSplitter->parentWidget());
                    if(grandParentSplitter){
                        if(parentSplitter->count() == 1){
                            int parentSplitterIndex = grandParentSplitter->indexOf(parentSplitter);
                            grandParentSplitter->insertWidget(parentSplitterIndex, parentSplitter->widget(0));
                        }
                        delete parentSplitter;
                    }
                }
            }
        }
        needToUpdateDefaultPaneAreas = true;
    }
}


void MainWindow::storeWindowStateConfig()
{
    impl->storeWindowStateConfig();
}


void MainWindowImpl::storeWindowStateConfig()
{
    config->write("fullScreen", self->isFullScreen());
    config->write("maximized", isMaximized);
    config->write("width", normalStateSize.width());
    config->write("height", normalStateSize.height());
    config->write("storeLastLayout", storeLastLayoutCheck->isChecked());
    
    if(storeLastLayoutCheck->isChecked()){
        toolBarArea->storeLayout(config);
    } else {
        toolBarArea->removeLayout(config);
    }
}
