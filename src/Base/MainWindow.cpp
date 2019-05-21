/**
   @author Shin'ichiro Nakaoka
*/

#include "MainWindow.h"
#include "ViewArea.h"
#include "InfoBar.h"
#include "ToolBarArea.h"
#include "MenuManager.h"
#include "AppConfig.h"
#include "TimeBar.h"
#include <QResizeEvent>
#include <QWindowStateChangeEvent>
#include <QApplication>
#include <QDesktopWidget>
#include <iostream>
#include "gettext.h"

using namespace std;
using namespace cnoid;
using namespace std::placeholders;

namespace {

const bool TRACE_FUNCTIONS = false;

MainWindow* mainWindow = 0;

QSize getAvailableScreenSize() {
    return QApplication::desktop()->availableGeometry().size();
}
#ifdef Q_OS_WIN32
QSize getScreenSize() {
    return QApplication::desktop()->screenGeometry().size();
}
#endif
    
}

namespace cnoid {
    
class MainWindowImpl
{
public:
    MainWindow* self;

    MainWindowImpl(MainWindow* self, const char* appName, ExtensionManager* ext);
    ~MainWindowImpl();

    QWidget* centralWidget;
    QVBoxLayout* centralVBox;

    std::vector<ToolBar*> toolBars;
    ToolBarArea* toolBarArea;

    ViewArea* viewArea;
    Action* showViewTabCheck;

    string appName;

    MappingPtr config;
    ArchivePtr initialLayoutArchive;
    Action* storeLastLayoutCheck;
        
    bool isBeforeShowing;
    bool isBeforeDoingInitialLayout;
    bool isMaximized;
    bool isMaximizedJustBeforeFullScreen;
    bool isFullScreen;
    bool isGoingToMaximized;
    QSize normalSize;
    QSize oldNormalSize;
    int lastWindowState;
    QString currentLayoutFolder;
    Action* fullScreenCheck;

    void setupMenus(ExtensionManager* ext);
    void showFirst();
    void onFullScreenToggled(bool on);
    void resizeEvent(QResizeEvent* event);
    void restoreLayout(ArchivePtr& archive);
    void resetLayout();
    void storeWindowStateConfig();
    void keyPressEvent(QKeyEvent* event);
};

}


MainWindow* MainWindow::initialize(const char* appName, ExtensionManager* ext)
{
    if(!mainWindow){
        new MainWindow(appName, ext);
    }
    return mainWindow;
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
    isFullScreen = false;
    isGoingToMaximized = false;
    
    config = AppConfig::archive()->openMapping("MainWindow");

    centralWidget = new QWidget(self);
    
    centralVBox = new QVBoxLayout(centralWidget);
    centralVBox->setSpacing(0);
    centralVBox->setContentsMargins(0, 0, 0, 0);

    toolBarArea = new ToolBarArea(centralWidget);
    centralVBox->addWidget(toolBarArea);

    viewArea = new ViewArea(centralWidget);
    centralVBox->addWidget(viewArea, 1);

    self->setCentralWidget(centralWidget);

    setupMenus(ext);

    self->setStatusBar(InfoBar::instance());

    toolBarArea->setInitialLayout(config);

    isBeforeShowing = true;

    if(TRACE_FUNCTIONS){
        cout << "size = (" << self->width() << ", " << self->height() << ")" << endl;
    }

    int width, height;
    if(config->read("width", width) && config->read("height", height)){
        normalSize = QSize(width, height);
    } else {
        normalSize = getAvailableScreenSize();

        if(TRACE_FUNCTIONS){
            cout << "AvailableScreenSize = (" << normalSize.width() << ", " << normalSize.height() << ")" << endl;
        }
        static const int defaultSizes[] = {
            3840, 2160,
            2560, 1600,
            1920, 1200,
            1680, 1050,
            1400, 1050,
            1600, 900,
            1280, 1024,
            1366, 768,
            1280, 800,
            1280, 768,
            1280, 720,
            1024, 768,
            800, 600,
            640, 480,
            -1, -1,
        };

        int i = 0;
        while(true){
            int w = defaultSizes[i++];
            int h = defaultSizes[i++];
            if(w < normalSize.width() && h < normalSize.height()){
                if(w > 0){
                    normalSize = QSize(w, h);
                }
                break;
            }
        }
    }
    oldNormalSize = normalSize;

    if(TRACE_FUNCTIONS){
        cout << "normalSize = (" << normalSize.width() << ", " << normalSize.height() << ")" << endl;
    }

    lastWindowState = self->windowState();
}


void MainWindow::setProjectTitle(const std::string& title)
{
    QString qtitle("%1 - %2");
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
        ->sigTriggered().connect(std::bind(&MainWindow::close, self));

    mm.setPath("/" N_("Edit"));

    Menu* viewMenu = static_cast<Menu*>(mm.setPath("/" N_("View")).current());

    mm.setPath(N_("Show Toolbar"));
    Menu* showToolBarMenu = static_cast<Menu*>(mm.current());
    showToolBarMenu->sigAboutToShow().connect(std::bind(&ToolBarArea::setVisibilityMenuItems, toolBarArea, showToolBarMenu));
    
    mm.setCurrent(viewMenu).setPath(N_("Show View"));
    mm.setCurrent(viewMenu).setPath(N_("Create View"));
    mm.setCurrent(viewMenu).setPath(N_("Delete View"));

    showViewTabCheck = mm.setCurrent(viewMenu).addCheckItem(_("Show View Tabs"));
    showViewTabCheck->sigToggled().connect(std::bind(&ViewArea::setViewTabsVisible, viewArea, _1));
    showViewTabCheck->setChecked(config->get("showViewTabs", true));

    mm.setCurrent(viewMenu).addSeparator();

    Action* showStatusBarCheck = mm.setCurrent(viewMenu).addCheckItem(_("Show Status Bar"));
    bool showStatusBar = config->get("showStatusBar", true);
    QWidget* statusBar = InfoBar::instance();
    showStatusBarCheck->setChecked(showStatusBar);
    showStatusBarCheck->sigToggled().connect(std::bind(&QWidget::setVisible, statusBar, _1));
    
    fullScreenCheck = mm.addCheckItem(_("Full Screen"));
    fullScreenCheck->setChecked(config->get("fullScreen", false));
    fullScreenCheck->sigToggled().connect(std::bind(&MainWindowImpl::onFullScreenToggled, this, _1));

    mm.setCurrent(viewMenu).setPath(N_("Layout"));
    
    storeLastLayoutCheck = mm.addCheckItem(_("Store Last Toolbar Layout"));
    storeLastLayoutCheck->setChecked(config->get("storeLastLayout", false));

    mm.addItem(_("Reset Layout"))->sigTriggered().connect(std::bind(&MainWindowImpl::resetLayout, this));
    
    mm.setPath("/" N_("Tools"));
    mm.setPath("/" N_("Filters"));
    mm.setPath("/" N_("Options"));
    mm.setPath("/").setBackwardMode().setPath(N_("Help"));
}


ToolBarArea* MainWindow::toolBarArea()
{
    return impl->toolBarArea;
}


ViewArea* MainWindow::viewArea()
{
    return impl->viewArea;
}


void MainWindow::addToolBar(ToolBar* toolbar)
{
    impl->toolBarArea->addToolBar(toolbar);
}


void MainWindow::removeToolBar(ToolBar* toolbar)
{
    impl->toolBarArea->removeToolBar(toolbar);
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

        QWindowStateChangeEvent* wsc = static_cast<QWindowStateChangeEvent*>(event);
        bool wasMaximized = wsc->oldState() & (Qt::WindowMaximized | Qt::WindowFullScreen);
        bool isMaximized = windowState() & (Qt::WindowMaximized | Qt::WindowFullScreen);
        if(isMaximized){
            impl->normalSize = impl->oldNormalSize;
            if(TRACE_FUNCTIONS){
                cout << "normalSize = oldNormalSize;" << endl;
            }
        }

        if(!impl->isGoingToMaximized && wasMaximized && !isMaximized){
            if(TRACE_FUNCTIONS){
                cout << "return to normal size" << endl;
            }
            resize(impl->normalSize);
        }

        if(!(windowState() & Qt::WindowFullScreen)){
            impl->isMaximized = (windowState() & Qt::WindowMaximized);
            if(TRACE_FUNCTIONS){
                cout << "impl->isMaximized = " << impl->isMaximized << endl;
            }
        }

        impl->lastWindowState = windowState();
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
        if(config->get("fullScreen", false)){
            isMaximizedJustBeforeFullScreen = config->get("maximized", true);
#ifdef Q_OS_WIN32
            self->resize(getScreenSize());
#endif
            isGoingToMaximized = true;
            if(TRACE_FUNCTIONS){
                cout << "showFullScreen();" << endl;
            }
            self->showFullScreen();
            //isGoingToMaximized = false;
        } else if(config->get("maximized", true)){
#ifdef Q_OS_WIN32
            self->resize(getAvailableScreenSize());
#endif
            isGoingToMaximized = true;
            if(TRACE_FUNCTIONS){
                cout << "showMaximized();" << endl;
            }
            self->showMaximized();
            //isGoingToMaximized = false;
        } else {
            self->resize(normalSize);
            if(TRACE_FUNCTIONS){
                cout << "showNormal();" << endl;
            }
            self->showNormal();
        }
        bool showStatusBar = config->get("showStatusBar", true);
        self->statusBar()->setVisible(showStatusBar);

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
            isGoingToMaximized = true;
            self->showFullScreen();
            isGoingToMaximized = false;
        }
    } else {
        if(self->isFullScreen()){
            if(isMaximizedJustBeforeFullScreen){
                isGoingToMaximized = true;
                self->showMaximized();
                isGoingToMaximized = false;
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
        cout << "window size = (" << self->width() << ", " << self->height() << ")" << endl;
        cout << ", windowState = " << self->windowState();
        cout << ", isVisible = " << self->isVisible() << endl;
    }
    
    if(isBeforeDoingInitialLayout){

#ifdef Q_OS_WIN32
        toolBarArea->resize(event->size().width(), toolBarArea->height());
        restoreLayout(initialLayoutArchive);
        initialLayoutArchive = 0;
        isBeforeDoingInitialLayout = false;

#else
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
            
            restoreLayout(initialLayoutArchive);
            initialLayoutArchive = 0;
            isBeforeDoingInitialLayout = false;
        }
#endif
    } else {
        static const int stateMask = (Qt::WindowMinimized | Qt::WindowMaximized | Qt::WindowFullScreen);
        if(TRACE_FUNCTIONS){
            cout << "lastWindowState: " << lastWindowState << endl;
            cout << "windowState: " << self->windowState() << endl;
        }
        if(!(lastWindowState & stateMask) && !(self->windowState() & stateMask)){
            oldNormalSize = normalSize;
            normalSize = self->size();
            if(TRACE_FUNCTIONS){
                cout << "normalSize = (" << normalSize.width() << ", " << normalSize.height() << ")" << endl;
            }
        }
        isGoingToMaximized = false;
        if(TRACE_FUNCTIONS){
            cout << "isGoingToMaximized = false;" << endl;
        }
    }
}


void MainWindow::restoreLayout(ArchivePtr archive)
{
    impl->restoreLayout(archive);
}


void MainWindowImpl::restoreLayout(ArchivePtr& archive)
{
    if(!isBeforeDoingInitialLayout){
        toolBarArea->restoreLayout(archive);
    } else {
        toolBarArea->doInitialLayout();
    }

    ViewArea::restoreAllViewAreaLayouts(archive);
}


void MainWindowImpl::resetLayout()
{
    toolBarArea->resetLayout(config);
    viewArea->resetLayout();
}


void MainWindow::storeLayout(ArchivePtr archive)
{
    try {
        ViewArea::storeAllViewAreaLayouts(archive);
        toolBarArea()->storeLayout(archive);
    }
    catch(const ValueNode::Exception& ex){
        std::cout << ex.message() << std::endl;
    }
}


void MainWindow::storeWindowStateConfig()
{
    impl->storeWindowStateConfig();
}


void MainWindowImpl::storeWindowStateConfig()
{
    config->write("showViewTabs", showViewTabCheck->isChecked());
    config->write("showStatusBar", InfoBar::instance()->isVisible());
    bool isFullScreen = self->isFullScreen();
    config->write("fullScreen", isFullScreen);
    if(isFullScreen){
        config->write("maximized", isMaximizedJustBeforeFullScreen);
    } else {
        config->write("maximized", isMaximized);
    }
    config->write("width", normalSize.width());
    config->write("height", normalSize.height());
    config->write("storeLastLayout", storeLastLayoutCheck->isChecked());
    
    if(storeLastLayoutCheck->isChecked()){
        toolBarArea->storeLayout(config);
    } else {
        toolBarArea->removeLayout(config);
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

    case Qt::Key_F12:
        viewArea->setViewTabsVisible(!viewArea->viewTabsVisible());
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
        event->ignore();
        break;
    }
}
