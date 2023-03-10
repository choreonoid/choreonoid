#include "MainWindow.h"
#include "ViewArea.h"
#include "InfoBar.h"
#include "ToolBarArea.h"
#include "ExtensionManager.h"
#include "App.h"
#include "AppConfig.h"
#include "TimeBar.h"
#include "UnifiedEditHistory.h"
#include "LayoutSwitcher.h"
#include "MessageView.h"
#include <cnoid/Sleep>
#include <QResizeEvent>
#include <QWindowStateChangeEvent>
#include <QApplication>
#include <QScreen>
#include <QMenuBar>
#include <iostream>
#include "gettext.h"

using namespace std;
using namespace cnoid;

namespace {

const bool TRACE_FUNCTIONS = false;

MainWindow* mainWindow = nullptr;
bool isLayoutSwitcherAvailable = true;

Signal<void(QKeyEvent* event)> sigKeyPressed_;
Signal<void(QKeyEvent* event)> sigKeyReleased_;

QSize getAvailableScreenSize()
{
    return QGuiApplication::primaryScreen()->availableSize();
}

#ifdef Q_OS_WIN32
QSize getScreenSize()
{
    return QGuiApplication::primaryScreen()->size();
}
#endif

#ifdef Q_OS_UNIX
class WindowActivationChecker : public QObject
{
public:
    bool isWindowActivated;

    WindowActivationChecker() : isWindowActivated(false) { }

    bool eventFilter(QObject* obj, QEvent* event) override {
        if(event->type() == QEvent::ActivationChange){
            isWindowActivated = true;
        }
        return false;
    }
};
#endif

}

namespace cnoid {
    
class MainWindow::Impl
{
public:
    MainWindow* self;

    Impl(MainWindow* self, const std::string& appName, ExtensionManager* ext);
    ~Impl();

    QWidget* centralWidget;
    QVBoxLayout* centralVBox;
    ToolBarArea* toolBarArea;
    ViewArea* viewArea;
    LayoutSwitcher* layoutSwitcher;
    string appName;
    MappingPtr config;
    ArchivePtr initialLayoutArchive;
    bool isBeforeShowing;
    bool isBeforeDoingInitialLayout;
    bool isMaximized;
    bool isMaximizedJustBeforeFullScreen;
    bool isFullScreen;
    bool isGoingToMaximized;
    QSize normalSize;
    QSize oldNormalSize;
    int lastWindowState;
    Signal<void(bool on)> sigFullScreenToggled;
    QString currentLayoutFolder;

#ifdef Q_OS_UNIX
    WindowActivationChecker windowActivationChecker;
#endif

    void showFirst();
    void resizeEvent(QResizeEvent* event);
    void restoreLayout(Archive* archive);
    void resetLayout();
    void storeWindowStateConfig();
    void keyPressEvent(QKeyEvent* event);
};

}


void MainWindow::setLayoutSwitcherAvailable(bool on)
{
    isLayoutSwitcherAvailable = on;
}


MainWindow* MainWindow::initialize(const std::string& appName, ExtensionManager* ext)
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


MainWindow::MainWindow(const std::string& appName, ExtensionManager* ext)
{
    mainWindow = this;

    setWindowTitle(appName.c_str());
    setFocusPolicy(Qt::WheelFocus);

    impl = new Impl(this, appName, ext);
}


MainWindow::Impl::Impl(MainWindow* self, const std::string& appName, ExtensionManager* ext)
    : self(self),
      appName(appName)
{
    config = AppConfig::archive()->openMapping("MainWindow");

    isBeforeDoingInitialLayout = true;
    isMaximized = false;
    isFullScreen = config->get({ "full_screen", "fullScreen" }, false);
    isGoingToMaximized = false;
    
    centralWidget = new QWidget(self);
    
    centralVBox = new QVBoxLayout(centralWidget);
    centralVBox->setSpacing(0);
    centralVBox->setContentsMargins(0, 0, 0, 0);

    toolBarArea = new ToolBarArea(centralWidget);
    centralVBox->addWidget(toolBarArea);

    viewArea = new ViewArea(centralWidget);
    centralVBox->addWidget(viewArea, 1);

    self->setCentralWidget(centralWidget);

    self->setStatusBar(InfoBar::instance());

    toolBarArea->setInitialLayout(config);

    if(!isLayoutSwitcherAvailable){
        layoutSwitcher = nullptr;
    } else {
        layoutSwitcher = new LayoutSwitcher;
        self->menuBar()->setCornerWidget(layoutSwitcher, Qt::TopRightCorner);
    }

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
    if(title.empty()){
        setWindowTitle(impl->appName.c_str());
    } else {
        QString qtitle("%1 - %2");
        setWindowTitle(qtitle.arg(title.c_str()).arg(impl->appName.c_str()));
    }
}


MainWindow::~MainWindow()
{
    if(impl){
        delete impl;
        impl = 0;
    }
}


MainWindow::Impl::~Impl()
{

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


std::vector<ToolBar*> MainWindow::toolBars() const
{
    return impl->toolBarArea->toolBars();
}


std::vector<ToolBar*> MainWindow::visibleToolBars() const
{
    return impl->toolBarArea->visibleToolBars();
}


void MainWindow::setInitialLayout(Archive* archive)
{
    if(TRACE_FUNCTIONS){
        cout << "MainWindow::setInitialLayout()" << endl;
    }
    if(impl->isBeforeDoingInitialLayout){
        auto viewLayout = archive->findListing("viewAreas");
        if(viewLayout->isValid()){
            impl->initialLayoutArchive = archive;
        }
        auto toolBarLayout = archive->findMapping("layoutOfToolBars");
        if(toolBarLayout->isValid()){
            impl->toolBarArea->setInitialLayout(archive);
        }
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
    /**
       This is necessary to avoid a crash when the MessageViwe's flush is called from
       the event handlers resulting from a main window resize event.
    */
    MessageView::blockFlush();
    
    impl->showFirst();
    
    MessageView::unblockFlush();
}


void MainWindow::Impl::showFirst()
{
    if(TRACE_FUNCTIONS){
        cout << "MainWindow::Impl::showFirst()" << endl;
    }

    if(!isBeforeShowing){
        self->QMainWindow::show();

    } else {

#ifdef Q_OS_UNIX
        self->installEventFilter(&windowActivationChecker);
#endif
        
        if(isFullScreen){
            isMaximizedJustBeforeFullScreen = config->get("maximized", false);
#ifdef Q_OS_WIN32
            self->resize(getScreenSize());
#endif
            isGoingToMaximized = true;
            if(TRACE_FUNCTIONS){
                cout << "showFullScreen();" << endl;
            }
            self->showFullScreen();
            //isGoingToMaximized = false;

            sigFullScreenToggled(isFullScreen);

        } else if(config->get("maximized", false)){
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

        viewArea->setViewTabsVisible(config->get({ "show_view_tabs" }, true));
        self->statusBar()->setVisible(config->get("show_status_bar", true));

        isBeforeShowing = false;
    }
}


bool MainWindow::isActivatedInWindowSystem() const
{
#ifdef Q_OS_UNIX
    return impl->windowActivationChecker.isWindowActivated;
#else
    return !impl->isBeforeShowing;
#endif
}


bool MainWindow::waitForWindowSystemToActivate()
{
#ifndef Q_OS_UNIX
    return isActivatedInWindowSystem();

#else
    /**
       There is a delay between the initial execution of the show function of a window and the window
       is actually displayed first. If the event loop is blocked by an operation that takes a long
       time before the window is displayed, the window will remain hidden or its contents will be black
       for a while. This behavior gives a user the bad impression that the application is slow to start.
       To avoid this problem, it is better to wait for the window system to actually display the window
       and then start the initialization process that may take some time. The following code wait for
       the window system to activate the main window on Linux.
    */
    int timeoutCounter = 0;
    while(true){
        App::updateGui();
        if(impl->windowActivationChecker.isWindowActivated){
            break;
        }
        msleep(1);
        ++timeoutCounter;
        if(timeoutCounter > 100){
            break;
        }
    }
    removeEventFilter(&impl->windowActivationChecker);

    return impl->windowActivationChecker.isWindowActivated;
#endif
}


void MainWindow::toggleFullScreen()
{
    setFullScreen(!impl->isFullScreen);
}


void MainWindow::setFullScreen(bool on)
{
    bool changed = false;
    
    if(on){
        if(!isFullScreen()){
            impl->isMaximizedJustBeforeFullScreen = impl->isMaximized;
            impl->isGoingToMaximized = true;
            showFullScreen();
            impl->isGoingToMaximized = false;
            changed = true;
        }
    } else {
        if(isFullScreen()){
            if(impl->isMaximizedJustBeforeFullScreen){
                impl->isGoingToMaximized = true;
                showMaximized();
                impl->isGoingToMaximized = false;
            } else {
                showNormal();
            }
            changed = true;
        }
    }
    impl->isFullScreen = on;

    if(changed){
        impl->sigFullScreenToggled(on);
    }
}


SignalProxy<void(bool on)> MainWindow::sigFullScreenToggled()
{
    return impl->sigFullScreenToggled;
}


void MainWindow::resizeEvent(QResizeEvent* event)
{
    QMainWindow::resizeEvent(event);
    impl->resizeEvent(event);
}


void MainWindow::Impl::resizeEvent(QResizeEvent* event)
{
    if(TRACE_FUNCTIONS){
        cout << "MainWindow::Impl::resizeEvent(): size = (";
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
                cout << "MainWindow::Impl::resizeEvent(): initializeLayout" << endl;
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


void MainWindow::restoreLayout(Archive* archive)
{
    impl->restoreLayout(archive);
}


void MainWindow::Impl::restoreLayout(Archive* archive)
{
    if(!isBeforeDoingInitialLayout){
        toolBarArea->restoreLayout(archive);
    } else {
        toolBarArea->doInitialLayout();
    }

    ViewArea::restoreAllViewAreaLayouts(archive);
}


void MainWindow::resetLayout()
{
    impl->toolBarArea->resetLayout(impl->config);
    impl->viewArea->resetLayout();
}


void MainWindow::storeLayout(Archive* archive)
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


void MainWindow::Impl::storeWindowStateConfig()
{
    config->clear();
    
    config->write("width", normalSize.width());
    config->write("height", normalSize.height());

    if(isFullScreen){
        config->write("maximized", isMaximizedJustBeforeFullScreen);
    } else {
        config->write("maximized", isMaximized);
    }
    config->write("full_screen", isFullScreen);

    config->write("show_view_tabs", viewArea->viewTabsVisible());
    config->write("show_status_bar", InfoBar::instance()->isVisible());

    /*
    config->write("storeLastLayout", storeLastLayoutCheck->isChecked());
    if(storeLastLayoutCheck->isChecked()){
        toolBarArea->storeLayout(config);
    } else {
        toolBarArea->removeLayout(config);
    }
    */
}


void MainWindow::keyPressEvent(QKeyEvent* event)
{
    impl->keyPressEvent(event);
}


void MainWindow::Impl::keyPressEvent(QKeyEvent* event)
{
    switch(event->key()){

    case Qt::Key_Z:
        if(event->modifiers() & Qt::ControlModifier){
            if(event->modifiers() & Qt::ShiftModifier){
                UnifiedEditHistory::instance()->redo();
            } else {
                UnifiedEditHistory::instance()->undo();
            }
        }
        break;

    case Qt::Key_Y:
        if (event->modifiers() & Qt::ControlModifier) {
            UnifiedEditHistory::instance()->redo();
        }
        break;
        
    case Qt::Key_F11:
        self->toggleFullScreen();
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

    sigKeyPressed_(event);
}


void MainWindow::keyReleaseEvent(QKeyEvent* event)
{
    sigKeyReleased_(event);
    event->ignore();
}


SignalProxy<void(QKeyEvent* event)> MainWindow::sigKeyPressed()
{
    return sigKeyPressed_;
}


SignalProxy<void(QKeyEvent* event)> MainWindow::sigKeyReleased()
{
    return sigKeyReleased_;
}
