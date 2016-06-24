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
#include <boost/bind.hpp>
#include <iostream>
#include "gettext.h"

using namespace std;
using namespace cnoid;

namespace {

const bool TRACE_FUNCTIONS = false;

MainWindow* mainWindow = 0;

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
    QSize normalStateSize;
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

    normalStateSize.setWidth(config->get("width", self->width()));
    normalStateSize.setHeight(config->get("height", self->width()));
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
        ->sigTriggered().connect(boost::bind(&MainWindow::close, self));

    mm.setPath("/" N_("Edit"));

    Menu* viewMenu = static_cast<Menu*>(mm.setPath("/" N_("View")).current());

    mm.setPath(N_("Show Toolbar"));
    Menu* showToolBarMenu = static_cast<Menu*>(mm.current());
    showToolBarMenu->sigAboutToShow().connect(boost::bind(&ToolBarArea::setVisibilityMenuItems, toolBarArea, showToolBarMenu));
    
    mm.setCurrent(viewMenu).setPath(N_("Show View"));
    mm.setCurrent(viewMenu).setPath(N_("Create View"));
    mm.setCurrent(viewMenu).setPath(N_("Delete View"));

    showViewTabCheck = mm.setCurrent(viewMenu).addCheckItem(_("Show View Tabs"));
    showViewTabCheck->sigToggled().connect(boost::bind(&ViewArea::setViewTabsVisible, viewArea, _1));
    showViewTabCheck->setChecked(config->get("showViewTabs", true));

    mm.setCurrent(viewMenu).addSeparator();

    Action* showStatusBarCheck = mm.setCurrent(viewMenu).addCheckItem(_("Show Status Bar"));
    bool showStatusBar = config->get("showStatusBar", true);
    QWidget* statusBar = InfoBar::instance();
    showStatusBarCheck->setChecked(showStatusBar);
    showStatusBarCheck->sigToggled().connect(boost::bind(&QWidget::setVisible, statusBar, _1));
    
    fullScreenCheck = mm.addCheckItem(_("Full Screen"));
    fullScreenCheck->setChecked(config->get("fullScreen", false));
    fullScreenCheck->sigToggled().connect(boost::bind(&MainWindowImpl::onFullScreenToggled, this, _1));

    mm.setCurrent(viewMenu).setPath(N_("Layout"));
    
    storeLastLayoutCheck = mm.addCheckItem(_("Store Last Toolbar Layout"));
    storeLastLayoutCheck->setChecked(config->get("storeLastLayout", false));

    mm.addItem(_("Reset Layout"))->sigTriggered().connect(boost::bind(&MainWindowImpl::resetLayout, this));
    
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
            self->showFullScreen();
        }
    } else {
        if(self->isFullScreen()){
            self->showNormal();
            if(isMaximizedJustBeforeFullScreen){
                self->showMaximized();
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
            
            restoreLayout(initialLayoutArchive);
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
