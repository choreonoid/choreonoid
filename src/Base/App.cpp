/**
   @author Shin'ichiro Nakaoka
*/

#include "App.h"
#include "AppUtil.h"
#include "AppConfig.h"
#include "ExtensionManager.h"
#include "OptionManager.h"
#include "PluginManager.h"
#include "ItemManager.h"
#include "ProjectManager.h"
#include "MenuManager.h"
#include "TimeSyncItemEngine.h"
#include "MainWindow.h"
#include "RootItem.h"
#include "FolderItem.h"
#include "ExtCommandItem.h"
#include "SceneItem.h"
#include "PointSetItem.h"
#include "MultiPointSetItem.h"
#include "ViewManager.h"
#include "MessageView.h"
#include "ItemTreeView.h"
#include "ItemPropertyView.h"
#include "SceneView.h"
#include "FileBar.h"
#include "ScriptBar.h"
#include "TimeBar.h"
#include "SceneBar.h"
#include "CaptureBar.h"
#include "ImageView.h"
#include "TaskView.h"
#include "GraphBar.h"
#include "MultiValueSeqGraphView.h"
#include "MultiSE3SeqGraphView.h"
#include "MultiValueSeqItem.h"
#include "MultiSE3SeqItem.h"
#include "MultiAffine3SeqItem.h"
#include "Vector3SeqItem.h"
#include "PathVariableEditor.h"
#include "Licenses.h"
#include "MovieGenerator.h"
#include "LazyCaller.h"
#include "TextEditView.h"
#include "DescriptionDialog.h"
#include <cnoid/Config>
#include <cnoid/ValueTree>
#include <QApplication>
#include <QTextCodec>
#include <QGLFormat>
#include <QTextStream>
#include <QFile>
#include <boost/bind.hpp>

#include <csignal>

#ifdef Q_OS_WIN32
#include <windows.h>
#endif

#include "gettext.h"

using namespace cnoid;

namespace {

View* lastFocusView_ = 0;
Signal<void(View*)> sigFocusViewChanged;

Signal<void()> sigAboutToQuit_;

void onCtrl_C_Input(int p)
{
    callLater(boost::bind(&MainWindow::close, MainWindow::instance()));
}

#ifdef Q_OS_WIN32
BOOL WINAPI consoleCtrlHandler(DWORD ctrlChar)
{
    callLater(boost::bind(&MainWindow::close, MainWindow::instance()));
    return FALSE;
}
#endif

}


namespace cnoid {

class AppImpl
{
    App* self;
    QApplication* qapplication;
    int& argc;
    char**& argv;
    ExtensionManager* ext;
    MainWindow* mainWindow;
    std::string appName;
    std::string vendorName;
    DescriptionDialog* descriptionDialog;
    bool doQuit;
        
    AppImpl(App* self, int& argc, char**& argv);
    ~AppImpl();
    void initialize(const char* appName, const char* vendorName, const QIcon& icon, const char* pluginPathList);
    int exec();
    void onMainWindowCloseEvent();
    void onSigOptionsParsed(boost::program_options::variables_map& v);
    bool processCommandLineOptions();
    void showInformationDialog();
    void onOpenGLVSyncToggled(bool on);

    friend class App;
    friend class View;
};

}


App::App(int& argc, char**& argv)
{
    impl = new AppImpl(this, argc, argv);
}

#include <iostream>
AppImpl::AppImpl(App* self, int& argc, char**& argv)
    : self(self),
      argc(argc),
      argv(argv)
{
    descriptionDialog = 0;

#if QT_VERSION < QT_VERSION_CHECK(5, 0, 0)
#if defined(__APPLE__) && defined(__MACH__)
    if(!getenv("QT_GRAPHICSSYSTEM")){
        QApplication::setGraphicsSystem("raster"); // to obtain better rendering performance
    }
#endif
#endif

    QCoreApplication::setAttribute(Qt::AA_X11InitThreads);

    doQuit = false;

    qapplication = new QApplication(argc, argv);

    self->connect(qapplication, SIGNAL(focusChanged(QWidget*, QWidget*)),
                  self, SLOT(onFocusChanged(QWidget*, QWidget*)));
}


void App::initialize(const char* appName, const char* vendorName, const QIcon& icon, const char* pluginPathList)
{
    impl->initialize(appName, vendorName, icon, pluginPathList);
}


void AppImpl::initialize( const char* appName, const char* vendorName, const QIcon& icon, const char* pluginPathList)
{
    this->appName = appName;
    this->vendorName = vendorName;

    setlocale(LC_ALL, ""); // for gettext

#if QT_VERSION < QT_VERSION_CHECK(5, 0, 0)
    QTextCodec::setCodecForCStrings(QTextCodec::codecForLocale());
#else

#endif
    qapplication->setApplicationName(appName);
    qapplication->setOrganizationName(vendorName);
    qapplication->setWindowIcon(icon);

    ext = new ExtensionManager("Base", false);

    AppConfig::initialize(appName, vendorName);

    // OpenGL settings
    Mapping* glConfig = AppConfig::archive()->openMapping("OpenGL");
    QGLFormat glfmt = QGLFormat::defaultFormat();
    glfmt.setSwapInterval(glConfig->get("vsync", 0));
    QGLFormat::setDefaultFormat(glfmt);
    
    mainWindow = MainWindow::initialize(appName, ext);
    
    ViewManager::initializeClass(ext);
    
    MessageView::initializeClass(ext);
    RootItem::initializeClass(ext);
    ProjectManager::initialize(ext);

    FileBar::initialize(ext);
    ScriptBar::initialize(ext);
    TimeBar::initialize(ext);
    ItemTreeView::initializeClass(ext);
    ItemPropertyView::initializeClass(ext);
    TextEditView::initializeClass(ext);
    SceneBar::initialize(ext);
    SceneView::initializeClass(ext);
    ImageView::initializeClass(ext);
    GraphBar::initialize(ext);
    MultiValueSeqGraphView::initializeClass(ext);
    MultiSE3SeqGraphView::initializeClass(ext);
    TaskView::initializeClass(ext);

    TimeSyncItemEngineManager::initialize();
    
    FolderItem::initializeClass(ext);
    ExtCommandItem::initializeClass(ext);
    MultiValueSeqItem::initializeClass(ext);
    MultiSE3SeqItem::initializeClass(ext);
    MultiAffine3SeqItem::initializeClass(ext);
    Vector3SeqItem::initializeClass(ext);
    SceneItem::initializeClass(ext);
    PointSetItem::initializeClass(ext);
    MultiPointSetItem::initializeClass(ext);

    initializeMovieGenerator(ext);

    CaptureBar::initialize(ext);
    
    PathVariableEditor::initialize(ext);
    
    ext->menuManager().setPath("/Help").addItem(_("About Choreonoid"))
        ->sigTriggered().connect(boost::bind(&AppImpl::showInformationDialog, this));

    // OpenGL settings
    Action* vsyncItem = ext->menuManager().setPath("/Options/OpenGL").addCheckItem(_("Vertical Sync"));
    vsyncItem->setChecked(glfmt.swapInterval() > 0);
    vsyncItem->sigToggled().connect(boost::bind(&AppImpl::onOpenGLVSyncToggled, this, _1));

    PluginManager::initialize(ext);
    PluginManager::instance()->doStartupLoading(pluginPathList);

    mainWindow->installEventFilter(self);

    OptionManager& om = ext->optionManager();
    om.addOption("quit", "quit the application just after it is invoked");
    om.sigOptionsParsed().connect(boost::bind(&AppImpl::onSigOptionsParsed, this, _1));

    // Some plugins such as OpenRTM plugin are driven by a library which tries to catch SIGINT.
    // This may block the normal termination by inputting Ctrl+C.
    // To avoid it, the following signal handliers are set.
    std::signal(SIGINT, onCtrl_C_Input);
    std::signal(SIGTERM, onCtrl_C_Input);

#ifdef Q_OS_WIN32
    // The above SIGINT handler seems to work even on Windows
    // when Choreonoid is compiled as a console-program,
    // and the following handler only works for a console-program, too.
    // Hence the folloing handler for Windows is currently disabled.
    // SetConsoleCtrlHandler(consoleCtrlHandler, TRUE);
#endif

#ifdef Q_OS_LINUX
    /**
       The following code is neccessary to avoid a crash when a view which has a widget such as
       QPlainTextEdit and has not been focused yet is first focused (clikced) during the camera
       image simulation processed by GLVisionSimulatorItem. The crash only occurs in Linux with
       the nVidia proprietary X driver. If the user clicks such a view to give the focus before
       the simulation started, the crash doesn't occur, so here the focus is forced to be given
       by the following code.
    */
    /**
       This is now executed in GLVisionSimulatorItem::initializeSimulation
       
       if(QWidget* textEdit = MessageView::instance()->findChild<QWidget*>("TextEdit")){
       textEdit->setFocus();
       textEdit->clearFocus();
       }
    */
#endif
}


App::~App()
{
    if(impl){
        delete impl;
    }
}


AppImpl::~AppImpl()
{
    AppConfig::flush();
    delete qapplication;
}


int App::exec()
{
    return impl->exec();
}


int AppImpl::exec()
{
    processCommandLineOptions();

    if(!mainWindow->isVisible()){
        mainWindow->show();
    }

    int result = 0;
    
    if(doQuit){
        MessageView::instance()->flush();
    } else {
        result = qapplication->exec();
    }

    PluginManager::finalize();
    delete ext;
    delete mainWindow;
    mainWindow = 0;
    
    return result;
}


bool App::eventFilter(QObject* watched, QEvent* event)
{
    if(watched == impl->mainWindow && event->type() == QEvent::Close){
        impl->onMainWindowCloseEvent();
        event->accept();
        return true;
    }
    return false;
}


void AppImpl::onMainWindowCloseEvent()
{
    sigAboutToQuit_();
    mainWindow->storeWindowStateConfig();

    QWidgetList windows = QApplication::topLevelWidgets();
    for(int i=0; i < windows.size(); ++i){
        QWidget* window = windows[i];
        if(window != mainWindow){
            window->close();
        }
    }
}    


SignalProxy<void()> cnoid::sigAboutToQuit()
{
    return sigAboutToQuit_;
}


void AppImpl::onSigOptionsParsed(boost::program_options::variables_map& v)
{
    if(v.count("quit")){
        doQuit = true;
    }
}
    

bool AppImpl::processCommandLineOptions()
{
    if(!ext->optionManager().parseCommandLine(argc, argv)){
        //put error messages
    }

    return false;
}


void AppImpl::showInformationDialog()
{
    if(!descriptionDialog){

        descriptionDialog = new DescriptionDialog();

        descriptionDialog->setWindowTitle(_("About Choreonoid"));

        QFile resource(":/Base/LICENSE");
        if(resource.open(QIODevice::ReadOnly | QIODevice::Text)){
            QTextStream license(&resource);
            descriptionDialog->setDescription(
                QString("Choreonoid Version %1\n\n").arg(CNOID_FULL_VERSION_STRING) +
                license.readAll());
        }
    }

    descriptionDialog->show();
}


void AppImpl::onOpenGLVSyncToggled(bool on)
{
    Mapping* glConfig = AppConfig::archive()->openMapping("OpenGL");
    glConfig->write("vsync", (on ? 1 : 0));

    // show messag dialog to prompt restart here
}


void App::onFocusChanged(QWidget* old, QWidget* now)
{
    while(now){
        View* view = dynamic_cast<View*>(now);
        if(view){
            lastFocusView_ = view;
            sigFocusViewChanged(lastFocusView_);
            break;
        }
        now = now->parentWidget();
    }
}


View* View::lastFocusView()
{
    return lastFocusView_;
}


SignalProxy<void(View*)> View::sigFocusChanged()
{
    return sigFocusViewChanged;
}


/**
   This function is called by a View oject when the object to delete
   is the current focus view.
*/
void App::clearFocusView()
{
    if(lastFocusView_){
        lastFocusView_ = 0;
        sigFocusViewChanged(0);
    }
}
