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
#include "SubProjectItem.h"
#include "ExtCommandItem.h"
#include "SceneItem.h"
#include "PointSetItem.h"
#include "MultiPointSetItem.h"
#include "LightingItem.h"
#include "MessageLogItem.h"
#include "MultiValueSeqItem.h"
#include "MultiSE3SeqItem.h"
#include "MultiSE3MatrixSeqItem.h"
#include "Vector3SeqItem.h"
#include "CoordinateFrameListItem.h"
#include "MultiCoordinateFrameListItem.h"
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
#include "CoordinateFrameListView.h"
#include "TextEditView.h"
#include "GeneralSliderView.h"
#include "VirtualJoystickView.h"
#include "PathVariableEditor.h"
#include "GLSceneRenderer.h"
#include "Licenses.h"
#include "MovieRecorder.h"
#include "LazyCaller.h"
#include "DescriptionDialog.h"
#include <cnoid/Config>
#include <cnoid/ValueTree>
#include <cnoid/CnoidUtil>
#include <cnoid/ParametricPathProcessor>
#include <fmt/format.h>
#include <Eigen/Core>
#include <QApplication>
#include <QTextCodec>
#include <QSurfaceFormat>
#include <QTextStream>
#include <QFile>
#include <QStyleFactory>
#include <iostream>
#include <csignal>

#ifdef Q_OS_WIN32
#include <windows.h>
#endif

#include "gettext.h"

using namespace std;
using namespace cnoid;

namespace {

View* lastFocusView_ = 0;
Signal<void(View*)> sigFocusViewChanged;

Signal<void()> sigAboutToQuit_;

void onCtrl_C_Input(int)
{
    callLater([](){ MainWindow::instance()->close(); });
}

#ifdef Q_OS_WIN32
BOOL WINAPI consoleCtrlHandler(DWORD ctrlChar)
{
    callLater([](){ MainWindow::instance()->close(); });
    return FALSE;
}
#endif

}

namespace cnoid {

class App::Impl : public QObject
{
    App* self;
    QApplication* qapplication;
    int& argc;
    char**& argv;
    ExtensionManager* ext;
    MainWindow* mainWindow;
    MessageView* messageView;
    string appName;
    string vendorName;
    DescriptionDialog* descriptionDialog;
    bool doQuit;
    
    Impl(App* self, int& argc, char**& argv);
    ~Impl();
    void initialize(const char* appName, const char* vendorName, const char* pluginPathList);
    int exec();
    void onMainWindowCloseEvent();
    void onSigOptionsParsed(boost::program_options::variables_map& v);
    void showInformationDialog();
    void onFocusChanged(QWidget* /* old */, QWidget* now);
    virtual bool eventFilter(QObject* watched, QEvent* event);

    friend class App;
    friend class View;
};

}


App::App(int& argc, char**& argv)
{
    impl = new Impl(this, argc, argv);
}


App::Impl::Impl(App* self, int& argc, char**& argv)
    : self(self),
      argc(argc),
      argv(argv)
{
    descriptionDialog = 0;

    QCoreApplication::setAttribute(Qt::AA_X11InitThreads);

    // OpenGL settings
    GLSceneRenderer::initializeClass();
    QSurfaceFormat format;
    switch(GLSceneRenderer::rendererType()){
    case GLSceneRenderer::GLSL_RENDERER:
        format.setVersion(3, 3);
        //format.setVersion(4, 4);
        format.setProfile(QSurfaceFormat::CoreProfile);
        break;
    case GLSceneRenderer::GL1_RENDERER:
    default:
        format.setVersion(1, 5);
        break;
    }
    QSurfaceFormat::setDefaultFormat(format);

    doQuit = false;

    qapplication = new QApplication(argc, argv);

    connect(qapplication, &QApplication::focusChanged,
            [&](QWidget* old, QWidget* now){ onFocusChanged(old, now); });
}


void App::initialize(const char* appName, const char* vendorName, const char* pluginPathList)
{
    impl->initialize(appName, vendorName, pluginPathList);
}


void App::Impl::initialize( const char* appName, const char* vendorName, const char* pluginPathList)
{
    this->appName = appName;
    this->vendorName = vendorName;

    setlocale(LC_ALL, ""); // for gettext

    QTextCodec::setCodecForLocale(QTextCodec::codecForName("UTF-8"));
    qapplication->setApplicationName(appName);
    qapplication->setOrganizationName(vendorName);

    QIcon icon;
    icon.addFile(":/Base/icons/choreonoid32.png");
    icon.addFile(":/Base/icons/choreonoid48.png");
    qapplication->setWindowIcon(icon);

    AppConfig::initialize(appName, vendorName);

    ParametricPathProcessor::instance()->setVariables(
        AppConfig::archive()->openMapping("pathVariables"));

    ext = new ExtensionManager("Base", false);

#ifdef CNOID_ENABLE_GETTEXT
    setCnoidUtilTextDomainCodeset();
#endif

    mainWindow = MainWindow::initialize(appName, ext);

    ViewManager::initializeClass(ext);
    
    MessageView::initializeClass(ext);
    messageView = MessageView::instance();
    RootItem::initializeClass(ext);
    ProjectManager::initializeClass(ext);

    FileBar::initialize(ext);
    ScriptBar::initialize(ext);
    TimeBar::initialize(ext);
    ItemTreeView::initializeClass(ext);
    ItemPropertyView::initializeClass(ext);
    SceneBar::initialize(ext);
    SceneView::initializeClass(ext);
    ImageViewBar::initialize(ext);
    ImageView::initializeClass(ext);
    TextEditView::initializeClass(ext);
    GeneralSliderView::initializeClass(ext);
    GraphBar::initialize(ext);
    MultiValueSeqGraphView::initializeClass(ext);
    MultiSE3SeqGraphView::initializeClass(ext);
    CoordinateFrameListView::initializeClass(ext);
    TaskView::initializeClass(ext);
    VirtualJoystickView::initializeClass(ext);

    TimeSyncItemEngineManager::initialize();
    
    FolderItem::initializeClass(ext);
    SubProjectItem::initializeClass(ext);
    ExtCommandItem::initializeClass(ext);
    MultiValueSeqItem::initializeClass(ext);
    MultiSE3SeqItem::initializeClass(ext);
    MultiSE3MatrixSeqItem::initializeClass(ext);
    Vector3SeqItem::initializeClass(ext);
    SceneItem::initializeClass(ext);
    PointSetItem::initializeClass(ext);
    MultiPointSetItem::initializeClass(ext);
    MessageLogItem::initializeClass(ext);
    LightingItem::initializeClass(ext);
    CoordinateFrameListItem::initializeClass(ext);
    MultiCoordinateFrameListItem::initializeClass(ext);

    MovieRecorder::initialize(ext);

    CaptureBar::initialize(ext);
    
    PathVariableEditor::initialize(ext);

    ext->menuManager().setPath("/Help").addItem(_("About Choreonoid"))
        ->sigTriggered().connect([&](){ showInformationDialog(); });

    messageView->putln(
        fmt::format(_("The Eigen library version {0}.{1}.{2} is used (SIMD intruction sets in use: {3})."),
                    EIGEN_WORLD_VERSION, EIGEN_MAJOR_VERSION, EIGEN_MINOR_VERSION,
                    Eigen::SimdInstructionSetsInUse()));

    PluginManager::initialize(ext);
    PluginManager::instance()->doStartupLoading(pluginPathList);

    mainWindow->installEventFilter(this);

    OptionManager& om = ext->optionManager();
    om.addOption("quit", "quit the application just after it is invoked");
    om.addOption("list-qt-styles", "list all the available qt styles");
    om.sigOptionsParsed().connect(
        [&](boost::program_options::variables_map& v){ onSigOptionsParsed(v); });

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
       
       if(QWidget* textEdit = messageView->findChild<QWidget*>("TextEdit")){
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


App::Impl::~Impl()
{
    AppConfig::flush();
    delete qapplication;
}


int App::exec()
{
    return impl->exec();
}


int App::Impl::exec()
{
    if(!ext->optionManager().parseCommandLine1(argc, argv)){
        //exit
    }

    if(!mainWindow->isVisible()){
        mainWindow->show();
    }

    ext->optionManager().parseCommandLine2();

    int result = 0;
    
    if(doQuit){
        messageView->flush();
    } else {
        result = qapplication->exec();
    }

    for(Item* item = RootItem::instance()->childItem(); item; ){
        Item* next = item->nextItem(); // detachFromParentItem() may deallocate item
        item->detachFromParentItem();
        item = next;
    }

    PluginManager::finalize();
    delete ext;
    delete mainWindow;
    mainWindow = 0;
    
    return result;
}


bool App::Impl::eventFilter(QObject* watched, QEvent* event)
{
    if(watched == mainWindow && event->type() == QEvent::Close){
        onMainWindowCloseEvent();
        event->accept();
        return true;
    }
    return false;
}


void App::Impl::onMainWindowCloseEvent()
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


void App::Impl::onSigOptionsParsed(boost::program_options::variables_map& v)
{
    if(v.count("quit")){
        doQuit = true;
    } else if(v.count("list-qt-styles")){
        cout << QStyleFactory::keys().join(" ").toStdString() << endl;
        doQuit = true;
    }
}
    

void App::Impl::showInformationDialog()
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


void App::Impl::onFocusChanged(QWidget* /* old */, QWidget* now)
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
        sigFocusViewChanged(nullptr);
    }
}
