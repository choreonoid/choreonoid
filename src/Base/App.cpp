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
#include "MessageView.h"
#include "RootItem.h"
#include "ProjectManager.h"
#include "UnifiedEditHistory.h"
#include "UnifiedEditHistoryView.h"
#include "ItemEditRecordManager.h"
#include "MenuManager.h"
#include "TimeSyncItemEngine.h"
#include "MainWindow.h"
#include "FolderItem.h"
#include "SubProjectItem.h"
#include "ExtCommandItem.h"
#include "SceneItem.h"
#include "PointSetItem.h"
#include "MultiPointSetItem.h"
#include "LightingItem.h"
#include "AbstractTextItem.h"
#include "ScriptItem.h"
#include "MessageLogItem.h"
#include "AbstractSeqItem.h"
#include "MultiValueSeqItem.h"
#include "MultiSE3SeqItem.h"
#include "MultiSE3MatrixSeqItem.h"
#include "Vector3SeqItem.h"
#include "ReferencedObjectSeqItem.h"
#include "CoordinateFrameListItem.h"
#include "CoordinateFrameItem.h"
#include "PositionTagGroupItem.h"
#include "ViewManager.h"
#include "ItemTreeView.h"
#include "ItemPropertyView.h"
#include "SceneView.h"
#include "LocationView.h"
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
#include <cnoid/FilePathVariableProcessor>
#include <fmt/format.h>
#include <Eigen/Core>
#include <QApplication>
#include <QTextCodec>
#include <QSurfaceFormat>
#include <QTextStream>
#include <QFile>
#include <QStyleFactory>
#include <QThread>
#include <iostream>
#include <csignal>
#include <cstdlib>

#ifdef Q_OS_WIN32
#include <windows.h>
#include <mbctype.h>
#endif

#include "gettext.h"

using namespace std;
using namespace cnoid;

namespace {

Signal<void()> sigAboutToQuit_;

void onCtrl_C_Input(int)
{
    callLater([](){ MainWindow::instance()->close(); });
}

#ifdef Q_OS_WIN32
int argc_winmain;
vector<char*> argv_winmain{ "application" };

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
public:
    App* self;
    QApplication* qapplication;
    int& argc;
    char** argv;
    ExtensionManager* ext;
    MainWindow* mainWindow;
    MessageView* messageView;
    string appName;
    string vendorName;
    DescriptionDialog* descriptionDialog;
    bool doQuit;
    
    Impl(App* self, int& argc, char** argv);
    ~Impl();
    void initialize(const char* appName, const char* vendorName, const char* pluginPathList);
    int exec();
    void onMainWindowCloseEvent();
    void onSigOptionsParsed(boost::program_options::variables_map& v);
    void showInformationDialog();
    virtual bool eventFilter(QObject* watched, QEvent* event);
};

}


App::App(int& argc, char** argv)
{
    impl = new Impl(this, argc, argv);
}


#ifdef _WIN32
App::App(HINSTANCE hInstance, HINSTANCE hPrevInstance, LPSTR lpCmdLine, int nCmdShow)
{
#ifndef UNICODE
    vector<string> args = boost::program_options::split_winmain(lpCmdLine);
    for(size_t i=0; i < args.size(); ++i){
        char* arg = new char[args[i].size() + 1];
        strcpy(arg, args[i].c_str());
        argv_winmain.push_back(arg);
    }
#else
    vector<wstring> wargs = boost::program_options::split_winmain(lpCmdLine);
    vector<char> buf;
    vector<string> args(wargs.size());
    int codepage = _getmbcp();
    for(size_t i=0; i < args.size(); ++i){
        const int size = WideCharToMultiByte(codepage, 0, &wargs[i][0], wargs[i].size(), NULL, 0, NULL, NULL);
        char* arg;
        if(size > 0){
            arg = new char[size + 1];
            WideCharToMultiByte(codepage, 0, &wargs[i][0], wargs[i].size(), arg, size + 1, NULL, NULL);
        } else {
            arg = new char[1];
            arg[0] = '\0';
        }
        argv_winmain.push_back(arg);
    }
#endif
    argc_winmain = argv_winmain.size();
    impl = new Impl(this, argc_winmain, &argv_winmain[0]);
}
#endif


App::Impl::Impl(App* self, int& argc, char** argv)
    : self(self),
      argc(argc),
      argv(argv)
{
    descriptionDialog = nullptr;
    doQuit = false;
}


void App::initialize(const char* appName, const char* vendorName, const char* pluginPathList)
{
    impl->initialize(appName, vendorName, pluginPathList);
}


void App::Impl::initialize( const char* appName, const char* vendorName, const char* pluginPathList)
{
    this->appName = appName;
    this->vendorName = vendorName;

    AppConfig::initialize(appName, vendorName);

    // OpenGL settings
    GLSceneRenderer::initializeClass();

    QSurfaceFormat glFormat = QSurfaceFormat::defaultFormat();
    
    switch(GLSceneRenderer::rendererType()){
    case GLSceneRenderer::GLSL_RENDERER:
        glFormat.setVersion(3, 3);
        glFormat.setProfile(QSurfaceFormat::CoreProfile);
        break;
    case GLSceneRenderer::GL1_RENDERER:
    default:
        glFormat.setVersion(1, 5);
        break;
    }
    
    auto glConfig = AppConfig::archive()->openMapping("open_gl");
    glFormat.setSwapInterval(glConfig->get("vsync", 0));

    QSurfaceFormat::setDefaultFormat(glFormat);

    QCoreApplication::setAttribute(Qt::AA_X11InitThreads);
    QCoreApplication::setAttribute(Qt::AA_UseHighDpiPixmaps);

    /*
      This attribute is necessary to render the scene on a scene view when the view is
      separated from the main window. Note that the default surface format must be
      initialized before creating the QApplication instance.
    */
    QCoreApplication::setAttribute(Qt::AA_ShareOpenGLContexts);

    qapplication = new QApplication(argc, argv);

#ifdef Q_OS_UNIX
    // See https://doc.qt.io/qt-5/qcoreapplication.html#locale-settings
    setlocale(LC_NUMERIC, "C");
#endif

    QTextCodec::setCodecForLocale(QTextCodec::codecForName("UTF-8"));
    qapplication->setApplicationName(appName);
    qapplication->setOrganizationName(vendorName);

    qapplication->setWindowIcon(QIcon(":/Base/icon/choreonoid.svg"));

    FilePathVariableProcessor::systemInstance()->setUserVariables(
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
    UnifiedEditHistory::initializeClass(ext);
    UnifiedEditHistoryView::initializeClass(ext);
    ItemEditRecordManager::initializeClass(ext);

    FileBar::initialize(ext);
    ScriptBar::initialize(ext);
    TimeBar::initialize(ext);
    ItemTreeView::initializeClass(ext);
    ItemPropertyView::initializeClass(ext);
    SceneView::initializeClass(ext);
    // SceneBar must be initialized after the initialization of SceneView
    SceneBar::initialize(ext);
    LocationView::initializeClass(ext);
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

    TimeSyncItemEngineManager::initializeClass(ext);
    
    FolderItem::initializeClass(ext);
    SubProjectItem::initializeClass(ext);
    ExtCommandItem::initializeClass(ext);
    AbstractSeqItem::initializeClass(ext);
    AbstractMultiSeqItem::initializeClass(ext);
    MultiValueSeqItem::initializeClass(ext);
    MultiSE3SeqItem::initializeClass(ext);
    MultiSE3MatrixSeqItem::initializeClass(ext);
    Vector3SeqItem::initializeClass(ext);
    ReferencedObjectSeqItem::initializeClass(ext);
    SceneItem::initializeClass(ext);
    PointSetItem::initializeClass(ext);
    MultiPointSetItem::initializeClass(ext);
    AbstractTextItem::initializeClass(ext);
    ScriptItem::initializeClass(ext);
    MessageLogItem::initializeClass(ext);
    
    LightingItem::initializeClass(ext);
    CoordinateFrameListItem::initializeClass(ext);
    CoordinateFrameItem::initializeClass(ext);
    PositionTagGroupItem::initializeClass(ext);

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
    if(!pluginPathList){
        pluginPathList = getenv("CNOID_PLUGIN_PATH");
    }
    PluginManager::instance()->doStartupLoading(pluginPathList);

    mainWindow->installEventFilter(this);

    OptionManager& om = ext->optionManager();
    om.addOption("quit", "quit the application just after it is invoked");
    om.addOption("list-qt-styles", "list all the available qt styles");
    om.sigOptionsParsed().connect(
        [&](boost::program_options::variables_map& v){ onSigOptionsParsed(v); });

    /*
      Some plugins such as OpenRTM plugin are driven by a library which tries to catch SIGINT.
      This may block the normal termination by inputting Ctrl+C.
      To avoid it, the following signal handliers are set.
    */
    std::signal(SIGINT, onCtrl_C_Input);
    std::signal(SIGTERM, onCtrl_C_Input);

#ifdef Q_OS_WIN32
    /*
      The above SIGINT handler seems to work even on Windows
      when Choreonoid is compiled as a console-program,
      and the following handler only works for a console-program, too.
      Hence the folloing handler for Windows is currently disabled.
    */
    // SetConsoleCtrlHandler(consoleCtrlHandler, TRUE);
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
        Item* next = item->nextItem(); // removeFromParentItem() may deallocate item
        item->removeFromParentItem();
        item = next;
    }

    PluginManager::finalize();
    delete ext;
    delete mainWindow;
    mainWindow = nullptr;
    
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


SignalProxy<void()> cnoid::sigAboutToQuit()
{
    return sigAboutToQuit_;
}


void cnoid::updateGui()
{
    QCoreApplication::processEvents(
        QEventLoop::ExcludeUserInputEvents | QEventLoop::ExcludeSocketNotifiers, 1.0);
}
