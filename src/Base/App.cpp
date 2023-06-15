#include "App.h"
#include "AppUtil.h"
#include "AppConfig.h"
#include "AppCustomizationUtil.h"
#include "ExtensionManager.h"
#include "PluginManager.h"
#include "Plugin.h"
#include "OptionManager.h"
#include "ItemManager.h"
#include "MessageView.h"
#include "RootItem.h"
#include "ProjectManager.h"
#include "UnifiedEditHistory.h"
#include "UnifiedEditHistoryView.h"
#include "ItemEditRecordManager.h"
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
#include "MultiVector3SeqItem.h"
#include "ReferencedObjectSeqItem.h"
#include "CoordinateFrameListItem.h"
#include "CoordinateFrameItem.h"
#include "PositionTagGroupItem.h"
#include "DistanceMeasurementItem.h"
#include "ViewManager.h"
#include "ItemTreeView.h"
#include "ItemPropertyView.h"
#include "SceneView.h"
#include "LocationView.h"
#include "FileBar.h"
#include "ScriptBar.h"
#include "TimeBar.h"
#include "DisplayValueFormatBar.h"
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
#include "MainMenu.h"
#include "Licenses.h"
#include "MovieRecorderBar.h"
#include "LazyCaller.h"
#include <cnoid/GLSceneRenderer>
#include <cnoid/MessageOut>
#include <cnoid/Config>
#include <cnoid/ValueTree>
#include <cnoid/FilePathVariableProcessor>
#include <cnoid/UTF8>
#include <fmt/format.h>
#include <Eigen/Core>
#include <QApplication>
#include <QTranslator>
#include <QTextCodec>
#include <QSurfaceFormat>
#include <QStyleFactory>
#include <QThread>
#include <QLibraryInfo>
#include <regex>
#include <iostream>
#include <csignal>

#ifdef Q_OS_WIN32
#include <windows.h>
#include <mbctype.h>
#endif

#include "gettext.h"

using namespace std;
using namespace cnoid;

namespace {

App* instance_ = nullptr;

Signal<void()> sigExecutionStarted_;
Signal<void()> sigAboutToQuit_;

bool isDoingInitialization_ = true;
bool isTestMode = false;
bool isNoWindowMode = false;
bool ctrl_c_pressed = false;
bool exitRequested = false;

void onCtrl_C_Input(int)
{
    callLater(
        [](){
            ctrl_c_pressed = true;
            MainWindow::instance()->close();
        });
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
    string appName;
    string organization;
    string pluginPathList;
    string iconFilename;
    string builtinProjectFile;
    MessageOut* mout;
    PluginManager* pluginManager;
    ExtensionManager* ext;
    MainWindow* mainWindow;
    MessageView* messageView;
    QTranslator translator;
    ErrorCode error;
    string errorMessage;
    int returnCode;
    bool isAppInitialized;
    bool doQuit;
    
    Impl(App* self, int& argc, char** argv, const std::string& appName, const std::string& organization);
    ~Impl();
    void initialize();
    int exec();
    void onMainWindowCloseEvent();
    void onSigOptionsParsed(boost::program_options::variables_map& v);
    void enableMessageViewRedirectToStdOut();
    virtual bool eventFilter(QObject* watched, QEvent* event);
};

}


App::App(int& argc, char** argv, const std::string& appName, const std::string& organization)
{
    impl = new Impl(this, argc, argv, appName, organization);
}


#ifdef _WIN32
App::App
(HINSTANCE hInstance, HINSTANCE hPrevInstance, LPSTR lpCmdLine, int nCmdShow,
 const std::string& appName, const std::string& organization)
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
    impl = new Impl(this, argc_winmain, &argv_winmain[0], appName, organization);
}
#endif


App::Impl::Impl(App* self, int& argc, char** argv, const std::string& appName, const std::string& organization)
    : self(self),
      qapplication(nullptr),
      argc(argc),
      argv(argv),
      appName(appName),
      organization(organization)
{
    instance_ = self;
    isDoingInitialization_ = true;

    mout = MessageOut::master();
    mout->setPendingMode(true);
    
    AppConfig::initialize(appName, organization);
    pluginManager = PluginManager::instance();

    ext = nullptr;
    mainWindow = nullptr;
    messageView = nullptr;
    error = NoError;
    returnCode = 0;
    isAppInitialized = false;
    doQuit = false;


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

    glFormat.setSwapInterval(
        AppConfig::archive()->openMapping("OpenGL")->get("vsync", false));

    QSurfaceFormat::setDefaultFormat(glFormat);

#if (QT_VERSION < QT_VERSION_CHECK(5, 6, 0))
    QCoreApplication::setAttribute(Qt::AA_X11InitThreads);
#endif
    
    QCoreApplication::setAttribute(Qt::AA_UseHighDpiPixmaps);

    /*
      This attribute is necessary to render the scene on a scene view when the view is
      separated from the main window. Note that the default surface format must be
      initialized before creating the QApplication instance.
    */
    QCoreApplication::setAttribute(Qt::AA_ShareOpenGLContexts);

#if (QT_VERSION >= QT_VERSION_CHECK(5, 10, 0))
    QCoreApplication::setAttribute(Qt::AA_DisableWindowContextHelpButton);
#endif

    qapplication = new QApplication(argc, argv);
    qapplication->setApplicationName(appName.c_str());
    qapplication->setOrganizationName(organization.c_str());

#ifdef Q_OS_UNIX
    // See https://doc.qt.io/qt-5/qcoreapplication.html#locale-settings
    setlocale(LC_NUMERIC, "C");
#endif

    QTextCodec::setCodecForLocale(QTextCodec::codecForName("UTF-8"));
}


bool App::requirePluginToCustomizeApplication(const std::string& pluginName)
{
    impl->pluginManager->loadPlugins(false);
    
    auto plugin = impl->pluginManager->findPlugin(pluginName);
    if(!plugin){
        impl->error = PluginNotFound;
        impl->errorMessage = impl->pluginManager->getErrorMessage(pluginName);
        return false;
    }

    AppCustomizationUtil util(this, impl->argc, impl->argv);
    if(!plugin->customizeApplication(util)){
        impl->error = CustomizationFailed;
        return false;
    }

    return true;
}


void App::setIcon(const std::string& filename)
{
    impl->iconFilename = filename;
    if(impl->qapplication){
        impl->qapplication->setWindowIcon(QIcon(filename.c_str()));
    }
}


void App::addPluginPath(const std::string& path)
{
    if(!path.empty()){
        impl->pluginManager->addPluginPath(path);
    }
}


void App::setBuiltinProject(const std::string& projectFile)
{
    impl->builtinProjectFile = projectFile;
}


void App::initialize()
{
    impl->initialize();
}


void App::Impl::initialize()
{
    if(checkCurrentLocaleLanguageSupport()){
        translator.load(
            "qt_" + QLocale::system().name(),
            QLibraryInfo::location(QLibraryInfo::TranslationsPath));
        qapplication->installTranslator(&translator);
    }

    qapplication->setWindowIcon(
        QIcon(!iconFilename.empty() ? iconFilename.c_str() : ":/Base/icon/choreonoid.svg"));

    auto fpvp = FilePathVariableProcessor::systemInstance();
    fpvp->restoreUserVariables(AppConfig::archive()->findMapping({ "path_variables", "pathVariables" }));
    FilePathVariableProcessor::setCurrentInstance(fpvp);

    ext = new ExtensionManager("Base", false);

    setUTF8ToModuleTextDomain("Util");

    OptionManager& om = ext->optionManager();
    om.addOption("quit", "stop the application without showing the main window");
    om.addOption("test-mode", "exit the application when an error occurs and put MessageView text to the standard output");
    om.addOption("no-window", "Do not show the application window and put MessageView text to the standard output");
    om.addOption("path-variable", boost::program_options::value<vector<string>>(), "Set a path variable in the format \"name=value\"");
    om.addOption("list-qt-styles", "list all the available qt styles");
    om.sigOptionsParsed().connect(
        [&](boost::program_options::variables_map& v){ onSigOptionsParsed(v); });

    mainWindow = MainWindow::initialize(appName, ext);

    ViewManager::initializeClass(ext);

    MessageView::initializeClass(ext);
    messageView = MessageView::instance();
    mout->flushPendingMessages();
    mout->setPendingMode(false);
    
    ItemManager::initializeClass(ext);
    ProjectManager::initializeClass(ext);
    RootItem::initializeClass(ext);
    UnifiedEditHistory::initializeClass(ext);
    UnifiedEditHistoryView::initializeClass(ext);
    ItemEditRecordManager::initializeClass(ext);

    /**
       Since the main menu may be customized by the main function of a custom application executable
       and the custom main menu may depend on plugins, the main menu setup is processed after
       initializing the plugin manager so that the custom main menu setup can use the functions of it.
    */
    MainMenu::instance()->setMenuItems();

    FileBar::initialize(ext);
    ScriptBar::initialize(ext);
    TimeBar::initialize(ext);
    DisplayValueFormatBar::initialize(ext);
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
    MultiVector3SeqItem::initializeClass(ext);
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
    DistanceMeasurementItem::initializeClass(ext);

    MovieRecorderBar::initializeClass(ext);
    CaptureBar::initialize(ext);
    
    messageView->putln(
        fmt::format(_("The Eigen library version {0}.{1}.{2} is used (SIMD intruction sets in use: {3})."),
                    EIGEN_WORLD_VERSION, EIGEN_MAJOR_VERSION, EIGEN_MINOR_VERSION,
                    Eigen::SimdInstructionSetsInUse()));

    pluginManager->doStartupLoading();

    mainWindow->installEventFilter(this);

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

    if(builtinProjectFile.empty()){
        builtinProjectFile = ":/Base/project/layout.cnoid";
    }
    ProjectManager::instance()->loadBuiltinProject(builtinProjectFile);

    isAppInitialized = true;
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
    delete pluginManager;
}


int App::exec()
{
    return impl->exec();
}


int App::Impl::exec()
{
    if(!isAppInitialized){
        initialize();
    }
        
    isDoingInitialization_ = false;
    
    if(!ext->optionManager().parseCommandLine1(argc, argv)){
        doQuit = true;
    }

    if(!doQuit){
        if(mainWindow->isVisible()){
            App::updateGui();
        } else {
            if(!isNoWindowMode){
                mainWindow->show();
                mainWindow->waitForWindowSystemToActivate();
            }
        }
    }

    if(!doQuit){
        callLater(
            [this](){
                ext->optionManager().parseCommandLine2();
                sigExecutionStarted_();
            });
        
        int result = qapplication->exec();

        if(result != 0){
            returnCode = result;
        }
    }

    if(returnCode == 0 && messageView->hasErrorMessages()){
        returnCode = 1;
    }

    UnifiedEditHistory::instance()->terminateRecording();
    RootItem::instance()->clearChildren();
    
    pluginManager->finalizePlugins();
    delete ext;
    delete mainWindow;
    mainWindow = nullptr;
    
    return returnCode;
}


App::ErrorCode App::error() const
{
    return impl->error;
}


const std::string& App::errorMessage() const
{
    return impl->errorMessage;
}


bool App::isDoingInitialization()
{
    return isDoingInitialization_;
}


ExtensionManager* App::baseModule()
{
    return instance_->impl->ext;
}


bool App::Impl::eventFilter(QObject* watched, QEvent* event)
{
    if(watched == mainWindow && event->type() == QEvent::Close){
        if(ctrl_c_pressed || exitRequested || ProjectManager::instance()->tryToCloseProject()){
            onMainWindowCloseEvent();
            event->accept();
        } else {
            event->ignore();
        }
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
    if(v.count("no-window")){
        isNoWindowMode = true;
        enableMessageViewRedirectToStdOut();
    }
    if(v.count("path-variable")){
        auto fpvp = FilePathVariableProcessor::systemInstance();
        static std::regex re("^([a-zA-Z][a-zA-Z_0-9]*)=([^;-?[\\]^'{-~]+)$");
        std::smatch match;
        for(auto& var : v["path-variable"].as<vector<string>>()){
            if(regex_match(var, match, re)){
                string name = match.str(1);
                string path = match.str(2);
                fpvp->addUserVariable(name, path);
            }
        }
    }
    if(v.count("quit")){
        doQuit = true;
    } else if(v.count("test-mode")){
        isTestMode = true;
        enableMessageViewRedirectToStdOut();
    } else if(v.count("list-qt-styles")){
        cout << QStyleFactory::keys().join(" ").toStdString() << endl;
        doQuit = true;
    }
}
    

void App::exit(int returnCode)
{
    if(instance_){
        auto impl = instance_->impl;
        impl->returnCode = returnCode;
        exitRequested = true;
        if(impl->mainWindow){
            if(!impl->mainWindow->close()){
                impl->qapplication->exit(returnCode);
            }
        }
    }
}


void App::Impl::enableMessageViewRedirectToStdOut()
{
    static bool isInitialized = false;

    if(!isInitialized){
        auto mv = instance_->impl->messageView;
        cout << fromUTF8(mv->messages());
        cout.flush();
        mv->sigMessage().connect(
            [this](const std::string& text){
                std::cout << fromUTF8(text);
                std::cout.flush();
            });
        isInitialized = true;
    }
}


void App::checkErrorAndExitIfTestMode()
{
    if(isTestMode){
        auto impl = instance_->impl;
        if(impl->messageView->hasErrorMessages()){
            App::updateGui();
            exit(1);
        }
    }
}


SignalProxy<void()> App::sigExecutionStarted()
{
    return sigExecutionStarted_;
}


SignalProxy<void(QKeyEvent* event)> AppUtil::sigKeyPressed()
{
    //! \todo Support top windows other than the main window
    return instance_->impl->mainWindow->sigKeyPressed();
}


SignalProxy<void(QKeyEvent* event)> AppUtil::sigKeyReleased()
{
    //! \todo Support top windows other than the main window
    return instance_->impl->mainWindow->sigKeyReleased();
}


SignalProxy<void()> App::sigAboutToQuit()
{
    return sigAboutToQuit_;
}


SignalProxy<void()> AppUtil::sigAboutToQuit()
{
    return sigAboutToQuit_;
}


SignalProxy<void()> cnoid::sigAboutToQuit()
{
    return sigAboutToQuit_;
}


void App::updateGui()
{
    QCoreApplication::processEvents(
        QEventLoop::ExcludeUserInputEvents | QEventLoop::ExcludeSocketNotifiers, 1.0);
}


void AppUtil::updateGui()
{
    return App::updateGui();
}


bool AppUtil::isNoWindowMode()
{
    return ::isNoWindowMode;
}


void cnoid::updateGui()
{
    return App::updateGui();
}
