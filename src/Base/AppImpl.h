/**
   @author Shin'ichiro Nakaoka
*/

#include "App.h"
#include "ExtensionManager.h"
#include "DescriptionDialog.h"
#include "OptionManager.h"
#include <QApplication>

namespace cnoid {

class MainWindow;

//class AppImpl : public QObject
class AppImpl
{
    //Q_OBJECT

    friend class App;
    friend class View;
        
    AppImpl(App* self, int& argc, char**& argv);
    ~AppImpl();
        
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
        
    void initialize(const char* appName, const char* vendorName, const QIcon& icon, const char* pluginPathList);
    int exec();
    void onMainWindowCloseEvent();
    void onSigOptionsParsed(boost::program_options::variables_map& v);
    bool processCommandLineOptions();
        
    void showInformationDialog();
    void onOpenGLVSyncToggled(bool on);
    static void clearFocusView();
                             
//private Q_SLOTS:
    //  void onFocusChanged(QWidget* old, QWidget* now);
};

}
