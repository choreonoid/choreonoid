#include "CorbaPlugin.h"
#include "MessageView_impl.h"
#include "NameServerView.h"
#include <cnoid/Plugin>
#include <cnoid/MessageView>
#include <cnoid/CorbaUtil>
#include <cnoid/Sleep>
#include <cnoid/AppConfig>
#include <cnoid/MenuManager>
#include <cnoid/Process>
#include <cnoid/ExecutablePath>
#include <cnoid/UTF8>
#include <cnoid/ValueTree>
#include <cnoid/stdx/filesystem>
#include <QTcpSocket>
#include <fmt/format.h>
#include <thread>
#include <iostream>
#include "gettext.h"

using namespace std;
using namespace cnoid;
using fmt::format;
namespace filesystem = cnoid::stdx::filesystem;

namespace {
bool commonInitializationDone;

#ifdef Q_OS_WIN32
const char* nameServerCommand = "cnoid-nameserver.exe";
bool useChoreonoidNameServerIfNecessary = true;
#else
const char* nameServerCommand = "cnoid-nameserver";
# ifdef Q_OS_DARWIN
bool useChoreonoidNameServerIfNecessary = true;
# else
bool useChoreonoidNameServerIfNecessary = false;
# endif
#endif

Process nameServerProcess;

}

namespace cnoid {

void checkOrInvokeCorbaNameServer()
{
    MessageView* mv = MessageView::instance();

    const Mapping& conf = *AppConfig::archive()->findMapping("CORBA");
    if(conf.isValid()){
        conf.read("useChoreonoidNameServerIfNecessary", useChoreonoidNameServerIfNecessary);
    }

    if(useChoreonoidNameServerIfNecessary){
        // Check if the CORBA iiop port needed for getting the existing NameServer can be accessed
        QTcpSocket socket;
        socket.connectToHost("localhost", 2809);
        if(socket.waitForConnected(50)){
            mv->putln(_("An external CORBA name server has been detected."));
        } else {
            mv->putln(_("No external CORBA name server was detected."));

            auto serverExecPath = executableTopDirPath() / "bin" / nameServerCommand;

            if(!filesystem::exists(serverExecPath)){
                mv->putln(format(_("Namer server {} is not found."), nameServerCommand));
                    
            } else {
                string command = serverExecPath.make_preferred().string();
#ifdef _WIN32
                nameServerProcess.start(QString("\"") + command.c_str() + "\"");
#else
                nameServerProcess.start(command.c_str(), QStringList());
#endif
                if(nameServerProcess.waitForStarted() && nameServerProcess.waitForReadyRead()){
                    mv->putln(format(_("Name server process {} has been invoked."), nameServerCommand));
                } else {
                    mv->putln(format(_("Name server \"{}\" cannot be invoked."), toUTF8(command)));
                }
            }
        }
    }
}

}

namespace {
    
bool initializeCorbaUtilAndNameServer(CORBA::ORB_ptr orb)
{
    if(orb){
        initializeCorbaUtil(orb, true);
                
    } else {
        checkOrInvokeCorbaNameServer();
        initializeCorbaUtil(true);
    }

    commonInitializationDone = true;
            
    return true;
}


class CorbaPlugin : public Plugin
{
    QAction* useChoreonoidNameServerIfNecessaryCheck;
    MessageView_impl* messageView;
    std::thread orbMainLoopThread;

public:
    CorbaPlugin() : Plugin("Corba") {
        commonInitializationDone = false;
    }
        
    virtual bool initialize() {

        bool doSetupCorbaMainLoop = !commonInitializationDone;

        if(!commonInitializationDone){
            initializeCorbaUtilAndNameServer(0);
        }

        MenuManager& mm = menuManager();
        mm.setPath("/Options").setPath("CORBA");
        useChoreonoidNameServerIfNecessaryCheck = mm.addCheckItem(_("Use Choreonoid's name server when no server is found"));
        useChoreonoidNameServerIfNecessaryCheck->setChecked(useChoreonoidNameServerIfNecessary);

        messageView = new MessageView_impl(getORB());

        NamingContextHelper* nc = getDefaultNamingContextHelper();
        if(!nc->bindObject(messageView->_this(), "MessageView")){
            MessageView::instance()->putln(nc->errorMessage());
        }

        NameServerView::initializeClass(this);

        if(doSetupCorbaMainLoop){
            orbMainLoopThread = std::thread(std::bind(&CorbaPlugin::orbMainLoop, this));
        }

        return true;
    }

    void orbMainLoop() {
        // setMainTread();
        getORB()->run();
        getORB()->destroy();
    }
        
    virtual bool finalize() {

        Mapping& conf = *AppConfig::archive()->openMapping("CORBA");
        conf.write("useChoreonoidNameServerIfNecessary", useChoreonoidNameServerIfNecessaryCheck->isChecked());

        if(orbMainLoopThread.joinable()){
            getORB()->shutdown(false);
            orbMainLoopThread.join();
        }

        if(nameServerProcess.state() != QProcess::NotRunning){
            nameServerProcess.kill();
            nameServerProcess.waitForFinished(100);
        }
            
        return true;
    }
};

}

CNOID_IMPLEMENT_PLUGIN_ENTRY(CorbaPlugin);


namespace cnoid {

bool takeOverCorbaPluginInitialization(CORBA::ORB_ptr orb)
{
    return initializeCorbaUtilAndNameServer(orb);
}

}
