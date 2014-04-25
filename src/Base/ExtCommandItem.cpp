/*!
  @file
  @author Shin'ichiro Nakaoka
*/

#include "ExtCommandItem.h"
#include "ItemManager.h"
#include <cnoid/MessageView>
#include <cnoid/Archive>
#include <boost/bind.hpp>
#include <boost/filesystem.hpp>
#include "gettext.h"

using namespace std;
using namespace boost;
using namespace cnoid;


void ExtCommandItem::initializeClass(ExtensionManager* ext)
{
    static bool initialized = false;
    if(!initialized){
        ItemManager& im = ext->itemManager();
        im.registerClass<ExtCommandItem>(N_("ExtCommandItem"));
        im.addCreationPanel<ExtCommandItem>();
        initialized = true;
    }
}


ExtCommandItem::ExtCommandItem()
{
    signalReadyStandardOutputConnected = false;
    doCheckExistingProcess = false;
    doExecuteOnLoading = true;

    process.sigReadyReadStandardOutput().connect(
        bind(&ExtCommandItem::onReadyReadServerProcessOutput, this));
}


ExtCommandItem::ExtCommandItem(const ExtCommandItem& org)
    : Item(org)
{
    command_ = org.command_;
    signalReadyStandardOutputConnected = false;
    doCheckExistingProcess = org.doCheckExistingProcess;
    doExecuteOnLoading = org.doExecuteOnLoading;

    process.sigReadyReadStandardOutput().connect(
        bind(&ExtCommandItem::onReadyReadServerProcessOutput, this));
}


ExtCommandItem::~ExtCommandItem()
{

}


void ExtCommandItem::onDisconnectedFromRoot()
{
    terminate();
}


ItemPtr ExtCommandItem::doDuplicate() const
{
    return new ExtCommandItem(*this);
}


void ExtCommandItem::setCommand(const std::string& command)
{
    terminate();
    command_ = command;
}


bool ExtCommandItem::execute()
{
    bool result = false;
    
    MessageView* mv = MessageView::instance();
    
    if(!command_.empty()){
        terminate();
        string actualCommand(command_);
#ifdef _WIN32
        if(filesystem::path(actualCommand).extension() != ".exe"){
            actualCommand += ".exe";
        }
        // quote the command string to support a path including spaces
        process.start(QString("\"") + actualCommand.c_str() + "\"");
#else
        process.start(actualCommand.c_str());
#endif

        if(process.waitForStarted()){
            mv->putln(fmt(_("External command \"%1%\" has been executed by item \"%2%\"."))
                      % actualCommand % name());
            result = true;

        } else {
            mv->put(fmt(_("External command \"%1%\" cannot be executed.")) % actualCommand);
            if(!filesystem::exists(actualCommand)){
                mv->putln(_(" The command does not exist."));
            } else {
                mv->putln("");
            }
        }
    }
    return result;
}


bool ExtCommandItem::terminate()
{
    if(process.state() != QProcess::NotRunning){
        process.kill();
        return process.waitForFinished(100);
    }
    return false;
}


void ExtCommandItem::onReadyReadServerProcessOutput()
{
    MessageView::instance()->put(QString(process.readAll()));
}


void ExtCommandItem::doPutProperties(PutPropertyFunction& putProperty)
{
    putProperty(_("Command"), command_,
                bind(&ExtCommandItem::setCommand, this, _1), true);
    putProperty(_("Execute on loading"), doExecuteOnLoading,
                changeProperty(doExecuteOnLoading));
}


bool ExtCommandItem::store(Archive& archive)
{
    //archive.writeRelocatablePath("command", command_);
    archive.write("command", command_);
    archive.write("executeOnLoading", doExecuteOnLoading);
    return true;
}


bool ExtCommandItem::restore(const Archive& archive)
{
    //archive.readRelocatablePath("command", command_);
    archive.read("command", command_);

    if(archive.read("executeOnLoading", doExecuteOnLoading)){
        if(doExecuteOnLoading){
            execute();
        }
    }
    return true;
}
