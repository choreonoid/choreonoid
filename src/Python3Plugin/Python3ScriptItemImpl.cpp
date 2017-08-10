/**
   \file
   \author Shin'ichiro Nakaoka
*/

#include "Python3ScriptItemImpl.h"
#include <cnoid/Archive>
#include <cnoid/FileUtil>
#include "gettext.h"

using namespace std;
namespace stdph = std::placeholders;
using namespace cnoid;
using boost::format;
namespace filesystem = boost::filesystem;
namespace py = pybind11;


Python3ScriptItemImpl::Python3ScriptItemImpl(ScriptItem* scriptItem)
    : scriptItem_(scriptItem),
      mv(MessageView::instance())
{
    executor.setBackgroundMode(false);
}


Python3ScriptItemImpl::Python3ScriptItemImpl(ScriptItem* scriptItem, const Python3ScriptItemImpl& org)
    : scriptItem_(scriptItem),
      scriptFilename_(org.scriptFilename_),
      mv(MessageView::instance()),
      executor(org.executor)
{
    
}


Python3ScriptItemImpl::~Python3ScriptItemImpl()
{

}


void Python3ScriptItemImpl::onDisconnectedFromRoot()
{
    terminate();
}


bool Python3ScriptItemImpl::setScriptFilename(const std::string& filename)
{
    filesystem::path scriptPath(filename);
    if(filesystem::exists(scriptPath)){
        scriptFilename_ = filename;
        if(scriptItem()->name().empty()){
            scriptItem()->setName(getFilename(filesystem::path(filename)));
        }
        return true;
    } else {
        mv->putln(format(_("Python script file \"%1%\" cannot be loaded. The file does not exist.")) % filename);
        return false;
    }
}


void Python3ScriptItemImpl::setBackgroundMode(bool on)
{
    if(on != executor.isBackgroundMode()){
        executor.setBackgroundMode(on);
        scriptItem_->notifyUpdate();
    }
}


bool Python3ScriptItemImpl::isBackgroundMode() const
{
    return executor.isBackgroundMode();
}


bool Python3ScriptItemImpl::isRunning() const
{
    return (executor.state() != Python3Executor::NOT_RUNNING);
}


bool Python3ScriptItemImpl::execute()
{
    const string iname = scriptItem()->identityName();

    Python3Executor::State state = executor.state();
    if(state == Python3Executor::RUNNING_FOREGROUND){
        showWarningDialog(
            format(_("Python script \"%1%\" is now running in the foreground. "
                     "The execution of the script cannot be overlapped.")) % iname);
        return false;
    } else if(state == Python3Executor::RUNNING_BACKGROUND){
        bool doRestart = showConfirmDialog(
            _("Python Script Termination"),
            str(format(_("Python script \"%1%\" is running now. "
                         "Do you want to terminate and restart it?")) % iname));
        if(!doRestart){
            return false;
        } else if(!terminate()){
            showWarningDialog(_("The python script cannot be terminated. It cannot be restarted."));
            return false;
        }
    }
    
    bool result = false;

    if(scriptFilename_.empty()){
        mv->putln(format(_(" Python script \"%1%\" is empty.")) % iname);

    } else {
        filesystem::path scriptPath(scriptFilename_);

        if(!filesystem::exists(scriptPath)){
            mv->putln(format(_("The file of Python script \"%1%\" does not exist.")) % iname);

        } else {
            mv->putln(format(_("Execution of Python script \"%1%\" has been started.")) % iname);

            sigFinishedConnection.disconnect();
            sigFinishedConnection =
                executor.sigFinished().connect(
                    std::bind(&Python3ScriptItemImpl::onScriptFinished, this));
            
            result = executor.execFile(scriptFilename_);
        }
    }
    return result;
}


bool Python3ScriptItemImpl::executeCode(const char* code)
{
    if(executor.state() != Python3Executor::NOT_RUNNING){
        mv->putln(
            format(_("Python script \"%1%\" is now running in the foreground. "
                     "The code cannot be executed now."))
            % scriptItem()->identityName());
        return false;
    }
    return executor.execCode(code);
}


bool Python3ScriptItemImpl::waitToFinish(double timeout)
{
    return executor.waitToFinish(timeout);
}


py::object Python3ScriptItemImpl::resultObject()
{
    return executor.resultObject();
}


const std::string Python3ScriptItemImpl::resultString() const
{
    return executor.resultString();
}


void Python3ScriptItemImpl::onScriptFinished()
{
    sigFinishedConnection.disconnect();
    
    const string iname = scriptItem()->identityName();

    if(executor.isTerminated()){
        mv->putln(format(_("The execution of Python script \"%1%\" has been terminated.")) % iname);
    } else if(executor.hasException()){
        mv->putln(format(_("The execution of Python script \"%1%\" failed.\n%2%"))
                  % iname % executor.exceptionText());
    } else {
        if(!executor.resultObject().is_none()){
            mv->putln(executor.resultString());
        }
        mv->putln(format(_("The execution of Python script \"%1%\" has been finished.")) % iname);
    }
    
    sigScriptFinished_();
}


bool Python3ScriptItemImpl::terminate()
{
    bool result = true;
    const string iname = scriptItem()->identityName();
    
    if(executor.state() == Python3Executor::RUNNING_BACKGROUND){
        if(executor.terminate()){
            mv->putln(format(_("Python script \"%1%\" has been terminated.")) % iname);
        } else {
            mv->putln(format(_("Python script \"%1%\" cannot be terminated. "
                               "Some internal errors may happen.")) % iname);
            result = false;
        }
    }
    return result;
}


void Python3ScriptItemImpl::doPutProperties(PutPropertyFunction& putProperty)
{
    putProperty(_("Background execution"), executor.isBackgroundMode(),
                std::bind(&Python3ScriptItemImpl::onBackgroundModeChanged, this, stdph::_1));
}


bool Python3ScriptItemImpl::onBackgroundModeChanged(bool on)
{
    executor.setBackgroundMode(on);
    return true;
}


bool Python3ScriptItemImpl::store(Archive& archive)
{
    archive.write("backgroundExecution", executor.isBackgroundMode());
    return true;
}


bool Python3ScriptItemImpl::restore(const Archive& archive)
{
    bool isBackgroundMode;
    if(archive.read("backgroundExecution", isBackgroundMode)){
        executor.setBackgroundMode(isBackgroundMode);
    }
    return true;
}
