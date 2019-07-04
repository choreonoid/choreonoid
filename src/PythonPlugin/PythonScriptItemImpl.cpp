/**
   \file
   \author Shin'ichiro Nakaoka
*/

#include "PythonScriptItemImpl.h"
#include <cnoid/Archive>
#include <cnoid/FileUtil>
#include <fmt/format.h>
#include "gettext.h"

using namespace std;
namespace stdph = std::placeholders;
using namespace cnoid;
using fmt::format;
namespace filesystem = cnoid::stdx::filesystem;


PythonScriptItemImpl::PythonScriptItemImpl(ScriptItem* scriptItem)
    : scriptItem_(scriptItem),
      mv(MessageView::instance())
{
    executor.setBackgroundMode(false);
}


PythonScriptItemImpl::PythonScriptItemImpl(ScriptItem* scriptItem, const PythonScriptItemImpl& org)
    : scriptItem_(scriptItem),
      scriptFilename_(org.scriptFilename_),
      mv(MessageView::instance()),
      executor(org.executor)
{
    
}


PythonScriptItemImpl::~PythonScriptItemImpl()
{

}


void PythonScriptItemImpl::onDisconnectedFromRoot()
{
    terminate();
}


bool PythonScriptItemImpl::setScriptFilename(const std::string& filename)
{
    filesystem::path scriptPath(filename);
    if(filesystem::exists(scriptPath)){
        scriptFilename_ = filename;
        if(scriptItem()->name().empty()){
            scriptItem()->setName(getFilename(filesystem::path(filename)));
        }
        return true;
    } else {
        mv->putln(format(_("Python script file \"{}\" cannot be loaded. The file does not exist."), filename));
        return false;
    }
}


void PythonScriptItemImpl::setBackgroundMode(bool on)
{
    if(on != executor.isBackgroundMode()){
        executor.setBackgroundMode(on);
        scriptItem_->notifyUpdate();
    }
}


bool PythonScriptItemImpl::isBackgroundMode() const
{
    return executor.isBackgroundMode();
}


bool PythonScriptItemImpl::isRunning() const
{
    return (executor.state() != PythonExecutor::NOT_RUNNING);
}


bool PythonScriptItemImpl::execute()
{
    const string iname = scriptItem()->identityName();

    PythonExecutor::State state = executor.state();
    if(state == PythonExecutor::RUNNING_FOREGROUND){
        showWarningDialog(
            format(_("Python script \"{}\" is now running in the foreground. "
                     "The execution of the script cannot be overlapped."), iname));
        return false;
    } else if(state == PythonExecutor::RUNNING_BACKGROUND){
        bool doRestart = showConfirmDialog(
            _("Python Script Termination"),
            format(_("Python script \"{}\" is running now. "
                     "Do you want to terminate and restart it?"), iname));
        if(!doRestart){
            return false;
        } else if(!terminate()){
            showWarningDialog(_("The python script cannot be terminated. It cannot be restarted."));
            return false;
        }
    }
    
    bool result = false;

    if(scriptFilename_.empty()){
        mv->putln(format(_(" Python script \"{}\" is empty."), iname));

    } else {
        filesystem::path scriptPath(scriptFilename_);

        if(!filesystem::exists(scriptPath)){
            mv->putln(format(_("The file of Python script \"{}\" does not exist."), iname));

        } else {
            mv->putln(format(_("Execution of Python script \"{}\" has been started."), iname));

            sigFinishedConnection.disconnect();
            sigFinishedConnection =
                executor.sigFinished().connect(
                    std::bind(&PythonScriptItemImpl::onScriptFinished, this));
            
            result = executor.execFile(scriptFilename_);
        }
    }
    return result;
}


bool PythonScriptItemImpl::executeCode(const char* code)
{
    if(executor.state() != PythonExecutor::NOT_RUNNING){
        mv->putln(
            format(_("Python script \"{}\" is now running in the foreground. "
                     "The code cannot be executed now."),
                     scriptItem()->identityName()));
        return false;
    }
    return executor.execCode(code);
}


bool PythonScriptItemImpl::waitToFinish(double timeout)
{
    return executor.waitToFinish(timeout);
}


python::object PythonScriptItemImpl::resultObject()
{
    return executor.resultObject();
}


const std::string PythonScriptItemImpl::resultString() const
{
    return executor.resultString();
}


void PythonScriptItemImpl::onScriptFinished()
{
    sigFinishedConnection.disconnect();
    
    const string iname = scriptItem()->identityName();

    if(executor.isTerminated()){
        mv->putln(format(_("The execution of Python script \"{}\" has been terminated."), iname));
    } else if(executor.hasException()){
        mv->putln(format(_("The execution of Python script \"{0}\" failed.\n{1}"),
                iname, executor.exceptionText()));
    } else {
        if(!executor.resultObject().is_none()){
            mv->putln(executor.resultString());
        }
        mv->putln(format(_("The execution of Python script \"{}\" has been finished."), iname));
    }
    
    sigScriptFinished_();
}


bool PythonScriptItemImpl::terminate()
{
    bool result = true;
    const string iname = scriptItem()->identityName();
    
    if(executor.state() == PythonExecutor::RUNNING_BACKGROUND){
        if(executor.terminate()){
            mv->putln(format(_("Python script \"{}\" has been terminated."), iname));
        } else {
            mv->putln(format(_("Python script \"{}\" cannot be terminated. "
                               "Some internal errors may happen."), iname));
            result = false;
        }
    }
    return result;
}


void PythonScriptItemImpl::doPutProperties(PutPropertyFunction& putProperty)
{
    putProperty(_("Background execution"), executor.isBackgroundMode(),
                std::bind(&PythonScriptItemImpl::onBackgroundModeChanged, this, stdph::_1));
}


bool PythonScriptItemImpl::onBackgroundModeChanged(bool on)
{
    executor.setBackgroundMode(on);
    return true;
}


bool PythonScriptItemImpl::store(Archive& archive)
{
    archive.write("backgroundExecution", executor.isBackgroundMode());
    return true;
}


bool PythonScriptItemImpl::restore(const Archive& archive)
{
    bool isBackgroundMode;
    if(archive.read("backgroundExecution", isBackgroundMode)){
        executor.setBackgroundMode(isBackgroundMode);
    }
    return true;
}
