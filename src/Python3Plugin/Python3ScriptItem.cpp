/**
   \file
   \author Shin'ichiro Nakaoka
*/

#include "Python3ScriptItem.h"
#include "Python3ScriptItemImpl.h"
#include <cnoid/ItemManager>
#include <cnoid/Archive>
#include <cnoid/FileUtil>
#include <cnoid/LazyCaller>
#include "gettext.h"

using namespace std;
namespace stdph = std::placeholders;
using namespace cnoid;


void Python3ScriptItem::initializeClass(ExtensionManager* ext)
{
    ext->itemManager().registerClass<Python3ScriptItem>(N_("Python3ScriptItem"));
    ext->itemManager().addLoader<Python3ScriptItem>(
        _("Python3 Script"), "PYTHON-SCRIPT-FILE", "py",
        std::bind(&Python3ScriptItem::setScriptFilename, stdph::_1, stdph::_2));
}


Python3ScriptItem::Python3ScriptItem()
{
    impl = new Python3ScriptItemImpl(this);
    doExecutionOnLoading = false;
}


Python3ScriptItem::Python3ScriptItem(const Python3ScriptItem& org)
    : ScriptItem(org)
{
    impl = new Python3ScriptItemImpl(this, *org.impl);
    doExecutionOnLoading = org.doExecutionOnLoading;
}


Python3ScriptItem::~Python3ScriptItem()
{
    delete impl;
}


void Python3ScriptItem::onDisconnectedFromRoot()
{
    impl->onDisconnectedFromRoot();
}


bool Python3ScriptItem::setScriptFilename(const std::string& filename)
{
    bool result = impl->setScriptFilename(filename);
    if(result && doExecutionOnLoading){
        callLater(std::bind(&Python3ScriptItem::execute, this), LazyCaller::PRIORITY_LOW);
    }
    return result;
}


const std::string& Python3ScriptItem::scriptFilename() const
{
    return impl->scriptFilename();
}


void Python3ScriptItem::setBackgroundMode(bool on)
{
    impl->setBackgroundMode(on);
}


bool Python3ScriptItem::isBackgroundMode() const
{
    return impl->isBackgroundMode();
}


bool Python3ScriptItem::isRunning() const
{
    return impl->isRunning();
}


bool Python3ScriptItem::execute()
{
    return impl->execute();
}


bool Python3ScriptItem::executeCode(const char* code)
{
    return impl->executeCode(code);
}


bool Python3ScriptItem::waitToFinish(double timeout)
{
    return impl->waitToFinish(timeout);
}
    

std::string Python3ScriptItem::resultString() const
{
    return impl->resultString();
}


SignalProxy<void()> Python3ScriptItem::sigScriptFinished()
{
    return impl->sigScriptFinished();
}


bool Python3ScriptItem::terminate()
{
    return impl->terminate();
}


Item* Python3ScriptItem::doDuplicate() const
{
    return new Python3ScriptItem(*this);
}


void Python3ScriptItem::doPutProperties(PutPropertyFunction& putProperty)
{
    putProperty(_("Script"), getFilename(filePath()));
    impl->doPutProperties(putProperty);
    putProperty(_("Execution on loading"), doExecutionOnLoading, changeProperty(doExecutionOnLoading));
}


bool Python3ScriptItem::store(Archive& archive)
{
    if(!filePath().empty()){
        archive.writeRelocatablePath("file", filePath());
    }
    archive.write("executionOnLoading", doExecutionOnLoading);
    return impl->store(archive);
}


bool Python3ScriptItem::restore(const Archive& archive)
{
    archive.read("executionOnLoading", doExecutionOnLoading);
    impl->restore(archive);
    string filename;
    if(archive.readRelocatablePath("file", filename)){
        bool doExecution = doExecutionOnLoading;
        doExecutionOnLoading = false; // disable execution in the load function
        bool loaded = load(filename);
        doExecutionOnLoading = doExecution;
        if(loaded && doExecution){
            archive.addPostProcess(std::bind(&Python3ScriptItem::execute, this));
        }
        return loaded;
    }
    return true;
}
