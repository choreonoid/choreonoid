/**
   \file
   \author Shin'ichiro Nakaoka
*/

#include "PythonScriptItem.h"
#include "PythonScriptItemImpl.h"
#include <cnoid/ItemManager>
#include <cnoid/PutPropertyFunction>
#include <cnoid/Archive>
#include <cnoid/LazyCaller>
#include "gettext.h"

using namespace std;
using namespace cnoid;


void PythonScriptItem::initializeClass(ExtensionManager* ext)
{
    ext->itemManager().registerClass<PythonScriptItem, ScriptItem>(N_("PythonScriptItem"));
    ext->itemManager().addLoader<PythonScriptItem>(
        _("Python Script"), "PYTHON-SCRIPT", "py",
        [](PythonScriptItem* item, const std::string& filename, std::ostream&, Item*){
            return item->setScriptFilename(filename); });
}


PythonScriptItem::PythonScriptItem()
{
    impl = new PythonScriptItemImpl(this);
    doExecuteOnProjectLoading = false;
}


PythonScriptItem::PythonScriptItem(const PythonScriptItem& org)
    : ScriptItem(org)
{
    impl = new PythonScriptItemImpl(this, *org.impl);
    doExecuteOnProjectLoading = org.doExecuteOnProjectLoading;
}


PythonScriptItem::~PythonScriptItem()
{
    delete impl;
}


void PythonScriptItem::onDisconnectedFromRoot()
{
    impl->onDisconnectedFromRoot();
}


bool PythonScriptItem::setScriptFilename(const std::string& filename)
{
    return impl->setScriptFilename(filename);
}


const std::string& PythonScriptItem::scriptFilename() const
{
    return impl->scriptFilename();
}


void PythonScriptItem::setBackgroundMode(bool on)
{
    impl->setBackgroundMode(on);
}


bool PythonScriptItem::isBackgroundMode() const
{
    return impl->isBackgroundMode();
}


bool PythonScriptItem::isRunning() const
{
    return impl->isRunning();
}


bool PythonScriptItem::execute()
{
    return impl->execute();
}


bool PythonScriptItem::executeCode(const char* code)
{
    return impl->executeCode(code);
}


bool PythonScriptItem::waitToFinish(double timeout)
{
    return impl->waitToFinish(timeout);
}
    

std::string PythonScriptItem::resultString() const
{
    return impl->exceptionText();
}


SignalProxy<void()> PythonScriptItem::sigScriptFinished()
{
    return impl->sigScriptFinished();
}


bool PythonScriptItem::terminate()
{
    return impl->terminate();
}


Item* PythonScriptItem::doDuplicate() const
{
    return new PythonScriptItem(*this);
}


void PythonScriptItem::doPutProperties(PutPropertyFunction& putProperty)
{
    putProperty(_("Script"), fileName());
    putProperty(_("Execution on project loading"),
                doExecuteOnProjectLoading, changeProperty(doExecuteOnProjectLoading));
    impl->doPutProperties(putProperty);
}


bool PythonScriptItem::store(Archive& archive)
{
    archive.writeFileInformation(this);
    archive.write("exeute_on_project_loading", doExecuteOnProjectLoading);
    return impl->store(archive);    
}


bool PythonScriptItem::restore(const Archive& archive)
{
    if(impl->restore(archive) && archive.loadFileTo(this)){
        archive.read({ "exeute_on_project_loading", "executionOnLoading" }, doExecuteOnProjectLoading);
        if(doExecuteOnProjectLoading){
            archive.addFinalProcess([this](){ execute(); });
        }
        return true;
    }
    return false;
}
