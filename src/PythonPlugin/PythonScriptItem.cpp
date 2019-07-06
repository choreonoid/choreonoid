/**
   \file
   \author Shin'ichiro Nakaoka
*/

#include "PythonScriptItem.h"
#include "PythonScriptItemImpl.h"
#include <cnoid/ItemManager>
#include <cnoid/Archive>
#include <cnoid/FileUtil>
#include <cnoid/LazyCaller>
#include "gettext.h"

using namespace std;
using namespace cnoid;


void PythonScriptItem::initializeClass(ExtensionManager* ext)
{
    ext->itemManager().registerClass<PythonScriptItem>(N_("PythonScriptItem"));
    ext->itemManager().addLoader<PythonScriptItem>(
        _("Python Script"), "PYTHON-SCRIPT-FILE", "py",
        [](PythonScriptItem* item, const std::string& filename, std::ostream&, Item*){
            return item->setScriptFilename(filename); });
}


PythonScriptItem::PythonScriptItem()
{
    impl = new PythonScriptItemImpl(this);
    doExecutionOnLoading = false;
}


PythonScriptItem::PythonScriptItem(const PythonScriptItem& org)
    : ScriptItem(org)
{
    impl = new PythonScriptItemImpl(this, *org.impl);
    doExecutionOnLoading = org.doExecutionOnLoading;
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
    bool result = impl->setScriptFilename(filename);
    if(result && doExecutionOnLoading){
        callLater(std::bind(&PythonScriptItem::execute, this), LazyCaller::PRIORITY_LOW);
    }
    return result;
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
    return impl->resultString();
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
    putProperty(_("Script"), getFilename(filePath()));
    impl->doPutProperties(putProperty);
    putProperty(_("Execution on loading"), doExecutionOnLoading, changeProperty(doExecutionOnLoading));
}


bool PythonScriptItem::store(Archive& archive)
{
    if(!filePath().empty()){
        archive.writeRelocatablePath("file", filePath());
    }
    archive.write("executionOnLoading", doExecutionOnLoading);
    return impl->store(archive);
}


bool PythonScriptItem::restore(const Archive& archive)
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
            archive.addPostProcess(std::bind(&PythonScriptItem::execute, this));
        }
        return loaded;
    }
    return true;
}
