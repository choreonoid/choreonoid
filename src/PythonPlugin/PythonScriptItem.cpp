/**
   \file
   \author Shin'ichiro Nakaoka
*/

#include "PythonScriptItem.h"
#include "PythonScriptItemImpl.h"
#include <cnoid/ItemManager>
#include <cnoid/Archive>
#include <cnoid/FileUtil>
#include "gettext.h"

using namespace std;
using namespace boost;
using namespace cnoid;


void PythonScriptItem::initializeClass(ExtensionManager* ext)
{
    ext->itemManager().registerClass<PythonScriptItem>(N_("PythonScriptItem"));
    ext->itemManager().addLoader<PythonScriptItem>(
        _("Python Script"), "PYTHON-SCRIPT-FILE", "py",
        boost::bind(&PythonScriptItem::setScriptFilename, _1, _2));
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
    return impl->setScriptFilename(filename);
}


const std::string& PythonScriptItem::scriptFilename() const
{
    return impl->scriptFilename();
}


bool PythonScriptItem::setBackgroundMode(bool on)
{
    return impl->setBackgroundMode(on);
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


ItemPtr PythonScriptItem::doDuplicate() const
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
    string filename;
    if(archive.readRelocatablePath("file", filename)){
        if(load(filename)){
            if(impl->restore(archive)){
                if(doExecutionOnLoading){
                    archive.addPostProcess(boost::bind(&PythonScriptItem::execute, this));
                }
                return true;
            }
        }
    }
    return false;
}
