/**
   \file
   \author Shin'ichiro Nakaoka
*/

#include "PythonSimScriptItem.h"
#include <cnoid/PythonScriptItemImpl>
#include <cnoid/ItemManager>
#include <cnoid/Archive>
#include "gettext.h"

using namespace std;
using namespace cnoid;


void PythonSimScriptItem::initialize(ExtensionManager* ext)
{
    ext->itemManager().registerClass<PythonSimScriptItem, SimulationScriptItem>(N_("PythonSimScriptItem"));
    ext->itemManager().addLoader<PythonSimScriptItem>(
        _("Python Script for Simulation"), "PYTHON-SCRIPT-FILE", "py",
        [](PythonSimScriptItem* item, const std::string& filename, std::ostream&, Item* /* parentItem */){
            return item->setScriptFilename(filename);
        });
}


PythonSimScriptItem::PythonSimScriptItem()
{
    impl = new PythonScriptItemImpl(this);
}


PythonSimScriptItem::PythonSimScriptItem(const PythonSimScriptItem& org)
    : SimulationScriptItem(org)
{
    impl = new PythonScriptItemImpl(this, *org.impl);
}


PythonSimScriptItem::~PythonSimScriptItem()
{
    delete impl;
}


void PythonSimScriptItem::onDisconnectedFromRoot()
{
    impl->onDisconnectedFromRoot();
}


bool PythonSimScriptItem::setScriptFilename(const std::string& filename)
{
    return impl->setScriptFilename(filename);
}


const std::string& PythonSimScriptItem::scriptFilename() const
{
    return impl->scriptFilename();
}


void PythonSimScriptItem::setBackgroundMode(bool on)
{
    impl->setBackgroundMode(on);
}


bool PythonSimScriptItem::isBackgroundMode() const
{
    return impl->isBackgroundMode();
}


bool PythonSimScriptItem::isRunning() const
{
    return impl->isRunning();
}


bool PythonSimScriptItem::executeAsSimulationScript()
{
    return impl->execute();
}


bool PythonSimScriptItem::executeCode(const char* code)
{
    return impl->executeCode(code);
}


bool PythonSimScriptItem::waitToFinish(double timeout)
{
    return impl->waitToFinish(timeout);
}
    

std::string PythonSimScriptItem::resultString() const
{
    return impl->exceptionText();
}


SignalProxy<void()> PythonSimScriptItem::sigScriptFinished()
{
    return impl->sigScriptFinished();
}


bool PythonSimScriptItem::terminate()
{
    return impl->terminate();
}


Item* PythonSimScriptItem::doDuplicate() const
{
    return new PythonSimScriptItem(*this);
}


void PythonSimScriptItem::doPutProperties(PutPropertyFunction& putProperty)
{
    impl->doPutProperties(putProperty);
    SimulationScriptItem::doPutProperties(putProperty);
}


bool PythonSimScriptItem::store(Archive& archive)
{
    if(SimulationScriptItem::store(archive) && impl->store(archive)){
        return archive.writeFileInformation(this);
    }
    return false;
}


bool PythonSimScriptItem::restore(const Archive& archive)
{
    if(SimulationScriptItem::restore(archive)){
        if(archive.loadFileTo(this)){
            return impl->restore(archive);
        }
    }
    return false;
}
