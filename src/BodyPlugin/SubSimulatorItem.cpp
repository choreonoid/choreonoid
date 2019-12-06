/*!
  @file
  @author Shin'ichiro Nakaoka
*/

#include "SubSimulatorItem.h"
#include <cnoid/ItemManager>
#include <cnoid/PutPropertyFunction>
#include <cnoid/Archive>
#include "gettext.h"

using namespace cnoid;


void SubSimulatorItem::initializeClass(ExtensionManager* ext)
{
    ext->itemManager().registerAbstractClass<SubSimulatorItem>();
}


SubSimulatorItem::SubSimulatorItem()
{
    isEnabled_ = true;
}


SubSimulatorItem::SubSimulatorItem(const SubSimulatorItem& org)
    : Item(org)
{
    isEnabled_ = org.isEnabled_;
}


bool SubSimulatorItem::isEnabled()
{
    return true;
}


bool SubSimulatorItem::setEnabled(bool on)
{
    isEnabled_ = on;
    return isEnabled_;
}
   

bool SubSimulatorItem::initializeSimulation(SimulatorItem* simulatorItem)
{
    return true;
}


void SubSimulatorItem::finalizeSimulation()
{

}


void SubSimulatorItem::doPutProperties(PutPropertyFunction& putProperty)
{
    putProperty(_("Enabled"), isEnabled(), [&](bool on){ return setEnabled(on); });
}


bool SubSimulatorItem::store(Archive& archive)
{
    archive.write("enabled", isEnabled());
    return true;
}


bool SubSimulatorItem::restore(const Archive& archive)
{
    bool on;
    if(archive.read("enabled", on)){
        setEnabled(on);
    }
    return true;
}
