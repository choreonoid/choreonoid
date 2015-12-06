/*!
  @file
  @author Shin'ichiro Nakaoka
*/

#include "SubSimulatorItem.h"
#include <cnoid/Archive>
#include <boost/bind.hpp>
#include "gettext.h"

using namespace cnoid;

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
    putProperty(_("Enabled"), isEnabled(),
                boost::bind(&SubSimulatorItem::setEnabled, this, _1));
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
