/*!
  @file
  @author Shin'ichiro Nakaoka
*/

#include "ControllerItem.h"
#include <cnoid/ItemManager>
#include <cnoid/PutPropertyFunction>
#include <cnoid/Archive>
#include "gettext.h"

using namespace std;
using namespace cnoid;


void ControllerItem::initializeClass(ExtensionManager* ext)
{
    ext->itemManager().registerAbstractClass<ControllerItem>();
}


ControllerItem::ControllerItem()
{
    isNoDelayMode_ = false;
}


ControllerItem::ControllerItem(const ControllerItem& org)
    : Item(org),
      optionString_(org.optionString_)
{
    isNoDelayMode_ = org.isNoDelayMode_;
}


ControllerItem::~ControllerItem()
{

}


bool ControllerItem::isActive() const
{
    return simulatorItem_ ? simulatorItem_->isRunning() : false;
}


bool ControllerItem::isNoDelayMode() const
{
    return isNoDelayMode_;
}


bool ControllerItem::setNoDelayMode(bool on)
{
    isNoDelayMode_ = on;
    return on;
}


const std::string& ControllerItem::optionString() const
{
    return optionString_;
}


void ControllerItem::setSimulatorItem(SimulatorItem* item)
{
    simulatorItem_ = item;
}


bool ControllerItem::initialize(ControllerIO* io)
{
    return true;
}


bool ControllerItem::start()
{
    return true;
}


void ControllerItem::stop()
{

}


void ControllerItem::onOptionsChanged()
{

}


void ControllerItem::doPutProperties(PutPropertyFunction& putProperty)
{
    putProperty(_("No delay mode"), isNoDelayMode_, changeProperty(isNoDelayMode_));

    putProperty(_("Controller options"), optionString_,
                [&](const string& options){
                    optionString_ = options;
                    onOptionsChanged();
                    return true;
                });
}


bool ControllerItem::store(Archive& archive)
{
    archive.write("isNoDelayMode", isNoDelayMode_);
    archive.write("controllerOptions", optionString_, DOUBLE_QUOTED);
    return true;
}


bool ControllerItem::restore(const Archive& archive)
{
    if(!archive.read("isNoDelayMode", isNoDelayMode_)){
        // For the backward compatibility
        archive.read("isImmediateMode", isNoDelayMode_); 
    }
    archive.read("controllerOptions", optionString_);
    return true;
}

#ifdef ENABLE_SIMULATION_PROFILING
void ControllerItem::getProfilingNames(vector<string>& profilingNames)
{

}


void ControllerItem::getProfilingTimes(vector<double>& profilingToimes)
{

}
#endif
