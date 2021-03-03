/*!
  @file
  @author Shin'ichiro Nakaoka
*/

#include "ControllerItem.h"
#include "ControllerLogItem.h"
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


void ControllerItem::setNoDelayMode(bool on)
{
    isNoDelayMode_ = on;
}


void ControllerItem::setSimulatorItem(SimulatorItem* item)
{
    simulatorItem_ = item;
}


double ControllerItem::timeStep() const
{
    return simulatorItem_ ? simulatorItem_->worldTimeStep() : 0.0;
}


bool ControllerItem::initialize(ControllerIO* io)
{
    return true;
}


ControllerLogItem* ControllerItem::createLogItem()
{
    return new ControllerLogItem;
}


bool ControllerItem::start()
{
    return true;
}


void ControllerItem::input()
{

}


bool ControllerItem::control()
{
    return false;
}


void ControllerItem::log()
{

}


void ControllerItem::output()
{

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
