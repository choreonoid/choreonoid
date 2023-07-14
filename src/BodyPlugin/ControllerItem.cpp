#include "ControllerItem.h"
#include <cnoid/ItemManager>
#include <cnoid/ReferencedObjectSeqItem>
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


void ControllerItem::onTreePathChanged()
{
    auto bodyItem = findOwnerItem<BodyItem>();
    if(targetBodyItem_.expired() || bodyItem != targetBodyItem_.lock()){
        targetBodyItem_ = bodyItem;
        onTargetBodyItemChanged(bodyItem);
    }
}


void ControllerItem::onTargetBodyItemChanged(BodyItem* /* bodyItem */)
{

}


void ControllerItem::setSimulatorItem(SimulatorItem* item)
{
    simulatorItem_ = item;
}


bool ControllerItem::isActive() const
{
    if(auto item = simulatorItem_.lock()){
        return item->isRunning();
    }
    return false;
}


void ControllerItem::setNoDelayMode(bool on)
{
    isNoDelayMode_ = on;
}


double ControllerItem::timeStep() const
{
    if(auto item = simulatorItem_.lock()){
        return item->worldTimeStep();
    }
    return 0.0;
}


bool ControllerItem::checkIfSubController(ControllerItem* /* controllerItem */) const
{
    return false;
}
    

bool ControllerItem::initialize(ControllerIO* io)
{
    return true;
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


void ControllerItem::output()
{

}


void ControllerItem::stop()
{

}


ReferencedObjectSeqItem* ControllerItem::createLogItem()
{
    return new ReferencedObjectSeqItem;
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
