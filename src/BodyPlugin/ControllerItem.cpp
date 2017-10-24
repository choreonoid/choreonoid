/*!
  @file
  @author Shin'ichiro Nakaoka
*/

#include "ControllerItem.h"
#include <cnoid/Archive>
#include "gettext.h"

using namespace std;
using namespace cnoid;


ControllerItem::ControllerItem()
{
    isNoDelayMode_ = true;
}


ControllerItem::ControllerItem(const ControllerItem& org)
    : Item(org)
{
    isNoDelayMode_ = org.isNoDelayMode_;
}


ControllerItem::~ControllerItem()
{

}


bool ControllerItem::setNoDelayMode(bool on)
{
    isNoDelayMode_ = on;
    return on;
}


bool ControllerItem::isActive() const
{
    return simulatorItem_ ? simulatorItem_->isRunning() : false;
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


std::string ControllerItem::getMessage()
{
    string message(message_);
    message_.clear();
    return message;
}


void ControllerItem::putMessage(const std::string& message)
{
    message_ += message;
    if(!sigMessage_.empty()){
        sigMessage_(message_);
        message_.clear();
    }
}


SignalProxy<void(const std::string& message)> ControllerItem::sigMessage()
{
    return sigMessage_;
}


void ControllerItem::doPutProperties(PutPropertyFunction& putProperty)
{
    putProperty(_("No delay mode"), isNoDelayMode_, changeProperty(isNoDelayMode_));
    putProperty(_("Controller options"), optionString_, changeProperty(optionString_));
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
