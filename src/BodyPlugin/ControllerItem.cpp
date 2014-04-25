/*!
  @file
  @author Shin'ichiro Nakaoka
*/

#include "ControllerItem.h"
#include <cnoid/Archive>
#include "gettext.h"

using namespace std;
using namespace boost;
using namespace cnoid;


ControllerItem::ControllerItem()
{
    isImmediateMode_ = true;
}


ControllerItem::ControllerItem(const ControllerItem& org)
    : Item(org)
{
    isImmediateMode_ = org.isImmediateMode_;
}


ControllerItem::~ControllerItem()
{

}


bool ControllerItem::isActive() const
{
    return simulatorItem_ ? simulatorItem_->isRunning() : false;
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


SignalProxy< boost::signal<void(const std::string& message)> > ControllerItem::sigMessage()
{
    return sigMessage_;
}


void ControllerItem::doPutProperties(PutPropertyFunction& putProperty)
{
    putProperty(_("Immediate mode"), isImmediateMode_, changeProperty(isImmediateMode_));
}


bool ControllerItem::store(Archive& archive)
{
    archive.write("isImmediateMode", isImmediateMode_);
    return true;
}


bool ControllerItem::restore(const Archive& archive)
{
    archive.read("isImmediateMode", isImmediateMode_);
    return true;
}
