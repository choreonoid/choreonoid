/*!
  @file
  @author Shin'ichiro Nakaoka
*/

#include "ControllerItem.h"
#include <cnoid/Archive>
#include <boost/tokenizer.hpp>
#include "gettext.h"

using namespace std;
using namespace boost;
using namespace cnoid;


double ControllerItemIO::worldTimeStep() const
{
    return timeStep();
}


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


bool ControllerItem::splitOptionString(const std::string& optionString, std::vector<std::string>& out_options) const
{
    out_options.clear();
    typedef boost::escaped_list_separator<char> separator;
    separator sep('\\', ' ');
    boost::tokenizer<separator> tok(optionString, sep);
    for(boost::tokenizer<separator>::iterator p = tok.begin(); p != tok.end(); ++p){
        const string& token = *p;
        if(!token.empty()){
            out_options.push_back(token);
        }
    }
    return !out_options.empty();
}


bool ControllerItem::isActive() const
{
    return simulatorItem_ ? simulatorItem_->isRunning() : false;
}


bool ControllerItem::initialize(ControllerItemIO* io)
{
    return true;
}


bool ControllerItem::start()
{
    return true;
}


bool ControllerItem::start(ControllerItemIO* io)
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
    putProperty(_("Immediate mode"), isImmediateMode_, changeProperty(isImmediateMode_));
    putProperty(_("Controller options"), optionString_, changeProperty(optionString_));
}


bool ControllerItem::store(Archive& archive)
{
    archive.write("isImmediateMode", isImmediateMode_);
    archive.write("controllerOptions", optionString_, DOUBLE_QUOTED);
    return true;
}


bool ControllerItem::restore(const Archive& archive)
{
    archive.read("isImmediateMode", isImmediateMode_);
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
