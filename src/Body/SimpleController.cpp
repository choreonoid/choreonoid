/**
   @author Shin'ichiro Nakaoka
*/

#include "SimpleController.h"
#include <stdexcept>

using namespace std;
using namespace cnoid;

std::string SimpleControllerIO::name() const
{
    return controllerName();
}

void SimpleControllerIO::setJointInput(int /* stateTypes */)
{
    throw std::logic_error(
        "SimpleControllerIO::setJointInput is deprecated and is not supported in the control system.");
}
    
void SimpleControllerIO::setJointOutput(int /* stateTypes */)
{
    throw std::logic_error(
        "SimpleControllerIO::setJointOutput is deprecated and is not supported in the control system.");
}
    
void SimpleControllerIO::setLinkInput(Link*, int /* stateTypes */)
{
    throw std::logic_error(
        "SimpleControllerIO::setLinkInput is deprecated and is not supported in the control system.");
}

void SimpleControllerIO::setLinkOutput(Link*, int /* stateTypes */)
{
    throw std::logic_error(
        "SimpleControllerIO::setLinkOutput is deprecated and is not supported in the control system.");
}


SimpleControllerConfig::SimpleControllerConfig(SimpleControllerIO* io)
    : io(io)
{

}

std::string SimpleControllerConfig::controllerName() const
{
    return io->controllerName();
}

Body* SimpleControllerConfig::body()
{
    return io->body();
}

std::string SimpleControllerConfig::optionString() const
{
    return io->optionString();
}

std::vector<std::string> SimpleControllerConfig::options() const
{
    return io->options();
}

std::ostream& SimpleControllerConfig::os() const
{
    return io->os();
}

SignalProxy<void()> SimpleControllerConfig::sigChanged()
{
    return sigChanged_;
}


SimpleController::SimpleController()
{

}

SimpleController::~SimpleController()
{

}

bool SimpleController::configure(SimpleControllerConfig*)
{
    return true;
}

bool SimpleController::start()
{
    return true;
}

void SimpleController::stop()
{

}

void SimpleController::unconfigure()
{

}
