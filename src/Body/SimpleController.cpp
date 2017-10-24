/**
   @author Shin'ichiro Nakaoka
*/

#include "SimpleController.h"
#include <stdexcept>

using namespace std;
using namespace cnoid;


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


SimpleController::SimpleController()
{

}


SimpleController::~SimpleController()
{

}


bool SimpleController::start()
{
    return true;
}
