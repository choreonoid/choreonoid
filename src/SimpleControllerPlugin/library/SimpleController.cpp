/**
   @author Shin'ichiro Nakaoka
*/

#include "SimpleController.h"

using namespace std;
using namespace boost;
using namespace cnoid;

SimpleController::SimpleController()
{
    io = 0;
}


SimpleController::~SimpleController()
{

}


bool SimpleController::initialize(SimpleControllerIO* io)
{
    return false;
}


bool SimpleController::initialize()
{
    return false;
}


bool SimpleController::control(SimpleControllerIO* io)
{
    return false;
}


bool SimpleController::control()
{
    return false;
}


void SimpleController::setIO(SimpleControllerIO* io)
{
    this->io = io;
}


void SimpleController::setJointOutput(bool on)
{
    if(on){
        io->setJointOutput(JOINT_TORQUE);
    } else {
        io->setJointOutput(0);
    }
}
    

void SimpleController::setJointOutput(int jointId, bool on)
{
    if(on){
        io->setLinkOutput(io->body()->joint(jointId), JOINT_TORQUE);
    } else {
        io->setLinkOutput(io->body()->joint(jointId), 0);
    }
}


Body* SimpleController::ioBody()
{
    return io->body();
}


double SimpleController::timeStep() const
{
    return io->timeStep();
}


std::ostream& SimpleController::os() const
{
    return io->os();
}
