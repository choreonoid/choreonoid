#include "RockDrillController.h"

using namespace std;
using namespace cnoid;

namespace {

RockDrillController* instance = nullptr;

const double amplitude = 0.04;
const double w = 50.0;

}


RockDrillController* RockDrillController::instance()
{
    return ::instance;
}


RockDrillController::RockDrillController()
{
    ::instance = this;
}


bool RockDrillController::initialize(SimpleControllerIO* io)
{
    this->io = io;
    
    pusher = io->body()->link("PUSHER");
    if(!pusher){
        io->os() << "PUSHER is not found in " << io->body()->name() << endl;
        return false;
    }
    pusher->setActuationMode(Link::JOINT_DISPLACEMENT);
    io->enableOutput(pusher);

    time = 0.0;
    dt = io->timeStep();
    isPowerOn = false;
    requestedPowerState = false;

    return true;
}
    

bool RockDrillController::control()
{
    if(requestedPowerState != isPowerOn){
        isPowerOn = requestedPowerState;
        if(isPowerOn){
            //io->os() << "Power on" << endl;
            time = 0.0;
        }
    }
    if(isPowerOn){
        pusher->q_target() = amplitude * sin(w * time);
        time += dt;
    }
    return true;
}


void RockDrillController::power(bool on)
{
    //io->os() << "RockDrillController::power(" << on << ")" << endl;
    requestedPowerState = on;
}


CNOID_IMPLEMENT_SIMPLE_CONTROLLER_FACTORY(RockDrillController)
