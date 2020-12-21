#include <cnoid/SimpleController>
#include <cnoid/SharedJoystick>
#include <fmt/format.h>

using namespace std;
using namespace cnoid;
using fmt::format;

namespace {

const double STICK_THRESH = 0.1;

}

class AizuWheelController : public SimpleController
{
    Body* body;
    double dt;
    int actuationMode;
    enum { L_WHEEL, R_WHEEL, NUM_WHEELS };
    vector<Link*> wheels;
    vector<double> qprev;
    SharedJoystickPtr joystick;
    int targetMode;

public:
    virtual bool initialize(SimpleControllerIO* io) override;
    bool initializeWheels(SimpleControllerIO* io, vector<string>& names);
    virtual bool control() override;
    void setWheelTorque(int id, double dq_target);
};


bool AizuWheelController::initialize(SimpleControllerIO* io)
{
    body = io->body();
    dt = io->timeStep();
    actuationMode = Link::JointTorque;

    string option = io->optionString();
    if(!option.empty()){
        if(option == "velocity" || option == "position"){
            actuationMode = Link::JointVelocity;
        } else if(option == "torque"){
            actuationMode = Link::JointTorque;
        } else {
            io->os() << format("Warning: Unknown option \"{}\".", option) << endl;
        }
    }

    vector<string> wheelNames = { "L_WHEEL", "R_WHEEL" };
    if(!initializeWheels(io, wheelNames)){
        return false;
    }

    joystick = io->getOrCreateSharedObject<SharedJoystick>("joystick");
    targetMode = joystick->addMode();

    return true;
}


bool AizuWheelController::initializeWheels(SimpleControllerIO* io, vector<string>& names)
{
    wheels.clear();
    qprev.clear();

    for(auto& name : names){
        auto link = body->link(name);
        if(!link){
            io->os() << format("{0} of {1} is not found", name, body->name()) << endl;
            return false;
        }
        link->setActuationMode(actuationMode);
        io->enableOutput(link);
        wheels.push_back(link);
        qprev.push_back(link->q());
    }
    
    return true;
}


bool AizuWheelController::control()
{
    joystick->updateState(targetMode);

    double hpos =
        joystick->getPosition(targetMode, Joystick::L_STICK_H_AXIS, STICK_THRESH) +
        0.8 * joystick->getPosition(Joystick::DIRECTIONAL_PAD_H_AXIS);

    double vpos = -(
        joystick->getPosition(targetMode, Joystick::L_STICK_V_AXIS, STICK_THRESH) +
        0.8 * joystick->getPosition(Joystick::DIRECTIONAL_PAD_V_AXIS));
    
    double dq_L = 4.0 * (vpos + 0.4 * hpos);
    double dq_R = 4.0 * (vpos - 0.4 * hpos);

    switch(actuationMode){

    case Link::JointVelocity:
        wheels[0]->dq_target() = dq_L;
        wheels[1]->dq_target() = dq_R;
        break;

    case Link::JointTorque:
        setWheelTorque(0, dq_L);
        setWheelTorque(1, dq_R);
        break;

    default:
        break;
    }

    return true;
}


void AizuWheelController::setWheelTorque(int id, double dq_target)
{
    Link* axis = wheels[id];
    double dq_current = (axis->q() - qprev[id]) / dt;
    axis->u() = 1.0 * (dq_target - dq_current);
    qprev[id] = axis->q();
}


CNOID_IMPLEMENT_SIMPLE_CONTROLLER_FACTORY(AizuWheelController)
