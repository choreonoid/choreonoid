#include <cnoid/SimpleController>
#include <iostream>

using namespace cnoid;

class HandyRockDrillController : public cnoid::SimpleController
{
private:
    double dt;
    Body* body;
    Link* pusher;
    double amplitude = 0.5;
    double w = 20.0;
public:
    virtual bool initialize(SimpleControllerIO* io) override;
    virtual bool control() override;
};

bool HandyRockDrillController::initialize(SimpleControllerIO* io)
{
    body = io->body();
    dt = io->timeStep();

    for(size_t i = 0; i < io->options().size(); i++){
        double value;
        try
        {
            value = std::stod(io->options()[i]);
        }
        catch (std::invalid_argument e)
        {
            std::cout << "RockDrillController: Options have invalid_arguments" << std::endl;
            continue;
        }

        if(i == 0) amplitude = value;
        if(i == 1) w = value;
    }

    pusher = body->link("PUSHER");
    if(!pusher) return false;
    pusher->setActuationMode(Link::JOINT_VELOCITY);
    io->enableOutput(pusher);

    return true;
}

bool HandyRockDrillController::control()
{
    static float t = 0.0;
    pusher->dq() = amplitude * sin(w * t);
    t += dt;

    //std::cout << pusher->dq() << std::endl;

    return true;
}


CNOID_IMPLEMENT_SIMPLE_CONTROLLER_FACTORY(HandyRockDrillController)
