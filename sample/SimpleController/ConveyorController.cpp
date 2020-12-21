/**
   Conveyor Controller Sample
*/

#include <cnoid/SimpleController>

using namespace cnoid;

class ConveyorController : public SimpleController
{
    Link* conveyorJoint;
    
public:
    virtual bool initialize(SimpleControllerIO* io) override
    {
        conveyorJoint = io->body()->joint(0);
        io->enableOutput(conveyorJoint, Link::JointVelocity);
        return true;
    }

    virtual bool control() override
    {
        conveyorJoint->dq_target() = 1.0;
        return true;
    }
};

CNOID_IMPLEMENT_SIMPLE_CONTROLLER_FACTORY(ConveyorController)
