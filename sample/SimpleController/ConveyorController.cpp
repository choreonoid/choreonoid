/**
   Conveyor Controller Sample
*/

#include <cnoid/SimpleController>

using namespace cnoid;

class ConveyorController : public SimpleController
{
    Link* conveyorJoint;
    
public:
    virtual bool initialize(SimpleControllerIO* io)
    {
        conveyorJoint = io->body()->joint(0);
        return true;
    }

    virtual bool control()
    {
        conveyorJoint->dq() = 1.0;
        return true;
    }
};


CNOID_IMPLEMENT_SIMPLE_CONTROLLER_FACTORY(ConveyorController)
