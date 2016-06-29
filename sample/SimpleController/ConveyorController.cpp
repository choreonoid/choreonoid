/**
   Conveyor Controller Sample
   @author Shin'ichiro Nakaoka
*/

#include <cnoid/SimpleController>

using namespace cnoid;

class ConveyorController : public cnoid::SimpleController
{
    Link* conveyorJoint;
    
public:
    virtual bool initialize(SimpleControllerIO* io)
    {
        conveyorJoint = io->body()->joint(0);
        io->setJointOutput(JOINT_VELOCITY);
        return true;
    }

    virtual bool control()
    {
        conveyorJoint->dq() = -1.0;
        return true;
    }
};


CNOID_IMPLEMENT_SIMPLE_CONTROLLER_FACTORY(ConveyorController)
