/**
   Conveyor Controller Sample
   @author Shin'ichiro Nakaoka
*/

#include <cnoid/SimpleController>

using namespace cnoid;

class ConveyorController : public cnoid::SimpleController
{ 
public:
    virtual bool initialize()
        {
            return true;
        }

    virtual bool control()
        {
            BodyPtr io = ioBody();
            io->joint(0)->u() = -1.0;
            return true;
        }
};


CNOID_IMPLEMENT_SIMPLE_CONTROLLER_FACTORY(ConveyorController)
