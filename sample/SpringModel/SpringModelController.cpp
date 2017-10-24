/**
   @author Shin'ichiro Nakaoka
*/
   
#include <cnoid/SimpleController>
#include <cnoid/Link>

using namespace cnoid;

class SpringModelController : public cnoid::SimpleController
{
    Link* spring;
    
public:

    virtual bool initialize(SimpleControllerIO* io) override
    {
        spring = io->body()->link("UPPER");

        if(!spring){
            io->os() << "Spring-damper joint \"UPPER\" cannot be detected." << std::endl;
            return false;
        }

        spring->setActuationMode(Link::JOINT_TORQUE);
        io->enableOutput(spring);
        io->enableInput(spring, JOINT_DISPLACEMENT | JOINT_VELOCITY);
        io->setNoDelayMode(true);

        return true;
    }

    virtual bool control() override
    {
        const double KP = 2000.0;
        const double KD = 5.0;

        spring->u() = -KP * spring->q() - KD * spring->dq();

        return true;
    }
};

CNOID_IMPLEMENT_SIMPLE_CONTROLLER_FACTORY(SpringModelController)
