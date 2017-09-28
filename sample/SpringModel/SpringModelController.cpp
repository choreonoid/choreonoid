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

    virtual bool initialize(SimpleControllerIO* io) {

        SimulationSimpleControllerIO* sio = dynamic_cast<SimulationSimpleControllerIO*>(io);

        if(!sio){
            return false;
        }

        spring = io->body()->link("UPPER");

        if(!spring){
            os() << "Spring-damper joint \"UPPER\" cannot be detected." << std::endl;
            return false;
        }
        
        io->enableInput(spring, JOINT_DISPLACEMENT | JOINT_VELOCITY);
        io->enableOutput(spring);

        sio->setImmediateMode(true);
        
        return true;
    }

    virtual bool control() {

        const double KP = 2000.0;
        const double KD = 5.0;

        spring->u() = -KP * spring->q() - KD * spring->dq();

        return true;
    }

};


CNOID_IMPLEMENT_SIMPLE_CONTROLLER_FACTORY(SpringModelController)
