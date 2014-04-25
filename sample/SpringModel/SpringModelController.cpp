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

    virtual bool initialize() {

        spring = ioBody()->link("UPPER");

        if(!spring){
            os() << "Spring-damper joint \"UPPER\" cannot be detected." << std::endl;
            return false;
        }
        if(!isImmediateMode()){
            os() << "Controller should be used with the immediate mode." << std::endl;
            return false;
        }
        
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
