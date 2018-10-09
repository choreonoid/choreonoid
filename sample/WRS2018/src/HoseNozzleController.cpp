/**
   @author Shin'ichiro Nakaoka
*/

#include <cnoid/SimpleController>
#include <cnoid/Device>
#include <cnoid/EigenUtil>

using namespace std;
using namespace cnoid;

class HoseNozzleController : public SimpleController
{
    Link* lever;
    Device* water;
    
public:

    virtual bool initialize(SimpleControllerIO* io) override
    {
        lever = io->body()->link("HOSE_NOZZLE_LEVER");
        water = io->body()->findDevice("WATER");

        if(lever && water){
            io->enableInput(lever, JOINT_DISPLACEMENT);
            return true;
        }
        
        return false;
    }

    virtual bool control() override
    {
        if(!water->on()){
            if(lever->q() < radian(-30.0)){
                water->on(true);
                water->notifyStateChange();
            }
        } else {
            if(lever->q() > radian(0.0)){
                water->on(false);
                water->notifyStateChange();
            }
        }
        return true;
    }
};

CNOID_IMPLEMENT_SIMPLE_CONTROLLER_FACTORY(HoseNozzleController)

