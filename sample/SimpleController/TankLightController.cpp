/**
   A controller that brinks the tank light
   @author Shin'ichiro Nakaoka
*/

#include <cnoid/SimpleController>
#include <cnoid/Lights>

using namespace std;
using namespace cnoid;

class TankLightController : public cnoid::SimpleController
{ 
    LightPtr light;
    int blinkCounter;

public:
    
    virtual bool initialize() {

        BodyPtr io = ioBody();
        setJointOutput(false);
        blinkCounter = 0;
        DeviceList<Light> lights(io->devices());
        if(!lights.empty()){
            light = lights.front();
        }
        blinkCounter = 0;

        return (light != 0);
    }

    virtual bool control() {

        if(++blinkCounter == 250){
            light->on(!light->on());
            light->notifyStateChange();
            blinkCounter = 0;
        }
        return true;
    }
};

CNOID_IMPLEMENT_SIMPLE_CONTROLLER_FACTORY(TankLightController)
