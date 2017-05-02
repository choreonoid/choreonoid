#include <cnoid/SimpleController>
#include <cnoid/SpotLight>
#include <cnoid/Joystick>

using namespace cnoid;

class LightController : public SimpleController
{
    SpotLight* light;
    Joystick joystick;
    bool oldButtonState;

public:
    virtual bool initialize(SimpleControllerIO* io)
    {
        light = io->body()->findDevice<SpotLight>("MainLight");
        oldButtonState = false;
        return true;
    }

    virtual bool control()
    {
        joystick.readCurrentState();

        bool changed = false;

        bool currentState = joystick.getButtonState(0);
        if(currentState && !oldButtonState){
            light->on(!light->on());
            changed = true;
        }
        oldButtonState = currentState;

        if(joystick.getButtonState(2)){
            light->setBeamWidth(std::min(0.7854f, light->beamWidth() + 0.001f));
            changed = true;
        } else if(joystick.getButtonState(3)){
            light->setBeamWidth(std::max(0.1f, light->beamWidth() - 0.001f));
            changed = true;
        }

        if(changed){
            light->notifyStateChange();
        }

        return true;
    }
};

CNOID_IMPLEMENT_SIMPLE_CONTROLLER_FACTORY(LightController)
