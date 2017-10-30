#include <cnoid/SimpleController>
#include <cnoid/SpotLight>
#include <cnoid/Joystick>

using namespace cnoid;

class LightController : public SimpleController
{
    SpotLight* light;
    Joystick joystick;
    bool prevButtonState;

public:
    virtual bool initialize(SimpleControllerIO* io) override
    {
        light = io->body()->findDevice<SpotLight>("Light");
        prevButtonState = false;
        return true;
    }

    virtual bool control() override
    {
        static const int buttonID[] = { 0, 2, 3 };
        
        joystick.readCurrentState();

        bool changed = false;

        bool currentState = joystick.getButtonState(buttonID[0]);
        if(currentState && !prevButtonState){
            light->on(!light->on());
            changed = true;
        }
        prevButtonState = currentState;

        if(joystick.getButtonState(buttonID[1])){
            light->setBeamWidth(std::max(0.1f, light->beamWidth() - 0.001f));
            changed = true;
        } else if(joystick.getButtonState(buttonID[2])){
            light->setBeamWidth(std::min(0.7854f, light->beamWidth() + 0.001f));
            changed = true;
        }

        if(changed){
            light->notifyStateChange();
        }

        return true;
    }
};

CNOID_IMPLEMENT_SIMPLE_CONTROLLER_FACTORY(LightController)
