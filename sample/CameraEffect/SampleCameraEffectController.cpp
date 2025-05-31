#include <cnoid/SimpleController>
#include <cnoid/Camera>
#include <cnoid/Joystick>

using namespace cnoid;

class SampleCameraEffectController : public SimpleController
{
    Camera* camera;
    Joystick joystick;

public:
    virtual bool initialize(SimpleControllerIO* io) override
    {
        camera = io->body()->findDevice<Camera>("Pepper");
        io->enableInput(camera);
        joystick.makeReady();
        return true;
    }

    virtual bool control() override
    {
        joystick.readCurrentState();

        if(camera) {
            bool stateChanged = false;
            double pepper_amount = camera->info()->get("pepper_amount", 0.0);
            if(joystick.getButtonState(Joystick::A_BUTTON)) {
                camera->info()->write("pepper_amount", std::max(0.0, pepper_amount - 0.001));
                stateChanged = true;
            } else if(joystick.getButtonState(Joystick::B_BUTTON)) {
                camera->info()->write("pepper_amount", std::min(1.0, pepper_amount + 0.001));
                stateChanged = true;
            }
            if(stateChanged) {
                camera->notifyInfoChange();
            }
        }

        return true;
    }
};

CNOID_IMPLEMENT_SIMPLE_CONTROLLER_FACTORY(SampleCameraEffectController)