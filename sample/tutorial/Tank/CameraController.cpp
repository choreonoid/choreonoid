#include <cnoid/SimpleController>
#include <cnoid/Camera>
#include <cnoid/Joystick>

using namespace cnoid;

class CameraController : public SimpleController
{
    Camera* camera;
    Joystick joystick;
    bool prevButtonState;
    std::ostream* os;
    
public:
    virtual bool initialize(SimpleControllerIO* io) override
    {
        camera = io->body()->findDevice<Camera>("Camera");
        io->enableInput(camera);
        prevButtonState = false;
        os = &io->os();
        return true;
    }

    virtual bool control() override
    {
        joystick.readCurrentState();

        bool currentState = joystick.getButtonState(1);
        if(currentState && !prevButtonState){
            const Image& image = camera->constImage();
            if(!image.empty()){
                std::string filename = camera->name() + ".png";
                camera->constImage().save(filename);
                (*os) << "The image of " << camera->name()
                      << " has been saved to \"" << filename << "\"."
                      << std::endl;
            }
        }
        prevButtonState = currentState;

        return true;
    }
};

CNOID_IMPLEMENT_SIMPLE_CONTROLLER_FACTORY(CameraController)
