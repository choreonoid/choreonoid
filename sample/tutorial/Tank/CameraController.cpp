#include <cnoid/SimpleController>
#include <cnoid/Camera>
#include <cnoid/Joystick>

using namespace cnoid;

class CameraController : public SimpleController
{
    Camera* camera;
    Joystick joystick;
    bool oldButtonState;
    
public:
    virtual bool initialize(SimpleControllerIO* io)
    {
        camera = io->body()->findDevice<Camera>("Camera");
        io->enableInput(camera);
        oldButtonState = false;
        return true;
    }

    virtual bool control()
    {
        joystick.readCurrentState();

        bool currentState = joystick.getButtonState(1);
        if(currentState && !oldButtonState){
            const Image& image = camera->constImage();
            if(!image.empty()){
                std::string filename = camera->name() + ".png";
                camera->constImage().save(filename);
                os() << "The image of " << camera->name() << " has been saved to \"" << filename << "\"." << std::endl;
            }
        }
        oldButtonState = currentState;

        return true;
    }
};

CNOID_IMPLEMENT_SIMPLE_CONTROLLER_FACTORY(CameraController)
