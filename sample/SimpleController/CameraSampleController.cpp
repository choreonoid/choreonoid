/**
   @author Shin'ichiro Nakaoka
*/

#include <cnoid/SimpleController>
#include <cnoid/Camera>

using namespace cnoid;

class CameraSampleController : public SimpleController
{
    DeviceList<Camera> cameras;
    double timeCounter;
    double timeStep;
    std::ostream* os;
    
public:
    virtual bool initialize(SimpleControllerIO* io) override
    {
        os = &io->os();
        
        cameras << io->body()->devices();

        for(size_t i=0; i < cameras.size(); ++i){
            Device* camera = cameras[i];
            io->enableInput(camera);
            *os << "Device type: " << camera->typeName()
                << ", id: " << camera->id()
                << ", name: " << camera->name() << std::endl;
        }
        
        timeCounter = 0.0;
        timeStep = io->timeStep();
        
        return true;
    }

    virtual bool control() override
    {
        timeCounter += timeStep;
        if(timeCounter >= 1.0){
            for(size_t i=0; i < cameras.size(); ++i){
                Camera* camera = cameras[i];
                std::string filename = camera->name() + ".png";
                if(camera->constImage().save(filename)){
                    *os << "The image of " << camera->name()
                        << " has been saved to \"" << filename << "\"." << std::endl;
                }
            }
            timeCounter = 0.0;
        }
        return false;
    }
};

CNOID_IMPLEMENT_SIMPLE_CONTROLLER_FACTORY(CameraSampleController)
