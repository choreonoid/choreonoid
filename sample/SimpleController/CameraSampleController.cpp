/**
   @author Shin'ichiro Nakaoka
*/

#include <cnoid/SimpleController>
#include <cnoid/Camera>

using namespace std;
using namespace cnoid;

class CameraSampleController : public SimpleController
{
    DeviceList<Camera> cameras;
    double timeCounter;
    
public:

    virtual bool initialize()
    {
        DeviceList<Camera> cameras(ioBody()->devices());

        for(size_t i=0; i < cameras.size(); ++i){
            Device* camera = cameras[i];
            os() << "Device type: " << camera->typeName()
                 << ", id: " << camera->id()
                 << ", name: " << camera->name() << endl;
        }
        
        setJointOutput(false);

        timeCounter = 0.0;
        
        return true;
    }

    virtual bool control()
    {
        timeCounter += timeStep();
        if(timeCounter >= 1.0){
            for(size_t i=0; i < cameras.size(); ++i){
                Camera* camera = cameras[i];
                string filename = camera->name() + ".png";
                camera->constImage().save(filename);
                os() << "The image of " << camera->name() << " has been saved to \"" << filename << "\"." << endl;
            }
            timeCounter = 0.0;
        }
        return false;
    }
};

CNOID_IMPLEMENT_SIMPLE_CONTROLLER_FACTORY(CameraSampleController)
