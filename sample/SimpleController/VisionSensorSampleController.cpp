/**
   @author Shin'ichiro Nakaoka
*/

#include <cnoid/SimpleController>
#include <cnoid/Camera>
#include <cnoid/RangeSensor>

using namespace std;
using namespace cnoid;

class VisionSensorSampleController : public SimpleController
{
    DeviceList<Camera> cameras;
    DeviceList<RangeCamera> rangeCameras;
    DeviceList<RangeSensor> rangeSensors;
    double timeCounter;
    
public:

    virtual bool initialize()
    {
        DeviceList<VisionSensor> sensors(ioBody()->devices());
        putSensorInformation(sensors);

        cameras << sensors;
        rangeCameras << sensors;
        rangeSensors << sensors;

        for(size_t i=0; i < rangeCameras.size(); ++i){
            rangeCameras[i]->setShotDataAsState(true);
        }

        setJointOutput(false);

        timeCounter = 0.0;
        
        return true;
    }

    void putSensorInformation(DeviceList<> sensors)
    {
        if(!sensors.empty()){
            os() << "Sensor Type: " << sensors.front()->typeName() << "\n";
            for(size_t i=0; i < sensors.size(); ++i){
                Device* sensor = sensors[i];
                os() << " name: " << sensor->name() << ", id: " << sensor->id() << "\n";
            }
            os().flush();
        }
    }

    virtual bool control()
    {
        timeCounter += timeStep();
        if(timeCounter >= 1.0){
            for(size_t i=0; i < rangeCameras.size(); ++i){
                Camera* camera = rangeCameras[i];
                const Image& image = camera->constImage();
                os() << camera->name() << "'s image.empty(): " << image.empty() << endl;
            }
            timeCounter = 0.0;
        }
        return true;
    }
};

CNOID_IMPLEMENT_SIMPLE_CONTROLLER_FACTORY(VisionSensorSampleController)
