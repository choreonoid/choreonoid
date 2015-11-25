/**
   @author Shin'ichiro Nakaoka
*/

#include <cnoid/SimpleController>
#include <cnoid/Camera>
#include <cnoid/RangeSensor>

using namespace cnoid;

class VisionSensorSampleController : public SimpleController
{
    DeviceList<Camera> cameras;
    DeviceList<RangeCamera> rangeCameras;
    DeviceList<RangeSensor> rangeSensors;

public:

    virtual bool initialize()
    {
        DeviceList<> devices = ioBody()->devices();
        
        devices.extract<RangeCamera>(rangeCameras);
        devices.extract<Camera>(cameras);
        devices.extract<RangeSensor>(rangeSensors);

        putSensorInformation(cameras);
        putSensorInformation(rangeCameras);
        putSensorInformation(rangeSensors);

        setJointOutput(false);
        
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
        return true;
    }
};

CNOID_IMPLEMENT_SIMPLE_CONTROLLER_FACTORY(VisionSensorSampleController)
