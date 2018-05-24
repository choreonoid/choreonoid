/**
   @author Shin'ichiro Nakaoka
   @author Shizuko Hattori
*/

#include <cnoid/BodyIoRTC>
#include <cnoid/RangeSensor>
#include <cnoid/RangeCamera>
#include <cnoid/ConnectionSet>
#include <cnoid/ThreadPool>
#include <rtm/DataOutPort.h>
#include <rtm/idl/BasicDataTypeSkel.h>
#include <rtm/idl/InterfaceDataTypes.hh>
#ifdef WIN32
#include <rtm/idl/CameraCommonInterface.hh>
#else
#include <rtm/ext/CameraCommonInterface.hh>
#endif
//#include <cnoid/corba/PointCloud.hh>


using namespace std;
using namespace cnoid;


class DeviceIo : public Referenced
{
protected:
    ScopedConnectionSet connections;
    
public:
    virtual void setPorts(BodyIoRTC* rtc) = 0;
    virtual bool initializeSimulation(Body* body) = 0;
    virtual void onStateChanged() = 0;
    void stopSimulation();
    virtual void clearSimulationDevice() = 0;
};

typedef ref_ptr<DeviceIo> DeviceIoPtr;


class CameraIo : public DeviceIo
{
    ThreadPool threadPool;
public:
    CameraPtr modelCamera;
    CameraPtr camera;
    Img::TimedCameraImage cameraImage;
    RTC::OutPort<Img::TimedCameraImage> cameraImageOut;
    std::shared_ptr<const Image> lastImage;

    CameraIo(Camera* camera);
    virtual void setPorts(BodyIoRTC* rtc) override;
    virtual bool initializeSimulation(Body* body) override;
    virtual void onStateChanged() override;
    void outputImage();
    virtual void clearSimulationDevice() override;
};


class RangeCameraIo : public CameraIo
{
    ThreadPool threadPool;
public:
    RangeCameraPtr modelRangeCamera;
    RangeCameraPtr rangeCamera;
    //PointCloudTypes::PointCloud pointCloud;
    //RTC::OutPort<PointCloudTypes::PointCloud> pointCloudOut;
    std::shared_ptr<const RangeCamera::PointData> lastPoints;
    std::shared_ptr<const Image> lastImage;

    RangeCameraIo(RangeCamera* rangeCamera);
    virtual void setPorts(BodyIoRTC* rtc) override;
    virtual bool initializeSimulation(Body* body) override;
    virtual void onStateChanged() override;
    void outputRangeData();
    virtual void clearSimulationDevice() override;
};    


class RangeSensorIo : public DeviceIo
{
    ThreadPool threadPool;
public:
    RangeSensorPtr modelRangeSensor;
    RangeSensorPtr rangeSensor;
    RTC::RangeData rangeData;
    RTC::OutPort<RTC::RangeData> rangeDataOut;
    std::shared_ptr<const RangeSensor::RangeData> lastRangeData;

    RangeSensorIo(RangeSensor* sensor);
    virtual void setPorts(BodyIoRTC* rtc) override;
    virtual bool initializeSimulation(Body* body) override;
    virtual void onStateChanged() override;
    void outputRangeData();
    virtual void clearSimulationDevice() override;
};    
    

class VisionSensorIoRTC : public BodyIoRTC
{
public:
    BodyPtr ioBody;
    vector<DeviceIoPtr> deviceIoList;

    VisionSensorIoRTC(RTC::Manager* manager);
    ~VisionSensorIoRTC();
    virtual bool initializeIO(ControllerIO* io) override;
    virtual bool initializeSimulation(ControllerIO* io) override;
    virtual void inputFromSimulator() override;
    virtual void outputToSimulator() override;
    virtual void stopSimulation() override;
};


extern "C"
{
    DLL_EXPORT void VisionSensorIoRTCInit(RTC::Manager* manager)
    {
        const char* spec[] = {
            "implementation_id", "VisionSensorIoRTC",
            "type_name",         "VisionSensorIoRTC",
            "description",       "Vision sensor I/O",
            "version",           "1.0.0",
            "vendor",            "Choreonoid",
            "category",          "Generic",
            "activity_type",     "DataFlowComponent",
            "max_instance",      "10",
            "language",          "C++",
            "lang_type",         "compile",
            ""
        };
        coil::Properties profile(spec);
        manager->registerFactory(
            profile, RTC::Create<VisionSensorIoRTC>, RTC::Delete<VisionSensorIoRTC>);
    }
};


VisionSensorIoRTC::VisionSensorIoRTC(RTC::Manager* manager)
    : BodyIoRTC(manager)
{

}


VisionSensorIoRTC::~VisionSensorIoRTC()
{

}


bool VisionSensorIoRTC::initializeIO(ControllerIO* io)
{
    deviceIoList.clear();
    
    DeviceList<> devices = io->body()->devices();

    for(auto& device : devices){
        if(auto rangeCamera = dynamic_pointer_cast<RangeCamera>(device)){
            deviceIoList.push_back(new RangeCameraIo(rangeCamera));
        } else if(auto camera = dynamic_pointer_cast<Camera>(device)){
            deviceIoList.push_back(new CameraIo(camera));
        } else if(auto rangeSensor = dynamic_pointer_cast<RangeSensor>(device)){
            deviceIoList.push_back(new RangeSensorIo(rangeSensor));
        }
    }
    for(auto& deviceIo : deviceIoList){
        deviceIo->setPorts(this);
    }

    return true;
}


bool VisionSensorIoRTC::initializeSimulation(ControllerIO* io)
{
    for(auto& deviceIo : deviceIoList){
        deviceIo->initializeSimulation(io->body());
    }
    return true;
}


void VisionSensorIoRTC::inputFromSimulator()
{

}


void VisionSensorIoRTC::outputToSimulator()
{

}


void VisionSensorIoRTC::stopSimulation()
{
    for(auto& deviceIo : deviceIoList){
        deviceIo->stopSimulation();
    }
}


void DeviceIo::stopSimulation()
{
    connections.disconnect();
    clearSimulationDevice();
}


CameraIo::CameraIo(Camera* camera)
    : modelCamera(camera),
      cameraImageOut(camera->name().c_str(), cameraImage)
{

}
        

void CameraIo::setPorts(BodyIoRTC* rtc)
{
    rtc->addOutPort(modelCamera->name().c_str(), cameraImageOut);
}


bool CameraIo::initializeSimulation(Body* body)
{
    camera = body->findDevice<Camera>(modelCamera->name());
    if(!camera){
        return false;
    }

    double fovy2 = camera->fieldOfView() / 2.0;
    double width = camera->resolutionX();
    double height = camera->resolutionY();
    double minlength = std::min(width, height);
    double fu, fv;
    fv = fu = minlength / tan(fovy2) / 2.0;
    double u0 = (width - 1)/2.0;
    double v0 = (height - 1)/2.0;

    cameraImage.data.intrinsic.matrix_element[0] = fu;
    cameraImage.data.intrinsic.matrix_element[1] = 0.0;
    cameraImage.data.intrinsic.matrix_element[2] = u0;
    cameraImage.data.intrinsic.matrix_element[3] = fv;
    cameraImage.data.intrinsic.matrix_element[4] = v0;
    cameraImage.data.intrinsic.distortion_coefficient.length(5);

    for(int i=0; i < 5; ++i){
        // zero distortion is natural in simulator
        cameraImage.data.intrinsic.distortion_coefficient[i] = 0.0;
    }
    for(int i=0; i < 4; ++i){
        for(int j=0; j < 4; ++j){
            cameraImage.data.extrinsic[i][j] = (i==j ? 1.0: 0.0);
        }
    }

    connections.add(
        camera->sigStateChanged().connect(
            [&](){ onStateChanged(); }));

    return true;
}


void CameraIo::onStateChanged()
{
    if(!threadPool.isRunning()){ // Skip if writing to the outport is not finished
        if(camera->sharedImage() != lastImage){
            lastImage = camera->sharedImage();
            if(!lastImage->empty()){
                threadPool.start([&](){ outputImage(); });
            }
        }
    }
}


void CameraIo::outputImage()
{
    int width,height;
    cameraImage.data.image.height = height = lastImage->height();
    cameraImage.data.image.width = width = lastImage->width();

    switch(camera->imageType()){
    case Camera::GRAYSCALE_IMAGE:
        cameraImage.data.image.format = Img::CF_GRAY;
        break;
    case Camera::COLOR_IMAGE:
        cameraImage.data.image.format = Img::CF_RGB;
        break;
    default:
        cameraImage.data.image.format = Img::CF_UNKNOWN;
        break;
    }
    size_t length = width * height * lastImage->numComponents() * sizeof(unsigned char);
    cameraImage.data.image.raw_data.length(length);
    void* src = (void*)lastImage->pixels();
    void* dis = (void*)cameraImage.data.image.raw_data.get_buffer();
    memcpy(dis, src, length);

    cameraImageOut.write();
}


void CameraIo::clearSimulationDevice()
{
    camera.reset();
}


RangeCameraIo::RangeCameraIo(RangeCamera* rangeCamera)
    : CameraIo(rangeCamera),
      modelRangeCamera(rangeCamera)
      //pointCloudOut(rangeCamera->name().c_str(), pointCloud)
{

}


void RangeCameraIo::setPorts(BodyIoRTC* rtc)
{
    CameraIo::setPorts(rtc);
    //rtc->addOutPort((modelRangeCamera->name() + "-depth").c_str(), pointCloudOut);
}


bool RangeCameraIo::initializeSimulation(Body* body)
{
    if(!CameraIo::initializeSimulation(body)){
        return false;
    }
    rangeCamera = dynamic_pointer_cast<RangeCamera>(camera);
    if(!rangeCamera){
        return false;
    }

    /*
    string format;
    switch(rangeCamera->imageType()){
    case Camera::COLOR_IMAGE:
        format = "xyzrgb";
        break;
    case Camera::GRAYSCALE_IMAGE:
    case Camera::NO_IMAGE:
    default:
        format = "xyz";
        break;
    }
    pointCloud.type = CORBA::string_dup(format.c_str());

    if(format == "xyz"){
        pointCloud.fields.length(3);
    } else if (format == "xyzrgb"){
        pointCloud.fields.length(6);
    }
    pointCloud.fields[0].name = "x";
    pointCloud.fields[0].offset = 0;
    pointCloud.fields[0].data_type = PointCloudTypes::FLOAT32;
    pointCloud.fields[0].count = 4;
    pointCloud.fields[1].name = "y";
    pointCloud.fields[1].offset = 4;
    pointCloud.fields[1].data_type = PointCloudTypes::FLOAT32;
    pointCloud.fields[1].count = 4;
    pointCloud.fields[2].name = "z";
    pointCloud.fields[2].offset = 8;
    pointCloud.fields[2].data_type = PointCloudTypes::FLOAT32;
    pointCloud.fields[2].count = 4;
    pointCloud.point_step = 12;
    if (format == "xyzrgb"){
        pointCloud.fields[3].name = "r";
        pointCloud.fields[3].offset = 12;
        pointCloud.fields[3].data_type = PointCloudTypes::UINT8;
        pointCloud.fields[3].count = 1;
        pointCloud.fields[4].name = "g";
        pointCloud.fields[4].offset = 13;
        pointCloud.fields[4].data_type = PointCloudTypes::UINT8;
        pointCloud.fields[4].count = 1;
        pointCloud.fields[5].name = "b";
        pointCloud.fields[5].offset = 14;
        pointCloud.fields[5].data_type = PointCloudTypes::UINT8;
        pointCloud.fields[5].count = 1;
        pointCloud.point_step = 16;
    }
    pointCloud.is_bigendian = false;
    pointCloud.is_dense = true;
    */
    
    return true;
}


void RangeCameraIo::onStateChanged()
{
    CameraIo::onStateChanged();

    if(!threadPool.isRunning()){
        if(rangeCamera->sharedPoints() != lastPoints){
            lastPoints = rangeCamera->sharedPoints();
            lastImage = rangeCamera->sharedImage();
            if(!lastPoints->empty()){
                if(!lastImage->empty()){
                    //pointCloud.height = lastImage->height();
                    //pointCloud.width = lastImage->width();
                } else {
                    if(rangeCamera->isOrganized()){
                        //pointCloud.height = rangeCamera->resolutionY();
                        //pointCloud.width = rangeCamera->resolutionX();
                    } else {
                        //pointCloud.height = 1;
                        //pointCloud.width = rangeCamera->numPoints();
                    }
                }
                threadPool.start([&](){ outputRangeData(); });
            }
        }
    }
}


void RangeCameraIo::outputRangeData()
{
    const vector<Vector3f>& points = *lastPoints;
    const Image& image = *lastImage;

    /*
    pointCloud.row_step = pointCloud.point_step * pointCloud.width;
    size_t length = points.size() * pointCloud.point_step;
    pointCloud.data.length(length);
    unsigned char* dis = (unsigned char*)pointCloud.data.get_buffer();
    const unsigned char* pixels = 0;
    if(!image.empty()){
        pixels = image.pixels();
    }
    for(size_t i=0; i < points.size(); i++, dis += pointCloud.point_step){
        memcpy(&dis[0], &points[i].x(), 4);
        memcpy(&dis[4], &points[i].y(), 4);
        memcpy(&dis[8], &points[i].z(), 4);
        if(pixels){
            dis[12] = *pixels++;
            dis[13] = *pixels++;
            dis[14] = *pixels++;
            dis[15] = 0;
        }
    }

    pointCloudOut.write();
    */
}


void RangeCameraIo::clearSimulationDevice()
{
    CameraIo::clearSimulationDevice();
    rangeCamera.reset();
}


RangeSensorIo::RangeSensorIo(RangeSensor* sensor)
    : modelRangeSensor(sensor),
      rangeDataOut(sensor->name().c_str(), rangeData)
{

}


void RangeSensorIo::setPorts(BodyIoRTC* rtc)
{
    rtc->addOutPort(modelRangeSensor->name().c_str(), rangeDataOut);
}


bool RangeSensorIo::initializeSimulation(Body* body)
{
    rangeSensor = body->findDevice<RangeSensor>(modelRangeSensor->name());
    if(!rangeSensor){
        return false;
    }
    
    rangeData.config.minAngle = -rangeSensor->yawRange() / 2.0;
    rangeData.config.maxAngle = rangeSensor->yawRange() / 2.0;
    rangeData.config.angularRes = rangeSensor->yawStep();
    rangeData.config.minRange = rangeSensor->minDistance();
    rangeData.config.maxRange = rangeSensor->maxDistance();
    rangeData.config.frequency = rangeSensor->frameRate();
    rangeData.config.rangeRes = 0;     // no data
    // Ignore geometry
    rangeData.geometry.geometry.pose.orientation.r = 0.0;
    rangeData.geometry.geometry.pose.orientation.p = 0.0;
    rangeData.geometry.geometry.pose.orientation.y = 0.0;
    rangeData.geometry.geometry.pose.position.x = 0.0;
    rangeData.geometry.geometry.pose.position.y = 0.0;
    rangeData.geometry.geometry.pose.position.z = 0.0;
    rangeData.geometry.geometry.size.w = 0.0;
    rangeData.geometry.geometry.size.l = 0.0;
    rangeData.geometry.geometry.size.h = 0.0;
    rangeData.geometry.elementGeometries.length(0);

    connections.add(
        rangeSensor->sigStateChanged().connect(
            [&](){ onStateChanged(); }));

    return true;
}


void RangeSensorIo::onStateChanged()
{
    if(!threadPool.isRunning()){
        if(rangeSensor->sharedRangeData() != lastRangeData){
            lastRangeData = rangeSensor->sharedRangeData();
            threadPool.start([&](){ outputRangeData(); });
        }
    }
}


void RangeSensorIo::outputRangeData()
{
    const RangeSensor::RangeData& src = *lastRangeData;
    rangeData.ranges.length(src.size());
    for(size_t i=0; i < src.size(); i++){
        rangeData.ranges[i] = src[i];
    }
    rangeDataOut.write();
}


void RangeSensorIo::clearSimulationDevice()
{
    rangeSensor.reset();
}
