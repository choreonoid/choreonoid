/**
 * Add VisionSensor to TankIoRTC.cpp
   @author Shizuko Hattori
*/

#include <cnoid/BodyIoRTC>
#include <cnoid/Light>
#include <cnoid/RangeSensor>
#include <cnoid/RangeCamera>
#include <cnoid/ConnectionSet>
#include <cnoid/corba/PointCloud.hh>
#include <cnoid/corba/CameraImage.hh>
#include <rtm/idl/BasicDataTypeSkel.h>
#include <rtm/idl/InterfaceDataTypes.hh>
#include <rtm/DataInPort.h>
#include <rtm/DataOutPort.h>

using namespace std;
using namespace cnoid;

namespace {

class TankVisionSensorIoRTC : public BodyIoRTC
{
public:
    TankVisionSensorIoRTC(RTC::Manager* manager);
    ~TankVisionSensorIoRTC();

    virtual bool initializeIO(ControllerIO* io) override;
    virtual bool initializeSimulation(ControllerIO* io) override;
    virtual void inputFromSimulator() override;
    virtual void outputToSimulator() override;

    void initializeCamera();
    void onCameraStateChanged();
    void initializeRangeCamera();
    void onRangeCameraStateChanged();
    void initializeRangeSensor();
    void onRangeSensorStateChanged();

    ScopedConnectionSet connections;

    BodyPtr ioBody;
    Link* turretY;
    Link* turretP;
    Link* trackL;
    Link* trackR;
    
    // DataInPort declaration
    RTC::TimedDoubleSeq torques;
    RTC::InPort<RTC::TimedDoubleSeq> torquesIn;

    RTC::TimedDoubleSeq velocities;
    RTC::InPort<RTC::TimedDoubleSeq> velocitiesIn;

    RTC::TimedBooleanSeq lightSwitch;
    RTC::InPort<RTC::TimedBooleanSeq> lightSwitchIn;
    LightPtr light;
    
    // DataOutPort declaration
    RTC::TimedDoubleSeq angles;
    RTC::OutPort<RTC::TimedDoubleSeq> anglesOut;

    Img::TimedCameraImage cameraImage;
    RTC::OutPort<Img::TimedCameraImage> cameraImageOut;
    CameraPtr camera;
    std::shared_ptr<const Image> prevImage;

    PointCloudTypes::PointCloud pointCloud;
    RTC::OutPort<PointCloudTypes::PointCloud> pointCloudOut;
    RangeCameraPtr rangeCamera;
    std::shared_ptr<const RangeCamera::PointData> prevPoints;

    RTC::RangeData rangeData;
    RTC::OutPort<RTC::RangeData> rangeDataOut;
    RangeSensorPtr rangeSensor;
    std::shared_ptr<const RangeSensor::RangeData> prevRangeData;
};

const char* spec[] =
{
    "implementation_id", "TankVisionSensorIoRTC",
    "type_name",         "TankVisionSensorIoRTC",
    "description",       "Tank I/O",
    "version",           "1.0",
    "vendor",            "AIST",
    "category",          "Generic",
    "activity_type",     "DataFlowComponent",
    "max_instance",      "10",
    "language",          "C++",
    "lang_type",         "compile",
    ""
};

}

TankVisionSensorIoRTC::TankVisionSensorIoRTC(RTC::Manager* manager)
    : BodyIoRTC(manager),
      torquesIn("u", torques),
      velocitiesIn("dq", velocities),
      lightSwitchIn("light", lightSwitch),
      anglesOut("q", angles),
      cameraImageOut("Camera_image", cameraImage),
      pointCloudOut("Camera_range", pointCloud),
      rangeDataOut("VelodyneVLP-16", rangeData)
{

}


TankVisionSensorIoRTC::~TankVisionSensorIoRTC()
{
    connections.disconnect();
}


bool TankVisionSensorIoRTC::initializeIO(ControllerIO* io)
{
    // Set InPort buffers
    addInPort("u", torquesIn);
    addInPort("dq", velocitiesIn);
    addInPort("light", lightSwitchIn);
    
    // Set OutPort buffer
    addOutPort("q", anglesOut);
    angles.data.length(2);

    addOutPort("Camera_image", cameraImageOut);
    addOutPort("Camera_range", pointCloudOut);
    addOutPort("VelodyneVLP-16", rangeDataOut);

    return true;
}


bool TankVisionSensorIoRTC::initializeSimulation(ControllerIO* io)
{
    ioBody = io->body();

    turretY = ioBody->link("TURRET_Y");
    turretP = ioBody->link("TURRET_P");
    turretY->setActuationMode(Link::JOINT_TORQUE);
    turretP->setActuationMode(Link::JOINT_TORQUE);

    trackL = ioBody->link("TRACK_L");
    trackR = ioBody->link("TRACK_R");
    trackL->setActuationMode(Link::JOINT_SURFACE_VELOCITY);
    trackR->setActuationMode(Link::JOINT_SURFACE_VELOCITY);
    
    light = ioBody->findDevice<Light>("Light");
    
    connections.disconnect();
    camera = ioBody->findDevice<Camera>("Camera");
    initializeCamera();

    rangeCamera = ioBody->findDevice<RangeCamera>("Camera");
    initializeRangeCamera();

    rangeSensor = ioBody->findDevice<RangeSensor>("VelodyneVLP-16");
    initializeRangeSensor();

    return true;
}


void TankVisionSensorIoRTC::inputFromSimulator()
{
    angles.data[0] = turretY->q();
    angles.data[1] = turretP->q();
    anglesOut.write();
}


void TankVisionSensorIoRTC::outputToSimulator()
{
    if(torquesIn.isNew()){
        torquesIn.read();
        if(torques.data.length() >= 2){
            turretY->u() = torques.data[0];
            turretP->u() = torques.data[1];
        }
    }
    if(velocitiesIn.isNew()){
        velocitiesIn.read();
        if(velocities.data.length() >= 2){
            trackL->dq() = velocities.data[0];
            trackR->dq() = velocities.data[1];
        }
    }
    if(light && lightSwitchIn.isNew()){
        lightSwitchIn.read();
        light->on(lightSwitch.data[0]);
        light->notifyStateChange();
    }
}


void TankVisionSensorIoRTC::initializeCamera()
{
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
    for (int i=0; i<5; i++)
        cameraImage.data.intrinsic.distortion_coefficient[i] = 0.0; // zero distortion is natural in simulator
    for(int i=0; i<4; i++)
        for(int j=0; j<4; j++)
            cameraImage.data.extrinsic[i][j] = i==j? 1.0: 0.0;

    connections.add(camera->sigStateChanged().connect(
            [&](){ onCameraStateChanged(); }));
}


void TankVisionSensorIoRTC::onCameraStateChanged()
{
    if(camera->sharedImage() != prevImage){
        const Image& image = camera->constImage();
        if(!image.empty()){
            int width,height;
            cameraImage.data.image.height = height = image.height();
            cameraImage.data.image.width = width = image.width();
            switch(camera->imageType()){
            case Camera::GRAYSCALE_IMAGE : cameraImage.data.image.format = Img::CF_GRAY;
            break;
            case Camera::COLOR_IMAGE : cameraImage.data.image.format = Img::CF_RGB;
            break;
            default : cameraImage.data.image.format = Img::CF_UNKNOWN;
            break;
            }
            size_t length = width * height * image.numComponents() * sizeof(unsigned char);
            cameraImage.data.image.raw_data.length(length);
            void* src = (void*)image.pixels();
            void* dis = (void*)cameraImage.data.image.raw_data.get_buffer();
            memcpy(dis, src, length);
        }
        prevImage = camera->sharedImage();
        cameraImageOut.write();
    }
}


void TankVisionSensorIoRTC::initializeRangeCamera()
{
    string format;
    switch(rangeCamera->imageType()){
    case Camera::COLOR_IMAGE :
        format = "xyzrgb";
        break;
    case Camera::GRAYSCALE_IMAGE :
    case Camera::NO_IMAGE :
    default :
        format = "xyz";
        break;
    }
    pointCloud.type = CORBA::string_dup(format.c_str());

    if (format == "xyz"){
        pointCloud.fields.length(3);
    }else if (format == "xyzrgb"){
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

    connections.add(rangeCamera->sigStateChanged().connect(
            [&](){ onRangeCameraStateChanged(); }));
}


void TankVisionSensorIoRTC::onRangeCameraStateChanged()
{
    if(rangeCamera->sharedPoints() != prevPoints){
        const vector<Vector3f>& points = rangeCamera->constPoints();
        const Image& image = rangeCamera->constImage();

        if(!points.empty()){
            if(!image.empty()){
                pointCloud.height = image.height();
                pointCloud.width = image.width();
            }else{
                if(rangeCamera->isOrganized()){
                    pointCloud.height = rangeCamera->resolutionY();
                    pointCloud.width = rangeCamera->resolutionX();
                }else{
                    pointCloud.height = 1;
                    pointCloud.width = rangeCamera->numPoints();
                }
            }
            pointCloud.row_step = pointCloud.point_step * pointCloud.width;
            size_t length = points.size() * pointCloud.point_step;
            pointCloud.data.length(length);
            unsigned char* dis = (unsigned char*)pointCloud.data.get_buffer();
            const unsigned char* pixels = 0;
            if(!image.empty())
                pixels = image.pixels();
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
        }

        prevPoints = rangeCamera->sharedPoints();
        pointCloudOut.write();
    }
}


void TankVisionSensorIoRTC::initializeRangeSensor()
{
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

    connections.add(rangeSensor->sigStateChanged().connect(
            [&](){ onRangeSensorStateChanged(); }));
}


void TankVisionSensorIoRTC::onRangeSensorStateChanged()
{
    if(rangeSensor->sharedRangeData() != prevRangeData){
        const RangeSensor::RangeData& src = rangeSensor->constRangeData();
        rangeData.ranges.length(src.size());
        for(size_t i=0; i < src.size(); i++)
            rangeData.ranges[i] = src[i];

        prevRangeData = rangeSensor->sharedRangeData();
        rangeDataOut.write();
    }
}


extern "C"
{
    DLL_EXPORT void TankVisionSensorIoRTCInit(RTC::Manager* manager)
    {
        coil::Properties profile(spec);
        manager->registerFactory(
            profile, RTC::Create<TankVisionSensorIoRTC>, RTC::Delete<TankVisionSensorIoRTC>);
    }
};
