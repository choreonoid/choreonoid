/**
   \file
   \author Shizuko Hattori
*/

#include "VirtualRobotPortHandler.h"
#include "BodyRTCItem.h"

#include <cnoid/corba/PointCloud.hh>

#ifdef USE_BUILTIN_CAMERA_IMAGE_IDL
# include <deprecated/corba/CameraImage.hh>
#else
# ifdef _WIN32
#  include <rtm/idl/CameraCommonInterface.hh>
# else
#  include <rtm/ext/CameraCommonInterface.hh>
# endif
#endif

#include <cnoid/EigenUtil>
#include <cnoid/DyBody>
#include <cnoid/Light>

using namespace std;
using namespace RTC;
using namespace cnoid;


PortHandler::PortHandler(PortInfo& info)
    : portName(info.portName)
{

} 


PortHandler::~PortHandler()
{

}


OutPortHandler::OutPortHandler(PortInfo& info, bool synchContorller) 
    : PortHandler(info), synchController(synchContorller)
{
    stepTime = info.stepTime;
}


SensorStateOutPortHandler::SensorStateOutPortHandler(PortInfo& info)
    : OutPortHandler(info), 
      outPort(info.portName.c_str(), values)
{
    dataTypeId = info.dataTypeId;
}


RTC::OutPortBase& SensorStateOutPortHandler::getOutPort()
{
    return outPort;
}


void SensorStateOutPortHandler::inputDataFromSimulator(BodyRTCItem* bodyRTC)
{
    auto body = bodyRTC->body();
    switch(dataTypeId) {
    case JOINT_VALUE:
    {
        const int n = body->numJoints();
        values.data.length(n);
        for(int i=0; i < n; i++){
            values.data[i] = body->joint(i)->q();
        }
        break;
    }
    case JOINT_VELOCITY:
    {
        const int n =body->numJoints();
        values.data.length(n);
        for(int i=0; i < n; i++){
            values.data[i] = body->joint(i)->dq();
        }
        break;
    }
    case JOINT_TORQUE:
    {
        const int n = body->numJoints();
        values.data.length(n);
        for(int i=0; i < n; i++){
            values.data[i] = body->joint(i)->u();
        }
        break;
    }
    case FORCE_SENSOR:
    {
        const DeviceList<ForceSensor>& forceSensors = bodyRTC->forceSensors();
        const int n = forceSensors.size();
        values.data.length(6 * n);
        for(int i=0; i < n; i++){
            Eigen::Map<Vector6>(&values.data[i*6]) = forceSensors[i]->F();
        }
    }
    break;
    case RATE_GYRO_SENSOR:
    {
        const DeviceList<RateGyroSensor>& rateGyroSensors = bodyRTC->rateGyroSensors();
        const int n = rateGyroSensors.size();
        values.data.length(3 * n);
        for(int i=0; i < n; i++){
            Eigen::Map<Vector3>(&values.data[i*3]) = rateGyroSensors[i]->w();
        }
    }
    break;
    case ACCELERATION_SENSOR:
    {
        const DeviceList<AccelerationSensor>& accelerationSensors = bodyRTC->accelerationSensors();
        const int n = accelerationSensors.size();
        values.data.length(3 * n);
        for(int i=0; i < n; i++){
            Eigen::Map<Vector3>(&values.data[i*3]) = accelerationSensors[i]->dv();
        }
    }
    break;
    default:
        break;
    }
    setTime(values, bodyRTC->controlTime());
}


void SensorStateOutPortHandler::writeDataToPort()
{
    outPort.write();
}


LinkDataOutPortHandler::LinkDataOutPortHandler(PortInfo& info)
    : OutPortHandler(info),
      outPort(info.portName.c_str(), value),
      linkNames(info.dataOwnerNames)
{
    linkDataType = info.dataTypeId;
}


RTC::OutPortBase& LinkDataOutPortHandler::getOutPort()
{
    return outPort;
}


void LinkDataOutPortHandler::inputDataFromSimulator(BodyRTCItem* bodyRTC)
{
    auto body = bodyRTC->body();
    size_t n = linkNames.size();
    switch(linkDataType) {
    case JOINT_VALUE:
        value.data.length(n);
        for(size_t i=0; i<n; i++){
            value.data[i] = body->link(linkNames[i])->q();
        }
        break;
    case JOINT_VELOCITY:
        value.data.length(n);
        for(size_t i=0; i<n; i++){
            value.data[i] = body->link(linkNames[i])->dq();
        }
        break;
    case JOINT_ACCELERATION:
        value.data.length(n);
        for(size_t i=0; i<n; i++){
            value.data[i] = body->link(linkNames[i])->ddq();
        }
        break;
    case JOINT_TORQUE:
        value.data.length(n);
        for(size_t i=0; i<n; i++){
            value.data[i] = body->link(linkNames[i])->u();
        }
        break;
    case ABS_TRANSFORM:
        value.data.length(12*n);
        for(size_t i=0, j=0; i<n; i++){
            Link* link = body->link(linkNames[i]);
            value.data[j++] = link->p()(0);
            value.data[j++] = link->p()(1);
            value.data[j++] = link->p()(2);
            const Matrix3 m33 = link->attitude();
            value.data[j++] = m33(0, 0);
            value.data[j++] = m33(0, 1);
            value.data[j++] = m33(0, 2);
            value.data[j++] = m33(1, 0);
            value.data[j++] = m33(1, 1);
            value.data[j++] = m33(1, 2);
            value.data[j++] = m33(2, 0);
            value.data[j++] = m33(2, 1);
            value.data[j++] = m33(2, 2);
        }
        break;
    case ABS_VELOCITY:
    {
        value.data.length(6*n);
        for(size_t i=0, j=0; i<n; i++){
            Link* link = body->link(linkNames[i]);
            value.data[j++] = link->v()(0);
            value.data[j++] = link->v()(1);
            value.data[j++] = link->v()(2);
            value.data[j++] = link->w()(0);
            value.data[j++] = link->w()(1);
            value.data[j++] = link->w()(2);
        }
    }
    break;
    case EXTERNAL_FORCE:
    {
        value.data.length(6*n);
        for(size_t i=0, j=0; i<n; i++){
            Link* link = body->link(linkNames[i]);
            value.data[j++] = link->f_ext()(0);
            value.data[j++] = link->f_ext()(1);
            value.data[j++] = link->f_ext()(2);
            value.data[j++] = link->tau_ext()(0);
            value.data[j++] = link->tau_ext()(1);
            value.data[j++] = link->tau_ext()(2);
        }
    }
    break;
    case CONSTRAINT_FORCE:   // linkName size must be one.
    {
        DyLink* link = dynamic_cast<DyLink*>(body->link(linkNames[0]));
        if(link){
            const vector<DyLink::ConstraintForce>& constraintForces = link->constraintForces();
            const size_t n = constraintForces.size();
            value.data.length(6*n);
            for(size_t i=0, j=0; i<n; i++){
                value.data[j++] = constraintForces[i].point(0);
                value.data[j++] = constraintForces[i].point(1);
                value.data[j++] = constraintForces[i].point(2);
                value.data[j++] = constraintForces[i].force(0);
                value.data[j++] = constraintForces[i].force(1);
                value.data[j++] = constraintForces[i].force(2);
            }
        }
    }
    break;
    default:
        break;
    }
    setTime(value, bodyRTC->controlTime());
}


void LinkDataOutPortHandler::writeDataToPort()
{
    outPort.write();
}


AbsTransformOutPortHandler::AbsTransformOutPortHandler(PortInfo& info)
    : OutPortHandler(info),
      outPort(info.portName.c_str(), value),
      linkNames(info.dataOwnerNames)
{
    linkDataType = info.dataTypeId;
}


RTC::OutPortBase& AbsTransformOutPortHandler::getOutPort()
{
    return outPort;
}


void AbsTransformOutPortHandler::inputDataFromSimulator(BodyRTCItem* bodyRTC)
{
    Link* link = bodyRTC->body()->link(linkNames[0]);
    value.data.position.x = link->p().x();
    value.data.position.y = link->p().y();
    value.data.position.z = link->p().z();
    Matrix3 R = link->attitude();
    const Vector3 rpy = rpyFromRot(R);
    value.data.orientation.r = rpy[0];
    value.data.orientation.p = rpy[1];
    value.data.orientation.y = rpy[2];
    setTime(value, bodyRTC->controlTime());
}


void AbsTransformOutPortHandler::writeDataToPort()
{
    outPort.write();
}


SensorDataOutPortHandler::SensorDataOutPortHandler(PortInfo& info)
    : OutPortHandler(info),
      outPort(info.portName.c_str(), value),
      sensorNames(info.dataOwnerNames)
{

}


RTC::OutPortBase& SensorDataOutPortHandler::getOutPort()
{
    return outPort;
}


void SensorDataOutPortHandler::inputDataFromSimulator(BodyRTCItem* bodyRTC)
{
    auto body = bodyRTC->body();
    if(!sensorNames.empty()){
        if(Device* sensor = body->findDevice(sensorNames[0])){
            const int dataSize = sensor->stateSize();
            value.data.length(dataSize);
            if(dataSize > 0){
                for(size_t i=0; i < sensorNames.size(); ++i){
                    if(Device* sensor = body->findDevice(sensorNames[i])){
                        sensor->writeState(&value.data[i * dataSize]);
                    }
                }
            }
        }
    }
    setTime(value, bodyRTC->controlTime());
}


void SensorDataOutPortHandler::writeDataToPort()
{
    outPort.write();
}


GyroSensorOutPortHandler::GyroSensorOutPortHandler(PortInfo& info)
    : OutPortHandler(info),
      outPort(info.portName.c_str(), value),
      sensorNames(info.dataOwnerNames)
{

}


RTC::OutPortBase& GyroSensorOutPortHandler::getOutPort()
{
    return outPort;
}


void GyroSensorOutPortHandler::inputDataFromSimulator(BodyRTCItem* bodyRTC)
{
    if(!sensorNames.empty()){
        if(RateGyroSensor* gyroSensor =bodyRTC->body()->findDevice<RateGyroSensor>(sensorNames[0])){
            value.data.avx = gyroSensor->w().x();
            value.data.avy = gyroSensor->w().y();
            value.data.avz = gyroSensor->w().z();
            setTime(value, bodyRTC->controlTime());
        }
    }
}


void GyroSensorOutPortHandler::writeDataToPort()
{
    outPort.write();
}


AccelerationSensorOutPortHandler::AccelerationSensorOutPortHandler(PortInfo& info)
    : OutPortHandler(info),
      outPort(info.portName.c_str(), value),
      sensorNames(info.dataOwnerNames)
{

}


RTC::OutPortBase& AccelerationSensorOutPortHandler::getOutPort()
{
    return outPort;
}


void AccelerationSensorOutPortHandler::inputDataFromSimulator(BodyRTCItem* bodyRTC)
{
    if(!sensorNames.empty()){
        if(AccelerationSensor* accelSensor = bodyRTC->body()->findDevice<AccelerationSensor>(sensorNames[0])){
            value.data.ax = accelSensor->dv().x();
            value.data.ay = accelSensor->dv().y();
            value.data.az = accelSensor->dv().z();
            setTime(value, bodyRTC->controlTime());
        }
    }
}


void AccelerationSensorOutPortHandler::writeDataToPort()
{
    outPort.write();
}


LightOnOutPortHandler::LightOnOutPortHandler(PortInfo& info)
    : OutPortHandler(info),
      outPort(info.portName.c_str(), value),
      lightNames(info.dataOwnerNames)
{

}


RTC::OutPortBase& LightOnOutPortHandler::getOutPort()
{
    return outPort;
}


void LightOnOutPortHandler::inputDataFromSimulator(BodyRTCItem* bodyRTC)
{
    auto body = bodyRTC->body();
    value.data.length(lightNames.size());
    size_t i=0;
    for(vector<string>::iterator it = lightNames.begin(); it != lightNames.end(); it++){
        Light* light = body->findDevice<Light>(*it);
        value.data[i++] = light->on(); 
    }
    
    setTime(value, bodyRTC->controlTime());
}


void LightOnOutPortHandler::writeDataToPort()
{
    outPort.write();
}


namespace cnoid {

class CameraImageOutPortHandlerImpl
{
public:
    CameraImageOutPortHandler* self;
    Img::TimedCameraImage value;
    RTC::OutPort<Img::TimedCameraImage> outPort;
    std::mutex mtx;
    Camera* camera;
    std::string cameraName;
    std::shared_ptr<const Image> prevImage;
    double controlTime;

    CameraImageOutPortHandlerImpl(CameraImageOutPortHandler* self, PortInfo& info, bool synchController);
    void initialize(Body* simBody);
    void onCameraStateChanged();
};

}


CameraImageOutPortHandler::CameraImageOutPortHandler(PortInfo& info, bool synchController)
    : OutPortHandler(info)
{
    impl = new CameraImageOutPortHandlerImpl(this, info, synchController);
}


CameraImageOutPortHandlerImpl::CameraImageOutPortHandlerImpl(CameraImageOutPortHandler* self, PortInfo& info, bool synchController)
    : self(self),
      outPort(info.portName.c_str(), value)
{
    if(!info.dataOwnerNames.empty()){
        cameraName = info.dataOwnerNames.at(0);
    } else {
        cameraName.clear();
    }
    value.data.image.format = Img::CF_UNKNOWN;
}


RTC::OutPortBase& CameraImageOutPortHandler::getOutPort()
{
    return impl->outPort;
}


void CameraImageOutPortHandler::initialize(Body* simBody)
{
    impl->initialize(simBody);
}


void CameraImageOutPortHandlerImpl::initialize(Body* simBody)
{
    camera = 0;
    if(!cameraName.empty()){
        camera = simBody->findDevice<Camera>(cameraName);
    }else{
        DeviceList<Camera> cameras(simBody->devices());
        if(!cameras.empty())
            camera = cameras[0];
    }

    if(camera){
        double fovy2 = camera->fieldOfView() / 2.0;
        double width = camera->resolutionX();
        double height = camera->resolutionY();
        double minlength = std::min(width, height);
        double fu, fv;
        fv = fu = minlength / tan(fovy2) / 2.0;
        double u0 = (width - 1)/2.0;
        double v0 = (height - 1)/2.0;

        value.data.intrinsic.matrix_element[0] = fu;
        value.data.intrinsic.matrix_element[1] = 0.0;
        value.data.intrinsic.matrix_element[2] = u0;
        value.data.intrinsic.matrix_element[3] = fv;
        value.data.intrinsic.matrix_element[4] = v0;
        value.data.intrinsic.distortion_coefficient.length(5);
        for (int i=0; i<5; i++)
          value.data.intrinsic.distortion_coefficient[i] = 0.0; // zero distortion is natural in simulator
        for(int i=0; i<4; i++)
            for(int j=0; j<4; j++)
                value.data.extrinsic[i][j] = i==j? 1.0: 0.0;

        camera->sigStateChanged().connect(std::bind(&CameraImageOutPortHandlerImpl::onCameraStateChanged, this));
    }
}


void CameraImageOutPortHandlerImpl::onCameraStateChanged()
{
    if(camera->sharedImage() != prevImage){
        const Image& image = camera->constImage();
        if(!image.empty()){
            int width,height;
            value.data.image.height = height = image.height();
            value.data.image.width = width = image.width();
            switch(camera->imageType()){
            case Camera::GRAYSCALE_IMAGE : value.data.image.format = Img::CF_GRAY;
                break;
            case Camera::COLOR_IMAGE : value.data.image.format = Img::CF_RGB;
                break;
            default : value.data.image.format = Img::CF_UNKNOWN;
                break;
            }
            size_t length = width * height * image.numComponents() * sizeof(unsigned char);
            value.data.image.raw_data.length(length);
            void* src = (void*)image.pixels();
            void* dis = (void*)value.data.image.raw_data.get_buffer();
            memcpy(dis, src, length);
        }
        prevImage = camera->sharedImage();
        std::lock_guard<std::mutex> lock(mtx);
        self->setTime(value, controlTime - camera->delay());
        outPort.write();
    }
}


void CameraImageOutPortHandler::inputDataFromSimulator(BodyRTCItem* bodyRTC)
{
    std::lock_guard<std::mutex> lock(impl->mtx);
    impl->controlTime = bodyRTC->controlTime();
}


void CameraImageOutPortHandler::writeDataToPort()
{

}


namespace cnoid {

class CameraRangeOutPortHandlerImpl
{
public:
    CameraRangeOutPortHandler* self;
    PointCloudTypes::PointCloud value;
    RTC::OutPort<PointCloudTypes::PointCloud> outPort;
    std::mutex mtx;
    RangeCamera* rangeCamera;
    std::string rangeCameraName;
    std::shared_ptr<const RangeCamera::PointData> prevPoints;
    std::shared_ptr<const Image> image;
    std::string format;
    double controlTime;

    CameraRangeOutPortHandlerImpl(CameraRangeOutPortHandler* self, PortInfo& info, bool synchController);
    void initialize(Body* simBody);
    void onCameraStateChanged();
};

}


CameraRangeOutPortHandler::CameraRangeOutPortHandler(PortInfo& info, bool synchController)
    : OutPortHandler(info)
{
    impl = new CameraRangeOutPortHandlerImpl(this, info, synchController);
}


CameraRangeOutPortHandlerImpl::CameraRangeOutPortHandlerImpl
(CameraRangeOutPortHandler* self, PortInfo& info, bool synchController)
    : self(self),
      outPort(info.portName.c_str(), value)
{
    if(!info.dataOwnerNames.empty()){
        rangeCameraName = info.dataOwnerNames.at(0);
    } else { 
        rangeCameraName.clear();
    }
}


CameraRangeOutPortHandler::~CameraRangeOutPortHandler()
{
    delete impl;
}


RTC::OutPortBase& CameraRangeOutPortHandler::getOutPort()
{
    return impl->outPort;
}


void CameraRangeOutPortHandler::initialize(Body* simBody)
{
    impl->initialize(simBody);
}


void CameraRangeOutPortHandlerImpl::initialize(Body* simBody)
{
    rangeCamera = nullptr;
    if(!rangeCameraName.empty()){
        rangeCamera = simBody->findDevice<RangeCamera>(rangeCameraName);
    }else{
        DeviceList<RangeCamera> cameras = simBody->devices();
        if(!cameras.empty())
            rangeCamera = cameras[0];
    }

    if(rangeCamera){
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
        value.type = CORBA::string_dup(format.c_str());

        if (format == "xyz"){
            value.fields.length(3);
        }else if (format == "xyzrgb"){
            value.fields.length(6);
        }
        value.fields[0].name = "x";
        value.fields[0].offset = 0;
        value.fields[0].data_type = PointCloudTypes::FLOAT32;
        value.fields[0].count = 4;
        value.fields[1].name = "y";
        value.fields[1].offset = 4;
        value.fields[1].data_type = PointCloudTypes::FLOAT32;
        value.fields[1].count = 4;
        value.fields[2].name = "z";
        value.fields[2].offset = 8;
        value.fields[2].data_type = PointCloudTypes::FLOAT32;
        value.fields[2].count = 4;
        value.point_step = 12;
        if (format == "xyzrgb"){
            value.fields[3].name = "r";
            value.fields[3].offset = 12;
            value.fields[3].data_type = PointCloudTypes::UINT8;
            value.fields[3].count = 1;
            value.fields[4].name = "g";
            value.fields[4].offset = 13;
            value.fields[4].data_type = PointCloudTypes::UINT8;
            value.fields[4].count = 1;
            value.fields[5].name = "b";
            value.fields[5].offset = 14;
            value.fields[5].data_type = PointCloudTypes::UINT8;
            value.fields[5].count = 1;
            value.point_step = 16;
        }
        // Originally is_bigendian has to be decided by CPU difference.
        value.is_bigendian = false;
        value.is_dense = true;
        
        rangeCamera->sigStateChanged().connect(std::bind(&CameraRangeOutPortHandlerImpl::onCameraStateChanged, this));
    }
}


void CameraRangeOutPortHandlerImpl::onCameraStateChanged()
{
    if(rangeCamera->sharedPoints() != prevPoints){
        const vector<Vector3f>& points = rangeCamera->constPoints(); 
        const Image& image = rangeCamera->constImage();

        if(!points.empty()){
            if(!image.empty()){
                value.height = image.height();
                value.width = image.width();
            }else{
                if(rangeCamera->isOrganized()){
                    value.height = rangeCamera->resolutionY();
                    value.width = rangeCamera->resolutionX();
                }else{
                    value.height = 1;
                    value.width = rangeCamera->numPoints();
                }
            }
            value.row_step = value.point_step * value.width;
            value.is_dense = rangeCamera->isDense();
            size_t length = points.size() * value.point_step;
            value.data.length(length);
            unsigned char* dis = (unsigned char*)value.data.get_buffer();
            const unsigned char* pixels = 0;
            if(!image.empty())
                pixels = image.pixels();
            for(size_t i=0; i < points.size(); i++, dis += value.point_step){
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
        std::lock_guard<std::mutex> lock(mtx);
        self->setTime(value, controlTime - rangeCamera->delay());
        outPort.write();
    }
}


void CameraRangeOutPortHandler::inputDataFromSimulator(BodyRTCItem* bodyRTC)
{
    std::lock_guard<std::mutex> lock(impl->mtx);
    impl->controlTime = bodyRTC->controlTime();
}


void CameraRangeOutPortHandler::writeDataToPort()
{

}


RangeSensorOutPortHandler::RangeSensorOutPortHandler(PortInfo& info, bool synchController) :
    OutPortHandler(info),
    outPort(info.portName.c_str(), value)
{
    if(!info.dataOwnerNames.empty())
        rangeSensorName = info.dataOwnerNames.at(0);
    else
        rangeSensorName.clear();
}


RTC::OutPortBase& RangeSensorOutPortHandler::getOutPort()
{
    return outPort;
}


void RangeSensorOutPortHandler::initialize(Body* simBody)
{
    rangeSensor = 0;
    if(!rangeSensorName.empty()){
        rangeSensor = simBody->findDevice<RangeSensor>(rangeSensorName);
    }else{
        DeviceList<RangeSensor> rangeSensors = simBody->devices();
        if(!rangeSensors.empty())
            rangeSensor = rangeSensors[0];
    }

    if(rangeSensor){
        value.config.minAngle = -rangeSensor->yawRange() / 2.0;
        value.config.maxAngle = rangeSensor->yawRange() / 2.0;
        value.config.angularRes = rangeSensor->yawStep();
        value.config.minRange = rangeSensor->minDistance();
        value.config.maxRange = rangeSensor->maxDistance();
        value.config.frequency = rangeSensor->frameRate();
        value.config.rangeRes = 0;     // no data
        // Ignore geometry
        value.geometry.geometry.pose.orientation.r = 0.0;
        value.geometry.geometry.pose.orientation.p = 0.0;
        value.geometry.geometry.pose.orientation.y = 0.0;
        value.geometry.geometry.pose.position.x = 0.0;
        value.geometry.geometry.pose.position.y = 0.0;
        value.geometry.geometry.pose.position.z = 0.0;
        value.geometry.geometry.size.w = 0.0;
        value.geometry.geometry.size.l = 0.0;
        value.geometry.geometry.size.h = 0.0;
        value.geometry.elementGeometries.length(0);
        //
        rangeSensor->sigStateChanged().connect(std::bind(&RangeSensorOutPortHandler::onRangeSensorStateChanged, this));
    }
}


void RangeSensorOutPortHandler::onRangeSensorStateChanged()
{
    if(rangeSensor->sharedRangeData() != prevRangeData){
        const RangeSensor::RangeData& src = rangeSensor->constRangeData();
        value.ranges.length(src.size());
        for(size_t i=0; i < src.size(); i++)
            value.ranges[i] = src[i];

        prevRangeData = rangeSensor->sharedRangeData();
        std::lock_guard<std::mutex> lock(mtx);
        setTime(value, controlTime - rangeSensor->delay());
        outPort.write();
    }
}


void RangeSensorOutPortHandler::inputDataFromSimulator(BodyRTCItem* bodyRTC)
{
    std::lock_guard<std::mutex> lock(mtx);
    controlTime = bodyRTC->controlTime();
}


void RangeSensorOutPortHandler::writeDataToPort()
{

}


JointDataSeqInPortHandler::JointDataSeqInPortHandler(PortInfo& info) :
    InPortHandler(info),
    inPort(info.portName.c_str(), values)
{
    linkDataType = info.dataTypeId;
}


void JointDataSeqInPortHandler::outputDataToSimulator(const BodyPtr& body)
{
    const int n = std::min(body->numJoints(), (int)(values.data.length()));
    //! \todo put warning if the number of joints is different from the port data size

    switch(linkDataType) {
    case JOINT_VALUE:
        for(int i=0; i < n; i++){
            body->joint(i)->q_target() = values.data[i];
        }
        break;
    case JOINT_VELOCITY:
        for(int i=0; i < n; i++){
            body->joint(i)->dq_target() = values.data[i];
        }
        break;
    case JOINT_ACCELERATION:
        for(int i=0; i<n; i++){
            body->joint(i)->ddq() = values.data[i];
        }
        break;
    case JOINT_TORQUE:
        for(int i=0; i < n; i++){
            body->joint(i)->u() = values.data[i];
        }
        break;
    default :
        break;
    }
}


void JointDataSeqInPortHandler::readDataFromPort()
{
    if( !inPort.isNew() ){
        values.data.length(0);
    }else
        inPort.read();
}

LinkDataInPortHandler::LinkDataInPortHandler(PortInfo& info) :
    InPortHandler(info),
    inPort(info.portName.c_str(), values),
    linkNames(info.dataOwnerNames)
{
    linkDataType = info.dataTypeId;
}

void LinkDataInPortHandler::outputDataToSimulator(const BodyPtr& body)
{
    size_t n=linkNames.size();
    if(!values.data.length())
        return;
    switch(linkDataType) {
    case JOINT_VALUE:
        for(size_t i=0; i<n; i++)
            body->link(linkNames[i])->q_target() = values.data[i];
        break;
    case JOINT_VELOCITY:
        for(size_t i=0; i<n; i++)
            body->link(linkNames[i])->dq_target() = values.data[i];
        break;
    case JOINT_ACCELERATION:
        for(size_t i=0; i<n; i++)
            body->link(linkNames[i])->ddq() = values.data[i];
        break;
    case JOINT_TORQUE:
        for(size_t i=0; i<n; i++)
            body->link(linkNames[i])->u() = values.data[i];
        break;
    case ABS_TRANSFORM:
        for(size_t i=0; i<n; i++){
            Link* link = body->link(linkNames[i]);
            link->p() =  Vector3( values.data[12*i], values.data[12*i+1], values.data[12*i+2]);
            Matrix3 R;
            R << values.data[12*i+3], values.data[12*i+4], values.data[12*i+5],
                values.data[12*i+6], values.data[12*i+7], values.data[12*i+8],
                values.data[12*i+9], values.data[12*i+10], values.data[12*i+11];
            link->setAttitude(R);
        }
        break;
    case ABS_VELOCITY:
        for(size_t i=0; i<n; i++){
            Link* link = body->link(linkNames[i]);
            link->v() = Vector3( values.data[6*i], values.data[6*i+1], values.data[6*i+2]);
            link->w() = Vector3( values.data[6*i+3], values.data[6*i+4], values.data[6*i+5]);
        }
        break;
    case EXTERNAL_FORCE:
        for(size_t i=0; i<n; i++){
            Link* link = body->link(linkNames[i]);
            link->f_ext() = Vector3( values.data[6*i], values.data[6*i+1], values.data[6*i+2]);
            link->tau_ext() = Vector3( values.data[6*i+3], values.data[6*i+4], values.data[6*i+5]);
        }
    default :
        break;
    }
}

void LinkDataInPortHandler::readDataFromPort()
{
    if( !inPort.isNew() ){
        values.data.length(0);
    }else
        inPort.read();
}

AbsTransformInPortHandler::AbsTransformInPortHandler(PortInfo& info) :
    InPortHandler(info),
    inPort(info.portName.c_str(), values),
    linkNames(info.dataOwnerNames)
{
    linkDataType = info.dataTypeId;
}


void AbsTransformInPortHandler::outputDataToSimulator(const BodyPtr& body)
{
    Link* link = body->link(linkNames[0]);
    link->p().x() = values.data.position.x;
    link->p().y() = values.data.position.y;
    link->p().z() = values.data.position.z;
    Matrix3 R = rotFromRpy(values.data.orientation.r,
                           values.data.orientation.p,
                           values.data.orientation.y);
    link->setAttitude(R);
    
}

void AbsTransformInPortHandler::readDataFromPort()
{
    if( !inPort.isNew() ){
        return;
    }else
        inPort.read();
}

LightOnInPortHandler::LightOnInPortHandler(PortInfo& info) :
    InPortHandler(info),
    inPort(info.portName.c_str(), values),
    lightNames(info.dataOwnerNames)
{

}

void LightOnInPortHandler::outputDataToSimulator(const BodyPtr& body)
{
    for(size_t i=0; i<values.data.length(); i++){
        Light* light = body->findDevice<Light>(lightNames[i]);
        light->on(values.data[i]);
        light->notifyStateChange();
    }
}

void LightOnInPortHandler::readDataFromPort()
{
    if( !inPort.isNew() ){
        values.data.length(0);
    }else
        inPort.read();
}
