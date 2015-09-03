/**
   \file
   \author Shizuko Hattori
*/

#include "VirtualRobotPortHandler.h"
#include "BodyRTCItem.h"
#include <cnoid/EigenUtil>
#include <cnoid/DyBody>
#include <boost/bind.hpp>

using namespace std;
using namespace RTC;
using namespace cnoid;


PortHandler::~PortHandler()
{

}


SensorStateOutPortHandler::SensorStateOutPortHandler(PortInfo& info) : 
    OutPortHandler(info), 
    outPort(info.portName.c_str(), values)
{
    dataTypeId = info.dataTypeId;
    stepTime = info.stepTime;
}

void SensorStateOutPortHandler::inputDataFromSimulator(BodyRTCItem* bodyRTC)
{
    const BodyPtr& body = bodyRTC->body();
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
        const DeviceList<RateGyroSensor>& gyroSensors = bodyRTC->gyroSensors();
        const int n = gyroSensors.size();
        values.data.length(3 * n);
        for(int i=0; i < n; i++){
            Eigen::Map<Vector3>(&values.data[i*3]) = gyroSensors[i]->w();
        }
    }
    break;
    case ACCELERATION_SENSOR:
    {
        const DeviceList<AccelSensor>& accelSensors = bodyRTC->accelSensors();
        const int n = accelSensors.size();
        values.data.length(3 * n);
        for(int i=0; i < n; i++){
            Eigen::Map<Vector3>(&values.data[i*3]) = accelSensors[i]->dv();
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


LinkDataOutPortHandler::LinkDataOutPortHandler(PortInfo& info) :
    OutPortHandler(info),
    outPort(info.portName.c_str(), value),
    linkNames(info.dataOwnerNames)
{
    linkDataType = info.dataTypeId;
    stepTime = info.stepTime;
}


void LinkDataOutPortHandler::inputDataFromSimulator(BodyRTCItem* bodyRTC)
{
    const BodyPtr& body = bodyRTC->body();
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

AbsTransformOutPortHandler::AbsTransformOutPortHandler(PortInfo& info) :
    OutPortHandler(info),
    outPort(info.portName.c_str(), value),
    linkNames(info.dataOwnerNames)
{
    linkDataType = info.dataTypeId;
    stepTime = info.stepTime;
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


SensorDataOutPortHandler::SensorDataOutPortHandler(PortInfo& info) :
    OutPortHandler(info),
    outPort(info.portName.c_str(), value),
    sensorNames(info.dataOwnerNames)
{
    stepTime = info.stepTime;
}


void SensorDataOutPortHandler::inputDataFromSimulator(BodyRTCItem* bodyRTC)
{
    const BodyPtr& body = bodyRTC->body();
    if(!sensorNames.empty()){
        if(Sensor* sensor = body->findDevice<Sensor>(sensorNames[0])){
            const int dataSize = sensor->stateSize();
            value.data.length(dataSize);
            if(dataSize > 0){
                for(size_t i=0; i < sensorNames.size(); ++i){
                    if(Sensor* sensor = body->findDevice<Sensor>(sensorNames[i])){
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


GyroSensorOutPortHandler::GyroSensorOutPortHandler(PortInfo& info) :
    OutPortHandler(info),
    outPort(info.portName.c_str(), value),
    sensorNames(info.dataOwnerNames)
{
    stepTime = info.stepTime;
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


AccelerationSensorOutPortHandler::AccelerationSensorOutPortHandler(PortInfo& info) :
    OutPortHandler(info),
    outPort(info.portName.c_str(), value),
    sensorNames(info.dataOwnerNames)
{
    stepTime = info.stepTime;
}


void AccelerationSensorOutPortHandler::inputDataFromSimulator(BodyRTCItem* bodyRTC)
{
    if(!sensorNames.empty()){
        if(AccelSensor* accelSensor = bodyRTC->body()->findDevice<AccelSensor>(sensorNames[0])){
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


LightOnOutPortHandler::LightOnOutPortHandler(PortInfo& info) :
    OutPortHandler(info),
    outPort(info.portName.c_str(), value),
    lightNames(info.dataOwnerNames)
{
    stepTime = info.stepTime;
}


void LightOnOutPortHandler::inputDataFromSimulator(BodyRTCItem* bodyRTC)
{
    const BodyPtr& body = bodyRTC->body();
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


CameraImageOutPortHandler::CameraImageOutPortHandler(PortInfo& info, bool synchController) :
    OutPortHandler(info),
    outPort(info.portName.c_str(), value)
{
    stepTime = info.stepTime;
    if(!info.dataOwnerNames.empty())
        cameraName = info.dataOwnerNames.at(0);
    else
        cameraName.clear();

    value.data.image.format = Img::CF_UNKNOWN;
}


void CameraImageOutPortHandler::initialize(Body* simBody)
{
    camera = 0;
    if(!cameraName.empty()){
        camera = simBody->findDevice<Camera>(cameraName);
    }else{
        DeviceList<Camera> cameras = simBody->devices();
        if(!cameras.empty())
            camera = cameras[0];
    }

    if(camera){
        double fovy2 = camera->fieldOfView() / 2.0;
        double near = camera->nearDistance();
        double width = camera->resolutionX();
        double height = camera->resolutionY();
        double aspect = width / height;
        double fv = 1.0 / tan(fovy2);
        double fu = fv / aspect;
        double v0 = near * tan(fovy2);
        double u0 = v0 * aspect;
        value.data.intrinsic.matrix_element[0] = fu;
        value.data.intrinsic.matrix_element[1] = 0.0;
        value.data.intrinsic.matrix_element[2] = u0;
        value.data.intrinsic.matrix_element[3] = fv;
        value.data.intrinsic.matrix_element[4] = v0;
        value.data.intrinsic.distortion_coefficient.length(0);
        for(int i=0; i<4; i++)
            for(int j=0; j<4; j++)
                value.data.extrinsic[i][j] = i==j? 1.0: 0.0;

        camera->sigStateChanged().connect(boost::bind(&CameraImageOutPortHandler::onCameraStateChanged, this));
    }
}


void CameraImageOutPortHandler::onCameraStateChanged()
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
        boost::lock_guard<boost::mutex> lock(mtx);
        setTime(value, controlTime - camera->delay());
        outPort.write();
    }
}


void CameraImageOutPortHandler::inputDataFromSimulator(BodyRTCItem* bodyRTC)
{
    boost::lock_guard<boost::mutex> lock(mtx);
    controlTime = bodyRTC->controlTime();
}


void CameraImageOutPortHandler::writeDataToPort()
{

}


CameraRangeOutPortHandler::CameraRangeOutPortHandler(PortInfo& info, bool synchController) :
    OutPortHandler(info),
    outPort(info.portName.c_str(), value)
{
    stepTime = info.stepTime;
    if(!info.dataOwnerNames.empty())
        rangeCameraName = info.dataOwnerNames.at(0);
    else
        rangeCameraName.clear();
}

void CameraRangeOutPortHandler::initialize(Body* simBody)
{
    rangeCamera = 0;
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

        bool colored = false;
        if (format == "xyz"){
            value.fields.length(3);
        }else if (format == "xyzrgb"){
            value.fields.length(6);
            colored = true;
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
        value.is_bigendian = false;
        value.is_dense = true;
        
        rangeCamera->sigStateChanged().connect(boost::bind(&CameraRangeOutPortHandler::onCameraStateChanged, this));
    }

}


void CameraRangeOutPortHandler::onCameraStateChanged()
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
            size_t length = points.size() * value.point_step;
            value.data.length(length);
            unsigned char* dis = (unsigned char*)value.data.get_buffer();
            const unsigned char* pixels = 0;
            if(!image.empty())
                pixels = image.pixels();
            for(int i=0; i<points.size(); i++, dis+=value.point_step){
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
        boost::lock_guard<boost::mutex> lock(mtx);
        setTime(value, controlTime - rangeCamera->delay());
        outPort.write();
    }
}


void CameraRangeOutPortHandler::inputDataFromSimulator(BodyRTCItem* bodyRTC)
{
    boost::lock_guard<boost::mutex> lock(mtx);
    controlTime = bodyRTC->controlTime();
}

void CameraRangeOutPortHandler::writeDataToPort()
{

}


RangeSensorOutPortHandler::RangeSensorOutPortHandler(PortInfo& info, bool synchController) :
    OutPortHandler(info),
    outPort(info.portName.c_str(), value)
{
    stepTime = info.stepTime;
    if(!info.dataOwnerNames.empty())
        rangeSensorName = info.dataOwnerNames.at(0);
    else
        rangeSensorName.clear();
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
        rangeSensor->sigStateChanged().connect(boost::bind(&RangeSensorOutPortHandler::onRangeSensorStateChanged, this));
    }
}


void RangeSensorOutPortHandler::onRangeSensorStateChanged()
{
    if(rangeSensor->sharedRangeData() != prevRangeData){
        const RangeSensor::RangeData& src = rangeSensor->constRangeData();
        value.ranges.length(src.size());
        for(int i=0; i<src.size(); i++)
            value.ranges[i] = src[i];

        prevRangeData = rangeSensor->sharedRangeData();
        boost::lock_guard<boost::mutex> lock(mtx);
        setTime(value, controlTime - rangeSensor->delay());
        outPort.write();
    }
}


void RangeSensorOutPortHandler::inputDataFromSimulator(BodyRTCItem* bodyRTC)
{
    boost::lock_guard<boost::mutex> lock(mtx);
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
            body->joint(i)->q() = values.data[i];
        }
        break;
    case JOINT_VELOCITY:
        for(int i=0; i < n; i++){
            body->joint(i)->dq() = values.data[i];
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
    if( inPort.isNew() == false ){
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
            body->link(linkNames[i])->q() = values.data[i];
        break;
    case JOINT_VELOCITY:
        for(size_t i=0; i<n; i++)
            body->link(linkNames[i])->dq() = values.data[i];
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
    if( inPort.isNew() == false ){
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
    if( inPort.isNew() == false ){
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
    if( inPort.isNew() == false ){
        values.data.length(0);
    }else
        inPort.read();
}
