/*!
  @file
  @author Shizuko Hattori
*/

#include "BodyInfo_impl.h"
#include "ModelLoaderUtil.h"
#include <cnoid/Camera>
#include <cnoid/RateGyroSensor>
#include <cnoid/AccelerationSensor>
#include <cnoid/RangeSensor>
#include <cnoid/SpotLight>
#include <cnoid/ValueTree>
#include <cnoid/EigenArchive>
#include <boost/algorithm/string.hpp>

using namespace std;
using namespace cnoid;
using namespace OpenHRP;
using namespace cnoid::openHRPModelloader;

namespace {

typedef vector<string> JointTypeMap;
JointTypeMap jointTypeMap;
typedef map<string,string> SensorTypeMap;
SensorTypeMap sensorTypeMap;
typedef vector<int> CameraTypeMap;
CameraTypeMap cameraTypeMap, rangeCameraTypeMap;
typedef map<string, OpenHRP::LightType> LightTypeMap;
LightTypeMap lightTypeMap;
typedef map<ExtraJoint::ExtraJointType, OpenHRP::ExtraJointType> ExtraJointTypeMap;
ExtraJointTypeMap extraJointTypeMap;

}

BodyInfo_impl::BodyInfo_impl(PortableServer::POA_ptr poa) :
    ShapeSetInfo_impl(poa)
{
    os = &cout;

    if(jointTypeMap.empty()){
        jointTypeMap.resize(5);
        jointTypeMap[Link::REVOLUTE_JOINT]          = "rotate";
        jointTypeMap[Link::PRISMATIC_JOINT]         = "slide";
        jointTypeMap[Link::FREE_JOINT]              = "free";
        jointTypeMap[Link::FIXED_JOINT]             = "fixed";
        jointTypeMap[Link::PSEUDO_CONTINUOUS_TRACK] = "clawler";
    }

    if(sensorTypeMap.empty()){
        sensorTypeMap["ForceSensor"]        = "Force";
        sensorTypeMap["RateGyroSensor"]     = "RateGyro";
        sensorTypeMap["AccelerationSensor"] = "Acceleration";
        sensorTypeMap["Camera"]             = "Vision";
        sensorTypeMap["RangeCamera"]        = "Vision";
        sensorTypeMap["RangeSensor"]        = "Range";
    }

    if(cameraTypeMap.empty()){
        cameraTypeMap.resize(3);
        cameraTypeMap[Camera::NO_IMAGE]        = OpenHRP::Camera::NONE;
        cameraTypeMap[Camera::COLOR_IMAGE]     = OpenHRP::Camera::COLOR;
        cameraTypeMap[Camera::GRAYSCALE_IMAGE] = OpenHRP::Camera::MONO;
    }

    if(rangeCameraTypeMap.empty()){
        rangeCameraTypeMap.resize(3);
        rangeCameraTypeMap[Camera::NO_IMAGE]        = OpenHRP::Camera::DEPTH;
        rangeCameraTypeMap[Camera::COLOR_IMAGE]     = OpenHRP::Camera::COLOR_DEPTH;
        rangeCameraTypeMap[Camera::GRAYSCALE_IMAGE] = OpenHRP::Camera::MONO_DEPTH;
    }

    if(lightTypeMap.empty()){
        lightTypeMap["PointLight"] = OpenHRP::POINT;
        lightTypeMap["SpotLight"]  = OpenHRP::SPOT;
    }

    if(extraJointTypeMap.empty()){
        extraJointTypeMap[ExtraJoint::EJ_BALL] = OpenHRP::ExtraJointType::EJ_XYZ;
        extraJointTypeMap[ExtraJoint::EJ_PISTON] = OpenHRP::ExtraJointType::EJ_XY;
    }

}


BodyInfo_impl::~BodyInfo_impl()
{

}


char* BodyInfo_impl::name()
{
    return CORBA::string_dup(name_.c_str());
}


char* BodyInfo_impl::url()
{
    return CORBA::string_dup(url_.c_str());
}


StringSequence* BodyInfo_impl::info()
{
    return new StringSequence(info_);
}


LinkInfoSequence* BodyInfo_impl::links()
{
    return new LinkInfoSequence(links_);
}


AllLinkShapeIndexSequence* BodyInfo_impl::linkShapeIndices()
{
    return new AllLinkShapeIndexSequence(linkShapeIndices_);
}

ExtraJointInfoSequence* BodyInfo_impl::extraJoints()
{
    return new ExtraJointInfoSequence(extraJoints_);
}


void BodyInfo_impl::loadModelFile(const std::string& url)
{
    BodyLoader loader;
    loader.setMessageSink(*os);

    body = loader.load(url);
    if(!body){
        throw ModelLoader::ModelLoaderException("The model file cannot be loaded.");
        return ;
    }

    url_ = url;
    name_ = body->modelName();

    string stringInfo;
    if(body->info()->read("humanoid info", stringInfo)){
        vector<string> infos;
        boost::split(infos, stringInfo, boost::is_any_of("\n"));
        info_.length(infos.size());
        for(int i=0; i<infos.size(); i++){
            info_[i] = CORBA::string_dup( infos[i].c_str() );
        }
    }

    setLinks();
    setDevices();

    linkShapeIndices_.length(body->numLinks());
    for(int i = 0 ; i < body->numLinks() ; ++i) {
        linkShapeIndices_[i] = links_[i].shapeIndices;
    }

    setExtraJoints();

}


void BodyInfo_impl::setExtraJoints()
{
    int n = body->numExtraJoints();
    extraJoints_.length(n);
    for(int i=0; i<n; i++){
        setExtraJoint(extraJoints_[i], body->extraJoint(i));
    }
}


void BodyInfo_impl::setExtraJoint(ExtraJointInfo& extraJointInfo, ExtraJoint& joint)
{
    extraJointInfo.name = "";
    extraJointInfo.jointType = extraJointTypeMap[joint.type];
    setVector3(joint.axis, extraJointInfo.axis);
    extraJointInfo.link[0] = CORBA::string_dup(joint.linkName[0].c_str());
    extraJointInfo.link[1] = CORBA::string_dup(joint.linkName[1].c_str());
    setVector3(joint.point[0], extraJointInfo.point[0]);
    setVector3(joint.point[1], extraJointInfo.point[1]);
}


void BodyInfo_impl::setLinks()
{
    int numLinks = body->numLinks();
    links_.length(numLinks);

    for(int i=0; i<numLinks; i++){
        LinkInfo& linkInfo = links_[i];
        Link& link = *body->link(i);

        setLink(linkInfo, link);
        setSegment(linkInfo, link);
        setHwc(linkInfo, link);
    }

    //In Choreonoid Body, do not set jointValue for each Link, but use standardPose to make settings.
    const Listing& pose = *body->info()->findListing("standardPose");
    if(pose.isValid()){
        const int n = std::min(pose.size(), body->numJoints());
        for(int jointIndex=0; jointIndex < n; jointIndex++){
            int i = body->joint(jointIndex)->index();
            LinkInfo& linkInfo = links_[i];
            linkInfo.jointValue = radian(pose[jointIndex].toDouble());   //TODO radian?
        }
    }

}


void BodyInfo_impl::setLink(LinkInfo& linkInfo, Link& link)
{
    if(link.isRoot()){
        linkInfo.parentIndex = -1;
    }else{
        linkInfo.parentIndex = link.parent()->index();
    }

    for(Link* link_ = link.child(); link_; link_ = link_->sibling()){
        int n = linkInfo.childIndices.length();
        linkInfo.childIndices.length( n + 1 );
        linkInfo.childIndices[n] = link_->index();
    }

    linkInfo.name =  CORBA::string_dup( link.name().c_str() );
    linkInfo.jointId = link.jointId();
    const Vector3& axis = link.jointAxis();
    setVector3(axis, linkInfo.jointAxis);

    string jointType = jointTypeMap[link.jointType()];
    linkInfo.jointType = CORBA::string_dup( jointType.c_str() );

    if(!link.isRoot()){
        Matrix3 parentRsi = link.parent()->Rs().transpose();
        Vector3 b_ =  parentRsi * link.b();
        setVector3(b_, linkInfo.translation);
        Matrix3 R_ = parentRsi * link.Rs();
        setMatrix3(R_, linkInfo.rotation);
    }else{
        setVector3( Vector3::Zero(), linkInfo.translation);
        setMatrix3( Matrix3::Identity(), linkInfo.rotation);
    }

    linkInfo.ulimit.length(CORBA::ULong(1));
    linkInfo.ulimit[0] = link.q_upper();
    linkInfo.llimit.length(CORBA::ULong(1));
    linkInfo.llimit[0] = link.q_lower();
    linkInfo.uvlimit.length(CORBA::ULong(1));
    linkInfo.uvlimit[0] = link.dq_upper();
    linkInfo.lvlimit.length(CORBA::ULong(1));
    linkInfo.lvlimit[0] = link.dq_lower();

    double w;
    if(link.info()->read("climit", w)){
        if(w == numeric_limits<double>::max()){
            linkInfo.climit.length(CORBA::ULong(0));
        }else{
            linkInfo.climit.length(CORBA::ULong(1));
            linkInfo.climit[0] = w;
        }
    }else{
        linkInfo.climit.length(CORBA::ULong(0));
    }

    linkInfo.gearRatio = link.info("gearRatio", 1.0);
    linkInfo.rotorInertia = link.info("rotorInertia", 0.0);
    linkInfo.rotorResistor = link.info("rotorResistor", 1.0);
    linkInfo.torqueConst = link.info("torqueConst", 0.0);
    linkInfo.encoderPulse = link.info("encoderPulse", 1.0);
    linkInfo.jointValue = 0.0;

    setShapeIndices(link.visualShape(), linkInfo.shapeIndices);
    linkInfo.inlinedShapeTransformMatrices.length(CORBA::ULong(0));

    //linkInfo.AABBmaxDepth
    //linkInfo.AABBmaxNum

}


void BodyInfo_impl::setSegment(LinkInfo& linkInfo, Link& link)
{
    linkInfo.mass = link.mass();
    setVector3(link.centerOfMass(), linkInfo.centerOfMass);
    setMatrix3(link.I(), linkInfo.inertia);

    //In the Choreonoid Body, multiple pieces of Segment node information are consolidated into one.
    linkInfo.segments.length(CORBA::ULong(1));
    SegmentInfo& segmentInfo = linkInfo.segments[0];
    segmentInfo.mass = link.mass();
    setVector3(link.centerOfMass(), segmentInfo.centerOfMass);
    setMatrix3(link.I(), segmentInfo.inertia);
    string name;
    if(!link.info()->read("segmentName", name)){
        name = link.name();
    }
    segmentInfo.name = CORBA::string_dup(name.c_str());
    TransformedShapeIndexSequence& shapeIndices = linkInfo.shapeIndices;
    segmentInfo.shapeIndices.length(shapeIndices.length());
    for(int i=0; i<shapeIndices.length(); i++){
        segmentInfo.shapeIndices[i] = i;
    }
    setTransformMatrix(Position::Identity(), segmentInfo.transformMatrix);

}


void BodyInfo_impl::setDevices()
{
    for(int i=0; i<body->numDevices(); i++){
        Device* device = body->device(i);
        string typeName(device->typeName());
        if( typeName == "ForceSensor"        ||
            typeName == "RateGyroSensor"     ||
            typeName == "AccelerationSensor" ||
            typeName == "Camera"             ||
            typeName == "RangeCamera"        ||
            typeName == "RangeSensor"          ){

            setSensor(device);

        }else if( typeName == "PointLight" ||
                   typeName == "SpotLight" ){

            setLight(device);

        }
    }
}


void BodyInfo_impl::setSensor(Device* device)
{
    LinkInfo& linkInfo = links_[device->link()->index()];
    int n = linkInfo.sensors.length();
    linkInfo.sensors.length(n+1);
    SensorInfo& sensorInfo = linkInfo.sensors[n];
    sensorInfo.name = CORBA::string_dup( device->name().c_str() );
    sensorInfo.id = device->id();
    string typeName(device->typeName());
    sensorInfo.type = CORBA::string_dup( sensorTypeMap[typeName].c_str() );
    setVector3(device->localTranslation(), sensorInfo.translation);
    setMatrix3(device->localRotation(), sensorInfo.rotation);

    if(typeName == "ForceSensor"){
        ForceSensor* forceSensor = dynamic_cast<ForceSensor*>(device);
        setFloatSequence(forceSensor->F_max(), sensorInfo.specValues);
    }else if(typeName == "RateGyroSensor") {
        RateGyroSensor* rateGyroSensor = dynamic_cast<RateGyroSensor*>(device);
        setFloatSequence(rateGyroSensor->w_max(), sensorInfo.specValues);
    }else if(typeName == "AccelerationSensor"){
        AccelerationSensor* accelerationSensor = dynamic_cast<AccelerationSensor*>(device);
        setFloatSequence(accelerationSensor->dv_max(), sensorInfo.specValues);
    }else if(typeName == "Camera" || typeName == "RangeCamera") {
        Camera* camera = dynamic_cast<Camera*>(device);
        sensorInfo.specValues.length( CORBA::ULong(7) );
        sensorInfo.specValues[0] = camera->nearClipDistance();
        sensorInfo.specValues[1] = camera->farClipDistance();
        sensorInfo.specValues[2] = camera->fieldOfView();
        if(typeName == "Camera"){
            sensorInfo.specValues[3] = cameraTypeMap[camera->imageType()];
        }else{   // typeName is "RangeCamera"
            sensorInfo.specValues[3] = rangeCameraTypeMap[camera->imageType()];
        }
        sensorInfo.specValues[4] = static_cast<CORBA::Float>(camera->resolutionX());
        sensorInfo.specValues[5] = static_cast<CORBA::Float>(camera->resolutionY());
        sensorInfo.specValues[6] = camera->frameRate();
    }else if(typeName == "RangeSensor"){
        sensorInfo.specValues.length( CORBA::ULong(4) );
        RangeSensor* rangeSensor = dynamic_cast<RangeSensor*>(device);
        sensorInfo.specValues[0] = rangeSensor->yawRange();
        sensorInfo.specValues[1] = rangeSensor->yawStep();
        sensorInfo.specValues[2] = rangeSensor->frameRate();
        sensorInfo.specValues[3] = rangeSensor->maxDistance();
    }

    //In choreonoid Body the Shape of the sensor is integrated into the shape of Link
    sensorInfo.shapeIndices.length(CORBA::ULong(0));
    sensorInfo.inlinedShapeTransformMatrices.length(CORBA::ULong(0));
}


void BodyInfo_impl::setLight(Device* device)
{
    LinkInfo& linkInfo = links_[device->link()->index()];
    int n = linkInfo.lights.length();
    linkInfo.lights.length(n+1);
    LightInfo& lightInfo = linkInfo.lights[n];

    setTransformMatrix(device->localRotation(), device->localTranslation(), lightInfo.transformMatrix);
    lightInfo.name = CORBA::string_dup( device->name().c_str() );
    string typeName(device->typeName());
    lightInfo.type = lightTypeMap[typeName];

    Light* light = dynamic_cast<Light*>(device);
    setVector3( light->color(), lightInfo.color);
    lightInfo.intensity = light->intensity();
    lightInfo.on = light->on();

    PointLight* pointlight = dynamic_cast<PointLight*>(device);
    if(pointlight){
        lightInfo.attenuation[0] = pointlight->constantAttenuation();
        lightInfo.attenuation[1] = pointlight->linearAttenuation();
        lightInfo.attenuation[2] = pointlight->quadraticAttenuation();
    }

    SpotLight* spotlight = dynamic_cast<SpotLight*>(device);
    if(spotlight){
        setVector3(spotlight->direction(), lightInfo.direction);
        lightInfo.beamWidth = spotlight->beamWidth();
        lightInfo.cutOffAngle = spotlight->cutOffAngle();
    }

    //lightInfo.ambientIntensity
    //lightInfo.location
    //lightInfo.radius

}

void BodyInfo_impl::setHwc(LinkInfo& linkInfo, Link& link)
{
    const Mapping& hwc = *link.info()->findMapping("Hwc");
    if(hwc.isValid()){
        int n = linkInfo.hwcs.length();
        linkInfo.hwcs.length(n+1);
        HwcInfo& hwcInfo = linkInfo.hwcs[n];
        string s;
        int i;
        Vector3 v;
        AngleAxis a;
        if(hwc.read("name", s)){
            hwcInfo.name = CORBA::string_dup( s.c_str() );;
        }
        if(hwc.read("id", i)){
            hwcInfo.id = i;
        }
        if(read(hwc, "translation", v)){
            setVector3(v, hwcInfo.translation);
        }
        if(read(hwc, "rotation", a)){
            setAngleAxis(a, hwcInfo.rotation);
            hwcInfo.rotation[3] = radian(hwcInfo.rotation[3]);             //TODO radian
        }
        if(hwc.read("url", s)){
            hwcInfo.url = CORBA::string_dup( s.c_str() );;
        }
        //In choreonoid Body the Shape of the sensor is integrated into the shape of Link
        hwcInfo.shapeIndices.length(CORBA::ULong(0));
        hwcInfo.inlinedShapeTransformMatrices.length(CORBA::ULong(0));
    }else{
        linkInfo.hwcs.length(CORBA::ULong(0));
    }

}
