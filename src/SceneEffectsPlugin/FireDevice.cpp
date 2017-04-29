/**
   @file
   @author Shin'ichiro Nakaoka
*/

#include "FireDevice.h"
#include "SceneFire.h"
#include <cnoid/YAMLBodyLoader>
#include <cnoid/SceneDevice>
#include <cnoid/EigenArchive>

using namespace std;
using namespace cnoid;

namespace {

bool readFireDevice(YAMLBodyLoader& loader, Mapping& node)
{
    FireDevicePtr fire = new FireDevice;
    Vector3f a;
    if(read(node, "acceleration", a)) fire->setAcceleration(a);
    double v;
    if(node.read("initialSpeedAverage", v)) fire->setInitialSpeedAverage(v);
    if(node.read("initialSpeedVariation", v)) fire->setInitialSpeedVariation(v);
    if(loader.readAngle(node, "initialVelocityAngleRange", v)) fire->setInitialVelocityAngleRange(v);
    return loader.readDevice(fire, node);
}


SceneDevice* createSceneFireDevice(Device* device)
{
    auto fire = static_cast<FireDevice*>(device);
    auto scene = new SceneFire;
    auto sceneDevice = new SceneDevice(fire, scene);

    sceneDevice->setFunctionOnStateChanged(
        [scene, fire](){
            scene->setAcceleration(fire->acceleration());
            scene->setInitialSpeedAverage(fire->initialSpeedAverage());
            scene->setInitialSpeedVariation(fire->initialSpeedVariation());
            scene->setInitialVelocityAngleRange(fire->initialVelocityAngleRange());
            scene->notifyUpdate();
        });

    sceneDevice->setFunctionOnTimeChanged(
        [scene](double time){
            scene->setTime(time);
            scene->notifyUpdate();
        });
            
    return sceneDevice;
}
                        
struct TypeRegistration
{
    TypeRegistration() {
        YAMLBodyLoader::addNodeType("FireDevice", readFireDevice);
        SceneDevice::registerSceneDeviceFactory<FireDevice>(createSceneFireDevice);
    }
} registration;

}


FireDevice::FireDevice()
{
    on_ = true;
    acceleration_ << 0.0f, 0.0f, 0.1f;
    initialSpeedAverage_ = 0.15f;
    initialSpeedVariation_ = 0.1f;
    initialVelocityAngleRange_ = PI / 3.0f;
}


FireDevice::FireDevice(const FireDevice& org, bool copyStateOnly)
    : Device(org, copyStateOnly)
{
    copyStateFrom(org);
}


const char* FireDevice::typeName()
{
    return "FireDevice";
}


void FireDevice::copyStateFrom(const FireDevice& other)
{
    on_ = other.on_;
    acceleration_ = other.acceleration_;
    initialSpeedAverage_ = other.initialSpeedAverage_;
    initialSpeedVariation_ = other.initialSpeedVariation_;
    initialVelocityAngleRange_ = other.initialVelocityAngleRange_;
}


void FireDevice::copyStateFrom(const DeviceState& other)
{
    if(typeid(other) != typeid(FireDevice)){
        throw std::invalid_argument("Type mismatch in the Device::copyStateFrom function");
    }
    copyStateFrom(static_cast<const FireDevice&>(other));
}


DeviceState* FireDevice::cloneState() const
{
    return new FireDevice(*this, false);
}


Device* FireDevice::clone() const
{
    return new FireDevice(*this);
}


void FireDevice::forEachActualType(std::function<bool(const std::type_info& type)> func)
{
    if(!func(typeid(FireDevice))){
        Device::forEachActualType(func);
    }
}


int FireDevice::stateSize() const
{
    return 7;
}


const double* FireDevice::readState(const double* buf)
{
    on_ = buf[0];
    acceleration_[0] = buf[1];
    acceleration_[1] = buf[2];
    acceleration_[2] = buf[3];
    initialSpeedAverage_ = buf[4];
    initialSpeedVariation_ = buf[5];
    initialVelocityAngleRange_ = buf[6];
    return buf + 7;
}


double* FireDevice::writeState(double* out_buf) const
{
    out_buf[0] = on_ ? 1.0 : 0.0;
    out_buf[1] = acceleration_[0];
    out_buf[2] = acceleration_[1];
    out_buf[3] = acceleration_[2];
    out_buf[4] = initialSpeedAverage_;
    out_buf[5] = initialSpeedVariation_;
    out_buf[6] = initialVelocityAngleRange_;
    return out_buf + 7;
}
