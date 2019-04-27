/**
   \file
   \author Shin'ichiro Nakaoka
*/

#include "MarkerDevice.h"
#include <cnoid/SceneDevice>
#include <cnoid/SceneMarkers>
#include <cnoid/YAMLBodyLoader>
#include <cnoid/EigenArchive>
#include <fmt/format.h>
#include "gettext.h"

using namespace std;
using namespace cnoid;

namespace {

YAMLBodyLoader::NodeTypeRegistration
registerMarkerDevice(
    "MarkerDevice",
    [](YAMLBodyLoader& loader, Mapping& node){
        MarkerDevicePtr device = new MarkerDevice;
        return device->readDescription(loader, node);
    });
        

int getSceneMarkerType(int type)
{
    switch(type){
    case MarkerDevice::CROSS_MARKER:
        return SceneMarker::CROSS_MARKER;
    case MarkerDevice::SPHERE_MARKER:
        return SceneMarker::SPHERE_MARKER;
    case MarkerDevice::AXES_MARKER:
        return SceneMarker::AXES_MARKER;
    default:
        break;
    }
    return SceneMarker::NO_MARKER;
}

struct SceneMarkerDevice : public SceneDevice
{
public:
    MarkerDevice* device;
    SceneMarkerPtr marker;
    bool isMarkerAttached;

    SceneMarkerDevice(Device* device_);
    void updateScene();
};

SceneDevice::FactoryRegistration<MarkerDevice>
registerSceneMarkerDeviceFactory(
    [](Device* device){ return new SceneMarkerDevice(device); });


SceneMarkerDevice::SceneMarkerDevice(Device* device_)
    : SceneDevice(device_),
      device(static_cast<MarkerDevice*>(device_))
{
    marker = new SceneMarker;
    marker->setName(device->name());
    isMarkerAttached = false;
    setFunctionOnStateChanged([&](){ updateScene(); });
}

void SceneMarkerDevice::updateScene()
{
    bool on = device->on();
    if(on != isMarkerAttached){
        if(on){
            addChildOnce(marker);
        } else {
            removeChild(marker);
        }
        isMarkerAttached = on;
    }

    marker->setColor(device->color());
    marker->setTransparency(device->transparency());
    marker->setPosition(device->offsetPosition());

    bool doUpdateMarker = false;
    int sceneMarkerType = getSceneMarkerType(device->markerType());
    if(sceneMarkerType != marker->markerType()){
        marker->setMarkerType(sceneMarkerType);
        doUpdateMarker = true;
    }
    if(device->markerSize() != marker->markerSize()){
        marker->setMarkerSize(device->markerSize());
        doUpdateMarker = true;
    }
    if(doUpdateMarker){
        marker->updateMarker(true);
    } else {
        marker->notifyUpdate();
    }
}

}
    

MarkerDevice::MarkerDevice()
{
    on_ = true;
    markerType_ = SPHERE_MARKER;
    markerSize_ = 0.01f;
    offsetPosition_.setIdentity();
    color_ << 1.0f, 0.0f, 0.0f;
    emission_ = 0.5f;
    transparency_ = 0.7f;
}


const char* MarkerDevice::typeName()
{
    return "MarkerDevice";
}


void MarkerDevice::copyMarkerDeviceStateFrom(const MarkerDevice& other)
{
    on_ = other.on_;
    markerType_ = other.markerType_; 
    markerSize_ = other.markerSize_;
    offsetPosition_ = other.offsetPosition_;
    color_ = other.color_;
    transparency_ = other.transparency_;
}


void MarkerDevice::copyStateFrom(const DeviceState& other)
{
    if(typeid(other) != typeid(MarkerDevice)){
        throw std::invalid_argument("Type mismatch in the Device::copyStateFrom function");
    }
    copyMarkerDeviceStateFrom(static_cast<const MarkerDevice&>(other));
}



MarkerDevice::MarkerDevice(const MarkerDevice& org, bool copyStateOnly)
    : Device(org, copyStateOnly)
{
    copyMarkerDeviceStateFrom(org);
}


DeviceState* MarkerDevice::cloneState() const
{
    return new MarkerDevice(*this, true);
}


Device* MarkerDevice::clone() const
{
    return new MarkerDevice(*this);
}

void MarkerDevice::forEachActualType(std::function<bool(const std::type_info& type)> func)
{
    if(!func(typeid(MarkerDevice))){
        Device::forEachActualType(func);
    }
}


bool MarkerDevice::on() const
{
    return on_;
}


void MarkerDevice::on(bool on)
{
    on_ = on;
}


int MarkerDevice::stateSize() const
{
    return 15;
}


const double* MarkerDevice::readState(const double* buf)
{
    int i = 0;
    on_ = buf[i++];
    markerType_ = buf[i++];
    markerSize_ = buf[i++];
    color_ = Eigen::Map<const Vector3>(buf + i).cast<float>();
    i += 3;
    emission_ = buf[i++];
    transparency_ = buf[i++];
    offsetPosition_.translation() << buf[i], buf[i+1], buf[i+2];
    i += 3;
    offsetPosition_.linear() = Quat(buf[i], buf[i+1], buf[i+2], buf[i+3]).toRotationMatrix();
    i += 4;
    return buf + i;
}


double* MarkerDevice::writeState(double* out_buf) const
{
    int i = 0;

    out_buf[i++] = on_ ? 1.0 : 0.0;
    out_buf[i++] = markerType_;
    out_buf[i++] = markerSize_;
    out_buf[i++] = color_[0];
    out_buf[i++] = color_[1];
    out_buf[i++] = color_[2];
    out_buf[i++] = emission_;
    out_buf[i++] = transparency_;

    auto p = offsetPosition_.translation();
    out_buf[i++] = p.x();
    out_buf[i++] = p.y();
    out_buf[i++] = p.z();

    Quat q(offsetPosition_.linear());
    out_buf[i++] = q.w();
    out_buf[i++] = q.x();
    out_buf[i++] = q.y();
    out_buf[i++] = q.z();
    
    return out_buf + i;
}


bool MarkerDevice::readDescription(YAMLBodyLoader& loader, Mapping& node)
{
    string type;
    if(node.read("markerType", type)){
        if(type == "cross"){
            setMarkerType(MarkerDevice::CROSS_MARKER);
        } else if(type == "sphere"){
            setMarkerType(MarkerDevice::SPHERE_MARKER);
        } else if(type == "axes"){
            setMarkerType(MarkerDevice::AXES_MARKER);
        } else {
            node.throwException(
                fmt::format(_("Unknown marker type '{}'"), type));
        }
    }

    setMarkerSize(node.get("size", markerSize()));
    setEmission(node.get("emission", emission()));
    setTransparency(node.get("transparency", transparency()));

    Vector3f c;
    if(read(node, "color", c)) setColor(c);

    Position T = Affine3::Identity();
    Vector3 p;
    if(read(node, "offsetTranslation", p)){
        T.translation() = p;
    }
    Matrix3 R;
    if(loader.readRotation(node, "offsetRotation", R)){
        T.linear() = R;
    }
    setOffsetPosition(T);

    return loader.readDevice(this, node);
}
