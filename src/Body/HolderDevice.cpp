#include "HolderDevice.h"
#include "Body.h"
#include "YAMLBodyLoader.h"
#include <cnoid/ValueTree>

using namespace std;
using namespace cnoid;

namespace {

YAMLBodyLoader::NodeTypeRegistration
registerHolderDevice(
    "Holder",
    [](YAMLBodyLoader& loader, Mapping& node){
        HolderDevicePtr holder = new HolderDevice;
        return holder->readDescription(loader, node);
    });

}


HolderDevice::HolderDevice()
{
    targetLocalPosition_.setIdentity();
    on_ = false;
    attachment_ = nullptr;
}


HolderDevice::HolderDevice(const HolderDevice& org, bool copyStateOnly)
    : Device(org, copyStateOnly)
{
    copyHolderDeviceStateFrom(org);

    attachment_ = nullptr;
    if(!copyStateOnly){
        if(org.attachment_){
            setAttachment(*org.attachment_);
        }
    }
}


HolderDevice::~HolderDevice()
{
    clearAttachment();
}


const char* HolderDevice::typeName()
{
    return "HolderDevice";
}


void HolderDevice::copyHolderDeviceStateFrom(const HolderDevice& other)
{
    targetLocalPosition_ = other.targetLocalPosition_;
    on_ = other.on_;
}


void HolderDevice::copyStateFrom(const DeviceState& other)
{
    if(typeid(other) != typeid(HolderDevice)){
        throw std::invalid_argument("Type mismatch in the Device::copyStateFrom function");
    }
    copyHolderDeviceStateFrom(static_cast<const HolderDevice&>(other));
}
    

DeviceState* HolderDevice::cloneState() const
{
    return new HolderDevice(*this, true);

}


Device* HolderDevice::clone() const
{
    return new HolderDevice(*this);
}


void HolderDevice::forEachActualType(std::function<bool(const std::type_info& type)> func)
{
    if(!func(typeid(HolderDevice))){
        Device::forEachActualType(func);
    }
}


bool HolderDevice::on() const
{
    return on_;
}


void HolderDevice::on(bool on)
{
    on_ = on;
}


std::string HolderDevice::attachment() const
{
    if(attachment_){
        return *attachment_;
    }
    return string();
}


void HolderDevice::setAttachment(const std::string& attachment)
{
    if(!attachment_){
        attachment_ = new string;
    }
    *attachment_ = attachment;
}


void HolderDevice::clearAttachment()
{
    if(attachment_){
        delete attachment_;
    }
    attachment_ = nullptr;
}
        

int HolderDevice::stateSize() const
{
    return 1;
}


const double* HolderDevice::readState(const double* buf)
{
    int i = 0;
    on_ = buf[i++];
    return buf + i;
}


double* HolderDevice::writeState(double* out_buf) const
{
    int i = 0;
    out_buf[i++] = on_ ? 1.0 : 0.0;
    return out_buf + i;
}


bool HolderDevice::readDescription(YAMLBodyLoader& loader, Mapping& node)
{
    string symbol;
    if(node.read("attachement", symbol)){
        setAttachment(symbol);
    } else {
        clearAttachment();
    }
    return loader.readDevice(this, node);
}
