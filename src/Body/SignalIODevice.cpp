#include "SignalIODevice.h"
#include "YAMLBodyLoader.h"
#include <cnoid/ValueTree>
#include <unordered_map>

using namespace std;
using namespace cnoid;

namespace {

YAMLBodyLoader::NodeTypeRegistration
registerHolderDevice(
    "SignalIO",
    [](YAMLBodyLoader& loader, Mapping& node){
        SignalIODevicePtr device = new SignalIODevice;
        return device->readDescription(loader, node);
    });
}

namespace cnoid {

class SignalIODevice::NonState
{
public:
    unordered_map<int, Signal<void(bool on)>> sigSignalOutputMap;
    unordered_map<int, Signal<void(bool on)>> sigSignalInputMap;
};

}


SignalIODevice::SignalIODevice()
{
    int n = 100;
    out_.resize(n, false);
    in_.resize(n, false);
    on_ = true;
    ns = new NonState;
}


SignalIODevice::SignalIODevice(const SignalIODevice& org, bool copyStateOnly, BodyCloneMap* cloneMap)
    : Device(org, copyStateOnly)
{
    copySignalIODeviceStateFrom(org);

    if(copyStateOnly){
        ns = nullptr;
    } else {
        ns = new NonState;
    }
}


SignalIODevice::~SignalIODevice()
{
    if(ns){
        delete ns;
    }
}


const char* SignalIODevice::typeName()
{
    return "SignalIODevice";
}


void SignalIODevice::copySignalIODeviceStateFrom(const SignalIODevice& other)
{
    out_ = other.out_;
    in_ = other.in_;
    on_ = other.on_;
}


void SignalIODevice::copyStateFrom(const DeviceState& other)
{
    if(typeid(other) != typeid(SignalIODevice)){
        throw std::invalid_argument("Type mismatch in the Device::copyStateFrom function");
    }
    copySignalIODeviceStateFrom(static_cast<const SignalIODevice&>(other));
}
    

DeviceState* SignalIODevice::cloneState() const
{
    return new SignalIODevice(*this, true, nullptr);
}


Device* SignalIODevice::doClone(BodyCloneMap* cloneMap) const
{
    return new SignalIODevice(*this, false, cloneMap);
}


void SignalIODevice::forEachActualType(std::function<bool(const std::type_info& type)> func)
{
    if(!func(typeid(SignalIODevice))){
        Device::forEachActualType(func);
    }
}


bool SignalIODevice::on() const
{
    return on_;
}


void SignalIODevice::on(bool on)
{
    on_ = on;
}


void SignalIODevice::setOut(int index, bool on, bool doNotify)
{
    out_[index] = on;

    if(doNotify && ns){
        ns->sigSignalOutputMap[index](on);
    }
}


void SignalIODevice::setIn(int index, bool on, bool doNotify)
{
    in_[index] = on;
    
    if(doNotify && ns){
        ns->sigSignalInputMap[index](on);
    }
}


SignalProxy<void(bool on)> SignalIODevice::sigSignalOutput(int index)
{
    return ns->sigSignalOutputMap[index];
}


SignalProxy<void(bool on)> SignalIODevice::sigSignalInput(int index)
{
    return ns->sigSignalInputMap[index];
}


int SignalIODevice::stateSize() const
{
    return 1;
}


const double* SignalIODevice::readState(const double* buf)
{
    int i = 0;
    on_ = buf[i++];
    return buf + i;
}


double* SignalIODevice::writeState(double* out_buf) const
{
    int i = 0;
    out_buf[i++] = on_ ? 1.0 : 0.0;
    return out_buf + i;
}


bool SignalIODevice::readDescription(YAMLBodyLoader& loader, Mapping& node)
{
    int n;
    if(node.read("numBinarySignals", n)){
        out_.resize(n, false);
        in_.resize(n, false);
    }
    return loader.readDevice(this, node);
}
