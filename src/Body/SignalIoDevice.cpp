#include "SignalIoDevice.h"
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
        SignalIoDevicePtr device = new SignalIoDevice;
        return device->readDescription(loader, node);
    });
}

namespace cnoid {

class SignalIoDevice::NonState
{
public:
    unordered_map<int, Signal<void(bool on)>> sigSignalOutputMap;
    unordered_map<int, Signal<void(bool on)>> sigSignalInputMap;
    unordered_map<int, string> outLabelMap;
    unordered_map<int, string> inLabelMap;
    string emptyLabel;
};

}


SignalIoDevice::SignalIoDevice()
{
    int n = 100;
    out_.resize(n, false);
    in_.resize(n, false);
    on_ = true;
    ns = new NonState;
}


SignalIoDevice::SignalIoDevice(const SignalIoDevice& org, bool copyStateOnly, BodyCloneMap* cloneMap)
    : Device(org, copyStateOnly)
{
    copySignalIoDeviceStateFrom(org);

    if(copyStateOnly){
        ns = nullptr;
    } else {
        ns = new NonState;
    }
}


SignalIoDevice::~SignalIoDevice()
{
    if(ns){
        delete ns;
    }
}


const char* SignalIoDevice::typeName()
{
    return "SignalIoDevice";
}


void SignalIoDevice::copySignalIoDeviceStateFrom(const SignalIoDevice& other)
{
    out_ = other.out_;
    in_ = other.in_;
    on_ = other.on_;
}


void SignalIoDevice::copyStateFrom(const DeviceState& other)
{
    if(typeid(other) != typeid(SignalIoDevice)){
        throw std::invalid_argument("Type mismatch in the Device::copyStateFrom function");
    }
    copySignalIoDeviceStateFrom(static_cast<const SignalIoDevice&>(other));
}
    

DeviceState* SignalIoDevice::cloneState() const
{
    return new SignalIoDevice(*this, true, nullptr);
}


Device* SignalIoDevice::doClone(BodyCloneMap* cloneMap) const
{
    return new SignalIoDevice(*this, false, cloneMap);
}


void SignalIoDevice::forEachActualType(std::function<bool(const std::type_info& type)> func)
{
    if(!func(typeid(SignalIoDevice))){
        Device::forEachActualType(func);
    }
}


bool SignalIoDevice::on() const
{
    return on_;
}


void SignalIoDevice::on(bool on)
{
    on_ = on;
}


void SignalIoDevice::setOut(int index, bool on, bool doNotify)
{
    out_[index] = on;

    if(doNotify && ns){
        ns->sigSignalOutputMap[index](on);
    }
}


void SignalIoDevice::setIn(int index, bool on, bool doNotify)
{
    in_[index] = on;
    
    if(doNotify && ns){
        ns->sigSignalInputMap[index](on);
    }
}


const std::string& SignalIoDevice::outLabel(int index) const
{
    auto iter = ns->outLabelMap.find(index);
    if(iter != ns->outLabelMap.end()){
        return iter->second;
    }
    return ns->emptyLabel;
}


void SignalIoDevice::setOutLabel(int index, const std::string& label)
{
    if(label.empty()){
        ns->outLabelMap.erase(index);
    } else {
        ns->outLabelMap[index] = label;
    }
}


const std::string& SignalIoDevice::inLabel(int index) const
{
    auto iter = ns->inLabelMap.find(index);
    if(iter != ns->inLabelMap.end()){
        return iter->second;
    }
    return ns->emptyLabel;
}


void SignalIoDevice::setInLabel(int index, const std::string& label)
{
    if(label.empty()){
        ns->inLabelMap.erase(index);
    } else {
        ns->inLabelMap[index] = label;
    }
}


SignalProxy<void(bool on)> SignalIoDevice::sigSignalOutput(int index)
{
    return ns->sigSignalOutputMap[index];
}


SignalProxy<void(bool on)> SignalIoDevice::sigSignalInput(int index)
{
    return ns->sigSignalInputMap[index];
}


int SignalIoDevice::stateSize() const
{
    return 1;
}


const double* SignalIoDevice::readState(const double* buf)
{
    int i = 0;
    on_ = buf[i++];
    return buf + i;
}


double* SignalIoDevice::writeState(double* out_buf) const
{
    int i = 0;
    out_buf[i++] = on_ ? 1.0 : 0.0;
    return out_buf + i;
}


bool SignalIoDevice::readDescription(YAMLBodyLoader& loader, Mapping& node)
{
    int n;
    if(node.read("numBinarySignals", n)){
        out_.resize(n, false);
        in_.resize(n, false);
    }
    return loader.readDevice(this, node);
}
