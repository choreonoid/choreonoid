#include "DigitalIoDevice.h"
#include "YAMLBodyLoader.h"
#include <cnoid/Body>
#include <cnoid/ValueTree>
#include <unordered_map>

using namespace std;
using namespace cnoid;

namespace {

YAMLBodyLoader::NodeTypeRegistration
registerHolderDevice(
    "DigitalIO",
    [](YAMLBodyLoader& loader, Mapping& node){
        DigitalIoDevicePtr device = new DigitalIoDevice;
        return device->readDescription(loader, node);
    });
}

namespace cnoid {

class DigitalIoDevice::NonState
{
public:
    unordered_map<int, Signal<void(bool on)>> sigOutputMap;
    unordered_map<int, Signal<void(bool on)>> sigInputMap;
    unordered_map<int, string> inputToDeviceOnActionMap;
    unordered_map<int, string> outLabelMap;
    unordered_map<int, string> inLabelMap;
    string emptyLabel;

    NonState();
    NonState(const NonState& org);
};

}


DigitalIoDevice::DigitalIoDevice()
{
    int n = 100;
    out_.resize(n, false);
    in_.resize(n, false);
    on_ = true;
    ns = new NonState;
}


DigitalIoDevice::NonState::NonState()
{

}


DigitalIoDevice::DigitalIoDevice(const DigitalIoDevice& org, bool copyStateOnly, CloneMap* cloneMap)
    : Device(org, copyStateOnly)
{
    copyDigitalIoDeviceStateFrom(org);

    if(copyStateOnly){
        ns = nullptr;
    } else {
        ns = new NonState(*org.ns);
    }
}


DigitalIoDevice::NonState::NonState(const NonState& org)
    : inputToDeviceOnActionMap(org.inputToDeviceOnActionMap),
      outLabelMap(org.outLabelMap),
      inLabelMap(org.inLabelMap)
{

}

DigitalIoDevice::~DigitalIoDevice()
{
    if(ns){
        delete ns;
    }
}


const char* DigitalIoDevice::typeName()
{
    return "DigitalIoDevice";
}


void DigitalIoDevice::copyDigitalIoDeviceStateFrom(const DigitalIoDevice& other)
{
    out_ = other.out_;
    in_ = other.in_;
    on_ = other.on_;
}


void DigitalIoDevice::copyStateFrom(const DeviceState& other)
{
    if(typeid(other) != typeid(DigitalIoDevice)){
        throw std::invalid_argument("Type mismatch in the Device::copyStateFrom function");
    }
    copyDigitalIoDeviceStateFrom(static_cast<const DigitalIoDevice&>(other));
}
    

DeviceState* DigitalIoDevice::cloneState() const
{
    return new DigitalIoDevice(*this, true, nullptr);
}


Referenced* DigitalIoDevice::doClone(CloneMap* cloneMap) const
{
    return new DigitalIoDevice(*this, false, cloneMap);
}


void DigitalIoDevice::forEachActualType(std::function<bool(const std::type_info& type)> func)
{
    if(!func(typeid(DigitalIoDevice))){
        Device::forEachActualType(func);
    }
}


bool DigitalIoDevice::on() const
{
    return on_;
}


void DigitalIoDevice::on(bool on)
{
    on_ = on;
}


void DigitalIoDevice::setOut(int index, bool on, bool doNotify)
{
    out_[index] = on;

    if(doNotify && ns){
        ns->sigOutputMap[index](on);
    }
}


void DigitalIoDevice::setIn(int index, bool on, bool doNotify)
{
    in_[index] = on;

    auto iter = ns->inputToDeviceOnActionMap.find(index);
    if(iter != ns->inputToDeviceOnActionMap.end()){
        auto& deviceName = iter->second;
        if(auto body_ = body()){
            if(auto device = body_->findDevice(deviceName)){
                device->on(on);
                if(doNotify){
                    device->notifyStateChange();
                }
            }
        }
    }
    
    if(doNotify && ns){
        ns->sigInputMap[index](on);
    }
}


const std::string& DigitalIoDevice::outLabel(int index) const
{
    auto iter = ns->outLabelMap.find(index);
    if(iter != ns->outLabelMap.end()){
        return iter->second;
    }
    return ns->emptyLabel;
}


void DigitalIoDevice::setOutLabel(int index, const std::string& label)
{
    if(label.empty()){
        ns->outLabelMap.erase(index);
    } else {
        ns->outLabelMap[index] = label;
    }
}


const std::string& DigitalIoDevice::inLabel(int index) const
{
    auto iter = ns->inLabelMap.find(index);
    if(iter != ns->inLabelMap.end()){
        return iter->second;
    }
    return ns->emptyLabel;
}


void DigitalIoDevice::setInLabel(int index, const std::string& label)
{
    if(label.empty()){
        ns->inLabelMap.erase(index);
    } else {
        ns->inLabelMap[index] = label;
    }
}


SignalProxy<void(bool on)> DigitalIoDevice::sigOutput(int index)
{
    return ns->sigOutputMap[index];
}


SignalProxy<void(bool on)> DigitalIoDevice::sigInput(int index)
{
    return ns->sigInputMap[index];
}


int DigitalIoDevice::stateSize() const
{
    return 1;
}


const double* DigitalIoDevice::readState(const double* buf)
{
    int i = 0;
    on_ = buf[i++];
    return buf + i;
}


double* DigitalIoDevice::writeState(double* out_buf) const
{
    int i = 0;
    out_buf[i++] = on_ ? 1.0 : 0.0;
    return out_buf + i;
}


bool DigitalIoDevice::readDescription(YAMLBodyLoader& loader, Mapping& node)
{
    int n;
    if(node.read("numSignalLines", n)){
        out_.resize(n, false);
        in_.resize(n, false);
    }

    ns->inputToDeviceOnActionMap.clear();
    auto action = node.findMapping("action");
    if(action->isValid()){
        auto input = action->findMapping("input");
        if(input->isValid()){
            for(auto& kv : *input){
                int signalIndex = stoi(kv.first);
                Listing* target = kv.second->toListing();
                if(target->size() == 3 &&
                   target->at(0)->toString() == "device" &&
                   target->at(2)->toString() == "on"){
                    auto& deviceName = target->at(1)->toString();
                    ns->inputToDeviceOnActionMap[signalIndex] = deviceName;
                }
            }
        }
    }

    return loader.readDevice(this, node);
}
