#include "DigitalIoDevice.h"
#include "StdBodyLoader.h"
#include "StdBodyFileUtil.h"
#include <cnoid/Body>
#include <cnoid/ValueTree>
#include <unordered_map>
#include "gettext.h"

using namespace std;
using namespace cnoid;

namespace cnoid {

class DigitalIoDevice::Impl
{
public:
    unordered_map<int, Signal<void(bool on)>> sigOutputMap;
    unordered_map<int, Signal<void(bool on)>> sigInputMap;
    unordered_map<int, string> inputToDeviceSwitchConnectionMap;
    unordered_map<int, string> outLabelMap;
    unordered_map<int, string> inLabelMap;
    string emptyLabel;

    Impl();
    Impl(const Impl& org);
    bool readInputToDeviceSwitchConnections(DigitalIoDevice* self, const Mapping* info);
    void readActions(DigitalIoDevice* self, const Mapping* info);
};

}


DigitalIoDevice::DigitalIoDevice()
{
    int n = 100;
    out_.resize(n, false);
    in_.resize(n, false);
    on_ = true;
    impl = new Impl;
}


DigitalIoDevice::Impl::Impl()
{

}


DigitalIoDevice::DigitalIoDevice(const DigitalIoDevice& org, bool copyStateOnly, CloneMap* cloneMap)
    : Device(org, copyStateOnly)
{
    copyDigitalIoDeviceStateFrom(org);

    if(copyStateOnly){
        impl = nullptr;
    } else {
        impl = new Impl(*org.impl);
    }
}


DigitalIoDevice::Impl::Impl(const Impl& org)
    : inputToDeviceSwitchConnectionMap(org.inputToDeviceSwitchConnectionMap),
      outLabelMap(org.outLabelMap),
      inLabelMap(org.inLabelMap)
{

}

DigitalIoDevice::~DigitalIoDevice()
{
    if(impl){
        delete impl;
    }
}


const char* DigitalIoDevice::typeName() const 
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


void DigitalIoDevice::setNumSignalLines(int n)
{
    out_.resize(n, false);
    in_.resize(n, false);
}


void DigitalIoDevice::setOut(int index, bool on, bool doNotify)
{
    out_[index] = on;

    if(doNotify && impl){
        impl->sigOutputMap[index](on);
        notifyStateChange();
    }
}


void DigitalIoDevice::setIn(int index, bool on, bool doNotify)
{
    in_[index] = on;

    auto iter = impl->inputToDeviceSwitchConnectionMap.find(index);
    if(iter != impl->inputToDeviceSwitchConnectionMap.end()){
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
    
    if(doNotify && impl){
        impl->sigInputMap[index](on);
        notifyStateChange();
    }
}


const std::string& DigitalIoDevice::outLabel(int index) const
{
    auto iter = impl->outLabelMap.find(index);
    if(iter != impl->outLabelMap.end()){
        return iter->second;
    }
    return impl->emptyLabel;
}


void DigitalIoDevice::setOutLabel(int index, const std::string& label)
{
    if(label.empty()){
        impl->outLabelMap.erase(index);
    } else {
        impl->outLabelMap[index] = label;
    }
}


std::vector<std::pair<int, std::string&>> DigitalIoDevice::getOutLabels() const
{
    std::vector<std::pair<int, std::string&>> labels;
    labels.reserve(impl->outLabelMap.size());
    for(auto& kv : impl->outLabelMap){
        labels.emplace_back(kv.first, kv.second);
    }
    return labels;
}


const std::string& DigitalIoDevice::inLabel(int index) const
{
    auto iter = impl->inLabelMap.find(index);
    if(iter != impl->inLabelMap.end()){
        return iter->second;
    }
    return impl->emptyLabel;
}


void DigitalIoDevice::setInLabel(int index, const std::string& label)
{
    if(label.empty()){
        impl->inLabelMap.erase(index);
    } else {
        impl->inLabelMap[index] = label;
    }
}


std::vector<std::pair<int, std::string&>> DigitalIoDevice::getInLabels() const
{
    std::vector<std::pair<int, std::string&>> labels;
    labels.reserve(impl->inLabelMap.size());
    for(auto& kv : impl->inLabelMap){
        labels.emplace_back(kv.first, kv.second);
    }
    return labels;
}



SignalProxy<void(bool on)> DigitalIoDevice::sigOutput(int index)
{
    return impl->sigOutputMap[index];
}


SignalProxy<void(bool on)> DigitalIoDevice::sigInput(int index)
{
    return impl->sigInputMap[index];
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


bool DigitalIoDevice::readDescription(const Mapping* info)
{
    return readSpecifications(info) && readConfiguration(info);
}


bool DigitalIoDevice::readSpecifications(const Mapping* info)
{
    int n;
    if(info->read("num_signal_lines", n)){
        setNumSignalLines(n);
    }
    return true;
}


bool DigitalIoDevice::readConfiguration(const Mapping* info)
{
    if(auto& outLabelsNode = *info->findMapping("out_labels")){
        for(auto& kv : outLabelsNode){
            setOutLabel(std::stoi(kv.first), kv.second->toString());
        }
    }
    if(auto& inLabelsNode = *info->findMapping("in_labels")){
        for(auto& kv : inLabelsNode){
            setInLabel(std::stoi(kv.first), kv.second->toString());
        }
    }
    impl->inputToDeviceSwitchConnectionMap.clear();
    if(!impl->readInputToDeviceSwitchConnections(this, info)){
        impl->readActions(this, info); // old format
    }
    return true;
}


bool DigitalIoDevice::Impl::readInputToDeviceSwitchConnections(DigitalIoDevice* self, const Mapping* info)
{
    bool done = false;
    if(auto& activationsNode = *info->findListing("input_to_device_switch_connections")){
        self->clearInputToDeviceSwitchConnections();
        for(auto& node : activationsNode){
            if(auto& nodePair = *node->toListing()){
                if(nodePair.size() != 2){
                    nodePair.throwException(
                        _("The element is not the pair of the input number and the target name"));
                }
                self->setInputToDeviceSwitchConnection(nodePair[0].toInt(), nodePair[1].toString());
                done = true;
            }
        }
    }
    return done;
}


void DigitalIoDevice::Impl::readActions(DigitalIoDevice* self, const Mapping* info)
{
    auto action = info->findMapping("action");
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
                    self->setInputToDeviceSwitchConnection(signalIndex, deviceName);
                }
            }
        }
    }
}


bool DigitalIoDevice::writeDescription(Mapping* info) const
{
    return writeSpecifications(info) && writeConfiguration(info);
}


bool DigitalIoDevice::writeSpecifications(Mapping* info) const
{
    info->write("num_signal_lines", numSignalLines());
    return true;
}


bool DigitalIoDevice::writeConfiguration(Mapping* info) const
{
    if(!impl->outLabelMap.empty()){
        auto outLabelsNode = info->openMapping("out_labels");
        for(auto& kv : impl->outLabelMap){
            outLabelsNode->write(std::to_string(kv.first), kv.second, DOUBLE_QUOTED);
        }
    }
    if(!impl->inLabelMap.empty()){
        auto inLabelsNode = info->openMapping("in_labels");
        for(auto& kv : impl->inLabelMap){
            inLabelsNode->write(std::to_string(kv.first), kv.second, DOUBLE_QUOTED);
        }
    }
    if(!impl->inputToDeviceSwitchConnectionMap.empty()){
        auto connectionsNode = info->openListing("input_to_device_switch_connections");
        for(auto& kv : impl->inputToDeviceSwitchConnectionMap){
            auto node = new Listing;
            node->setFlowStyle();
            node->append(kv.first);
            node->append(kv.second, DOUBLE_QUOTED);
            connectionsNode->append(node);
        }
    }

    return true;
}


void DigitalIoDevice::setInputToDeviceSwitchConnection(int inputIndex, const std::string& deviceName)
{
    impl->inputToDeviceSwitchConnectionMap[inputIndex] = deviceName;
}


std::vector<std::pair<int, std::string&>> DigitalIoDevice::getInputToDeviceSwitchConnections() const
{
    std::vector<std::pair<int, std::string&>> list;
    list.reserve(impl->inputToDeviceSwitchConnectionMap.size());
    for(auto& kv : impl->inputToDeviceSwitchConnectionMap){
        list.emplace_back(kv.first, kv.second);
    }
    return list;
}


void DigitalIoDevice::removeInputToDeviceSwitchConnection(int inputIndex)
{
    impl->inputToDeviceSwitchConnectionMap.erase(inputIndex);
}


void DigitalIoDevice::clearInputToDeviceSwitchConnections()
{
    impl->inputToDeviceSwitchConnectionMap.clear();
}


namespace {

StdBodyFileDeviceTypeRegistration<DigitalIoDevice>
registerDigitalIoDevice(
    "DigitalIO",
    [](StdBodyLoader* loader, const Mapping* info){
        DigitalIoDevicePtr device = new DigitalIoDevice;
        if(device->readDescription(info)){
            return loader->readDevice(device, info);
        }
        return false;
    },
    [](StdBodyWriter* /* writer */, Mapping* info, const DigitalIoDevice* device){
        return device->writeSpecifications(info);
    });
    
}
