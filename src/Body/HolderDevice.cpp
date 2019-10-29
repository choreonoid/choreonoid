#include "HolderDevice.h"
#include "AttachmentDevice.h"
#include "Body.h"
#include "YAMLBodyLoader.h"
#include <cnoid/CloneMap>
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
    on_ = false;
    category_ = nullptr;
}


HolderDevice::HolderDevice(const HolderDevice& org, bool copyStateOnly, CloneMap* cloneMap)
    : Device(org, copyStateOnly)
{
    copyHolderDeviceStateFrom(org);

    if(org.attachment_ && cloneMap){
        attachment_ = cloneMap->findCloneOrReplaceLater<AttachmentDevice>(
            org.attachment_, [&](AttachmentDevice* clone){ attachment_ = clone; });
    }

    category_ = nullptr;
    if(!copyStateOnly){
        if(org.category_){
            setCategory(*org.category_);
        }
    }
}


HolderDevice::~HolderDevice()
{
    clearCategory();
}


const char* HolderDevice::typeName()
{
    return "HolderDevice";
}


void HolderDevice::copyHolderDeviceStateFrom(const HolderDevice& other)
{
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
    return new HolderDevice(*this, true, nullptr);

}


Referenced* HolderDevice::doClone(CloneMap* cloneMap) const
{
    return new HolderDevice(*this, false, cloneMap);
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


AttachmentDevice* HolderDevice::attachment()
{
    return attachment_;
}


void HolderDevice::setAttachment(AttachmentDevice* attachment)
{
    attachment_ = attachment;
}


std::string HolderDevice::category() const
{
    if(category_){
        return *category_;
    }
    return string();
}


void HolderDevice::setCategory(const std::string& category)
{
    clearCategory();
    category_ = new string;
    *category_ = category;
}


void HolderDevice::clearCategory()
{
    if(category_){
        delete category_;
    }
    category_ = nullptr;
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
    if(node.read("category", symbol)){
        setCategory(symbol);
    } else {
        clearCategory();
    }
    return loader.readDevice(this, node);
}
