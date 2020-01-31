#include "AttachmentDevice.h"
#include "HolderDevice.h"
#include "Body.h"
#include "YAMLBodyLoader.h"
#include <cnoid/CloneMap>
#include <cnoid/ValueTree>

using namespace std;
using namespace cnoid;

namespace {

YAMLBodyLoader::NodeTypeRegistration
registerAttachmentDevice(
    "Attachment",
    [](YAMLBodyLoader& loader, Mapping& node){
        AttachmentDevicePtr holder = new AttachmentDevice;
        return holder->readDescription(loader, node);
    });

}


AttachmentDevice::AttachmentDevice()
{
    on_ = false;
    category_ = nullptr;
}


AttachmentDevice::AttachmentDevice(const AttachmentDevice& org, bool copyStateOnly, CloneMap* cloneMap)
    : Device(org, copyStateOnly)
{
    copyAttachmentDeviceStateFrom(org);

    auto orgHolder = org.weak_holder.lock();
    if(orgHolder && cloneMap){
        weak_holder = cloneMap->findCloneOrReplaceLater<HolderDevice>(
            orgHolder, [&](HolderDevice* clone){ weak_holder = clone; });
    }

    category_ = nullptr;
    if(!copyStateOnly){
        if(org.category_){
            setCategory(*org.category_);
        }
    }
}


AttachmentDevice::~AttachmentDevice()
{
    clearCategory();
}


const char* AttachmentDevice::typeName()
{
    return "AttachmentDevice";
}


void AttachmentDevice::copyAttachmentDeviceStateFrom(const AttachmentDevice& other)
{
    on_ = other.on_;
}


void AttachmentDevice::copyStateFrom(const DeviceState& other)
{
    if(typeid(other) != typeid(AttachmentDevice)){
        throw std::invalid_argument("Type mismatch in the Device::copyStateFrom function");
    }
    copyAttachmentDeviceStateFrom(static_cast<const AttachmentDevice&>(other));
}
    

DeviceState* AttachmentDevice::cloneState() const
{
    return new AttachmentDevice(*this, true, nullptr);

}


Referenced* AttachmentDevice::doClone(CloneMap* cloneMap) const
{
    return new AttachmentDevice(*this, false, cloneMap);
}


void AttachmentDevice::forEachActualType(std::function<bool(const std::type_info& type)> func)
{
    if(!func(typeid(AttachmentDevice))){
        Device::forEachActualType(func);
    }
}


bool AttachmentDevice::on() const
{
    return on_;
}


void AttachmentDevice::on(bool on)
{
    on_ = on;
}


HolderDevice* AttachmentDevice::holder()
{
    return weak_holder.lock();
}


void AttachmentDevice::setHolder(HolderDevice* holder)
{
    weak_holder = holder;
}


void AttachmentDevice::detach()
{
    if(auto holder = weak_holder.lock()){
        holder->removeAttachment(this);
    }
}


std::string AttachmentDevice::category() const
{
    if(category_){
        return *category_;
    }
    return string();
}


void AttachmentDevice::setCategory(const std::string& category)
{
    clearCategory();
    category_ = new string;
    *category_ = category;
}


void AttachmentDevice::clearCategory()
{
    if(category_){
        delete category_;
    }
    category_ = nullptr;
}


int AttachmentDevice::stateSize() const
{
    return 1;
}
        

const double* AttachmentDevice::readState(const double* buf)
{
    int i = 0;
    on_ = buf[i++];
    return buf + i;
}


double* AttachmentDevice::writeState(double* out_buf) const
{
    int i = 0;
    out_buf[i++] = on_ ? 1.0 : 0.0;
    return out_buf + i;
}


bool AttachmentDevice::readDescription(YAMLBodyLoader& loader, Mapping& node)
{
    string symbol;
    if(node.read("category", symbol)){
        setCategory(symbol);
    } else {
        clearCategory();
    }
    return loader.readDevice(this, node);
}
