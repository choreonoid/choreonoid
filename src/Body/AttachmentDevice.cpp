#include "AttachmentDevice.h"
#include "HolderDevice.h"
#include "Body.h"
#include "StdBodyLoader.h"
#include "StdBodyFileUtil.h"
#include <cnoid/CloneMap>
#include <cnoid/ValueTree>

using namespace std;
using namespace cnoid;


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


const char* AttachmentDevice::typeName() const
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


bool AttachmentDevice::isAttaching() const
{
    return static_cast<bool>(weak_holder);
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


bool AttachmentDevice::readSpecifications(const Mapping* info)
{
    string symbol;
    if(info->read("category", symbol)){
        setCategory(symbol);
    } else {
        clearCategory();
    }
    return true;
}


bool AttachmentDevice::writeSpecifications(Mapping* info) const
{
    info->write("category", category_);
    return true;
}


namespace {

StdBodyFileDeviceTypeRegistration<AttachmentDevice>
registerAttachmentDevice(
    "Attachment",
    [](StdBodyLoader* loader, const Mapping* info){
        AttachmentDevicePtr attachment = new AttachmentDevice;
        if(attachment->readSpecifications(info)){
            return loader->readDevice(attachment, info);
        }
        return false;
    },
    [](StdBodyWriter* /* writer */, Mapping* info, const AttachmentDevice* attachment){
        return attachment->writeSpecifications(info);
    });

}
