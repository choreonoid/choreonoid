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

namespace cnoid {

class HolderDevice::NonState
{
public:
    std::string category;
    vector<AttachmentDevicePtr> attachments;
    NonState(){ }
    NonState(const NonState& org, CloneMap* cloneMap);
};

}


HolderDevice::HolderDevice()
{
    on_ = false;
    ns = new NonState;
}


HolderDevice::HolderDevice(const HolderDevice& org, bool copyStateOnly, CloneMap* cloneMap)
    : Device(org, copyStateOnly)
{
    copyHolderDeviceStateFrom(org);

    if(copyStateOnly){
        ns = nullptr;
    } else {
        ns = new NonState(*org.ns, cloneMap);
    }
}


HolderDevice::NonState::NonState(const NonState& org, CloneMap* cloneMap)
    : category(org.category)
{
    if(cloneMap && !org.attachments.empty()){
        for(size_t i=0; i < org.attachments.size(); ++i){
            attachments.push_back(
                cloneMap->findCloneOrReplaceLater<AttachmentDevice>(
                    org.attachments[i],
                    [this, i](AttachmentDevice* clone){ attachments[i] = clone; }));
        }
    }
}
    

HolderDevice::~HolderDevice()
{
    if(ns){
        delete ns;
    }
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


std::string HolderDevice::category() const
{
    if(ns){
        return ns->category;
    }
    return string();
}


void HolderDevice::setCategory(const std::string& category)
{
    if(ns){
        ns->category = category;
    }
}


int HolderDevice::numAttachments() const
{
    if(ns){
        return ns->attachments.size();
    }
    return 0;
}


AttachmentDevice* HolderDevice::attachment(int index)
{
    if(ns){
        return ns->attachments[index];
    }
    return nullptr;
}

    
bool HolderDevice::addAttachment(AttachmentDevice* attachment)
{
    if(ns){
        auto& a = ns->attachments;
        if(std::find(a.begin(), a.end(), attachment) == a.end()){
            a.push_back(attachment);
            attachment->setHolder(this);
            attachment->on(true);
            on_ = true;
            return true;
        }
    }
    return false;
}
    

bool HolderDevice::removeAttachment(AttachmentDevice* attachment)
{
    if(ns){
        auto& a = ns->attachments;
        auto iter = a.begin();
        while(iter != a.end()){
            if(*iter == attachment){
                attachment->setHolder(nullptr);
                attachment->on(false);
                a.erase(iter);
                if(a.empty()){
                    on_ = false;
                }
                return true;
            }
            ++iter;
        }
    }
    return false;
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
    if(ns){
        if(!node.read("category", ns->category)){
            ns->category.clear();
        }
    }
    return loader.readDevice(this, node);
}
