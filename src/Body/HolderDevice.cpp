#include "HolderDevice.h"
#include "AttachmentDevice.h"
#include "Body.h"
#include "StdBodyFileUtil.h"
#include <cnoid/CloneMap>
#include <cnoid/ValueTree>

using namespace std;
using namespace cnoid;

namespace cnoid {

class HolderDevice::NonState
{
public:
    vector<std::string> categories;
    int holdCondition;
    double maxHoldDistance;
    std::string holdTargetName;
    vector<AttachmentDevicePtr> attachments;
    NonState();
    NonState(const NonState& org, CloneMap* cloneMap);
};

}


HolderDevice::HolderDevice()
{
    on_ = false;
    ns = new NonState;
}


HolderDevice::NonState::NonState()
{
    holdCondition = Distance;
    maxHoldDistance = 0.1;
}


HolderDevice::HolderDevice(const HolderDevice& org, bool copyStateOnly, CloneMap* cloneMap)
    : Device(org, copyStateOnly)
{
    copyHolderDeviceStateFrom(&org);

    if(copyStateOnly){
        ns = nullptr;
    } else {
        ns = new NonState(*org.ns, cloneMap);
    }
}


HolderDevice::NonState::NonState(const NonState& org, CloneMap* cloneMap)
    : categories(org.categories),
      holdTargetName(org.holdTargetName)
{
    holdCondition = org.holdCondition;
    maxHoldDistance = org.maxHoldDistance;
    
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


const char* HolderDevice::typeName() const
{
    return "HolderDevice";
}


void HolderDevice::copyHolderDeviceStateFrom(const HolderDevice* other)
{
    on_ = other->on_;
}


void HolderDevice::copyStateFrom(const DeviceState& other)
{
    if(typeid(other) != typeid(HolderDevice)){
        throw std::invalid_argument("Type mismatch in the Device::copyStateFrom function");
    }
    copyHolderDeviceStateFrom(static_cast<const HolderDevice*>(&other));
}
    

DeviceState* HolderDevice::cloneState() const
{
    return new HolderDevice(*this, true, nullptr);

}


Referenced* HolderDevice::doClone(CloneMap* cloneMap) const
{
    return new HolderDevice(*this, false, cloneMap);
}


void HolderDevice::copyHolderDeviceFrom(const HolderDevice* other)
{
    ns->categories = other->ns->categories;
    ns->holdCondition = other->ns->holdCondition;
    ns->maxHoldDistance = other->ns->maxHoldDistance;
    ns->holdTargetName = other->ns->holdTargetName;
    copyHolderDeviceStateFrom(other);
}
    

bool HolderDevice::copyFrom(const Device* other)
{
    if(auto otherHolder = dynamic_cast<const HolderDevice*>(other)){
        copyHolderDeviceFrom(otherHolder);
        return true;
    }
    return false;
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


std::vector<std::string> HolderDevice::categories() const
{
    if(ns){
        return ns->categories;
    }
    return vector<string>();
}


bool HolderDevice::hasCategories() const
{
    return ns && !ns->categories.empty();
}


bool HolderDevice::hasCategory(const char* category) const
{
    if(ns){
        for(auto& c : ns->categories){
            if(c == category){
                return true;
            }
        }
    }
    return false;
}


bool HolderDevice::hasCategory(const std::string& category) const
{
    return hasCategory(category.c_str());
}

            
void HolderDevice::addCategory(const std::string& category)
{
    if(!category.empty() && ns){
        ns->categories.push_back(category);
    }
}


void HolderDevice::clearCategories()
{
    if(ns){
        ns->categories.clear();
    }
}


std::string HolderDevice::category() const
{
    if(ns && !ns->categories.empty()){
        return ns->categories.front();
    }
    return string();
}


void HolderDevice::setCategory(const std::string& category)
{
    if(ns){
        ns->categories.clear();
        ns->categories.push_back(category);
    }
}


int HolderDevice::holdCondition() const
{
    return ns->holdCondition;
}


void HolderDevice::setHoldCondition(int condition)
{
    ns->holdCondition = condition;
}


double HolderDevice::maxHoldDistance() const
{
    return ns->maxHoldDistance;
}


void HolderDevice::setMaxHoldDistance(double distance)
{
    ns->maxHoldDistance = distance;
}


const std::string& HolderDevice::holdTargetName() const
{
    return ns->holdTargetName;
}


void HolderDevice::setHoldTargetName(const std::string& name)
{
    ns->holdTargetName = name;
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
            return true;
        }
    }
    return false;
}


void HolderDevice::removeAttachment(int index)
{
    if(ns){
        auto& attachments = ns->attachments;
        auto& attachment = attachments[index];
        attachment->setHolder(nullptr);
        attachments.erase(attachments.begin() + index);
    }
}


bool HolderDevice::removeAttachment(AttachmentDevice* attachment)
{
    if(ns){
        auto& attachments = ns->attachments;
        const int n = attachments.size();
        for(int i=0; i < n; ++i){
            if(attachments[i] == attachment){
                removeAttachment(i);
                return true;
            }
        }
    }
    return false;
}


void HolderDevice::clearAttachments()
{
    if(ns){
        for(auto& attachment : ns->attachments){
            attachment->setHolder(nullptr);
        }
        ns->attachments.clear();
    }
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


bool HolderDevice::readDescription(const Mapping* info)
{
    string symbol;
    
    clearCategories();
    if(auto& categories = *info->findListing("categories")){
        for(auto& category : categories){
            addCategory(category->toString());
        }
    } else if(info->read("category", symbol)){
        addCategory(symbol);
    }

    if(info->read("hold_condition", symbol)){
        if(symbol == "distance"){
            ns->holdCondition = Distance;
        } else if(symbol == "collision"){
            ns->holdCondition = Collision;
        } else if(symbol == "name"){
            ns->holdCondition = Name;
        }
    }
    info->read("max_hold_distance", ns->maxHoldDistance);
    info->read("hold_target_name", ns->holdTargetName);
    
    return true;
}


bool HolderDevice::writeDescription(Mapping* info) const
{
    if(!ns->categories.empty()){
        if(ns->categories.size() == 1){
            // For backward compatibility
            info->write("category", ns->categories.front());
        } else {
            auto categories = info->openListing("categories");
            for(auto& category : ns->categories){
                categories->append(category);
            }
        }
    }
    string condition;
    switch(ns->holdCondition){
    case HolderDevice::Collision: condition = "collision"; break;
    case HolderDevice::Name: condition = "name"; break;
    default: /* HolderDevice::Distance */ condition = "distance"; break;
    }
    info->write("hold_condition", condition);

    info->write("max_hold_distance", ns->maxHoldDistance);
    if(!ns->holdTargetName.empty()){
        info->write("hold_target_name", ns->holdTargetName);
    }
    return true;
}


namespace {

StdBodyFileDeviceTypeRegistration<HolderDevice>
registerHolderDevice(
    "Holder",
    [](StdBodyLoader* loader, const Mapping* info){
        HolderDevicePtr holder = new HolderDevice;
        if(holder->readDescription(info)){
            return loader->readDevice(holder, info);
        }
        return false;
    },
    [](StdBodyWriter* /* writer */, Mapping* info, const HolderDevice* holder){
        return holder->writeDescription(info);
    });
}
