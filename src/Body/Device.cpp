#include "Device.h"
#include "Link.h"

using namespace cnoid;


// Implementation for backward compatibility
const double* DeviceState::readState(const double* buf, int size)
{
    if(stateSize() <= size){
        return readState(buf);
    } else {
        return buf;
    }
}


const double* DeviceState::readState(const double* buf)
{
    return buf;
}


Device::Device()
{
    ns = new NonState;
    ns->index = -1;
    ns->id = -1;
    ns->link = nullptr;
    T_local().setIdentity();

    info_ = nullptr;
}


Device::Device(const Device& org, bool copyStateOnly)
{
    if(copyStateOnly){
        ns = nullptr;
    } else {
        ns = new NonState;
        ns->index = -1;
        ns->link = nullptr;
        copySpecFrom(&org);
    }
    if(org.info_){
        info_ = org.info_->deepClone();
    } else {
        info_ = nullptr;
    }
}


void Device::copySpecFrom(const Device* other)
{
    ns->id = other->ns->id;
    ns->name = other->ns->name;
    ns->T_local = other->ns->T_local;
}


bool Device::copyFrom(const Device* other)
{
    copySpecFrom(other);
    return true;
}


Device::~Device()
{
    if(ns){
        delete ns;
    }
}


void Device::forEachActualType(std::function<bool(const std::type_info& type)> func)
{
    func(typeid(Device));
}


const Body* Device::body() const
{
    return ns->link ? ns->link->body() : nullptr;
}


Body* Device::body()
{
    return ns->link ? ns->link->body() : nullptr;
}


void Device::setLocalAttitude(const Isometry3& Ta)
{
    T_local() = Ta;
}


void Device::clearState()
{

}


bool Device::on() const
{
    return true;
}


void Device::on(bool)
{

}
