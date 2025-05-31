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

    config = new Config;
    config->info = nullptr;
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

    config = new Config;
    copyConfigFrom(org);
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
    if(config){
        delete config;
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


void Device::copyInfoFrom(const Device& other)
{
    config->info = other.config->info->cloneMapping();
}


void Device::copyConfigFrom(const DeviceConfig& other)
{
    copyInfoFrom(static_cast<const Device&>(other));
}