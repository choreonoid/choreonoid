/**
   \file
   \author Shin'ichiro Nakaoka
*/

#include "Device.h"
#include "Link.h"

using namespace cnoid;


Device::Device()
{
    ns = new NonState;
    ns->index = -1;
    ns->id = -1;
    ns->link = nullptr;
    T_local().setIdentity();
    setCycle(20.0);
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
}


void Device::copySpecFrom(const Device* other)
{
    ns->id = other->ns->id;
    ns->name = other->ns->name;
    ns->T_local = other->ns->T_local;
    setCycle(other->cycle());
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


Position Device::T_local_org() const
{
    Position T;
    const auto& Rs = link()->Rs();
    T.linear() = Rs.transpose() * T_local().linear();
    T.translation() = Rs.transpose() * T_local().translation();
    return T;
}


void Device::setLocalAttitude(const Position& Ta)
{
    const auto& Rs = link()->Rs();
    T_local().linear() = Rs * Ta.linear();
    T_local().translation() = Rs * Ta.translation();
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
