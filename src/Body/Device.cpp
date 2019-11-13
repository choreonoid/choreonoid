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
        ns->id = org.ns->id;
        ns->name = org.ns->name;
        ns->link = nullptr;
        T_local() = org.T_local();
        setCycle(org.cycle());
    }
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
