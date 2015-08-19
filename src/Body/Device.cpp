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
    ns->link = 0;
    //T().setIdentity();
    T_local().setIdentity();
    setCycle(20.0);
}


Device::Device(const Device& org, bool copyAll)
{
    if(!copyAll){
        ns = 0;
    } else {
        ns = new NonState;
        ns->index = -1;
        ns->id = org.ns->id;
        ns->name = org.ns->name;
        ns->link = 0;
        //T() = org.T();
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


/*
  void Device::updatePosition()
  {
  setRotation(link()->attitude() * localRotation());
  setTranslation(link()->attitude() * localTranslation() + link()->p);
  }
*/


void Device::forEachActualType(boost::function<bool(const std::type_info& type)> func)
{
    func(typeid(Device));
}


void ActiveDevice::forEachActualType(boost::function<bool(const std::type_info& type)> func)
{
    if(!func(typeid(ActiveDevice))){
        Device::forEachActualType(func);
    }
}


void Device::clearState()
{

}

