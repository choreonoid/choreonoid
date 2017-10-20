/**
   \file
   \author Shin'ichiro Nakaoka
*/

#ifndef CNOID_BODY_DEVICE_LIST_H
#define CNOID_BODY_DEVICE_LIST_H

#include "Device.h"
#include <cnoid/PolymorphicReferencedArray>
#include "exportdecl.h"

namespace cnoid {

template <class DeviceType = Device, class PointerType = ref_ptr<DeviceType> >
class DeviceList : public PolymorphicReferencedArray<DeviceType, Device, PointerType>
{
    typedef PolymorphicReferencedArray<DeviceType, Device, PointerType> ArrayBase;

public:
    DeviceList() { }

    template <class RhsDeviceType>
    DeviceList(const DeviceList<RhsDeviceType>& rhs)
        : ArrayBase(rhs) { }

    DeviceList getSortedById() const {
        DeviceList sorted;
        for(size_t i=0; i < ArrayBase::size(); ++i){
            DeviceType* device = (*this)[i];
            const int id = device->id();
            if(id >= 0){
                if(static_cast<int>(sorted.size()) <= id){
                    sorted.resize(id + 1);
                }
                sorted[id] = device;
            }
        }
        return sorted;
    }
};

}

#endif
