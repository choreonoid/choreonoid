#ifndef CNOID_BODY_PYDEVICELIST_H
#define CNOID_BODY_PYDEVICELIST_H

#include "../DeviceList.h"
#include <cnoid/PyUtil>
//#include "exportdecl.h"

namespace pybind11 { namespace detail {
    template<typename DeviceType> struct type_caster<cnoid::DeviceList<DeviceType>> {
    public:
        PYBIND11_TYPE_CASTER(cnoid::DeviceList<DeviceType>, _("DeviceList"));

        //Conversion part 1 (Python->C++)
        bool load(handle src, bool) {
            return false;
        }

        //Conversion part 2 (C++ -> Python)
        static handle cast(cnoid::DeviceList<DeviceType> src, return_value_policy, handle ) {
            pybind11::list retval;
            for(size_t i=0; i < src.size(); i++){
                retval.append(src[i]);
            }
            return  retval.inc_ref();
        }
    };
}}


namespace cnoid {

//CNOID_EXPORT
pybind11::list getPyDeviceList(const DeviceList<>& orgDeviceList);
//CNOID_EXPORT
pybind11::list getPyNarrowedDeviceList(const DeviceList<>& orgDeviceList, pybind11::object deviceClass);
//CNOID_EXPORT
pybind11::list getPyNarrowedDeviceList(pybind11::list orgDeviceList, const PyTypeObject* deviceClass);
//CNOID_EXPORT
pybind11::object getPyNarrowedFirstDevice(const DeviceList<>& orgDeviceList, pybind11::object deviceClass);

template<typename DeviceType>
class PyDeviceList
{
    static pybind11::object deviceType;
    
    static pybind11::list construct1(pybind11::list devices){
        auto &types = pybind11::detail::get_internals().registered_types_cpp;
        auto it = types.find(std::type_index(typeid(DeviceType)));
        pybind11::detail::type_info* deviceClass = 0;
        if (it != types.end())
            deviceClass = (pybind11::detail::type_info *) it->second;

        return getPyNarrowedDeviceList(devices, deviceClass->type);
    }
    static pybind11::list construct2(pybind11::list devices){
        return getPyNarrowedDeviceList(devices, (PyTypeObject*)deviceType.ptr());
    }
public:
    PyDeviceList(pybind11::module m, const char* name){
        m.def(name, &PyDeviceList::construct1);
    }
    /**
       Use this constructor to avoid a compile error for a pure abstract class
    */
    PyDeviceList(pybind11::module m, const char* name, pybind11::object deviceType_){
        deviceType = deviceType_;
        m.def(name, &PyDeviceList::construct2);
    }
};

template<typename DeviceType> pybind11::object PyDeviceList<DeviceType>::deviceType;

}

#endif
