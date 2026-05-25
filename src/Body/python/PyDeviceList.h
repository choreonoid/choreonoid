#ifndef CNOID_BODY_PYTHON_PYDEVICELIST_H
#define CNOID_BODY_PYTHON_PYDEVICELIST_H

#include "../DeviceList.h"
#include <cnoid/PyUtil>

/*
   A cnoid::DeviceList<DeviceType> is exposed to Python as a plain list of the
   bound device objects. The type_caster below handles the C++ -> Python
   direction (used e.g. by Body::devices()); there is no Python -> C++ direction.

   In addition, a function named after each device type (e.g. "DeviceList",
   "ForceSensorList") is bound so that Python code can narrow an existing list of
   devices down to those that are instances of that type. The narrowing uses
   nanobind's nb::isinstance<DeviceType>(), which checks against the Python type
   registered for DeviceType, so it needs no access to nanobind internals (unlike
   the pybind11 version, which reached into get_internals()).
*/

namespace nanobind {
namespace detail {

template<typename DeviceType>
struct type_caster<cnoid::DeviceList<DeviceType>>
{
    NB_TYPE_CASTER(cnoid::DeviceList<DeviceType>, const_name("DeviceList"))

    bool from_python(handle, uint8_t, cleanup_list*) noexcept {
        return false;
    }

    static handle from_cpp(const cnoid::DeviceList<DeviceType>& src, rv_policy, cleanup_list*) noexcept {
        list retval;
        for(size_t i = 0; i < src.size(); ++i){
            retval.append(cast(src[i]));
        }
        return retval.release();
    }
};

} // namespace detail
} // namespace nanobind

namespace cnoid {
namespace python {

namespace nb = nanobind;

// Returns the subset of the given device list whose elements are instances of
// DeviceType (in the Python type sense).
template<typename DeviceType>
nb::list narrowDeviceList(nb::list devices)
{
    nb::list narrowed;
    for(auto item : devices){
        if(nb::isinstance<DeviceType>(item)){
            narrowed.append(item);
        }
    }
    return narrowed;
}

// Binds a module-level function 'name' that narrows a device list to DeviceType.
template<typename DeviceType>
void exportDeviceListNarrowFunction(nb::module_& m, const char* name)
{
    m.def(name, &narrowDeviceList<DeviceType>);
}

} // namespace python
} // namespace cnoid

#endif
