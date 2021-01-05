#include "PyDeviceList.h"

using namespace cnoid;
namespace py = pybind11;

namespace {

void checkIfDeviceSubclass(PyObject* typeObject)
{
    auto &types = py::detail::get_internals().registered_types_cpp;
    auto it = types.find(std::type_index(typeid(Device)));
    pybind11::detail::type_info* deviceClass = nullptr;
    if (it != types.end())
        deviceClass = (pybind11::detail::type_info *) it->second;

    int isSubclass = PyObject_IsSubclass(typeObject, (PyObject*)deviceClass->type);
    if(isSubclass <= 0){
        PyErr_SetString(PyExc_TypeError, "argument for specifying the device list type must be an Device class");
        throw py::error_already_set();
    }
}

}

namespace cnoid {

py::list getPyDeviceList(const DeviceList<>& orgDeviceList)
{
    py::list deviceList;
    for(size_t i=0; i < orgDeviceList.size(); ++i){
        py::object device(py::cast(orgDeviceList[i]));
        deviceList.append(device);
    }
    return deviceList;
}


py::list getPyNarrowedDeviceList(const DeviceList<>& orgDeviceList, py::object deviceClass)
{
    checkIfDeviceSubclass(deviceClass.ptr());

    py::list narrowedDeviceList;
    for(size_t i=0; i < orgDeviceList.size(); ++i){
        py::object device(py::cast(orgDeviceList[i]));
        if(PyObject_IsInstance(device.ptr(), deviceClass.ptr()) > 0){
            narrowedDeviceList.append(device);
        }
    }
    return narrowedDeviceList;
}


py::list getPyNarrowedDeviceList(py::list orgDeviceList, const PyTypeObject* deviceClass)
{
    checkIfDeviceSubclass((PyObject*)deviceClass);
    
    py::list narrowedDeviceList;
    const int n = py::len(orgDeviceList);
    for(int i=0; i < n; ++i){
        py::object device = orgDeviceList[i];
        if(PyObject_IsInstance(device.ptr(), (PyObject*)deviceClass) > 0){
            narrowedDeviceList.append(device);
        }
    }
    return narrowedDeviceList;
}


py::object getPyNarrowedFirstDevice(const DeviceList<>& orgDeviceList, py::object deviceClass)
{
    checkIfDeviceSubclass(deviceClass.ptr());

    for(size_t i=0; i < orgDeviceList.size(); ++i){
        py::object device(py::cast(orgDeviceList[i]));
        if(PyObject_IsInstance(device.ptr(), deviceClass.ptr()) > 0){
            return device;
        }
    }
    return py::object();
}

}
