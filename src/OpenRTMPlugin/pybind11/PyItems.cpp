/*!
  @author Shin'ichiro Nakaoka
*/

#include "../RTCItem.h"
#include "../ControllerRTCItem.h"
#include "../BodyIoRTCItem.h"
#include "../deprecated/BodyRTCItem.h"
#if defined(_WINDOWS) && defined(HAVE_UNISTD_H)
#undef HAVE_UNISTD_H
#endif
#include <cnoid/PyBase>

using namespace cnoid;
namespace py = pybind11;

namespace cnoid {

void exportItems(py::module m)
{
    py::class_<RTCItem, RTCItemPtr, Item> rtcItemClass(m, "RTCItem");
    rtcItemClass
        .def("setModuleName", &RTCItem::setModuleName)
        .def("setPeriodicType", &RTCItem::setPeriodicType)
        .def("setPeriodicRate", &RTCItem::setPeriodicRate);

    py::enum_<RTCItem::PERIODIC_TYPE>(rtcItemClass, "PeriodicType")
        .value("PERIODIC_EXECUTION_CONTEXT", RTCItem::PERIODIC_TYPE::PERIODIC_EXECUTION_CONTEXT)
        .value("SYNCH_EXT_TRIGGER", RTCItem::PERIODIC_TYPE::SYNCH_EXT_TRIGGER)
        .value("EXT_TRIG_EXECUTION_CONTEXT", RTCItem::PERIODIC_TYPE::EXT_TRIG_EXECUTION_CONTEXT)
        .value("SIMULATION_EXECUTION_CONTEXT", RTCItem::PERIODIC_TYPE::SIMULATION_EXECUTION_CONTEXT)
        .value("N_PERIODIC_TYPE", RTCItem::PERIODIC_TYPE::N_PERIODIC_TYPE)
        .export_values();
    
    PyItemList<RTCItem>(m, "RTCItemList");

    py::class_<ControllerRTCItem, ControllerRTCItemPtr, ControllerItem>(m, "ControllerRTCItem")
        .def(py::init<>())
        .def_property("rtcModuleName", &ControllerRTCItem::rtcModuleName, &ControllerRTCItem::setRTCModule)
        .def_property("rtcInstanceName", &ControllerRTCItem::rtcInstanceName, &ControllerRTCItem::setRTCInstanceName)
        ;

    PyItemList<ControllerRTCItem>(m, "ControllerRTCItemList");

    py::class_<BodyIoRTCItem, BodyIoRTCItemPtr, ControllerRTCItem>(m, "BodyIoRTCItem")
        .def(py::init<>())
        ;

    PyItemList<BodyIoRTCItem>(m, "BodyIoRTCItemList");

    py::class_<BodyRTCItem, BodyRTCItemPtr, ControllerItem> bodyRTCItemClass(m, "BodyRTCItem");
    bodyRTCItemClass
        .def("setControllerModule", &BodyRTCItem::setControllerModule)
        .def("setConfigMode", &BodyRTCItem::setConfigMode)
        .def("setConfigFile", &BodyRTCItem::setConfigFile)
        .def("setAutoConnectionMode", &BodyRTCItem::setAutoConnectionMode)
        .def("setPeriodicRate", &BodyRTCItem::setPeriodicRate);

    PyItemList<BodyRTCItem>(m, "BodyRTCItemList");

    py::enum_<BodyRTCItem::ConfigMode>(bodyRTCItemClass, "ConfigMode")
        .value("FILE", BodyRTCItem::ConfigMode::CONF_FILE_MODE)
        .value("ALL", BodyRTCItem::ConfigMode::CONF_ALL_MODE)
        .export_values();
}

}
