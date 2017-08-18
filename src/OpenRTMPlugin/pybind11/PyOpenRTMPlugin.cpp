/*!
  @author Shin'ichiro Nakaoka
*/

#include "../RTCItem.h"
#include "../deprecated/BodyRTCItem.h"

#ifdef _WIN32
#undef HAVE_UNISTD_H
#if _MSC_VER < 1800
#undef HAVE_INTTYPES_H
#endif
#endif

#include <cnoid/PyUtil>

using namespace cnoid;
namespace py = pybind11;

PYBIND11_MODULE(OpenRTMPlugin, m)
{
    m.doc() = "Choreonoid OpenRTMPlugin module";

    py::module::import("cnoid.Base");

    py::class_<RTCItem, RTCItemPtr, Item> rtcItemClass(m, "RTCItem");
    rtcItemClass
        .def("setModuleName", &RTCItem::setModuleName)
        .def("setPeriodicType", &RTCItem::setPeriodicType)
        .def("setPeriodicRate", &RTCItem::setPeriodicRate);

    py::class_<BodyRTCItem, BodyRTCItemPtr, Item> bodyRTCItemClass(m, "BodyRTCItem");
    bodyRTCItemClass
        .def("setControllerModule", &BodyRTCItem::setControllerModule)
        .def("setConfigMode", &BodyRTCItem::setConfigMode)
        .def("setConfigFile", &BodyRTCItem::setConfigFile)
        .def("setAutoConnectionMode", &BodyRTCItem::setAutoConnectionMode)
        .def("setPeriodicRate", &BodyRTCItem::setPeriodicRate);

    py::enum_<BodyRTCItem::ConfigMode>(bodyRTCItemClass, "ConfigMode")
        .value("FILE", BodyRTCItem::ConfigMode::CONF_FILE_MODE)
        .value("ALL", BodyRTCItem::ConfigMode::CONF_ALL_MODE)
        .export_values();
             
    py::enum_<RTCItem::PERIODIC_TYPE>(rtcItemClass, "PeriodicType")
        .value("PERIODIC_EXECUTION_CONTEXT", RTCItem::PERIODIC_TYPE::PERIODIC_EXECUTION_CONTEXT)
        .value("SYNCH_EXT_TRIGGER", RTCItem::PERIODIC_TYPE::SYNCH_EXT_TRIGGER)
        .value("EXT_TRIG_EXECUTION_CONTEXT", RTCItem::PERIODIC_TYPE::EXT_TRIG_EXECUTION_CONTEXT)
        .value("CHOREONOID_EXECUTION_CONTEXT", RTCItem::PERIODIC_TYPE::CHOREONOID_EXECUTION_CONTEXT)
        .value("N_PERIODIC_TYPE", RTCItem::PERIODIC_TYPE::N_PERIODIC_TYPE)
        .export_values();
}
