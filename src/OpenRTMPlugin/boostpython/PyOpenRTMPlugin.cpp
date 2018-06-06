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
namespace py = boost::python;

BOOST_PYTHON_MODULE(OpenRTMPlugin)
{
    py::class_<RTCItem, RTCItemPtr, py::bases<Item>>("RTCItem")
        .def("setModuleName", &RTCItem::setModuleName)
        .def("setPeriodicType", &RTCItem::setPeriodicType)
        .def("setPeriodicRate", &RTCItem::setPeriodicRate);

    py::class_<BodyRTCItem, BodyRTCItemPtr, py::bases<Item>> bodyRTCItemClass("BodyRTCItem");
    bodyRTCItemClass
        .def("setControllerModule", &BodyRTCItem::setControllerModule)
        .def("setConfigMode", &BodyRTCItem::setConfigMode)
        .def("setConfigFile", &BodyRTCItem::setConfigFile)
        .def("setAutoConnectionMode", &BodyRTCItem::setAutoConnectionMode)
        .def("setPeriodicRate", &BodyRTCItem::setPeriodicRate);

    {
        py::scope bodyRTCItemScope = bodyRTCItemClass;
        py::enum_<BodyRTCItem::ConfigMode>("ConfigMode")
            .value("FILE", BodyRTCItem::CONF_FILE_MODE)
            .value("ALL", BodyRTCItem::CONF_ALL_MODE);
    }
             

    py::enum_<RTCItem::PERIODIC_TYPE>("PeriodicType")
        .value("PERIODIC_EXECUTION_CONTEXT", RTCItem::PERIODIC_EXECUTION_CONTEXT) 
        .value("SYNCH_EXT_TRIGGER", RTCItem::SYNCH_EXT_TRIGGER) 
        .value("EXT_TRIG_EXECUTION_CONTEXT", RTCItem::EXT_TRIG_EXECUTION_CONTEXT) 
        .value("SIMULATION_EXECUTION_CONTEXT", RTCItem::SIMULATION_EXECUTION_CONTEXT)
        .value("N_PERIODIC_TYPE", RTCItem::N_PERIODIC_TYPE);
}
