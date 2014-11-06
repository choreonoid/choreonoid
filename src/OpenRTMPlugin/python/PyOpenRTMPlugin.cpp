/*!
  @author Shin'ichiro Nakaoka
*/

#include "../RTCItem.h"
#include "../BodyRTCItem.h"
#include <cnoid/PyUtil>

using namespace boost::python;
using namespace cnoid;

BOOST_PYTHON_MODULE(OpenRTMPlugin)
{
    class_< RTCItem, RTCItemPtr, bases<Item> >("RTCItem")
            .def("setModuleName", &RTCItem::setModuleName)
            .def("setPeriodicType", &RTCItem::setPeriodicType)
            .def("setPeriodicRate", &RTCItem::setPeriodicRate);

    class_< BodyRTCItem, BodyRTCItemPtr, bases<Item> >("BodyRTCItem")
            .def("setModuleName", &BodyRTCItem::setModuleName)
            .def("setAutoConnectionMode", &BodyRTCItem::setAutoConnectionMode);

    enum_<RTCItem::PERIODIC_TYPE>("PeriodicType")
        .value("PERIODIC_EXECUTION_CONTEXT", RTCItem::PERIODIC_EXECUTION_CONTEXT) 
        .value("SYNCH_EXT_TRIGGER", RTCItem::SYNCH_EXT_TRIGGER) 
        .value("EXT_TRIG_EXECUTION_CONTEXT", RTCItem::EXT_TRIG_EXECUTION_CONTEXT) 
        .value("CHOREONOID_EXECUTION_CONTEXT", RTCItem::CHOREONOID_EXECUTION_CONTEXT) 
        .value("N_PERIODIC_TYPE", RTCItem::N_PERIODIC_TYPE);
}



