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

using namespace boost::python;
using namespace cnoid;

BOOST_PYTHON_MODULE(OpenRTMPlugin)
{
    class_< RTCItem, RTCItemPtr, bases<Item> >("RTCItem")
        .def("setModuleName", &RTCItem::setModuleName)
        .def("setPeriodicType", &RTCItem::setPeriodicType)
        .def("setPeriodicRate", &RTCItem::setPeriodicRate);

    class_< BodyRTCItem, BodyRTCItemPtr, bases<Item> > bodyRTCItemClass("BodyRTCItem");
    bodyRTCItemClass
        .def("setControllerModule", &BodyRTCItem::setControllerModule)
        .def("setConfigMode", &BodyRTCItem::setConfigMode)
        .def("setConfigFile", &BodyRTCItem::setConfigFile)
        .def("setAutoConnectionMode", &BodyRTCItem::setAutoConnectionMode)
        .def("setPeriodicRate", &BodyRTCItem::setPeriodicRate);

    {
        scope bodyRTCItemScope = bodyRTCItemClass;
        enum_<BodyRTCItem::ConfigMode>("ConfigMode")
            .value("FILE", BodyRTCItem::CONF_FILE_MODE)
            .value("ALL", BodyRTCItem::CONF_ALL_MODE);
    }
             

    enum_<RTCItem::PERIODIC_TYPE>("PeriodicType")
        .value("PERIODIC_EXECUTION_CONTEXT", RTCItem::PERIODIC_EXECUTION_CONTEXT) 
        .value("SYNCH_EXT_TRIGGER", RTCItem::SYNCH_EXT_TRIGGER) 
        .value("EXT_TRIG_EXECUTION_CONTEXT", RTCItem::EXT_TRIG_EXECUTION_CONTEXT) 
        .value("CHOREONOID_EXECUTION_CONTEXT", RTCItem::CHOREONOID_EXECUTION_CONTEXT) 
        .value("N_PERIODIC_TYPE", RTCItem::N_PERIODIC_TYPE);
}



