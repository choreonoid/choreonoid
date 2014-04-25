/*!
 * @author Shin'ichiro Nakaoka
 * @author Hisashi Ikari
 */
#include <boost/python.hpp>

// This header is an "EXPLICIT" item of dependence.
// And this items are located in the "cnoid/include" or same directory.
// Its meaning is exposed externally. For reason, we can will be selected here.

#include <cnoid/Referenced>
#include <cnoid/Item>
#include <cnoid/RootItem>
#include <cnoid/BodyItem>
#include <cnoid/WorldItem>

#include "../../Base/python/PyBase.h"
#include "../RTCItem.h"
#include "../BodyRTCItem.h"

// We recommend the use of minieigen.
using namespace boost::python;
using namespace cnoid;

namespace cnoid
{
    /*!
     * @brief Reference types are explicitly declare a function pointer. 
     *        Reference types share a reference to the original(in C++).
     *        But, it does not share the type of destination(in python).
     *        (Reason-1. Primitive types can not be a reference type.)
     *        (Reason-2. Minieigen create an object always.)
     *        Therefore, we must use the setter if you want to use the python.
     *        We will define the settings of the following variables.       
     */
    BOOST_PYTHON_MODULE(OpenRTMPlugin)
    {
        /*!
         * @brief Define the interface for RTCItem.
         */
        class_ < RTCItem, ref_ptr<RTCItem>, bases<Item>, boost::noncopyable >("RTCItem", init<>())
            .def("__init__", boost::python::make_constructor(&createInstance<RTCItem>))
            .def("setModuleName", &RTCItem::setModuleName)
            .def("setPeriodicType", &RTCItem::setPeriodicType)
            .def("setPeriodicRate", &RTCItem::setPeriodicRate);


        /*!
         * @brief Define the interface for RTCItem.
         */
        class_ < BodyRTCItem, ref_ptr<BodyRTCItem>, bases<Item>, boost::noncopyable >("BodyRTCItem", init<>())
            .def("__init__", boost::python::make_constructor(&createInstance<BodyRTCItem>))
            .def("setModuleName", &BodyRTCItem::setModuleName)
            .def("setAutoConnectionMode", &BodyRTCItem::setAutoConnectionMode);


        /*!
         * @brief Provides following enum value of the RTCItem.
         */
        enum_ <RTCItem::PERIODIC_TYPE>("PeriodicType")
            .value("PERIODIC_EXECUTION_CONTEXT", RTCItem::PERIODIC_EXECUTION_CONTEXT) 
            .value("SYNCH_EXT_TRIGGER", RTCItem::SYNCH_EXT_TRIGGER) 
            .value("EXT_TRIG_EXECUTION_CONTEXT", RTCItem::EXT_TRIG_EXECUTION_CONTEXT) 
            .value("CHOREONOID_EXECUTION_CONTEXT", RTCItem::CHOREONOID_EXECUTION_CONTEXT) 
            .value("N_PERIODIC_TYPE", RTCItem::N_PERIODIC_TYPE);


    }

}; // end of namespace


