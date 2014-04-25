/*!
 * @author Hisashi Ikari
 */
#include <boost/python.hpp>

// This header is an "EXPLICIT" item of dependence.
// And this items are located in the "cnoid/include" or same directory.
// Its meaning is exposed externally. For reason, we can will be selected here.

#include <cnoid/Referenced>
#include <cnoid/Item>
#include <cnoid/RootItem>
#include <cnoid/WorldItem>
#include <cnoid/SimulatorItem>

#include "../../Base/python/PyBase.h"
#include "../ODESimulatorItem.h"

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
    BOOST_PYTHON_MODULE(ODEPlugin)
    {
        /*!
         * @brief Define the interface for ODESimulatorItem.
         * @note  !!! IMPORTANT !!! Please load the cnoid.BodyPlugin before the reading of this module. 
         *                          BodyPlugin loads the SimulatorItem.
         */
        class_ < ODESimulatorItem, ref_ptr<ODESimulatorItem>, 
            bases<SimulatorItem>, boost::noncopyable >("ODESimulatorItem", init<>())
            .def("__init__", boost::python::make_constructor(&createInstance<ODESimulatorItem>))
            .def("setAllLinkPositionOutputMode", &ODESimulatorItem::setAllLinkPositionOutputMode)
            .def("setStepMode", &ODESimulatorItem::setStepMode)
            .def("setGravity", &ODESimulatorItem::setGravity)
            .def("setFriction", &ODESimulatorItem::setFriction)
            .def("setJointLimitMode", &ODESimulatorItem::setJointLimitMode)
            .def("set2Dmode", &ODESimulatorItem::set2Dmode)
            .def("setGlobalERP", &ODESimulatorItem::setGlobalERP)
            .def("setGlobalCFM", &ODESimulatorItem::setGlobalCFM)
            .def("setNumIterations", &ODESimulatorItem::setNumIterations)
            .def("setOverRelaxation", &ODESimulatorItem::setOverRelaxation)
            .def("setCorrectingVelocityLimitMode", &ODESimulatorItem::setCorrectingVelocityLimitMode)
            .def("setMaxCorrectingVelocity", &ODESimulatorItem::setMaxCorrectingVelocity)
            .def("setSurfaceLayerDepth", &ODESimulatorItem::setSurfaceLayerDepth)
            .def("useWorldCollisionDetector", &ODESimulatorItem::useWorldCollisionDetector);
 

        /*!
         * @brief Provides following enum value of the BodyItem.
         */
        enum_ <ODESimulatorItem::StepMode>("ODEStepMode")
            .value("STEP_ITERATIVE", ODESimulatorItem::STEP_ITERATIVE)
            .value("STEP_BIG_MATRIX", ODESimulatorItem::STEP_BIG_MATRIX)
            .value("NUM_STEP_MODES", ODESimulatorItem::NUM_STEP_MODES);    
        
    }

}; // end of namespace
