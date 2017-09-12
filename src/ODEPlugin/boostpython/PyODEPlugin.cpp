/*!
  @author Shin'ichiro Nakaoka
*/

#include "../ODESimulatorItem.h"
#include <cnoid/PyBase>

using namespace cnoid;
namespace py = boost::python;

BOOST_PYTHON_MODULE(ODEPlugin)
{
    py::import("cnoid.BodyPlugin");

    {
        py::scope odeSimulatorItemScope = 
            py::class_<ODESimulatorItem, ODESimulatorItemPtr, py::bases<SimulatorItem>>("ODESimulatorItem")
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
            .def("useWorldCollisionDetector", &ODESimulatorItem::useWorldCollisionDetector)
            ;

        py::enum_<ODESimulatorItem::StepMode>("StepMode")
            .value("STEP_ITERATIVE", ODESimulatorItem::STEP_ITERATIVE)
            .value("STEP_BIG_MATRIX", ODESimulatorItem::STEP_BIG_MATRIX)
            .value("NUM_STEP_MODES", ODESimulatorItem::NUM_STEP_MODES);    
    }

    py::implicitly_convertible<ODESimulatorItemPtr, SimulatorItemPtr>();
    PyItemList<ODESimulatorItem>("ODESimulatorItemList");
}
