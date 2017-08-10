/*!
  @author Shin'ichiro Nakaoka
*/

#include "../ODESimulatorItem.h"
#include <cnoid/Py3Base>

namespace py = pybind11;
using namespace cnoid;

PYBIND11_PLUGIN(ODEPlugin)
{
    py::module::import("cnoid.BodyPlugin");

    py::module m("ODEPlugin", "ODEPlugin Python Module");

    py::class_< ODESimulatorItem, ODESimulatorItemPtr, SimulatorItem > odeSimulatorItemScope (m,"ODESimulatorItem");

    odeSimulatorItemScope
        .def(py::init<>())
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

    py::enum_<ODESimulatorItem::StepMode>(odeSimulatorItemScope, "StepMode")
        .value("STEP_ITERATIVE", ODESimulatorItem::StepMode::STEP_ITERATIVE)
        .value("STEP_BIG_MATRIX", ODESimulatorItem::StepMode::STEP_BIG_MATRIX)
        .value("NUM_STEP_MODES", ODESimulatorItem::StepMode::NUM_STEP_MODES)
        .export_values();

    PyItemList<ODESimulatorItem>(m, "ODESimulatorItemList");

    return m.ptr();
}

