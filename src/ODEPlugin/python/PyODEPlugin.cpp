#include "../ODESimulatorItem.h"
#include <cnoid/PyBase>
#include <cnoid/PyEigenTypes>

using namespace cnoid;
namespace nb = nanobind;

NB_MODULE(ODEPlugin, m)
{
    m.doc() = "Choreonoid ODEPlugin module";

    nb::module_::import_("cnoid.BodyPlugin");

    nb::class_<ODESimulatorItem, SimulatorItem> odeSimulatorItemScope(m, "ODESimulatorItem");

    odeSimulatorItemScope
        .def(nb::init<>())
        .def("setStepMode", &ODESimulatorItem::setStepMode)
        .def("setGravity",
             [](ODESimulatorItem& self, const python::Vector3Arg& g){ self.setGravity(g.value); })
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

    nb::enum_<ODESimulatorItem::StepMode>(odeSimulatorItemScope, "StepMode")
        .value("STEP_ITERATIVE", ODESimulatorItem::StepMode::STEP_ITERATIVE)
        .value("STEP_BIG_MATRIX", ODESimulatorItem::StepMode::STEP_BIG_MATRIX)
        .value("NUM_STEP_MODES", ODESimulatorItem::StepMode::NUM_STEP_MODES)
        .export_values();

    PyItemList<ODESimulatorItem>(m, "ODESimulatorItemList");
}
