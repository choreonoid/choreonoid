#include "../AGXSimulatorItem.h"
#include <cnoid/PyBase>

using namespace cnoid;
namespace nb = nanobind;

NB_MODULE(AGXDynamicsPlugin, m)
{
    m.doc() = "Choreonoid AGXDynamicsPlugin module";

    nb::module_::import_("cnoid.BodyPlugin");

    nb::class_<AGXSimulatorItem, SimulatorItem> agxSimulatorItemScope(m, "AGXSimulatorItem");

    agxSimulatorItemScope
        .def(nb::init<>())
        .def("getGravity", &AGXSimulatorItem::getGravity)
        .def("setNumThreads", &AGXSimulatorItem::setNumThreads)
        .def("setEnableContactReduction", &AGXSimulatorItem::setEnableContactReduction)
        .def("setContactReductionBinResolution", &AGXSimulatorItem::setContactReductionBinResolution)
        .def("setContactReductionThreshhold", &AGXSimulatorItem::setContactReductionThreshhold)
        .def("setEnableContactWarmstarting", &AGXSimulatorItem::setEnableContactWarmstarting)
        .def("setEnableAMOR", &AGXSimulatorItem::setEnableAMOR)
        ;

    PyItemList<AGXSimulatorItem>(m, "AGXSimulatorItemList");
}
