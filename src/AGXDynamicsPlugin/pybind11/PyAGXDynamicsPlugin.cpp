/*!
  @author Ikumi Susa
*/


#include "../AGXSimulatorItem.h"
#include <cnoid/PyBase>

using namespace cnoid;
namespace py = pybind11;

PYBIND11_MODULE(AGXDynamicsPlugin, m)
{
    m.doc() = "Choreonoid AGXDynamicsPlugin module";

    py::class_<AGXSimulatorItem, SimulatorItem, AGXSimulatorItemPtr> agxSimulatorItemScope (m, "AGXSimulatorItem");

    agxSimulatorItemScope
        .def(py::init<>())
        .def("getGravity", (Vector3 (AGXSimulatorItem::*)()) &AGXSimulatorItem::getGravity)
        .def("setNumThreads", &AGXSimulatorItem::setNumThreads)
        .def("setEnableContactReduction", &AGXSimulatorItem::setEnableContactReduction)
        .def("setContactReductionBinResolution", &AGXSimulatorItem::setContactReductionBinResolution)
        .def("setContactReductionThreshhold", &AGXSimulatorItem::setContactReductionThreshhold)
        .def("setEnableContactWarmstarting", &AGXSimulatorItem::setEnableContactWarmstarting)
        .def("setEnableAMOR", &AGXSimulatorItem::setEnableAMOR)
        ;

    PyItemList<AGXSimulatorItem>(m, "AGXSimulatorItemList");
}
