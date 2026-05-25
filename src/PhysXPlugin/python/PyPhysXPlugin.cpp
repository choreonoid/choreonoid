#include "../PhysXSimulatorItem.h"
#include <cnoid/PyBase>
#include <cnoid/PyEigenTypes>

using namespace cnoid;
namespace nb = nanobind;

NB_MODULE(PhysXPlugin, m)
{
    m.doc() = "Choreonoid PhysXPlugin module";

    nb::module_::import_("cnoid.BodyPlugin");

    nb::class_<PhysXSimulatorItem, SimulatorItem>(m, "PhysXSimulatorItem")
        .def(nb::init<>())
        .def("setNumThreads", &PhysXSimulatorItem::setNumThreads)
        .def("setGravity",
             [](PhysXSimulatorItem& self, const python::Vector3Arg& g){ self.setGravity(g.value); })
        .def_prop_ro("gravity", &PhysXSimulatorItem::gravity)
        ;

    PyItemList<PhysXSimulatorItem>(m, "PhysXSimulatorItemList");
}
