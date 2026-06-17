#include "../MuJoCoSimulatorItem.h"
#include <cnoid/PyBase>
#include <cnoid/PyEigenTypes>

using namespace cnoid;
namespace nb = nanobind;

NB_MODULE(MuJoCoPlugin, m)
{
    m.doc() = "Choreonoid MuJoCoPlugin module";

    nb::module_::import_("cnoid.BodyPlugin");

    nb::class_<MuJoCoSimulatorItem, SimulatorItem>(m, "MuJoCoSimulatorItem")
        .def(nb::init<>())
        .def("setGravity",
             [](MuJoCoSimulatorItem& self, const python::Vector3Arg& g){ self.setGravity(g.value); })
        .def_prop_ro("gravity", &MuJoCoSimulatorItem::gravity)
        ;

    PyItemList<MuJoCoSimulatorItem>(m, "MuJoCoSimulatorItemList");
}
