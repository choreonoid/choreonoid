#include "../MulticopterSimulatorItem.h"
#include <cnoid/PyBase>

using namespace cnoid;
namespace nb = nanobind;

NB_MODULE(MulticopterPlugin, m)
{
    m.doc() = "Choreonoid MulticopterPlugin module";

    nb::module_::import_("cnoid.BodyPlugin");

    nb::class_<MulticopterSimulatorItem, SubSimulatorItem>(m, "MulticopterSimulatorItem")
        .def(nb::init<>())
        .def("setAirDefinitionFile", &MulticopterSimulatorItem::setAirDefinitionFile);

    PyItemList<MulticopterSimulatorItem>(m, "MulticopterSimulatorItemList");
}
