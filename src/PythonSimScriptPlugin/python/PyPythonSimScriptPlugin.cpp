#include "../PythonSimScriptItem.h"
#include <cnoid/PyBase>

using namespace cnoid;
namespace nb = nanobind;

NB_MODULE(PythonSimScriptPlugin, m)
{
    m.doc() = "Choreonoid PythonSimScriptPlugin module";

    nb::module_::import_("cnoid.BodyPlugin");

    nb::class_<PythonSimScriptItem, SimulationScriptItem>(m, "PythonSimScriptItem")
        .def(nb::init<>())
        .def("setScriptFilename", &PythonSimScriptItem::setScriptFilename);

    PyItemList<PythonSimScriptItem>(m, "PythonSimScriptItemList");
}
