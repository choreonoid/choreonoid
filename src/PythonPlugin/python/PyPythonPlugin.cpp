#include "PythonPlugin.h"
#include <cnoid/PyBase>

using namespace cnoid;
namespace nb = nanobind;

NB_MODULE(PythonPlugin, m)
{
    m.doc() = "Choreonoid PythonPlugin module";

    nb::module_::import_("cnoid.Base");

    nb::class_<PythonPlugin, Plugin>(m, "PythonPlugin")
        .def_prop_ro_static(
            "instance",
            [](nb::handle){ return PythonPlugin::instance(); },
            nb::rv_policy::reference);
}
