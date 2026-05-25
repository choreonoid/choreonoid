#include "../App.h"
#include <cnoid/PyUtil>

using namespace cnoid;
namespace nb = nanobind;

namespace cnoid {

void exportPyAppUtil(nb::module_ m)
{
    nb::class_<App>(m, "App")
        .def_static("updateGui", &App::updateGui, nb::arg("allEvents") = false)
        .def_static("exit", &App::exit, nb::arg("returnCode") = 0)
        .def_prop_ro_static("sigAboutToQuit", [](nb::handle){ return App::sigAboutToQuit(); })
        ;
}

}
