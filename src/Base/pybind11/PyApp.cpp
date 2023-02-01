#include "../App.h"
#include <cnoid/PyUtil>

using namespace cnoid;
namespace py = pybind11;

namespace cnoid {

void exportPyAppUtil(py::module m)
{
    py::class_<App>(m, "App")
        .def_static("updateGui", &App::updateGui)
        .def_static("exit", &App::exit, py::arg("returnCode") = 0)
        .def_property_readonly_static("sigAboutToQuit", [](py::object){ return App::sigAboutToQuit(); })
        ;
    
    // Deprecated functions defined in AppUtil.h
    m.def("updateGui", &App::updateGui);
}

}
