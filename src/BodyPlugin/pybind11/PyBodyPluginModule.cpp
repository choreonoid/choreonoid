/*!
  @author Shin'ichiro Nakaoka
*/

#include <cnoid/PyUtil>

using namespace cnoid;
namespace py = pybind11;

namespace cnoid {

void exportBodyItem(py::module m);
void exportItems(py::module m);
void exportSimulationClasses(py::module m);

}

PYBIND11_PLUGIN(BodyPlugin)
{
    py::module m("BodyPlugin", "BodyPlugin Python Module");

    py::module::import("cnoid.Base");
    py::module::import("cnoid.Body");

    exportBodyItem(m);
    exportItems(m);
    exportSimulationClasses(m);

    return m.ptr();
}
