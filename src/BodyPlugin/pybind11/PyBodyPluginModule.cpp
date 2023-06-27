#include <cnoid/PyUtil>

using namespace cnoid;
namespace py = pybind11;

namespace cnoid {

void exportBodyItem(py::module m);
void exportItems(py::module m);
void exportSimulationClasses(py::module m);

}

PYBIND11_MODULE(BodyPlugin, m)
{
    m.doc() = "Choreonoid BodyPlugin module";

    auto base = py::module::import("cnoid.Base");
    py::module::import("cnoid.Body");

    exportBodyItem(m);
    exportItems(m);
    exportSimulationClasses(m);

    // For backward compatibility
    m.attr("ControllerLogItem") = base.attr("ReferencedObjectSeqItem");
}
