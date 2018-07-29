/*!
  @author Shin'ichiro Nakaoka
*/

#include <cnoid/PyUtil>

namespace py = pybind11;

namespace cnoid {

void exportItems(py::module m);

}

PYBIND11_MODULE(OpenRTMPlugin, m)
{
    m.doc() = "Choreonoid OpenRTMPlugin module";

    py::module::import("cnoid.Base");
    py::module::import("cnoid.BodyPlugin");

    cnoid::exportItems(m);
}
