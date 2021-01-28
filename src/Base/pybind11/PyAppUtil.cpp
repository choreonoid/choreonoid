#include "../AppUtil.h"
#include <cnoid/PyUtil>

using namespace cnoid;
namespace py = pybind11;

namespace cnoid {

void exportPyAppUtil(py::module m)
{
    m.def("updateGui", &cnoid::updateGui);
}

}
