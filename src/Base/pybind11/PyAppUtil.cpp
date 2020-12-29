#include "../AppUtil.h"
#include <pybind11/pybind11.h>

using namespace cnoid;
namespace py = pybind11;

namespace cnoid {

void exportPyAppUtil(py::module m)
{
    m.def("updateGui", &cnoid::updateGui);
}

}


