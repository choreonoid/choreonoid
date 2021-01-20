#include "PyUtil.h"

namespace py = pybind11;

namespace cnoid {

void exportPyReferenced(py::module& m)
{
    py::class_<Referenced, ReferencedPtr>(m, "Referenced");
}

}
