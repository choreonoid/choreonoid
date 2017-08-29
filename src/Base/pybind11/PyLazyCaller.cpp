/*!
  @author Shin'ichiro Nakaoka
*/

#include "../LazyCaller.h"
#include <pybind11/pybind11.h>

using namespace cnoid;
namespace py = pybind11;

namespace {

struct PyFunc
{
    py::function func;

    PyFunc(py::function f) : func(f) { }

    void operator()() {
        py::gil_scoped_acquire lock;
        try {
            func();
        } catch(const py::error_already_set& ex) {
            py::print(ex.what());
        }
    }
};

}

namespace cnoid {

void exportPyLazyCaller(py::module m)
{
    m.def("callLater", [](py::function func){ cnoid::callLater(PyFunc(func)); });
    m.def("callSynchronously", [](py::function func){ cnoid::callSynchronously(PyFunc(func)); });
}

}
