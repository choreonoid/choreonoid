/*!
  @author Shin'ichiro Nakaoka
*/

#include "../LazyCaller.h"
#include <cnoid/PyUtil>
#include <cnoid/PythonUtil>

using namespace cnoid;
namespace py = pybind11;

namespace {

struct PyFunc
{
    py::object func;
    PyFunc(py::object f) : func(f) {
        if(!PyFunction_Check(f.ptr()) && !PyMethod_Check(f.ptr())){
            PyErr_SetString(PyExc_TypeError, "Task command must be a function type object");
            throw py::error_already_set();
        }
    }
    void operator()() {
        py::gil_scoped_acquire lock;
        try {
            func();
        } catch(py::error_already_set const& ex) {
            cnoid::handlePythonException();
        }
    }
};

}

namespace cnoid {

void exportPyLazyCaller(py::module m)
{
    m.def("callLater", [](py::object func){ cnoid::callLater(PyFunc(func)); });
    m.def("callSynchronously", [](py::object func){ cnoid::callSynchronously(PyFunc(func)); });
}

}
