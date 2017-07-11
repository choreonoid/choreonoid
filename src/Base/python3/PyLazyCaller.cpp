/*!
  @author Shin'ichiro Nakaoka
*/

#include "../LazyCaller.h"
#include <cnoid/Py3Util>
#include <cnoid/PythonUtil>

namespace py = pybind11;
using namespace cnoid;

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
            handlePythonException();
        }
    }
};


void cnoid_callLater(py::object func)
{
    cnoid::callLater(PyFunc(func));
}


void cnoid_callSynchronously(py::object func)
{
    cnoid::callSynchronously(PyFunc(func));
}


} // namespace

namespace cnoid {

void exportPyLazyCaller(py::module m)
{
    m.def("callLater", cnoid_callLater);
    m.def("callSynchronously", cnoid_callSynchronously);
}

}
