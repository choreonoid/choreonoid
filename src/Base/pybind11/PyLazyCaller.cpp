/*!
  @author Shin'ichiro Nakaoka
*/

#include "../LazyCaller.h"
#include <cnoid/PyUtil>

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
    py::class_<LazyCaller> lazyCaller(m, "LazyCaller");

    py::enum_<LazyCaller::Priority>(lazyCaller, "Priority")
        .value("HighPriority", LazyCaller::HighPriority)
        .value("NormalPriority", LazyCaller::NormalPriority)
        .value("LowPriority", LazyCaller::LowPriority)
        .value("MinimumPriority", LazyCaller::MinimumPriority)
        ;
    
    m.def("callLater",
          [](py::function func, int priority){
              cnoid::callLater(PyFunc(func), priority); },
          py::arg("func"), py::arg("priority") = LazyCaller::NormalPriority);
    
    m.def("callSynchronously",
          [](py::function func, int priority){
              cnoid::callSynchronously(PyFunc(func), priority); },
          py::arg("func"), py::arg("priority") = LazyCaller::NormalPriority);
}

}
