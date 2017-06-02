/*
 * @author Shin'ichiro Nakaoka
 */

#include <pybind11/pybind11.h>
#include <pybind11/eval.h>

namespace py = pybind11;

PYBIND11_PLUGIN(PythonPlugin)
{
    py::module m("PythonPlugin", "PythonPlugin Python Module");

    // define the ExitException class which inherits the built-in Exception class
    py::module mainModule = py::module::import("__main__");
    py::object mainName = mainModule.attr("__name__");
    mainModule.attr("__name__") = m.attr("__name__");
    py::eval<py::eval_single_statement>("class ExitException (Exception): pass\n",
          mainModule.attr("__dict__"), m.attr("__dict__"));
    //python object::ExitException = python::scope().attr("ExitException");
    mainModule.attr("__name__") = mainName;

    return m.ptr();
}
