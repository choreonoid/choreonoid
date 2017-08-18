/*
 * @author Shin'ichiro Nakaoka
 */
#include <cnoid/Config>

#ifdef CNOID_USE_PYBIND11

#include <pybind11/eval.h>

namespace py = pybind11;

PYBIND11_MODULE(PythonPlugin, m)
{
    m.doc() = "Choreonoid PythonPlugin module";

    // define the ExitException class which inherits the built-in Exception class
    py::module mainModule = py::module::import("__main__");
    py::object mainName = mainModule.attr("__name__");
    mainModule.attr("__name__") = m.attr("__name__");
    py::eval<py::eval_single_statement>("class ExitException (Exception): pass\n",
          mainModule.attr("__dict__"), m.attr("__dict__"));
    //python object::ExitException = python::scope().attr("ExitException");
    mainModule.attr("__name__") = mainName;
}

#else

#include <boost/python.hpp>
namespace py = boost::python;

BOOST_PYTHON_MODULE(PythonPlugin)
{
    // define the ExitException class which inherits the built-in Exception class
    py::object mainModule = py::import("__main__");
    py::object mainName = mainModule.attr("__name__");
    mainModule.attr("__name__") = py::scope().attr("__name__");
    py::exec("class ExitException (Exception): pass\n",
         mainModule.attr("__dict__"), py::scope().attr("__dict__"));
    //py object::ExitException = py::scope().attr("ExitException");
    mainModule.attr("__name__") = mainName;
}

#endif
