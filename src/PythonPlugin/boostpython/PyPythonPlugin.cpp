/*
 * @author Shin'ichiro Nakaoka
 */

#include <boost/python.hpp>

namespace python = boost::python;

BOOST_PYTHON_MODULE(PythonPlugin)
{
    // define the ExitException class which inherits the built-in Exception class
    python::object mainModule = python::import("__main__");
    python::object mainName = mainModule.attr("__name__");
    mainModule.attr("__name__") = python::scope().attr("__name__");
    python::exec("class ExitException (Exception): pass\n",
         mainModule.attr("__dict__"), python::scope().attr("__dict__"));
    //python object::ExitException = python::scope().attr("ExitException");
    mainModule.attr("__name__") = mainName;
}
