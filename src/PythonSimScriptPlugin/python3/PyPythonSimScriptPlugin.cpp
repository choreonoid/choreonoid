/*!
  @author Shin'ichiro Nakaoka
*/

#include "../PythonSimScriptItem.h"
#include <cnoid/Py3Base>

namespace py = pybind11;
using namespace cnoid;

PYBIND11_PLUGIN(PythonSimScriptPlugin)
{
    py::module m("PythonSimScriptPlugin", "PYthonSimScriptPlugin Python Module");

    py::class_< PythonSimScriptItem, PythonSimScriptItemPtr, SimulationScriptItem >(m, "PythonSimScriptItem")
        .def(py::init<>())
        .def("setScriptFilename", &PythonSimScriptItem::setScriptFilename);

    PyItemList<PythonSimScriptItem>(m, "PythonSimScriptItemList");

    return m.ptr();
};
