/*!
  @author Shin'ichiro Nakaoka
*/

#include "../PythonSimScriptItem.h"
#include <cnoid/PyBase>

using namespace cnoid;

PYBIND11_MODULE(PythonSimScriptPlugin, m)
{
    m.doc() = "Choreonoid PythonSimScriptPlugin module";

    pybind11::class_< PythonSimScriptItem, PythonSimScriptItemPtr, SimulationScriptItem >(m, "PythonSimScriptItem")
        .def(pybind11::init<>())
        .def("setScriptFilename", &PythonSimScriptItem::setScriptFilename);

    PyItemList<PythonSimScriptItem>(m, "PythonSimScriptItemList");
};
