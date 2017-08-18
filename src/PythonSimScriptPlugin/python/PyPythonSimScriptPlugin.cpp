/*!
  @author Shin'ichiro Nakaoka
*/

#include "../PythonSimScriptItem.h"
#include <cnoid/PyBase>

using namespace cnoid;

#ifdef CNOID_USE_PYBIND11

PYBIND11_MODULE(PythonSimScriptPlugin, m)
{
    m.doc() = "Choreonoid PythonSimScriptPlugin module";

    pybind11::class_< PythonSimScriptItem, PythonSimScriptItemPtr, SimulationScriptItem >(m, "PythonSimScriptItem")
        .def(pybind11::init<>())
        .def("setScriptFilename", &PythonSimScriptItem::setScriptFilename);

    PyItemList<PythonSimScriptItem>(m, "PythonSimScriptItemList");
};

#else

BOOST_PYTHON_MODULE(PythonSimScriptPlugin)
{
    boost::python::class_<
        PythonSimScriptItem, PythonSimScriptItemPtr, boost::python::bases<SimulationScriptItem>>
        ("PythonSimScriptItem")
        .def("setScriptFilename", &PythonSimScriptItem::setScriptFilename);

    boost::python::implicitly_convertible<PythonSimScriptItemPtr, SimulationScriptItemPtr>();
    PyItemList<PythonSimScriptItem>("PythonSimScriptItemList");
};

#endif
