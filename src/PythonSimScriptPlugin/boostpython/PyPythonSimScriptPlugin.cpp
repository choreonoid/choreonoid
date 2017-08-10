/*!
  @author Shin'ichiro Nakaoka
*/

#include "../PythonSimScriptItem.h"
#include <cnoid/PyBase>

using namespace boost::python;
using namespace cnoid;

BOOST_PYTHON_MODULE(PythonSimScriptPlugin)
{
    class_< PythonSimScriptItem, PythonSimScriptItemPtr, bases<SimulationScriptItem> >("PythonSimScriptItem")
        .def("setScriptFilename", &PythonSimScriptItem::setScriptFilename);

    implicitly_convertible<PythonSimScriptItemPtr, SimulationScriptItemPtr>();
    PyItemList<PythonSimScriptItem>("PythonSimScriptItemList");
};
