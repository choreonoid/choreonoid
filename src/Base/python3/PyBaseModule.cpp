/*!
  @author Shin'ichiro Nakaoka
*/

#include <pybind11/pybind11.h>

namespace cnoid {

void exportPyQtExTypes(pybind11::module m);
void exportPyItems(pybind11::module m);
void exportPyMainWindow(pybind11::module m);
void exportPyToolBars(pybind11::module m);
void exportPyViews(pybind11::module m);
void exportPyItemTreeView(pybind11::module m);
void exportPySceneTypes(pybind11::module m);
void exportPyLazyCaller(pybind11::module m);
void exportPyPluginManager(pybind11::module m);

PYBIND11_PLUGIN(Base)
{
    //! \todo check if this module is imported from the Choreonoid process with PythonPlugin

    pybind11::module m("Base", "Base Python Module");

    pybind11::module::import("cnoid.Util");
    pybind11::module::import("cnoid.QtGui");
    
    exportPyQtExTypes(m);
    exportPyItems(m);
    exportPyMainWindow(m);
    exportPyToolBars(m);
    exportPyViews(m);
    exportPyItemTreeView(m);
    exportPySceneTypes(m);
    exportPyLazyCaller(m);
    exportPyPluginManager(m);

    return m.ptr();
}

}
