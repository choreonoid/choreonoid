/*!
  @author Shin'ichiro Nakaoka
*/

#include <pybind11/pybind11.h>

namespace py = pybind11;

namespace cnoid {

void exportPyQtExTypes(py::module m);
void exportPyItems(py::module m);
void exportPyMainWindow(py::module m);
void exportPyToolBars(py::module m);
void exportPyViews(py::module m);
void exportPyItemTreeView(py::module m);
void exportPySceneTypes(py::module m);
void exportPyLazyCaller(py::module m);
void exportPyPluginManager(py::module m);
void exportPyProjectManager(py::module m);

}

using namespace cnoid;

PYBIND11_MODULE(Base, m)
{
    m.doc() = "Choreonoid Base module";

    py::module::import("cnoid.Util");
    py::module::import("cnoid.QtGui");
    
    exportPyQtExTypes(m);
    exportPyItems(m);
    exportPyMainWindow(m);
    exportPyToolBars(m);
    exportPyViews(m);
    exportPyItemTreeView(m);
    exportPySceneTypes(m);
    exportPyLazyCaller(m);
    exportPyPluginManager(m);
    exportPyProjectManager(m);
}
