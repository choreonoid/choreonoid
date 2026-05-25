#include <cnoid/PyUtil>

namespace nb = nanobind;

namespace cnoid {

void exportPyQtExTypes(nb::module_ m);
void exportPyArchive(nb::module_ m);
void exportPyAppUtil(nb::module_ m);
void exportPyItems(nb::module_ m);
void exportPySeqItems(nb::module_ m);
void exportPyMainWindow(nb::module_ m);
void exportPyToolBars(nb::module_ m);
void exportPyViews(nb::module_ m);
void exportPyItemTreeView(nb::module_ m);
void exportPySceneTypes(nb::module_ m);
void exportPyLazyCaller(nb::module_ m);
void exportPyPluginManager(nb::module_ m);
void exportPyProjectManager(nb::module_ m);
void exportPyExtensionManagers(nb::module_ m);

}

using namespace cnoid;

NB_MODULE(Base, m)
{
    m.doc() = "Choreonoid Base module";

    nb::module_::import_("cnoid.Util");
    nb::module_::import_("cnoid.QtGui");
    nb::module_::import_("cnoid.QtWidgets");

    exportPyQtExTypes(m);
    exportPyArchive(m);
    exportPyAppUtil(m);
    exportPyItems(m);
    exportPySeqItems(m);
    exportPyMainWindow(m);
    exportPyToolBars(m);
    exportPyViews(m);
    exportPyItemTreeView(m);
    exportPySceneTypes(m);
    exportPyLazyCaller(m);
    exportPyPluginManager(m);
    exportPyProjectManager(m);
    exportPyExtensionManagers(m);
}
