/*!
  @author Shin'ichiro Nakaoka
*/

#include <boost/python.hpp>

using namespace cnoid;

namespace cnoid {

void exportPyQtExTypes();
void exportPyItems();
void exportPyMainWindow();
void exportPyToolBars();
void exportPyViews();
void exportPyItemTreeView();
void exportPySceneTypes();
void exportPyLazyCaller();
void exportPyPluginManager();

}

BOOST_PYTHON_MODULE(Base)
{
    boost::python::import("cnoid.Util");
    boost::python::import("cnoid.QtGui");
    
    exportPyQtExTypes();
    exportPyItems();
    exportPyMainWindow();
    exportPyToolBars();
    exportPyViews();
    exportPyItemTreeView();
    exportPySceneTypes();
    exportPyLazyCaller();
    exportPyPluginManager();
}
