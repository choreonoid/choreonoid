/*!
  @author Shin'ichiro Nakaoka
*/

#include <boost/python.hpp>

namespace cnoid {

void exportPyQtExTypes();
void exportPyItems();
void exportPyToolBars();
void exportPyViews();
void exportPyItemTreeView();

BOOST_PYTHON_MODULE(Base)
{
    //! \todo check if this module is imported from the Choreonoid process with PythonPlugin

    
    boost::python::import("cnoid.Util");
    boost::python::import("cnoid.QtGui");
    
    exportPyQtExTypes();
    exportPyItems();
    exportPyToolBars();
    exportPyViews();
    exportPyItemTreeView();
}

}
