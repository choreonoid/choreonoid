/*!
  @author Shin'ichiro Nakaoka
*/

#include <boost/python.hpp>

namespace cnoid {

void exportQtExTypes();
void exportItems();
void exportToolBars();
void exportViews();
void exportItemTreeView();

BOOST_PYTHON_MODULE(Base)
{
    //! \todo check if this module is imported from the Choreonoid process with PythonPlugin

    
    
    boost::python::import("cnoid.Util");
    boost::python::import("cnoid.QtGui");
    
    exportQtExTypes();
    exportItems();
    exportToolBars();
    exportViews();
    exportItemTreeView();
}

}
