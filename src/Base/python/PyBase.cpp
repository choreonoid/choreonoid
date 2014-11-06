/*!
  @author Shin'ichiro Nakaoka
*/

#include <boost/python.hpp>

void exportQtExTypes();
void exportItems();
void exportToolBars();
void exportViews();

BOOST_PYTHON_MODULE(Base)
{
    boost::python::import("cnoid.Util");
    boost::python::import("cnoid.QtGui");
    
    exportQtExTypes();
    exportItems();
    exportToolBars();
    //exportViews();
}
