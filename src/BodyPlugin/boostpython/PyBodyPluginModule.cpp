/*!
  @author Shin'ichiro Nakaoka
*/

#include <cnoid/PyUtil>

using namespace boost::python;
using namespace cnoid;

void exportBodyItem();
void exportItems();
void exportSimulationClasses();

BOOST_PYTHON_MODULE(BodyPlugin)
{
    boost::python::import("cnoid.Base");
    boost::python::import("cnoid.Body");

    exportBodyItem();
    exportItems();
    exportSimulationClasses();
}
