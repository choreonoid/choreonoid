/*!
  @author Shin'ichiro Nakaoka
*/

#include "../SimulationBar.h"
#include "../SimulatorItem.h"
#include <cnoid/PyUtil>

using namespace boost::python;
using namespace cnoid;

void exportItems();

BOOST_PYTHON_MODULE(BodyPlugin)
{
    boost::python::import("cnoid.Base");
    boost::python::import("cnoid.Body");
    
    exportItems();

    void (SimulationBar::*SimulationBar_startSimulation1)(SimulatorItem*, bool) = &SimulationBar::startSimulation;
    void (SimulationBar::*SimulationBar_startSimulation2)(bool) = &SimulationBar::startSimulation;
    
    class_<SimulationBar, SimulationBar*, boost::noncopyable>("SimulationBar", no_init)
        .def("instance", &SimulationBar::instance, return_value_policy<reference_existing_object>()).staticmethod("instance")
        .def("startSimulation", SimulationBar_startSimulation1)
        .def("startSimulation", SimulationBar_startSimulation2)
        .def("stopSimulation", &SimulationBar::stopSimulation)
        .def("pauseSimulation", &SimulationBar::pauseSimulation)
        ;
}
