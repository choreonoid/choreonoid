/*!
  @author Shizuko Hattori
*/

#include "../AgXSimulatorItem.h"
#include <cnoid/PyBase>

using namespace boost::python;
using namespace cnoid;

BOOST_PYTHON_MODULE(AgXPlugin)
{
    {
        scope agxSimulatorItemScope =
                class_< AgXSimulatorItem, AgXSimulatorItemPtr, bases<SimulatorItem> >("AgXSimulatorItem")
                .def("setJointControlMode", &AgXSimulatorItem::setJointControlMode)
                ;

        enum_<AgXSimulatorItem::ControlMode>("ControlMode")
                .value("HIGH_GAIN", AgXSimulatorItem::HIGH_GAIN)
                .value("TORQUE", AgXSimulatorItem::TORQUE)
                .value("FREE", AgXSimulatorItem::FREE);
    }

    implicitly_convertible<AgXSimulatorItemPtr, SimulatorItemPtr>();
    PyItemList<AgXSimulatorItem>("AgXSimulatorItemList");

}

