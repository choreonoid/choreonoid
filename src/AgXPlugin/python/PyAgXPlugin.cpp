/*!
  @author Shizuko Hattori
*/

#include "../AgXSimulatorItem.h"
#include <cnoid/PyBase>

using namespace boost::python;
using namespace cnoid;

namespace
{
void  (AgXSimulatorItem::*setContactMaterialDamping1)(Link*, Link*, double) = &AgXSimulatorItem::setContactMaterialDamping;
void  (AgXSimulatorItem::*setContactMaterialDamping2)(Body*, Body*, double) = &AgXSimulatorItem::setContactMaterialDamping;
void  (AgXSimulatorItem::*setContactMaterialYoungsModulus1)(Link*, Link*, double) = &AgXSimulatorItem::setContactMaterialYoungsModulus;
void  (AgXSimulatorItem::*setContactMaterialYoungsModulus2)(Body*, Body*, double) = &AgXSimulatorItem::setContactMaterialYoungsModulus;
}

namespace cnoid
{

BOOST_PYTHON_MODULE(AgXPlugin)
{
    {
        scope agxSimulatorItemScope =
                class_< AgXSimulatorItem, AgXSimulatorItemPtr, bases<SimulatorItem> >("AgXSimulatorItem")
                .def("setJointControlMode", &AgXSimulatorItem::setJointControlMode)
                .def("setJointCompliance", &AgXSimulatorItem::setJointCompliance)
                .def("setContactMaterialDamping", setContactMaterialDamping1)
                .def("setContactMaterialDamping", setContactMaterialDamping2)
                .def("setContactMaterialYoungsModulus", setContactMaterialYoungsModulus1)
                .def("setContactMaterialYoungsModulus", setContactMaterialYoungsModulus2)
                ;

        enum_<AgXSimulatorItem::ControlMode>("ControlMode")
                .value("HIGH_GAIN", AgXSimulatorItem::HIGH_GAIN)
                .value("TORQUE", AgXSimulatorItem::TORQUE)
                .value("FREE", AgXSimulatorItem::FREE);
    }

    implicitly_convertible<AgXSimulatorItemPtr, SimulatorItemPtr>();
    PyItemList<AgXSimulatorItem>("AgXSimulatorItemList");

}

}
