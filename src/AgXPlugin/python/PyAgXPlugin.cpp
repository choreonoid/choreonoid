/*!
  @author Shizuko Hattori
*/

#include "../AgXSimulatorItem.h"
#include <cnoid/PyBase>

using namespace boost::python;
using namespace cnoid;

namespace
{
void  (AgXSimulatorItem::*setContactMaterialFriction1)(Link*, Link*, double) = &AgXSimulatorItem::setContactMaterialFriction;
void  (AgXSimulatorItem::*setContactMaterialViscosity1)(Link*, Link*, AgXSimulatorItem::FrictionDirection, double) = &AgXSimulatorItem::setContactMaterialViscosity;
void  (AgXSimulatorItem::*setContactMaterialAdhesion1)(Link*, Link*, double, double) = &AgXSimulatorItem::setContactMaterialAdhesion;
void  (AgXSimulatorItem::*setContactMaterialRestitution1)(Link*, Link*, double) = &AgXSimulatorItem::setContactMaterialRestitution;
void  (AgXSimulatorItem::*setContactMaterialDamping1)(Link*, Link*, double) = &AgXSimulatorItem::setContactMaterialDamping;
void  (AgXSimulatorItem::*setContactMaterialYoungsModulus1)(Link*, Link*, double) = &AgXSimulatorItem::setContactMaterialYoungsModulus;
void  (AgXSimulatorItem::*setContactMaterialFrictionModelsolveType1)(Link*,  Link*, AgXSimulatorItem::FrictionModelType, AgXSimulatorItem::FrictionSolveType) = &AgXSimulatorItem::setContactMaterialFrictionModelsolveType;
void  (AgXSimulatorItem::*setContactMaterialFriction2)(Body*, Body*, double) = &AgXSimulatorItem::setContactMaterialFriction;
void  (AgXSimulatorItem::*setContactMaterialViscosity2)(Body*, Body*, AgXSimulatorItem::FrictionDirection, double) = &AgXSimulatorItem::setContactMaterialViscosity;
void  (AgXSimulatorItem::*setContactMaterialAdhesion2)(Body*, Body*, double, double) = &AgXSimulatorItem::setContactMaterialAdhesion;
void  (AgXSimulatorItem::*setContactMaterialRestitution2)(Body*, Body*, double) = &AgXSimulatorItem::setContactMaterialRestitution;
void  (AgXSimulatorItem::*setContactMaterialDamping2)(Body*, Body*, double) = &AgXSimulatorItem::setContactMaterialDamping;
void  (AgXSimulatorItem::*setContactMaterialYoungsModulus2)(Body*, Body*, double) = &AgXSimulatorItem::setContactMaterialYoungsModulus;
void  (AgXSimulatorItem::*setContactMaterialFrictionModelsolveType2)(Body*,  Body*, AgXSimulatorItem::FrictionModelType, AgXSimulatorItem::FrictionSolveType) = &AgXSimulatorItem::setContactMaterialFrictionModelsolveType;
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
                .def("setContactMaterialFriction", setContactMaterialFriction1)
                .def("setContactMaterialViscosity", setContactMaterialViscosity1)
                .def("setContactMaterialAdhesion", setContactMaterialAdhesion1)
                .def("setContactMaterialRestitution", setContactMaterialRestitution1)
                .def("setContactMaterialDamping", setContactMaterialDamping1)
                .def("setContactMaterialYoungsModulus", setContactMaterialYoungsModulus1)
                .def("setContactMaterialFrictionModelsolveType", setContactMaterialFrictionModelsolveType1)
                .def("setContactMaterialFriction", setContactMaterialFriction2)
                .def("setContactMaterialViscosity", setContactMaterialViscosity2)
                .def("setContactMaterialAdhesion", setContactMaterialAdhesion2)
                .def("setContactMaterialRestitution", setContactMaterialRestitution2)
                .def("setContactMaterialDamping", setContactMaterialDamping2)
                .def("setContactMaterialYoungsModulus", setContactMaterialYoungsModulus2)
                .def("setContactMaterialFrictionModelsolveType", setContactMaterialFrictionModelsolveType2)
                ;

        enum_<AgXSimulatorItem::ControlMode>("ControlMode")
                .value("HIGH_GAIN", AgXSimulatorItem::HIGH_GAIN)
                .value("TORQUE", AgXSimulatorItem::TORQUE)
                .value("FREE", AgXSimulatorItem::FREE);
        enum_<AgXSimulatorItem::FrictionModelType>("FrictionModelType")
                .value("MODEL_DEFAULT", AgXSimulatorItem::MODEL_DEFAULT)
                .value("BOX", AgXSimulatorItem::BOX)
                .value("SCALE_BOX", AgXSimulatorItem::SCALE_BOX)
                .value("ITERATIVE_PROJECTED", AgXSimulatorItem::ITERATIVE_PROJECTED);
        enum_<AgXSimulatorItem::FrictionSolveType>("FrictionSolveType")
                .value("SOLVE_DEFAULT", AgXSimulatorItem::SOLVE_DEFAULT)
                .value("DIRECT", AgXSimulatorItem::DIRECT)
                .value("ITERATIVE", AgXSimulatorItem::ITERATIVE)
                .value("SPLIT", AgXSimulatorItem::SPLIT)
                .value("DIRECT_AND_ITERATIVE", AgXSimulatorItem::DIRECT_AND_ITERATIVE);
        enum_<AgXSimulatorItem::FrictionDirection>("FrictionDirection")
                .value("PRIMARY_DIRECTION", AgXSimulatorItem::PRIMARY_DIRECTION)
                .value("SECONDARY_DIRECTION", AgXSimulatorItem::SECONDARY_DIRECTION)
                .value("BOTH_PRIMARY_AND_SECONDARY", AgXSimulatorItem::BOTH_PRIMARY_AND_SECONDARY);

    }

    implicitly_convertible<AgXSimulatorItemPtr, SimulatorItemPtr>();
    PyItemList<AgXSimulatorItem>("AgXSimulatorItemList");

}

}
