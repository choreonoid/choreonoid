/*!
  @author Shizuko Hattori
*/


#include "../AgXSimulatorItem.h"
#include <cnoid/PyBase>

using namespace cnoid;
namespace py = pybind11;

PYBIND11_MODULE(AgXPlugin, m)
{
    m.doc() = "Choreonoid AgXPlugin module";

    py::class_<AgXSimulatorItem, SimulatorItem, AgXSimulatorItemPtr> agxSimulatorItem(m, " AgXSimulatorItem");

    agxSimulatorItem
            .def(py::init<>())
            .def("setJointControlMode", &AgXSimulatorItem::setJointControlMode)
            .def("setJointCompliance", &AgXSimulatorItem::setJointCompliance)
            .def("setContactMaterialFriction", (void (AgXSimulatorItem::*)(Link*, Link*, double)) &AgXSimulatorItem::setContactMaterialFriction)   // Overloaded methods
            .def("setContactMaterialFriction", (void (AgXSimulatorItem::*)(Body*, Body*, double)) &AgXSimulatorItem::setContactMaterialFriction)
            .def("setContactMaterialViscosity", (void (AgXSimulatorItem::*)(Link*, Link*, AgXSimulatorItem::FrictionDirection, double)) &AgXSimulatorItem::setContactMaterialViscosity)
            .def("setContactMaterialViscosity", (void (AgXSimulatorItem::*)(Body*, Body*, AgXSimulatorItem::FrictionDirection, double)) &AgXSimulatorItem::setContactMaterialViscosity)
            .def("setContactMaterialAdhesion", (void (AgXSimulatorItem::*)(Link*, Link*, double, double)) &AgXSimulatorItem::setContactMaterialAdhesion)
            .def("setContactMaterialAdhesion", (void (AgXSimulatorItem::*)(Body*, Body*, double, double)) &AgXSimulatorItem::setContactMaterialAdhesion)
            .def("setContactMaterialRestitution", (void (AgXSimulatorItem::*)(Link*, Link*, double)) &AgXSimulatorItem::setContactMaterialRestitution)
            .def("setContactMaterialRestitution", (void (AgXSimulatorItem::*)(Body*, Body*, double)) &AgXSimulatorItem::setContactMaterialRestitution)
            .def("setContactMaterialDamping", (void (AgXSimulatorItem::*)(Link*, Link*, double)) &AgXSimulatorItem::setContactMaterialDamping)
            .def("setContactMaterialDamping", (void (AgXSimulatorItem::*)(Body*, Body*, double)) &AgXSimulatorItem::setContactMaterialDamping)
            .def("setContactMaterialYoungsModulus", (void (AgXSimulatorItem::*)(Link*, Link*, double)) &AgXSimulatorItem::setContactMaterialYoungsModulus)
            .def("setContactMaterialYoungsModulus", (void (AgXSimulatorItem::*)(Body*, Body*, double)) &AgXSimulatorItem::setContactMaterialYoungsModulus)
            .def("setContactMaterialFrictionModelsolveType", (void (AgXSimulatorItem::*)(Link*,  Link*, AgXSimulatorItem::FrictionModelType, AgXSimulatorItem::FrictionSolveType)) &AgXSimulatorItem::setContactMaterialFrictionModelsolveType)
            .def("setContactMaterialFrictionModelsolveType", (void (AgXSimulatorItem::*)(Body*,  Body*, AgXSimulatorItem::FrictionModelType, AgXSimulatorItem::FrictionSolveType)) &AgXSimulatorItem::setContactMaterialFrictionModelsolveType)
            ;

    py::enum_<AgXSimulatorItem::ControlMode>(agxSimulatorItem, "ControlMode")
            .value("HIGH_GAIN", AgXSimulatorItem::ControlMode::HIGH_GAIN)
            .value("TORQUE", AgXSimulatorItem::ControlMode::TORQUE)
            .value("FREE", AgXSimulatorItem::ControlMode::FREE)
            .export_values();

    py::enum_<AgXSimulatorItem::FrictionModelType>(agxSimulatorItem, "FrictionModelType")
            .value("MODEL_DEFAULT", AgXSimulatorItem::FrictionModelType::MODEL_DEFAULT)
            .value("BOX", AgXSimulatorItem::FrictionModelType::BOX)
            .value("SCALE_BOX", AgXSimulatorItem::FrictionModelType::SCALE_BOX)
            .value("ITERATIVE_PROJECTED", AgXSimulatorItem::FrictionModelType::ITERATIVE_PROJECTED)
            .export_values();

    py::enum_<AgXSimulatorItem::FrictionSolveType>(agxSimulatorItem, "FrictionSolveType")
            .value("SOLVE_DEFAULT", AgXSimulatorItem::FrictionSolveType::SOLVE_DEFAULT)
            .value("DIRECT", AgXSimulatorItem::FrictionSolveType::DIRECT)
            .value("ITERATIVE", AgXSimulatorItem::FrictionSolveType::ITERATIVE)
            .value("SPLIT", AgXSimulatorItem::FrictionSolveType::SPLIT)
            .value("DIRECT_AND_ITERATIVE", AgXSimulatorItem::FrictionSolveType::DIRECT_AND_ITERATIVE)
            .export_values();

    py::enum_<AgXSimulatorItem::FrictionDirection>(agxSimulatorItem, "FrictionDirection")
            .value("PRIMARY_DIRECTION", AgXSimulatorItem::FrictionDirection::PRIMARY_DIRECTION)
            .value("SECONDARY_DIRECTION", AgXSimulatorItem::FrictionDirection::SECONDARY_DIRECTION)
            .value("BOTH_PRIMARY_AND_SECONDARY", AgXSimulatorItem::FrictionDirection::BOTH_PRIMARY_AND_SECONDARY)
            .export_values();

    PyItemList<AgXSimulatorItem>(m, "AgXSimulatorItemList");
}
