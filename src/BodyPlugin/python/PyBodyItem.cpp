#include "../BodyItem.h"
#include <cnoid/BodyState>
#include <cnoid/PyBase>

using namespace std;
using namespace cnoid;
namespace nb = nanobind;

namespace cnoid {

void exportBodyItem(nb::module_ m)
{
    m.def("loadBodyItem", [](const string& filename) {
            BodyItem* bodyItem = new BodyItem;
            bodyItem->load(filename);
            return bodyItem;
        });

    nb::class_<BodyItem, Item> bodyItem(m, "BodyItem");

    bodyItem
        .def(nb::init<>())
        .def(nb::init<const std::string&>())
        .def_prop_ro("body", &BodyItem::body)
        .def("moveToOrigin", &BodyItem::moveToOrigin)
        .def("setPresetPose", &BodyItem::setPresetPose)
        .def_prop_ro("currentBaseLink", &BodyItem::currentBaseLink)
        .def("setCurrentBaseLink", &BodyItem::setCurrentBaseLink)
        .def("calcForwardKinematics",
             &BodyItem::calcForwardKinematics,
             nb::arg("calcVelocity") = false, nb::arg("calcAcceleration") = false)
        // The C++ store/get functions take an output BodyState argument; here
        // they return a new BodyState, which is the natural form for Python.
        .def("storeKinematicState",
             [](BodyItem& self){ BodyState state; self.storeKinematicState(state); return state; })
        .def("restoreKinematicState", &BodyItem::restoreKinematicState)
        .def("storeInitialState", &BodyItem::storeInitialState)
        .def("restoreInitialState", &BodyItem::restoreInitialState, nb::arg("doNotify") = true)
        .def("getInitialState",
             [](BodyItem& self){ BodyState state; self.getInitialState(state); return state; })
        .def_prop_ro("sigKinematicStateChanged", &BodyItem::sigKinematicStateChanged)
        .def("notifyKinematicStateChange",
             (void (BodyItem::*)(bool, bool, bool)) &BodyItem::notifyKinematicStateChange,
             nb::arg("requestFK") = false, nb::arg("requestVelFK") = false, nb::arg("requestAccFK") = false)
        .def("notifyKinematicStateChange",
             (void (BodyItem::*)(Connection&, bool, bool, bool)) &BodyItem::notifyKinematicStateChange,
             nb::arg("connectionToBlock"),
             nb::arg("requestFK") = false, nb::arg("requestVelFK") = false, nb::arg("requestAccFK") = false)
        .def("notifyKinematicStateUpdate",
             &BodyItem::notifyKinematicStateUpdate, nb::arg("doNotifyStateChange") = true)
        .def("isCollisionDetectionEnabled", &BodyItem::isCollisionDetectionEnabled)
        .def("setCollisionDetectionEnabled", &BodyItem::setCollisionDetectionEnabled)
        .def("isSelfCollisionDetectionEnabled", &BodyItem::isSelfCollisionDetectionEnabled)
        .def("setSelfCollisionDetectionEnabled", &BodyItem::setSelfCollisionDetectionEnabled)
        .def("clearCollisions", &BodyItem::clearCollisions)
        .def_prop_ro("centerOfMass", &BodyItem::centerOfMass)
        .def("doLegIkToMoveCm", &BodyItem::doLegIkToMoveCm)
        .def("setStance", &BodyItem::setStance)

        .def("notifyModelUpdate", &BodyItem::notifyModelUpdate)
        .def("setBody", &BodyItem::setBody)
        ;

    nb::enum_<BodyItem::PresetPoseID>(bodyItem, "PresetPoseID")
        .value("INITIAL_POSE", BodyItem::PresetPoseID::INITIAL_POSE)
        .value("STANDARD_POSE", BodyItem::PresetPoseID::STANDARD_POSE)
        .export_values();

    nb::enum_<BodyItem::PositionType>(bodyItem, "PositionType")
        .value("CM_PROJECTION", BodyItem::PositionType::CM_PROJECTION)
        .value("HOME_COP", BodyItem::PositionType::HOME_COP)
        .value("RIGHT_HOME_COP", BodyItem::PositionType::RIGHT_HOME_COP)
        .value("LEFT_HOME_COP", BodyItem::PositionType::LEFT_HOME_COP)
        .value("ZERO_MOMENT_POINT", BodyItem::PositionType::ZERO_MOMENT_POINT)
        .export_values();

    nb::enum_<BodyItem::ModelUpdateFlag>(bodyItem, "ModelUpdateFlag", nb::is_arithmetic())
        .value("LinkSetUpdate",    BodyItem::LinkSetUpdate)
        .value("LinkSpecUpdate",   BodyItem::LinkSpecUpdate)
        .value("DeviceSetUpdate",  BodyItem::DeviceSetUpdate)
        .value("DeviceSpecUpdate", BodyItem::DeviceSpecUpdate)
        .value("ShapeUpdate",      BodyItem::ShapeUpdate)
        .export_values();

    PyItemList<BodyItem>(m, "BodyItemList");
}

}
