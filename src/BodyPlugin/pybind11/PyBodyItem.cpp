/*!
  @author Shin'ichiro Nakaoka
*/

#include "../BodyItem.h"
#include <cnoid/BodyState>
#include <cnoid/PyBase>

using namespace std;
using namespace cnoid;
namespace py = pybind11;

namespace cnoid {

void exportBodyItem(py::module m)
{
    m.def("loadBodyItem", [](const string& filename) {
            BodyItem* bodyItem = new BodyItem;
            bodyItem->load(filename);
            return bodyItem;
        });
    
    py::class_<BodyItem, BodyItemPtr, Item> bodyItem(m, "BodyItem", py::multiple_inheritance());

    bodyItem
        .def(py::init<>())
        .def(py::init<const std::string&>())
        .def_property_readonly("body", &BodyItem::body)
        .def("makeBodyStatic", &BodyItem::makeBodyStatic)
        .def("makeBodyDynamic", &BodyItem::makeBodyDynamic)
        .def("moveToOrigin", &BodyItem::moveToOrigin)
        .def("setPresetPose", &BodyItem::setPresetPose)
        .def_property_readonly("currentBaseLink", &BodyItem::currentBaseLink)
        .def("setCurrentBaseLink", &BodyItem::setCurrentBaseLink)
        .def("calcForwardKinematics",
             &BodyItem::calcForwardKinematics,
             py::arg("calcVelocity") = false, py::arg("calcAcceleration") = false)
        .def("copyKinematicState", &BodyItem::copyKinematicState)
        .def("pasteKinematicState", &BodyItem::pasteKinematicState)
        .def("storeKinematicState", &BodyItem::storeKinematicState)
        .def("restoreKinematicState", &BodyItem::restoreKinematicState)
        .def("storeInitialState", &BodyItem::storeInitialState)
        .def("restoreInitialState", &BodyItem::restoreInitialState)
        .def("getInitialState", &BodyItem::getInitialState)
        .def_property_readonly("sigKinematicStateChanged", &BodyItem::sigKinematicStateChanged)
        .def("notifyKinematicStateChange",
             (void (BodyItem::*)(bool, bool, bool)) &BodyItem::notifyKinematicStateChange,
             py::arg("requestFK") = false, py::arg("requestVelFK") = false, py::arg("requestAccFK") = false)
        .def("notifyKinematicStateChange",
             (void (BodyItem::*)(Connection&, bool, bool, bool)) &BodyItem::notifyKinematicStateChange,
             py::arg("connectionToBlock"),
             py::arg("requestFK") = false, py::arg("requestVelFK") = false, py::arg("requestAccFK") = false)
        .def("notifyKinematicStateUpdate",
             &BodyItem::notifyKinematicStateUpdate, py::arg("doNotifyStateChange") = true)
        .def("isCollisionDetectionEnabled", &BodyItem::isCollisionDetectionEnabled)
        .def("setCollisionDetectionEnabled", &BodyItem::setCollisionDetectionEnabled)
        .def("isSelfCollisionDetectionEnabled", &BodyItem::isSelfCollisionDetectionEnabled)
        .def("setSelfCollisionDetectionEnabled", &BodyItem::setSelfCollisionDetectionEnabled)
        .def("clearCollisions", &BodyItem::clearCollisions)
        .def_property_readonly("centerOfMass", &BodyItem::centerOfMass)
        .def("doLegIkToMoveCm", &BodyItem::doLegIkToMoveCm)
        .def_property_readonly("zmp", &BodyItem::zmp)
        .def("setZmp", &BodyItem::setZmp)
        .def("setStance", &BodyItem::setStance)

        // deprecated
        .def("enableCollisionDetection", &BodyItem::setCollisionDetectionEnabled)
        .def("enableSelfCollisionDetection", &BodyItem::setSelfCollisionDetectionEnabled)
        .def("getBody", &BodyItem::body)
        .def("getCurrentBaseLink", &BodyItem::currentBaseLink)
        .def("getSigKinematicStateChanged", &BodyItem::sigKinematicStateChanged)
        .def("getCenterOfMass", &BodyItem::centerOfMass)
        .def("getZmp", &BodyItem::zmp)
        ;

    py::enum_<BodyItem::PresetPoseID>(bodyItem, "PresetPoseID")
        .value("INITIAL_POSE", BodyItem::PresetPoseID::INITIAL_POSE)
        .value("STANDARD_POSE", BodyItem::PresetPoseID::STANDARD_POSE)
        .export_values();

    py::enum_<BodyItem::PositionType>(bodyItem, "PositionType")
        .value("CM_PROJECTION", BodyItem::PositionType::CM_PROJECTION)
        .value("HOME_COP", BodyItem::PositionType::HOME_COP)
        .value("RIGHT_HOME_COP", BodyItem::PositionType::RIGHT_HOME_COP)
        .value("LEFT_HOME_COP", BodyItem::PositionType::LEFT_HOME_COP)
        .value("ZERO_MOMENT_POINT", BodyItem::PositionType::ZERO_MOMENT_POINT)
        .export_values();

    PyItemList<BodyItem>(m, "BodyItemList");
}

}
