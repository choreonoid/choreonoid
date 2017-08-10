/*!
  @author Shin'ichiro Nakaoka
*/

#include "../BodyItem.h"
#include <cnoid/BodyState>
#include <cnoid/Py3Base>

namespace py = pybind11;
using namespace cnoid;

namespace {

BodyItemPtr loadBodyItem(const std::string& filename) {
    BodyItem* bodyItem = new BodyItem;
    bodyItem->load(filename);
    return bodyItem;
}

}


void exportBodyItem(py::module m)
{
    m.def("loadBodyItem", loadBodyItem);
    
    py::class_<BodyItem, BodyItemPtr, Item> bodyItem(m, "BodyItem");

    bodyItem
        .def(py::init<>())
        .def("loadModelFile", &BodyItem::loadModelFile)
        .def("setName", &BodyItem::setName)
        .def("body", [](BodyItem& self){ return BodyPtr(self.body()); })
        .def("isEditable", &BodyItem::isEditable)
        .def("moveToOrigin", &BodyItem::moveToOrigin)
        .def("setPresetPose", &BodyItem::setPresetPose)
        .def("currentBaseLink", [](BodyItem& self){ return LinkPtr(self.currentBaseLink()); })
        .def("setCurrentBaseLink", &BodyItem::setCurrentBaseLink)
        .def("calcForwardKinematics", &BodyItem::calcForwardKinematics,
                py::arg("calcVelocity")=false, py::arg("calcAcceleration")=false)
        .def("copyKinematicState", &BodyItem::copyKinematicState)
        .def("pasteKinematicState", &BodyItem::pasteKinematicState)
        .def("storeKinematicState", &BodyItem::storeKinematicState)
        .def("restoreKinematicState", &BodyItem::restoreKinematicState)
        .def("storeInitialState", &BodyItem::storeInitialState)
        .def("restoreInitialState", &BodyItem::restoreInitialState)
        .def("getInitialState", &BodyItem::getInitialState)
        .def("beginKinematicStateEdit", &BodyItem::beginKinematicStateEdit)
        .def("acceptKinematicStateEdit", &BodyItem::acceptKinematicStateEdit)
        .def("undoKinematicState", &BodyItem::undoKinematicState)
        .def("redoKinematicState", &BodyItem::redoKinematicState)
        .def("sigKinematicStateChanged", &BodyItem::sigKinematicStateChanged)
        .def("notifyKinematicStateChange", (void (BodyItem::*)(bool, bool, bool)) &BodyItem::notifyKinematicStateChange,
            py::arg("requestFK")=false, py::arg("requestVelFK")=false, py::arg("requestAccFK")=false)
        .def("notifyKinematicStateChange", (void (BodyItem::*)(Connection&, bool, bool, bool)) &BodyItem::notifyKinematicStateChange,
            py::arg("connectionToBlock"), py::arg("requestFK")=false, py::arg("requestVelFK")=false, py::arg("requestAccFK")=false)
        .def("enableCollisionDetection", &BodyItem::enableCollisionDetection)
        .def("isCollisionDetectionEnabled", &BodyItem::isCollisionDetectionEnabled)
        .def("enableSelfCollisionDetection", &BodyItem::enableSelfCollisionDetection)
        .def("isSelfCollisionDetectionEnabled", &BodyItem::isSelfCollisionDetectionEnabled)
        .def("clearCollisions", &BodyItem::clearCollisions)
        .def("centerOfMass", &BodyItem::centerOfMass, py::return_value_policy::reference)
        .def("doLegIkToMoveCm", &BodyItem::doLegIkToMoveCm)
        .def("zmp", &BodyItem::zmp, py::return_value_policy::reference)
        .def("setZmp", &BodyItem::setZmp)
        .def("setStance", &BodyItem::setStance)
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
