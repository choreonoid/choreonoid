/*!
  @author Shin'ichiro Nakaoka
*/

#include "../BodyItem.h"
#include <cnoid/BodyState>
#include <cnoid/PyBase>

using namespace boost::python;
using namespace cnoid;

namespace {

BodyItemPtr loadBodyItem(const std::string& filename) {
    BodyItem* bodyItem = new BodyItem;
    bodyItem->load(filename);
    return bodyItem;
}

BodyPtr BodyItem_body(BodyItem& self) { return self.body(); }
LinkPtr BodyItem_currentBaseLink(BodyItem& self) { return self.currentBaseLink(); }

BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(BodyItem_calcForwardKinematics_overloads, calcForwardKinematics, 0, 2)

void (BodyItem::*BodyItem_notifyKinematicStateChange1)(bool, bool, bool) = &BodyItem::notifyKinematicStateChange;
BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(BodyItem_notifyKinematicStateChange1_overloads, notifyKinematicStateChange, 0, 2)

void (BodyItem::*BodyItem_notifyKinematicStateChange2)(Connection&, bool, bool, bool) = &BodyItem::notifyKinematicStateChange;
BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(BodyItem_notifyKinematicStateChange2_overloads, notifyKinematicStateChange, 1, 3)

}

void exportBodyItem()
{
    def("loadBodyItem", loadBodyItem);
    
    {
        scope bodyItemScope = 
            class_< BodyItem, BodyItemPtr, bases<Item, SceneProvider> >("BodyItem")
            .def("loadModelFile", &BodyItem::loadModelFile)
            .def("setName", &BodyItem::setName)
            .def("body", BodyItem_body)
            .def("getBody", BodyItem_body)
            .def("isEditable", &BodyItem::isEditable)
            .def("moveToOrigin", &BodyItem::moveToOrigin)
            .def("setPresetPose", &BodyItem::setPresetPose)
            .def("currentBaseLink", BodyItem_currentBaseLink)
            .def("getCurrentBaseLink", BodyItem_currentBaseLink)
            .def("setCurrentBaseLink", &BodyItem::setCurrentBaseLink)
            .def("calcForwardKinematics", &BodyItem::calcForwardKinematics, BodyItem_calcForwardKinematics_overloads())
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
            .def("getSigKinematicStateChanged", &BodyItem::sigKinematicStateChanged)
            .def("notifyKinematicStateChange", BodyItem_notifyKinematicStateChange1, BodyItem_notifyKinematicStateChange1_overloads())
            .def("notifyKinematicStateChange", BodyItem_notifyKinematicStateChange2, BodyItem_notifyKinematicStateChange2_overloads())
            .def("enableCollisionDetection", &BodyItem::enableCollisionDetection)
            .def("isCollisionDetectionEnabled", &BodyItem::isCollisionDetectionEnabled)
            .def("enableSelfCollisionDetection", &BodyItem::enableSelfCollisionDetection)
            .def("isSelfCollisionDetectionEnabled", &BodyItem::isSelfCollisionDetectionEnabled)
            .def("clearCollisions", &BodyItem::clearCollisions)
            .def("centerOfMass", &BodyItem::centerOfMass, return_value_policy<copy_const_reference>())
            .def("getCenterOfMass", &BodyItem::centerOfMass, return_value_policy<copy_const_reference>())
            .def("doLegIkToMoveCm", &BodyItem::doLegIkToMoveCm)
            .def("zmp", &BodyItem::zmp, return_value_policy<copy_const_reference>())
            .def("getZmp", &BodyItem::zmp, return_value_policy<copy_const_reference>())
            .def("setZmp", &BodyItem::setZmp)
            .def("setStance", &BodyItem::setStance)
            ;

        enum_<BodyItem::PresetPoseID>("PresetPoseID")
            .value("INITIAL_POSE", BodyItem::INITIAL_POSE) 
            .value("STANDARD_POSE", BodyItem::STANDARD_POSE);

        enum_<BodyItem::PositionType>("PositionType")
            .value("CM_PROJECTION", BodyItem::CM_PROJECTION)
            .value("HOME_COP", BodyItem::HOME_COP)
            .value("RIGHT_HOME_COP", BodyItem::RIGHT_HOME_COP)
            .value("LEFT_HOME_COP", BodyItem::LEFT_HOME_COP)
            .value("ZERO_MOMENT_POINT", BodyItem::ZERO_MOMENT_POINT);
    }

    implicitly_convertible<BodyItemPtr, ItemPtr>();
    PyItemList<BodyItem>("BodyItemList");
}
