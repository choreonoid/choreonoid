/*!
  @author Shin'ichiro Nakaoka
*/

#include "../WorldItem.h"
#include "../WorldLogFileItem.h"
#include "../BodyMotionItem.h"
#include "../BodyTrackingCameraItem.h"
#include <cnoid/PyBase>
#include <cnoid/SceneCameras>
#include <cnoid/MaterialTable>

using namespace cnoid;
namespace py = pybind11;

namespace cnoid {

void exportItems(py::module m)
{
    py::class_<WorldItem, WorldItemPtr, Item>(m, "WorldItem", py::multiple_inheritance())
        .def(py::init<>())
        .def("storeCurrentBodyPositionsAsInitialPositions", &WorldItem::storeCurrentBodyPositionsAsInitialPositions)
        .def("restoreInitialBodyPositions", &WorldItem::restoreInitialBodyPositions, py::arg("doNotify") = true)
        .def("selectCollisionDetector", &WorldItem::selectCollisionDetector)
        .def("isCollisionDetectionEnabled", &WorldItem::isCollisionDetectionEnabled)
        .def("setCollisionDetectionEnabled", &WorldItem::setCollisionDetectionEnabled)
        .def("updateCollisionDetectorLater", &WorldItem::updateCollisionDetectorLater)
        .def("updateCollisionDetector", &WorldItem::updateCollisionDetector)
        .def("updateCollisions", &WorldItem::updateCollisions)
        .def_property_readonly("sigCollisionsUpdated", &WorldItem::sigCollisionsUpdated)
        .def("setDefaultMaterialTableFile", &WorldItem::setDefaultMaterialTableFile)
        .def_property_readonly("defaultMaterialTable", [](WorldItem& self){ return self.defaultMaterialTable(); })
        .def_property_readonly("materialTable", &WorldItem::materialTable)
        ;

    PyItemList<WorldItem>(m, "WorldItemList");

    py::class_<WorldLogFileItem, WorldLogFileItemPtr, Item>(m, "WorldLogFileItem")
        .def(py::init<>())
        .def("setLogFile", &WorldLogFileItem::setLogFile)
        .def_property("logFile", &WorldLogFileItem::logFile, &WorldLogFileItem::setLogFile)
        .def("setTimeStampSuffixEnabled", &WorldLogFileItem::setTimeStampSuffixEnabled)
        .def_property("isTimeStampSuffixEnabled",
                      &WorldLogFileItem::isTimeStampSuffixEnabled, &WorldLogFileItem::setTimeStampSuffixEnabled)
        .def("setRecordingFrameRate", &WorldLogFileItem::setRecordingFrameRate)
        .def_property("recordingFrameRate",
                      &WorldLogFileItem::recordingFrameRate, &WorldLogFileItem::setRecordingFrameRate)
        .def("recallStateAtTime", &WorldLogFileItem::recallStateAtTime)
        ;

    PyItemList<WorldLogFileItem>(m, "WorldLogFileItemList");
    
    py::class_<BodyMotionItem, BodyMotionItemPtr, AbstractSeqItem>(m, "BodyMotionItem")
        .def(py::init<>())
        .def_property_readonly("motion", [](BodyMotionItem* item){ return item->motion(); })
        .def_property_readonly("jointPosSeqItem", (MultiValueSeqItem*(BodyMotionItem::*)())&BodyMotionItem::jointPosSeqItem)
        .def_property_readonly("jointPosSeq", &BodyMotionItem::jointPosSeq)
        .def_property_readonly("linkPosSeqItem", (MultiSE3SeqItem*(BodyMotionItem::*)())&BodyMotionItem::linkPosSeqItem)
        .def_property_readonly("linkPosSeq", &BodyMotionItem::linkPosSeq)
        .def_property_readonly("numExtraSeqItems", &BodyMotionItem::numExtraSeqItems)
        .def("extraSeqKey", &BodyMotionItem::extraSeqKey)
        .def("extraSeqItem", (AbstractSeqItem*(BodyMotionItem::*)(int))&BodyMotionItem::extraSeqItem)
        .def("updateExtraSeqItems", &BodyMotionItem::updateExtraSeqItems)

        // deprecated
        .def("getMotion", [](BodyMotionItem* item){ return item->motion(); })
        .def("getJointPosSeqItem", (MultiValueSeqItem*(BodyMotionItem::*)())&BodyMotionItem::jointPosSeqItem)
        .def("getJointPosSeq", &BodyMotionItem::jointPosSeq)
        .def("getLinkPosSeqItem", (MultiSE3SeqItem*(BodyMotionItem::*)())&BodyMotionItem::linkPosSeqItem)
        .def("getLinkPosSeq", &BodyMotionItem::linkPosSeq)
        .def("getNumExtraSeqItems", &BodyMotionItem::numExtraSeqItems)
        ;

    PyItemList<BodyMotionItem>(m, "BodyMotionItemList");

    py::class_<BodyTrackingCameraItem, BodyTrackingCameraItemPtr, Item>(m, "BodyTrackingCameraItem", py::multiple_inheritance())
        .def(py::init<>())
        .def("setTargetLink", &BodyTrackingCameraItem::setTargetLink)
        .def_property("targetLinkName", &BodyTrackingCameraItem::targetLinkName, &BodyTrackingCameraItem::setTargetLink)
        .def("setRotationSyncEnabled", &BodyTrackingCameraItem::setRotationSyncEnabled)
        .def_property(
            "isRotationSyncEnabled",
            &BodyTrackingCameraItem::isRotationSyncEnabled, &BodyTrackingCameraItem::setRotationSyncEnabled)
        .def("setCameraType", &BodyTrackingCameraItem::setCameraType)
        .def_property(
            "CameraType",
            &BodyTrackingCameraItem::cameraType, &BodyTrackingCameraItem::setCameraType)
        .def_property_readonly("currentCamera", &BodyTrackingCameraItem::currentCamera)
        .def_property_readonly("cameraTransform", &BodyTrackingCameraItem::cameraTransform)
        ;

    PyItemList<BodyTrackingCameraItem>(m, "BodyTrackingCameraItemList");
}

}
