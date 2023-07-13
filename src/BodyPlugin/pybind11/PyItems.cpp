/*!
  @author Shin'ichiro Nakaoka
*/

#include "../WorldItem.h"
#include "../WorldLogFileItem.h"
#include "../BodyMotionItem.h"
#include "../BodySyncCameraItem.h"
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
        .def_property_readonly("numExtraSeqItems", &BodyMotionItem::numExtraSeqItems)
        .def("getExtraSeqContentName", &BodyMotionItem::extraSeqContentName)
        .def("getExtraSeqItem", (AbstractSeqItem*(BodyMotionItem::*)(int))&BodyMotionItem::extraSeqItem)
        .def("updateExtraSeqItems", &BodyMotionItem::updateExtraSeqItems)

        // deprecated
        .def("extraSeqKey", &BodyMotionItem::extraSeqContentName)
        .def("extraSeqItem", (AbstractSeqItem*(BodyMotionItem::*)(int))&BodyMotionItem::extraSeqItem)
        .def("getMotion", [](BodyMotionItem* item){ return item->motion(); })
        .def("getNumExtraSeqItems", &BodyMotionItem::numExtraSeqItems)
        ;

    PyItemList<BodyMotionItem>(m, "BodyMotionItemList");

    py::class_<BodySyncCameraItem, BodySyncCameraItemPtr, Item>
        bodySyncCameraItem(m, "BodySyncCameraItem", py::multiple_inheritance());

    bodySyncCameraItem
        .def(py::init<>())
        .def("setTargetLink", &BodySyncCameraItem::setTargetLink)
        .def_property("targetLinkName", &BodySyncCameraItem::targetLinkName, &BodySyncCameraItem::setTargetLink)
        .def("setCameraType", &BodySyncCameraItem::setCameraType)
        .def_property(
            "CameraType",
            &BodySyncCameraItem::cameraType, &BodySyncCameraItem::setCameraType)
        .def_property_readonly("currentCamera", &BodySyncCameraItem::currentCamera)
        .def_property_readonly("cameraTransform", &BodySyncCameraItem::cameraTransform)
        .def("setParallelTrackingMode", &BodySyncCameraItem::setParallelTrackingMode)
        .def("isParallelTrackingMode", &BodySyncCameraItem::isParallelTrackingMode)

        // deprecated
        .def("setRotationSyncEnabled",
             [](BodySyncCameraItem* item, bool on){ item->setParallelTrackingMode(!on); })
        .def_property_readonly(
            "isRotationSyncEnabled",
            [](BodySyncCameraItem* item){ return !item->isParallelTrackingMode(); })
        ;

    // For the backward compatibility
    m.attr("BodyTrackingCameraItem") = bodySyncCameraItem;

    PyItemList<BodySyncCameraItem> bodySyncCameraItemList(m, "BodySyncCameraItemList");

    m.attr("BodyTrackingCameraItemList") = m.attr("BodySyncCameraItemList");
}

}
