#include "../WorldItem.h"
#include "../WorldLogFileItem.h"
#include "../BodyMotionItem.h"
#include "../BodySyncCameraItem.h"
#include <cnoid/PyBase>
#include <cnoid/SceneCameras>
#include <cnoid/MaterialTable>

using namespace cnoid;
namespace nb = nanobind;

namespace cnoid {

void exportItems(nb::module_ m)
{
    nb::class_<WorldItem, Item>(m, "WorldItem")
        .def(nb::init<>())
        .def("storeCurrentBodyPositionsAsInitialPositions", &WorldItem::storeCurrentBodyPositionsAsInitialPositions)
        .def("restoreInitialBodyPositions", &WorldItem::restoreInitialBodyPositions, nb::arg("doNotify") = true)
        .def("selectCollisionDetector", &WorldItem::selectCollisionDetector)
        .def("isCollisionDetectionEnabled", &WorldItem::isCollisionDetectionEnabled)
        .def("setCollisionDetectionEnabled", &WorldItem::setCollisionDetectionEnabled)
        .def("updateCollisionDetectionBodies", &WorldItem::updateCollisionDetectionBodies)
        .def("updateCollisionDetectionBodiesLater", &WorldItem::updateCollisionDetectionBodiesLater)
        .def("updateCollisions", &WorldItem::updateCollisions)
        .def_prop_ro("sigCollisionsUpdated", &WorldItem::sigCollisionsUpdated)
        .def("setDefaultMaterialTableFile", &WorldItem::setDefaultMaterialTableFile)
        .def_prop_ro("defaultMaterialTable", [](WorldItem& self){ return self.defaultMaterialTable(); })
        .def_prop_ro("materialTable", &WorldItem::materialTable)
        ;

    PyItemList<WorldItem>(m, "WorldItemList");

    nb::class_<WorldLogFileItem, Item>(m, "WorldLogFileItem")
        .def(nb::init<>())
        .def("setLogFile", &WorldLogFileItem::setLogFile)
        .def_prop_rw("logFile", &WorldLogFileItem::logFile, &WorldLogFileItem::setLogFile)
        .def("setTimeStampSuffixEnabled", &WorldLogFileItem::setTimeStampSuffixEnabled)
        .def_prop_rw("isTimeStampSuffixEnabled",
                     &WorldLogFileItem::isTimeStampSuffixEnabled, &WorldLogFileItem::setTimeStampSuffixEnabled)
        .def("setRecordingFrameRate", &WorldLogFileItem::setRecordingFrameRate)
        .def_prop_rw("recordingFrameRate",
                     &WorldLogFileItem::recordingFrameRate, &WorldLogFileItem::setRecordingFrameRate)
        .def("recallStateAtTime", &WorldLogFileItem::recallStateAtTime)
        .def("setLivePlaybackReadInterval", &WorldLogFileItem::setLivePlaybackReadInterval)
        .def("setLivePlaybackReadTimeout", &WorldLogFileItem::setLivePlaybackReadTimeout)
        .def("startLivePlayback", &WorldLogFileItem::startLivePlayback)
        .def("stopLivePlayback", &WorldLogFileItem::stopLivePlayback)
        .def("showPlaybackArchiveSaveDialog", &WorldLogFileItem::showPlaybackArchiveSaveDialog)
        .def("saveProjectAsPlaybackArchive", &WorldLogFileItem::saveProjectAsPlaybackArchive)
        ;

    PyItemList<WorldLogFileItem>(m, "WorldLogFileItemList");

    nb::class_<BodyMotionItem, AbstractSeqItem>(m, "BodyMotionItem")
        .def(nb::init<>())
        .def_prop_ro("motion", [](BodyMotionItem* item){ return item->motion(); })
        .def_prop_ro("numExtraSeqItems", &BodyMotionItem::numExtraSeqItems)
        .def("getExtraSeqContentName", &BodyMotionItem::extraSeqContentName)
        .def("getExtraSeqItem", (AbstractSeqItem*(BodyMotionItem::*)(int))&BodyMotionItem::extraSeqItem)
        .def("updateExtraSeqItems", &BodyMotionItem::updateExtraSeqItems)
        ;

    PyItemList<BodyMotionItem>(m, "BodyMotionItemList");

    nb::class_<BodySyncCameraItem, Item>(m, "BodySyncCameraItem")
        .def(nb::init<>())
        .def("setTargetLink", &BodySyncCameraItem::setTargetLink)
        .def_prop_rw("targetLinkName", &BodySyncCameraItem::targetLinkName, &BodySyncCameraItem::setTargetLink)
        .def("setCameraType", &BodySyncCameraItem::setCameraType)
        .def_prop_rw(
            "CameraType",
            &BodySyncCameraItem::cameraType, &BodySyncCameraItem::setCameraType)
        .def_prop_ro("currentCamera", &BodySyncCameraItem::currentCamera)
        .def_prop_ro("cameraTransform", &BodySyncCameraItem::cameraTransform)
        .def("setParallelTrackingMode", &BodySyncCameraItem::setParallelTrackingMode)
        .def("isParallelTrackingMode", &BodySyncCameraItem::isParallelTrackingMode)
        ;

    PyItemList<BodySyncCameraItem>(m, "BodySyncCameraItemList");
}

}
