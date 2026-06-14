#include "../SimulatorItem.h"
#include "../AISTSimulatorItem.h"
#include "../SubSimulatorItem.h"
#include "../SimulationScriptItem.h"
#include "../SimulationBar.h"
#include "../BodyItem.h"
#include "../SimpleControllerItem.h"
#include "../BodyContactPointLoggerItem.h"
#include "../BodyContactPointLogItem.h"
#include <cnoid/PyBase>
#include <cnoid/PyEigenTypes>
#include <nanobind/stl/vector.h>

using namespace cnoid;
namespace nb = nanobind;

namespace cnoid {

void exportSimulationClasses(nb::module_ m)
{
    nb::class_<SimulatorItem, Item> simulatorItemClass(m, "SimulatorItem");

    simulatorItemClass
        .def_static("findActiveSimulatorItemFor", &SimulatorItem::findActiveSimulatorItemFor)
        .def_prop_ro("worldTimeStep", &SimulatorItem::worldTimeStep)
        .def("setTimeStep", &SimulatorItem::setTimeStep)
        .def("startSimulation", &SimulatorItem::startSimulation, nb::arg("doReset") = true)
        .def("stopSimulation", &SimulatorItem::stopSimulation, nb::arg("isForced") = false)
        .def("pauseSimulation", &SimulatorItem::pauseSimulation)
        .def("resumeSimulation", &SimulatorItem::resumeSimulation)
        .def("restartSimulation", &SimulatorItem::resumeSimulation)
        .def("isRunning", &SimulatorItem::isRunning)
        .def_prop_ro("sigSimulationStarted", &SimulatorItem::sigSimulationStarted)
        .def_prop_ro("sigSimulationPaused", &SimulatorItem::sigSimulationPaused)
        .def_prop_ro("sigSimulationResumed", &SimulatorItem::sigSimulationResumed)
        .def_prop_ro("sigSimulationFinished", &SimulatorItem::sigSimulationFinished)
        .def_prop_ro("currentFrame", &SimulatorItem::currentFrame)
        .def_prop_ro("currentTime", &SimulatorItem::currentTime)
        .def("setRecordingMode",
             [](SimulatorItem& self, SimulatorItem::RecordingMode mode){ self.setRecordingMode(mode); })
        .def_prop_ro("recordingMode", &SimulatorItem::recordingMode)
        .def("setTimeRangeMode",
             [](SimulatorItem& self, SimulatorItem::TimeRangeMode mode){ self.setTimeRangeMode(mode); })
        .def("setTimeLength", &SimulatorItem::setTimeLength)
        .def("setActiveControlTimeRangeMode", &SimulatorItem::setActiveControlTimeRangeMode)
        .def("isActiveControlTimeRangeMode", &SimulatorItem::isActiveControlTimeRangeMode)
        .def("isRecordingEnabled", &SimulatorItem::isRecordingEnabled)
        .def("isDeviceStateOutputEnabled", &SimulatorItem::isDeviceStateOutputEnabled)
        .def("setRealtimeSyncMode",
             [](SimulatorItem& self, SimulatorItem::RealtimeSyncMode mode){ self.setRealtimeSyncMode(mode); })
        .def("setDeviceStateOutputEnabled", &SimulatorItem::setDeviceStateOutputEnabled)
        .def("isAllLinkPositionOutputMode", &SimulatorItem::isAllLinkPositionOutputMode)
        .def("setAllLinkPositionOutputMode", &SimulatorItem::setAllLinkPositionOutputMode)
        .def("setSceneViewEditModeBlockedDuringSimulation", &SimulatorItem::setSceneViewEditModeBlockedDuringSimulation)
        .def("setExternalForce",
             [](SimulatorItem& self, BodyItem* bodyItem, Link* link,
                const python::Vector3Arg& point, const python::Vector3Arg& f, double time){
                 self.setExternalForce(bodyItem, link, point, f, time); },
             nb::arg("bodyItem"), nb::arg("link"), nb::arg("point"), nb::arg("f"), nb::arg("time") = 0.0)
        .def("clearExternalForces", &SimulatorItem::clearExternalForces)
        .def("setForcedPosition", &SimulatorItem::setForcedPosition)
        .def("clearForcedPositions", &SimulatorItem::clearForcedPositions)
        ;

    nb::enum_<SimulatorItem::RecordingMode>(simulatorItemClass, "RecordingMode")
        .value("FullRecording", SimulatorItem::FullRecording)
        .value("TailRecording", SimulatorItem::TailRecording)
        .value("NoRecording", SimulatorItem::NoRecording)
        .value("NumRecordingModes", SimulatorItem::NumRecordingModes)
        .export_values();

    nb::enum_<SimulatorItem::TimeRangeMode>(simulatorItemClass, "TimeRangeMode")
        .value("UnlimitedTime", SimulatorItem::UnlimitedTime)
        .value("SpecifiedTime", SimulatorItem::SpecifiedTime)
        .value("TimeBarTime", SimulatorItem::TimeBarTime)
        .value("NumTimeRangeModes", SimulatorItem::NumTimeRangeModes)
        .export_values();

    nb::enum_<SimulatorItem::RealtimeSyncMode>(simulatorItemClass, "RealtimeSyncMode")
        .value("NonRealtimeSync", SimulatorItem::NonRealtimeSync)
        .value("CompensatoryRealtimeSync", SimulatorItem::CompensatoryRealtimeSync)
        .value("ConservativeRealtimeSync", SimulatorItem::ConservativeRealtimeSync)
        .value("NumRealtimeSyncModes", SimulatorItem::NumRealtimeSyncModes)
        .export_values();

    PyItemList<SimulatorItem>(m, "SimulatorItemList", simulatorItemClass);

    nb::class_<AISTSimulatorItem, SimulatorItem>
        aistSimulatorItemClass(m, "AISTSimulatorItem");

    aistSimulatorItemClass
        .def(nb::init<>())
        .def("setIntegrationMode",
             [](AISTSimulatorItem& self, AISTSimulatorItem::IntegrationMode mode){ self.setIntegrationMode(mode); })
        .def("setGravity",
             [](AISTSimulatorItem& self, const python::Vector3Arg& g){ self.setGravity(g.value); })
        .def("setFriction", (void (AISTSimulatorItem::*)(double, double)) &AISTSimulatorItem::setFriction)
        .def("setContactCullingDistance", &AISTSimulatorItem::setContactCullingDistance)
        .def("setContactCullingDepth", &AISTSimulatorItem::setContactCullingDepth)
        .def("setErrorCriterion", &AISTSimulatorItem::setErrorCriterion)
        .def("setMaxNumIterations", &AISTSimulatorItem::setMaxNumIterations)
        .def("setMaxNumContactPoints", &AISTSimulatorItem::setMaxNumContactPoints)
        .def("setContactCorrectionDepth", &AISTSimulatorItem::setContactCorrectionDepth)
        .def("setContactCorrectionVelocityRatio", &AISTSimulatorItem::setContactCorrectionVelocityRatio)
        .def("setEpsilon", &AISTSimulatorItem::setEpsilon)
        .def("set2Dmode", &AISTSimulatorItem::set2Dmode)
        .def("setKinematicWalkingEnabled", &AISTSimulatorItem::setKinematicWalkingEnabled)
        .def("clearExtraJoints", &AISTSimulatorItem::clearExtraJoints)
        .def("addExtraJoint", &AISTSimulatorItem::addExtraJoint)
        ;

    nb::enum_<AISTSimulatorItem::DynamicsMode>(aistSimulatorItemClass, "DynamicsMode")
        .value("ForwardDynamicsMode", AISTSimulatorItem::ForwardDynamicsMode)
        .value("KinematicsMode", AISTSimulatorItem::KinematicsMode)
        .export_values();

    nb::enum_<AISTSimulatorItem::IntegrationMode>(aistSimulatorItemClass, "IntegrationMode")
        .value("SemiImplicitEuler", AISTSimulatorItem::SemiImplicitEuler)
        .value("RungeKutta", AISTSimulatorItem::RungeKutta)
        .export_values();

    PyItemList<AISTSimulatorItem>(m, "AISTSimulatorItemList");

    nb::class_<SubSimulatorItem, Item>(m, "SubSimulatorItem")
        .def(nb::init<>())
        .def("isEnabled", &SubSimulatorItem::isEnabled)
        .def("setEnabled", &SubSimulatorItem::setEnabled);

    PyItemList<SubSimulatorItem>(m, "SubSimulatorItemList");

    nb::class_<SimulationScriptItem, ScriptItem> simulationScriptItemClass(m, "SimulationScriptItem");

    simulationScriptItemClass
        .def_prop_rw("executionTiming", &SimulationScriptItem::executionTiming, &SimulationScriptItem::setExecutionTiming)
        .def("setExecutionTiming", &SimulationScriptItem::setExecutionTiming)
        .def_prop_rw("executionDelay", &SimulationScriptItem::executionDelay, &SimulationScriptItem::setExecutionDelay)
        .def("setExecutionDelay", &SimulationScriptItem::setExecutionDelay)
        ;

    nb::enum_<SimulationScriptItem::ExecutionTiming>(simulationScriptItemClass, "ExecutionTiming")
        .value("BEFORE_INITIALIZATION", SimulationScriptItem::ExecutionTiming::BEFORE_INITIALIZATION)
        .value("DURING_INITIALIZATION", SimulationScriptItem::ExecutionTiming::DURING_INITIALIZATION)
        .value("AFTER_INITIALIZATION", SimulationScriptItem::ExecutionTiming::AFTER_INITIALIZATION)
        .value("DURING_FINALIZATION", SimulationScriptItem::ExecutionTiming::DURING_FINALIZATION)
        .value("AFTER_FINALIZATION", SimulationScriptItem::ExecutionTiming::AFTER_FINALIZATION)
        .value("NUM_TIMINGS", SimulationScriptItem::ExecutionTiming::NUM_TIMINGS)
        .export_values();

    //PyItemList<SimulationScriptItem>("SimulationScriptItemList");

    nb::class_<SimulationBar, ToolBar>(m, "SimulationBar")
        .def_prop_ro_static("instance", [](nb::handle){ return SimulationBar::instance(); }, nb::rv_policy::reference)
        .def("startSimulation", (void (SimulationBar::*)(bool)) &SimulationBar::startSimulation)
        ;

    nb::class_<ControllerItem, Item>(m, "ControllerItem")
        .def("isActive", &ControllerItem::isActive)
        .def("isNoDelayMode", &ControllerItem::isNoDelayMode)
        .def("setNoDelayMode", &ControllerItem::setNoDelayMode)
        .def("optionString", &ControllerItem::optionString)
        .def("setOptions", &ControllerItem::setOptions)
        .def("timeStep", &ControllerItem::timeStep)
        ;

    nb::class_<SimpleControllerItem, ControllerItem>(m, "SimpleControllerItem")
        .def(nb::init<>())
        .def("setController", &SimpleControllerItem::setController)
        ;

    nb::class_<BodyContactPointLogItem, ReferencedObjectSeqItem>
        bodyContactPointLogItem(m, "BodyContactPointLogItem");

    bodyContactPointLogItem
        .def(nb::init<>())
        .def("getLogFrame", [](BodyContactPointLogItem& self, int frameIndex){ return self.logFrame(frameIndex); })
        ;

    nb::class_<BodyContactPointLogItem::LogFrame>(bodyContactPointLogItem, "LogFrame")
        .def_prop_ro("bodyContactPoints",
            [](BodyContactPointLogItem::LogFrame& self){ return self.bodyContactPoints(); })
        .def("getLinkContactPoints",
            [](BodyContactPointLogItem::LogFrame& self, int linkIndex){ return self.linkContactPoints(linkIndex); })
        ;

    nb::class_<BodyContactPointLoggerItem, ControllerItem>(m, "BodyContactPointLoggerItem")
        .def(nb::init<>())
        .def("setLogFrameToVisualize", &BodyContactPointLoggerItem::setLogFrameToVisualize)
        ;
}

}
