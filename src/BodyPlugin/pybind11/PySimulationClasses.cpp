#include "../SimulatorItem.h"
#include "../AISTSimulatorItem.h"
#include "../SubSimulatorItem.h"
#include "../GLVisionSimulatorItem.h"
#include "../SimulationScriptItem.h"
#include "../SimulationBar.h"
#include "../BodyItem.h"
#include "../SimpleControllerItem.h"
#include "../BodyContactPointLoggerItem.h"
#include "../BodyContactPointLogItem.h"
#include <cnoid/PyBase>

using namespace cnoid;
namespace py = pybind11;

namespace cnoid {

void exportSimulationClasses(py::module m)
{
    py::class_<SimulatorItem, SimulatorItemPtr, Item> simulatorItemClass(m, "SimulatorItem");

    simulatorItemClass
        .def_static("findActiveSimulatorItemFor", &SimulatorItem::findActiveSimulatorItemFor)
        .def_property_readonly("worldTimeStep", &SimulatorItem::worldTimeStep)
        .def("setTimeStep", &SimulatorItem::setTimeStep)
        .def("startSimulation", &SimulatorItem::startSimulation, py::arg("doReset") = true)
        .def("stopSimulation", &SimulatorItem::stopSimulation, py::arg("isForced") = false)
        .def("pauseSimulation", &SimulatorItem::pauseSimulation)
        .def("restartSimulation", &SimulatorItem::restartSimulation)
        .def("isRunning", &SimulatorItem::isRunning)
        .def_property_readonly("sigSimulationStarted", &SimulatorItem::sigSimulationStarted)
        .def_property_readonly("sigSimulationPaused", &SimulatorItem::sigSimulationPaused)
        .def_property_readonly("sigSimulationResumed", &SimulatorItem::sigSimulationResumed)
        .def_property_readonly("sigSimulationFinished", &SimulatorItem::sigSimulationFinished)
        .def_property_readonly("currentFrame", &SimulatorItem::currentFrame)
        .def_property_readonly("currentTime", &SimulatorItem::currentTime)
        .def("setRecordingMode", &SimulatorItem::setRecordingMode)
        .def_property_readonly("recordingMode", &SimulatorItem::recordingMode)
        .def("setTimeRangeMode", &SimulatorItem::setTimeRangeMode)
        .def("setTimeLength", &SimulatorItem::setTimeLength)
        .def("setActiveControlTimeRangeMode", &SimulatorItem::setActiveControlTimeRangeMode)
        .def("isActiveControlTimeRangeMode", &SimulatorItem::isActiveControlTimeRangeMode)
        .def("isRecordingEnabled", &SimulatorItem::isRecordingEnabled)
        .def("isDeviceStateOutputEnabled", &SimulatorItem::isDeviceStateOutputEnabled)
        .def("setRealtimeSyncMode", [](SimulatorItem& self, int mode){ self.setRealtimeSyncMode(mode); })
        .def("setDeviceStateOutputEnabled", &SimulatorItem::setDeviceStateOutputEnabled)
        .def("isAllLinkPositionOutputMode", &SimulatorItem::isAllLinkPositionOutputMode)
        .def("setAllLinkPositionOutputMode", &SimulatorItem::setAllLinkPositionOutputMode)
        .def("setSceneViewEditModeBlockedDuringSimulation", &SimulatorItem::setSceneViewEditModeBlockedDuringSimulation)
        .def("setExternalForce", &SimulatorItem::setExternalForce,
             py::arg("bodyItem"), py::arg("link"), py::arg("point"), py::arg("f"), py::arg("time") = 0.0)
        .def("clearExternalForces", &SimulatorItem::clearExternalForces)
        .def("setForcedPosition", &SimulatorItem::setForcedPosition)
        .def("clearForcedPositions", &SimulatorItem::clearForcedPositions)

        // deprecated
        .def("setRealtimeSyncMode",
             [](SimulatorItem& self, bool on){
                 self.setRealtimeSyncMode(
                     on ? SimulatorItem::CompensatoryRealtimeSync : SimulatorItem::NonRealtimeSync);
             })
        .def("setSpecifiedRecordingTimeLength", &SimulatorItem::setTimeLength)
        .def("getWorldTimeStep", &SimulatorItem::worldTimeStep)
        .def("getCurrentFrame", &SimulatorItem::currentFrame)
        .def("getCurrentTime", &SimulatorItem::currentTime)
        .def("getRecordingMode", &SimulatorItem::recordingMode)
        .def("getSigSimulationStarted", &SimulatorItem::sigSimulationStarted)
        .def("getSigSimulationPaused", &SimulatorItem::sigSimulationPaused)
        .def("getSigSimulationResumed", &SimulatorItem::sigSimulationResumed)
        .def("getSigSimulationFinished", &SimulatorItem::sigSimulationFinished)

        ;

    py::enum_<SimulatorItem::RecordingMode>(simulatorItemClass, "RecordingMode")
        .value("FullRecording", SimulatorItem::FullRecording)
        .value("TailRecording", SimulatorItem::TailRecording)
        .value("NoRecording", SimulatorItem::NoRecording)
        .value("NumRecordingModes", SimulatorItem::NumRecordingModes)

        // deprecated
        .value("REC_FULL", SimulatorItem::FullRecording)
        .value("REC_TAIL", SimulatorItem::TailRecording)
        .value("REC_NONE", SimulatorItem::NoRecording)
        .value("N_RECORDING_MODES", SimulatorItem::NumRecordingModes)
        .export_values();
        
    py::enum_<SimulatorItem::TimeRangeMode>(simulatorItemClass, "TimeRangeMode")
        .value("UnlimitedTime", SimulatorItem::UnlimitedTime)
        .value("SpecifiedTime", SimulatorItem::SpecifiedTime)
        .value("TimeBarTime", SimulatorItem::TimeBarTime)
        .value("NumTimeRangeModes", SimulatorItem::NumTimeRangeModes)

        // deprecated
        .value("ActiveControlTime", SimulatorItem::ActiveControlTime)
        .value("TR_UNLIMITED", SimulatorItem::UnlimitedTime)
        .value("TR_ACTIVE_CONTROL", SimulatorItem::ActiveControlTime)
        .value("TR_SPECIFIED", SimulatorItem::SpecifiedTime)
        .value("TR_TIMEBAR", SimulatorItem::TimeBarTime)
        .value("N_TIME_RANGE_MODES", SimulatorItem::NumTimeRangeModes)
        .export_values();

    py::enum_<SimulatorItem::RealtimeSyncMode>(simulatorItemClass, "RealtimeSyncMode")
        .value("NonRealtimeSync", SimulatorItem::NonRealtimeSync)
        .value("CompensatoryRealtimeSync", SimulatorItem::CompensatoryRealtimeSync)
        .value("ConservativeRealtimeSync", SimulatorItem::ConservativeRealtimeSync)
        .value("NumRealtimeSyncModes", SimulatorItem::NumRealtimeSyncModes)
        .export_values();

    PyItemList<SimulatorItem>(m, "SimulatorItemList", simulatorItemClass);

    py::class_<AISTSimulatorItem, AISTSimulatorItemPtr, SimulatorItem>
        aistSimulatorItemClass(m, "AISTSimulatorItem");

    aistSimulatorItemClass
        .def(py::init<>())
        .def("setIntegrationMode", &AISTSimulatorItem::setIntegrationMode)
        .def("setGravity", &AISTSimulatorItem::setGravity)
        .def("setFriction", (void (AISTSimulatorItem::*)(double, double)) &AISTSimulatorItem::setFriction)
        .def("setContactCullingDistance", &AISTSimulatorItem::setContactCullingDistance)
        .def("setContactCullingDepth", &AISTSimulatorItem::setContactCullingDepth)
        .def("setErrorCriterion", &AISTSimulatorItem::setErrorCriterion)
        .def("setMaxNumIterations", &AISTSimulatorItem::setMaxNumIterations)
        .def("setContactCorrectionDepth", &AISTSimulatorItem::setContactCorrectionDepth)
        .def("setContactCorrectionVelocityRatio", &AISTSimulatorItem::setContactCorrectionVelocityRatio)
        .def("setEpsilon", &AISTSimulatorItem::setEpsilon)
        .def("set2Dmode", &AISTSimulatorItem::set2Dmode)
        .def("setKinematicWalkingEnabled", &AISTSimulatorItem::setKinematicWalkingEnabled)
        .def("clearExtraJoints", &AISTSimulatorItem::clearExtraJoints)
        .def("addExtraJoint", &AISTSimulatorItem::addExtraJoint)

        // deprecated
        .def("setFriction", (void (AISTSimulatorItem::*)(Link*, Link*, double, double)) &AISTSimulatorItem::setFriction)
        ;

    py::enum_<AISTSimulatorItem::DynamicsMode>(aistSimulatorItemClass, "DynamicsMode")
        .value("ForwardDynamicsMode", AISTSimulatorItem::ForwardDynamicsMode)
        .value("KinematicsMode", AISTSimulatorItem::KinematicsMode)
        // deprecated
        .value("FORWARD_DYNAMICS", AISTSimulatorItem::ForwardDynamicsMode)
        .value("KINEMATICS", AISTSimulatorItem::KinematicsMode)
        .export_values();

    py::enum_<AISTSimulatorItem::IntegrationMode>(aistSimulatorItemClass, "IntegrationMode")
        .value("SemiImplicitEuler", AISTSimulatorItem::SemiImplicitEuler)
        .value("RungeKutta", AISTSimulatorItem::RungeKutta)
        // deprecated
        .value("EULER_INTEGRATION", AISTSimulatorItem::SemiImplicitEuler)
        .value("RUNGE_KUTTA_INTEGRATION", AISTSimulatorItem::RungeKutta)
        .export_values();

    PyItemList<AISTSimulatorItem>(m, "AISTSimulatorItemList");

    py::class_<SubSimulatorItem, SubSimulatorItemPtr, Item>(m, "SubSimulatorItem")
        .def(py::init<>())
        .def("isEnabled", &SubSimulatorItem::isEnabled)
        .def("setEnabled", &SubSimulatorItem::setEnabled);

    PyItemList<SubSimulatorItem>(m, "SubSimulatorItemList");

    py::class_<GLVisionSimulatorItem, GLVisionSimulatorItemPtr, SubSimulatorItem>
        glVisionSimulatorItemClass(m, "GLVisionSimulatorItem");

    glVisionSimulatorItemClass
        .def(py::init<>())
        .def("setTargetBodies", &GLVisionSimulatorItem::setTargetBodies)
        .def("setTargetSensors", &GLVisionSimulatorItem::setTargetSensors)
        .def("setMaxFrameRate", &GLVisionSimulatorItem::setMaxFrameRate)
        .def("setMaxLatency", &GLVisionSimulatorItem::setMaxLatency)
        .def("setVisionDataRecordingEnabled", &GLVisionSimulatorItem::setVisionDataRecordingEnabled)
        .def("setThreadMode", &GLVisionSimulatorItem::setThreadMode)
        .def("setBestEffortMode", &GLVisionSimulatorItem::setBestEffortMode)
        .def("setRangeSensorPrecisionRatio", &GLVisionSimulatorItem::setRangeSensorPrecisionRatio)
        .def("setAllSceneObjectsEnabled", &GLVisionSimulatorItem::setAllSceneObjectsEnabled)
        .def("setHeadLightEnabled", &GLVisionSimulatorItem::setHeadLightEnabled)
        .def("setAdditionalLightsEnabled", &GLVisionSimulatorItem::setAdditionalLightsEnabled)

        // deprecated
        .def("setDedicatedSensorThreadsEnabled",
             [](GLVisionSimulatorItem& self, bool on){
                 self.setThreadMode(
                     on ? GLVisionSimulatorItem::SENSOR_THREAD_MODE : GLVisionSimulatorItem::SINGLE_THREAD_MODE);
             })
        ;

    py::enum_<GLVisionSimulatorItem::ThreadMode>(glVisionSimulatorItemClass, "ThreadMode")
        .value("SINGLE_THREAD_MODE", GLVisionSimulatorItem::SINGLE_THREAD_MODE)
        .value("SENSOR_THREAD_MODE", GLVisionSimulatorItem::SENSOR_THREAD_MODE)
        .value("SCREEN_THREAD_MODE", GLVisionSimulatorItem::SCREEN_THREAD_MODE)
        .value("N_THREAD_MODES", GLVisionSimulatorItem::N_THREAD_MODES)
        .export_values();

    PyItemList<GLVisionSimulatorItem>(m, "GLVisionSimulatorItemList");

    py::class_<SimulationScriptItem, SimulationScriptItemPtr, ScriptItem> simulationScriptItemClass(m,"SimulationScriptItem");

    simulationScriptItemClass
        .def_property("executionTiming", &SimulationScriptItem::executionTiming, &SimulationScriptItem::setExecutionTiming)
        .def("setExecutionTiming", &SimulationScriptItem::setExecutionTiming)
        .def_property("executionDelay", &SimulationScriptItem::executionDelay, &SimulationScriptItem::setExecutionDelay)
        .def("setExecutionDelay", &SimulationScriptItem::setExecutionDelay)

        // deprecated
        .def("getExecutionTiming", &SimulationScriptItem::executionTiming)
        .def("getExecutionDelay", &SimulationScriptItem::executionDelay)
        ;

    py::enum_<SimulationScriptItem::ExecutionTiming>(simulationScriptItemClass, "ExecutionTiming")
        .value("BEFORE_INITIALIZATION", SimulationScriptItem::ExecutionTiming::BEFORE_INITIALIZATION)
        .value("DURING_INITIALIZATION", SimulationScriptItem::ExecutionTiming::DURING_INITIALIZATION)
        .value("AFTER_INITIALIZATION", SimulationScriptItem::ExecutionTiming::AFTER_INITIALIZATION)
        .value("DURING_FINALIZATION", SimulationScriptItem::ExecutionTiming::DURING_FINALIZATION)
        .value("AFTER_FINALIZATION", SimulationScriptItem::ExecutionTiming::AFTER_FINALIZATION)
        .value("NUM_TIMINGS", SimulationScriptItem::ExecutionTiming::NUM_TIMINGS)
        .export_values();

    //PyItemList<SimulationScriptItem>("SimulationScriptItemList");

    py::class_<SimulationBar, PyQObjectHolder<SimulationBar>, ToolBar>(m, "SimulationBar")
        .def_property_readonly_static("instance", [](py::object){ return SimulationBar::instance(); })
        .def("startSimulation", (void (SimulationBar::*)(bool)) &SimulationBar::startSimulation)
        ;

    py::class_<ControllerItem, ControllerItemPtr, Item>(m, "ControllerItem")
        .def("isActive", &ControllerItem::isActive)
        .def("isNoDelayMode", &ControllerItem::isNoDelayMode)
        .def("setNoDelayMode", &ControllerItem::setNoDelayMode)
        .def("optionString", &ControllerItem::optionString)
        .def("setOptions", &ControllerItem::setOptions)
        .def("timeStep", &ControllerItem::timeStep)
        ;
    
    py::class_<SimpleControllerItem, SimpleControllerItemPtr, ControllerItem>(m, "SimpleControllerItem")
        .def(py::init<>())
        .def("setController", &SimpleControllerItem::setController)
        ;

    py::class_<BodyContactPointLogItem, BodyContactPointLogItemPtr, ReferencedObjectSeqItem>
        bodyContactPointLogItem(m, "BodyContactPointLogItem");

    bodyContactPointLogItem
        .def(py::init<>())
        .def("getLogFrame", [](BodyContactPointLogItem& self, int frameIndex){ return self.logFrame(frameIndex); })
        ;

    py::class_<BodyContactPointLogItem::LogFrame, BodyContactPointLogItem::LogFramePtr>(bodyContactPointLogItem, "LogFrame")
        .def_property_readonly("bodyContactPoints",
            [](BodyContactPointLogItem::LogFrame& self){ return self.bodyContactPoints(); })
        .def("getLinkContactPoints",
            [](BodyContactPointLogItem::LogFrame& self, int linkIndex){ return self.linkContactPoints(linkIndex); })
        ;

    py::class_<BodyContactPointLoggerItem, BodyContactPointLoggerItemPtr, ControllerItem>(m, "BodyContactPointLoggerItem")
        .def(py::init<>())
        .def("setLogFrameToVisualize", &BodyContactPointLoggerItem::setLogFrameToVisualize)
        ;
}

}
