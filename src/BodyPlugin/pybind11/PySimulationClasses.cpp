/*!
  @author Shin'ichiro Nakaoka
*/

#include "../SimulatorItem.h"
#include "../AISTSimulatorItem.h"
#include "../SubSimulatorItem.h"
#include "../GLVisionSimulatorItem.h"
#include "../SimulationScriptItem.h"
#include "../SimulationBar.h"
#include "../BodyItem.h"
#include "../SimpleControllerItem.h"
#include "../ControllerLogItem.h"
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
        .def("startSimulation", &SimulatorItem::startSimulation)
        .def("startSimulation", [](SimulatorItem& self){ return self.startSimulation(); })
        .def("stopSimulation", &SimulatorItem::stopSimulation)
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
        .def("setRealtimeSyncMode", &SimulatorItem::setRealtimeSyncMode)
        .def("setDeviceStateOutputEnabled", &SimulatorItem::setDeviceStateOutputEnabled)
        .def("isAllLinkPositionOutputMode", &SimulatorItem::isAllLinkPositionOutputMode)
        .def("setAllLinkPositionOutputMode", &SimulatorItem::setAllLinkPositionOutputMode)
        .def("setExternalForce", &SimulatorItem::setExternalForce,
             py::arg("bodyItem"), py::arg("link"), py::arg("point"), py::arg("f"), py::arg("time") = 0.0)
        .def("clearExternalForces", &SimulatorItem::clearExternalForces)
        .def("setForcedPosition", &SimulatorItem::setForcedPosition)
        .def("clearForcedPositions", &SimulatorItem::clearForcedPositions)

        // deprecated
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
        .value("UNLIMITED", SimulatorItem::UnlimitedTime)
        .value("ACTIVE_CONTROL", SimulatorItem::ActiveControlTime)
        .value("SPECIFIED", SimulatorItem::SpecifiedTime)
        .value("TIMEBAR", SimulatorItem::TimeBarTime)
        .value("N_TIME_RANGE_MODES", SimulatorItem::NumTimeRangeModes)
        .export_values();

    PyItemList<SimulatorItem>(m, "SimulatorItemList", simulatorItemClass);

    py::class_<AISTSimulatorItem, AISTSimulatorItemPtr, SimulatorItem> aistSimulatorItemClass(m, "AISTSimulatorItem");

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
        .value("FORWARD_DYNAMICS", AISTSimulatorItem::DynamicsMode::FORWARD_DYNAMICS)
        .value("KINEMATICS", AISTSimulatorItem::DynamicsMode::KINEMATICS)
        .value("N_DYNAMICS_MODES", AISTSimulatorItem::DynamicsMode::N_DYNAMICS_MODES)
        .export_values();

    py::enum_<AISTSimulatorItem::IntegrationMode>(aistSimulatorItemClass, "IntegrationMode")
        .value("EULER_INTEGRATION", AISTSimulatorItem::IntegrationMode::EULER_INTEGRATION)
        .value("RUNGE_KUTTA_INTEGRATION", AISTSimulatorItem::IntegrationMode::RUNGE_KUTTA_INTEGRATION)
        .value("N_INTEGRATION_MODES", AISTSimulatorItem::IntegrationMode::N_INTEGRATION_MODES)
        .export_values();

    PyItemList<AISTSimulatorItem>(m, "AISTSimulatorItemList");

    py::class_<SubSimulatorItem, SubSimulatorItemPtr, Item>(m, "SubSimulatorItem")
        .def(py::init<>())
        .def("isEnabled", &SubSimulatorItem::isEnabled)
        .def("setEnabled", &SubSimulatorItem::setEnabled);

    PyItemList<SubSimulatorItem>(m, "SubSimulatorItemList");

    py::class_<GLVisionSimulatorItem, GLVisionSimulatorItemPtr, SubSimulatorItem>(m, "GLVisionSimulatorItem")
        .def(py::init<>())
        .def("setTargetBodies", &GLVisionSimulatorItem::setTargetBodies)
        .def("setTargetSensors", &GLVisionSimulatorItem::setTargetSensors)
        .def("setMaxFrameRate", &GLVisionSimulatorItem::setMaxFrameRate)
        .def("setMaxLatency", &GLVisionSimulatorItem::setMaxLatency)
        .def("setVisionDataRecordingEnabled", &GLVisionSimulatorItem::setVisionDataRecordingEnabled)
        .def("setDedicatedSensorThreadsEnabled", &GLVisionSimulatorItem::setDedicatedSensorThreadsEnabled)
        .def("setBestEffortMode", &GLVisionSimulatorItem::setBestEffortMode)
        .def("setRangeSensorPrecisionRatio", &GLVisionSimulatorItem::setRangeSensorPrecisionRatio)
        .def("setAllSceneObjectsEnabled", &GLVisionSimulatorItem::setAllSceneObjectsEnabled)
        .def("setHeadLightEnabled", &GLVisionSimulatorItem::setHeadLightEnabled)
        .def("setAdditionalLightsEnabled", &GLVisionSimulatorItem::setAdditionalLightsEnabled)
        ;

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

    py::class_<ControllerLogItem, ControllerLogItemPtr, ReferencedObjectSeqItem>(m, "ControllerLogItem")
        .def(py::init<>())
        .def_property_readonly("log", &ControllerLogItem::log)
        .def("resetLog", &ControllerLogItem::resetLog)
        ;

    PyItemList<ControllerLogItem>(m, "ControllerLogItemList");
}

}
