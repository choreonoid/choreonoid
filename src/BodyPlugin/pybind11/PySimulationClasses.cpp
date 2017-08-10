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
#include <cnoid/Py3Base>
#include <cnoid/EigenTypes>

namespace py = pybind11;
using namespace cnoid;

// for MSVC++2015 Update3
//CNOID_PYTHON_DEFINE_GET_POINTER(SimulatorItem)
CNOID_PYTHON_DEFINE_GET_POINTER(SimulationBar)

void exportSimulationClasses(py::module m)
{
    py::class_<SimulationBody, SimulationBodyPtr, Referenced>(m, "SimulationBody")
        .def("bodyItem", [](SimulationBody& self) { return BodyItemPtr(self.bodyItem()); })
        .def("body", [](SimulationBody& self) { return BodyPtr(self.body()); });

    // The following code cannot be compiled with VC++2015 Update3
#ifndef _MSC_VER
    
    py::class_<SimulatorItem, SimulatorItemPtr, Item> simulatorItemClass(m, "SimulatorItem");

    simulatorItemClass
        .def_static("findActiveSimulatorItemFor", [](Item* item) {
            return SimulatorItemPtr(SimulatorItem::findActiveSimulatorItemFor(item));
        })
        .def("worldTimeStep", &SimulatorItem::worldTimeStep)
        .def("startSimulation", &SimulatorItem::startSimulation, py::arg("doReset")=true)
        .def("stopSimulation", &SimulatorItem::stopSimulation)
        .def("pauseSimulation", &SimulatorItem::pauseSimulation)
        .def("restartSimulation", &SimulatorItem::restartSimulation)
        .def("isRunning", &SimulatorItem::isRunning)
        .def("currentFrame", &SimulatorItem::currentFrame)
        .def("currentTime", &SimulatorItem::currentTime)
        .def("sigSimulationFinished", &SimulatorItem::sigSimulationFinished)
        .def("setRecordingMode", &SimulatorItem::setRecordingMode)
        .def("recordingMode", &SimulatorItem::recordingMode)
        .def("setTimeRangeMode", &SimulatorItem::setTimeRangeMode)
        .def("setRealtimeSyncMode", &SimulatorItem::setRealtimeSyncMode)
        .def("setDeviceStateOutputEnabled", &SimulatorItem::setDeviceStateOutputEnabled)
        .def("isRecordingEnabled", &SimulatorItem::isRecordingEnabled)
        .def("isDeviceStateOutputEnabled", &SimulatorItem::isDeviceStateOutputEnabled)
        .def("isAllLinkPositionOutputMode", &SimulatorItem::isAllLinkPositionOutputMode)
        .def("setAllLinkPositionOutputMode", &SimulatorItem::setAllLinkPositionOutputMode)
        .def("setExternalForce", &SimulatorItem::setExternalForce,
            py::arg("bodyItem"), py::arg("link"), py::arg("point"), py::arg("f"), py::arg("time")=0.0)
        .def("clearExternalForces", &SimulatorItem::clearExternalForces)
        .def("setForcedPosition", &SimulatorItem::setForcedPosition)
        .def("clearForcedPositions", &SimulatorItem::clearForcedPositions)
        ;

    py::enum_<SimulatorItem::RecordingMode>(simulatorItemClass, "RecordingMode")
        .value("REC_FULL", SimulatorItem::RecordingMode::REC_FULL)
        .value("REC_TAIL", SimulatorItem::RecordingMode::REC_TAIL)
        .value("REC_NONE", SimulatorItem::RecordingMode::REC_NONE)
        .value("N_RECORDING_MODES", SimulatorItem::RecordingMode::N_RECORDING_MODES)
        .export_values();
        
    py::enum_<SimulatorItem::TimeRangeMode>(simulatorItemClass, "TimeRangeMode")
        .value("TR_UNLIMITED", SimulatorItem::TimeRangeMode::TR_UNLIMITED)
        .value("TR_ACTIVE_CONTROL", SimulatorItem::TimeRangeMode::TR_ACTIVE_CONTROL)
        .value("TR_SPECIFIED", SimulatorItem::TimeRangeMode::TR_SPECIFIED)
        .value("TR_TIMEBAR", SimulatorItem::TimeRangeMode::TR_TIMEBAR)
        .value("N_TIME_RANGE_MODES", SimulatorItem::TimeRangeMode::N_TIME_RANGE_MODES)
        .export_values();

    PyItemList<SimulatorItem>(m, "SimulatorItemList", simulatorItemClass);

#ifdef _MSC_VER
    register_ptr_to_python<SimulatorItemPtr>();
#endif

    py::class_< AISTSimulatorItem, AISTSimulatorItemPtr, SimulatorItem> aistSimulatorItemClass(m, "AISTSimulatorItem");

    aistSimulatorItemClass
        .def(py::init<>())
        .def("setIntegrationMode", &AISTSimulatorItem::setIntegrationMode)
        .def("setGravity", &AISTSimulatorItem::setGravity)
        .def("setFriction", (void (AISTSimulatorItem::*)(double, double)) &AISTSimulatorItem::setFriction)
        .def("setFriction", (void (AISTSimulatorItem::*)(Link*, Link*, double, double)) &AISTSimulatorItem::setFriction)
        .def("collisionHandlerId", &AISTSimulatorItem::collisionHandlerId)
        .def("setCollisionHandler", &AISTSimulatorItem::setCollisionHandler)
        .def("setContactCullingDistance", &AISTSimulatorItem::setContactCullingDistance)
        .def("setContactCullingDepth", &AISTSimulatorItem::setContactCullingDepth)
        .def("setErrorCriterion", &AISTSimulatorItem::setErrorCriterion)
        .def("setMaxNumIterations", &AISTSimulatorItem::setMaxNumIterations)
        .def("setContactCorrectionDepth", &AISTSimulatorItem::setContactCorrectionDepth)
        .def("setContactCorrectionVelocityRatio", &AISTSimulatorItem::setContactCorrectionVelocityRatio)
        .def("setEpsilon", &AISTSimulatorItem::setEpsilon)
        .def("set2Dmode", &AISTSimulatorItem::set2Dmode)
        .def("setKinematicWalkingEnabled", &AISTSimulatorItem::setKinematicWalkingEnabled)
        .def("setConstraintForceOutputEnabled", &AISTSimulatorItem::setConstraintForceOutputEnabled)
        .def("clearExtraJoint", &AISTSimulatorItem::clearExtraJoint)
        .def("addExtraJoint", &AISTSimulatorItem::addExtraJoint)
        ;

    py::enum_<AISTSimulatorItem::DynamicsMode>(aistSimulatorItemClass, "DynamicsMode")
        .value("FORWARD_DYNAMICS", AISTSimulatorItem::DynamicsMode::FORWARD_DYNAMICS)
        .value("HG_DYNAMICS", AISTSimulatorItem::DynamicsMode::HG_DYNAMICS)
        .value("KINEMATICS", AISTSimulatorItem::DynamicsMode::KINEMATICS)
        .value("N_DYNAMICS_MODES", AISTSimulatorItem::DynamicsMode::N_DYNAMICS_MODES)
        .export_values();

    py::enum_<AISTSimulatorItem::IntegrationMode>(aistSimulatorItemClass, "IntegrationMode")
        .value("EULER_INTEGRATION", AISTSimulatorItem::IntegrationMode::EULER_INTEGRATION)
        .value("RUNGE_KUTTA_INTEGRATION", AISTSimulatorItem::IntegrationMode::RUNGE_KUTTA_INTEGRATION)
        .value("N_INTEGRATION_MODES", AISTSimulatorItem::IntegrationMode::N_INTEGRATION_MODES)
        .export_values();

    PyItemList<AISTSimulatorItem>(m, "AISTSimulatorItemList");

    py::class_< SubSimulatorItem, SubSimulatorItemPtr, Item>(m, "SubSimulatorItem")
        .def(py::init<>())
        .def("isEnabled", &SubSimulatorItem::isEnabled)
        .def("setEnabled", &SubSimulatorItem::setEnabled);

    PyItemList<SubSimulatorItem>(m, "SubSimulatorItemList");

    py::class_< GLVisionSimulatorItem, GLVisionSimulatorItemPtr, SubSimulatorItem>(m, "GLVisionSimulatorItem")
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

#endif
    
    py::class_< SimulationScriptItem, SimulationScriptItemPtr, ScriptItem> simulationScriptItemClass(m,"SimulationScriptItem");

    simulationScriptItemClass
        .def("executionTiming", &SimulationScriptItem::executionTiming)
        .def("setExecutionTiming", &SimulationScriptItem::setExecutionTiming)
        .def("executionDelay", &SimulationScriptItem::executionDelay)
        .def("setExecutionDelay", &SimulationScriptItem::setExecutionDelay);

    py::enum_<SimulationScriptItem::ExecutionTiming>(simulationScriptItemClass, "ExecutionTiming")
        .value("BEFORE_INITIALIZATION", SimulationScriptItem::ExecutionTiming::BEFORE_INITIALIZATION)
        .value("DURING_INITIALIZATION", SimulationScriptItem::ExecutionTiming::DURING_INITIALIZATION)
        .value("AFTER_INITIALIZATION", SimulationScriptItem::ExecutionTiming::AFTER_INITIALIZATION)
        .value("DURING_FINALIZATION", SimulationScriptItem::ExecutionTiming::DURING_FINALIZATION)
        .value("AFTER_FINALIZATION", SimulationScriptItem::ExecutionTiming::AFTER_FINALIZATION)
        .value("NUM_TIMINGS", SimulationScriptItem::ExecutionTiming::NUM_TIMINGS)
        .export_values();

    //PyItemList<SimulationScriptItem>("SimulationScriptItemList");

    py::class_<SimulationBar>(m, "SimulationBar")
        .def_static("instance", &SimulationBar::instance, py::return_value_policy::reference)
        .def("startSimulation", (void (SimulationBar::*)(SimulatorItem*, bool)) &SimulationBar::startSimulation)
        .def("startSimulation", (void (SimulationBar::*)(bool)) &SimulationBar::startSimulation)
        .def("stopSimulation", &SimulationBar::stopSimulation)
        .def("pauseSimulation", &SimulationBar::pauseSimulation)
        ;

}
