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
#include <cnoid/PyBase>

using namespace boost::python;
using namespace cnoid;

// for MSVC++2015 Update3
//CNOID_PYTHON_DEFINE_GET_POINTER(SimulatorItem)
CNOID_PYTHON_DEFINE_GET_POINTER(SimulationBar)

namespace {

BodyItemPtr SimulationBody_bodyItem(SimulationBody& self) { return self.bodyItem(); }
BodyPtr SimulationBody_body(SimulationBody& self) { return self.body(); }

SimulatorItemPtr SimulatorItem_findActiveSimulatorItemFor(Item* item)
{
    return SimulatorItem::findActiveSimulatorItemFor(item);
}

BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(SimulatorItem_startSimulation_overloads, startSimulation, 0, 1)
BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(SimulatorItem_setExternalForce_overloads, setExternalForce, 4, 5)

}

void exportSimulationClasses()
{
    class_<SimulationBody, SimulationBodyPtr, bases<Referenced>, boost::noncopyable>("SimulationBody", no_init)
        .def("bodyItem", SimulationBody_bodyItem)
        .def("getBodyItem", SimulationBody_bodyItem)
        .def("body", SimulationBody_body)
        .def("getBody", SimulationBody_body);

    implicitly_convertible<SimulationBodyPtr, ReferencedPtr>();

    // The following code cannot be compiled with VC++2015 Update3
#ifndef _MSC_VER
    
    class_<SimulatorItem, SimulatorItemPtr, bases<Item>, boost::noncopyable>
        simulatorItemClass("SimulatorItem", no_init);

    simulatorItemClass
        .def("findActiveSimulatorItemFor", SimulatorItem_findActiveSimulatorItemFor).staticmethod("findActiveSimulatorItemFor")
        .def("worldTimeStep", &SimulatorItem::worldTimeStep)
        .def("getWorldTimeStep", &SimulatorItem::worldTimeStep)
        .def("setTimeStep", &SimulatorItem::setTimeStep)
        .def("startSimulation", &SimulatorItem::startSimulation, SimulatorItem_startSimulation_overloads())
        .def("stopSimulation", &SimulatorItem::stopSimulation)
        .def("pauseSimulation", &SimulatorItem::pauseSimulation)
        .def("restartSimulation", &SimulatorItem::restartSimulation)
        .def("isRunning", &SimulatorItem::isRunning)
        .def("currentFrame", &SimulatorItem::currentFrame)
        .def("getCurrentFrame", &SimulatorItem::currentFrame)
        .def("currentTime", &SimulatorItem::currentTime)
        .def("getCurrentTime", &SimulatorItem::currentTime)
        .def("sigSimulationFinished", &SimulatorItem::sigSimulationFinished)
        .def("getSigSimulationFinished", &SimulatorItem::sigSimulationFinished)
        .def("setRecordingMode", &SimulatorItem::setRecordingMode)
        .def("recordingMode", &SimulatorItem::recordingMode)
        .def("getRecordingMode", &SimulatorItem::recordingMode)
        .def("setTimeRangeMode", &SimulatorItem::setTimeRangeMode)
        .def("setRealtimeSyncMode", &SimulatorItem::setRealtimeSyncMode)
        .def("setDeviceStateOutputEnabled", &SimulatorItem::setDeviceStateOutputEnabled)
        .def("isRecordingEnabled", &SimulatorItem::isRecordingEnabled)
        .def("isDeviceStateOutputEnabled", &SimulatorItem::isDeviceStateOutputEnabled)
        .def("isAllLinkPositionOutputMode", &SimulatorItem::isAllLinkPositionOutputMode)
        .def("setAllLinkPositionOutputMode", &SimulatorItem::setAllLinkPositionOutputMode)
        .def("setExternalForce", &SimulatorItem::setExternalForce, SimulatorItem_setExternalForce_overloads())
        .def("clearExternalForces", &SimulatorItem::clearExternalForces)
        .def("setForcedPosition", &SimulatorItem::setForcedPosition)
        .def("clearForcedPositions", &SimulatorItem::clearForcedPositions)
        ;
    {
        scope simulatorItemScope = simulatorItemClass;

        enum_<SimulatorItem::RecordingMode>("RecordingMode")
            .value("REC_FULL", SimulatorItem::REC_FULL) 
            .value("REC_TAIL", SimulatorItem::REC_TAIL)
            .value("REC_NONE", SimulatorItem::REC_NONE)
            .value("N_RECORDING_MODES", SimulatorItem::N_RECORDING_MODES);
        
        enum_<SimulatorItem::TimeRangeMode>("TimeRangeMode")
            .value("UNLIMITED", SimulatorItem::TR_UNLIMITED)
            .value("ACTIVE_CONTROL", SimulatorItem::TR_ACTIVE_CONTROL)
            .value("SPECIFIED", SimulatorItem::TR_SPECIFIED)
            .value("TIMEBAR", SimulatorItem::TR_TIMEBAR) 
            .value("N_TIME_RANGE_MODES", SimulatorItem::N_TIME_RANGE_MODES)
            .value("TR_UNLIMITED", SimulatorItem::TR_UNLIMITED) // deprecated
            .value("TR_ACTIVE_CONTROL", SimulatorItem::TR_ACTIVE_CONTROL) // deprecated
            .value("TR_SPECIFIED", SimulatorItem::TR_SPECIFIED) // deprecated
            .value("TR_TIMEBAR", SimulatorItem::TR_TIMEBAR);  // deprecated
    }

    implicitly_convertible<SimulatorItemPtr, ItemPtr>();
    PyItemList<SimulatorItem>("SimulatorItemList", simulatorItemClass);

#ifdef _MSC_VER
    register_ptr_to_python<SimulatorItemPtr>();
#endif

    {
        scope aistSimulatorItemScope = 
            class_<AISTSimulatorItem, AISTSimulatorItemPtr, bases<SimulatorItem>>("AISTSimulatorItem")
            .def("setDynamicsMode", &AISTSimulatorItem::setDynamicsMode)
            .def("setIntegrationMode", &AISTSimulatorItem::setIntegrationMode)
            .def("setGravity", &AISTSimulatorItem::setGravity)
            .def("setFriction", (void(AISTSimulatorItem::*)(double,double)) &AISTSimulatorItem::setFriction)
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

            // deprecated
            .def("setFriction", (void(AISTSimulatorItem::*)(Link*,Link*,double,double)) &AISTSimulatorItem::setFriction)
            ;

        enum_<AISTSimulatorItem::DynamicsMode>("DynamicsMode")
            .value("FORWARD_DYNAMICS", AISTSimulatorItem::FORWARD_DYNAMICS) 
            .value("KINEMATICS", AISTSimulatorItem::KINEMATICS)
            .value("N_DYNAMICS_MODES", AISTSimulatorItem::N_DYNAMICS_MODES);

        enum_<AISTSimulatorItem::IntegrationMode>("IntegrationMode")
            .value("EULER_INTEGRATION", AISTSimulatorItem::EULER_INTEGRATION)
            .value("RUNGE_KUTTA_INTEGRATION", AISTSimulatorItem::RUNGE_KUTTA_INTEGRATION)
            .value("N_INTEGRATION_MODES", AISTSimulatorItem::N_INTEGRATION_MODES);
    }

    implicitly_convertible<AISTSimulatorItemPtr, SimulatorItemPtr>();
    PyItemList<AISTSimulatorItem>("AISTSimulatorItemList");

    class_< SubSimulatorItem, SubSimulatorItemPtr, bases<Item>, boost::noncopyable>("SubSimulatorItem", no_init)
        .def("isEnabled", &SubSimulatorItem::isEnabled)
        .def("setEnabled", &SubSimulatorItem::setEnabled);

    implicitly_convertible<SubSimulatorItemPtr, ItemPtr>();
    PyItemList<SubSimulatorItem>("SubSimulatorItemList");

    class_< GLVisionSimulatorItem, GLVisionSimulatorItemPtr, bases<SubSimulatorItem> >("GLVisionSimulatorItem")
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

    implicitly_convertible<GLVisionSimulatorItemPtr, SubSimulatorItemPtr>();
    PyItemList<GLVisionSimulatorItem>("GLVisionSimulatorItemList");

#endif
    
    {
        scope simulationScriptItemScope = 
            class_< SimulationScriptItem, SimulationScriptItemPtr, bases<ScriptItem>, boost::noncopyable >
            ("SimulationScriptItem", no_init)
            .def("executionTiming", &SimulationScriptItem::executionTiming)
            .def("setExecutionTiming", &SimulationScriptItem::setExecutionTiming)
            .def("executionDelay", &SimulationScriptItem::executionDelay)
            .def("setExecutionDelay", &SimulationScriptItem::setExecutionDelay);

        enum_<SimulationScriptItem::ExecutionTiming>("ExecutionTiming")
            .value("BEFORE_INITIALIZATION", SimulationScriptItem::BEFORE_INITIALIZATION)
            .value("DURING_INITIALIZATION", SimulationScriptItem::DURING_INITIALIZATION)
            .value("AFTER_INITIALIZATION", SimulationScriptItem::AFTER_INITIALIZATION)
            .value("DURING_FINALIZATION", SimulationScriptItem::DURING_FINALIZATION)
            .value("AFTER_FINALIZATION", SimulationScriptItem::AFTER_FINALIZATION)
            .value("NUM_TIMINGS", SimulationScriptItem::NUM_TIMINGS);
    }

    implicitly_convertible<SimulationScriptItemPtr, ScriptItemPtr>();
    //PyItemList<SimulationScriptItem>("SimulationScriptItemList");

    void (SimulationBar::*SimulationBar_startSimulation1)(SimulatorItem*, bool) = &SimulationBar::startSimulation;
    void (SimulationBar::*SimulationBar_startSimulation2)(bool) = &SimulationBar::startSimulation;
    
    class_<SimulationBar, SimulationBar*, boost::noncopyable>("SimulationBar", no_init)
        .def("instance", &SimulationBar::instance, return_value_policy<reference_existing_object>()).staticmethod("instance")
        .def("getInstance", &SimulationBar::instance, return_value_policy<reference_existing_object>()).staticmethod("getInstance")
        .def("startSimulation", SimulationBar_startSimulation1)
        .def("startSimulation", SimulationBar_startSimulation2)
        .def("stopSimulation", &SimulationBar::stopSimulation)
        .def("pauseSimulation", &SimulationBar::pauseSimulation)
        ;

    class_<SimpleControllerItem, SimpleControllerItemPtr, bases<Item>>("SimpleControllerItem")
        .def("setController", &SimpleControllerItem::setController);
}
