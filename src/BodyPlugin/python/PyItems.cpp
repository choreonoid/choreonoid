/*!
  @author Shin'ichiro Nakaoka
*/

#include "../BodyItem.h"
#include "../BodyMotionItem.h"
#include "../WorldItem.h"
#include "../SimulatorItem.h"
#include "../AISTSimulatorItem.h"
#include "../SubSimulatorItem.h"
#include "../GLVisionSimulatorItem.h"
#include "../SimulationScriptItem.h"
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

BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(BodyItem_calcForwardKinematics_overloads, calcForwardKinematics, 0, 2)

void (BodyItem::*BodyItem_notifyKinematicStateChange1)(bool, bool, bool) = &BodyItem::notifyKinematicStateChange;
BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(BodyItem_notifyKinematicStateChange1_overloads, notifyKinematicStateChange, 0, 2)

void (BodyItem::*BodyItem_notifyKinematicStateChange2)(Connection&, bool, bool, bool) = &BodyItem::notifyKinematicStateChange;
BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(BodyItem_notifyKinematicStateChange2_overloads, notifyKinematicStateChange, 1, 3)

MultiValueSeqItemPtr BodyMotionItem_jointPosSeqItem(BodyMotionItem& self) { return self.jointPosSeqItem(); }
MultiSE3SeqItemPtr BodyMotionItem_linkPosSeqItem(BodyMotionItem& self) { return self.linkPosSeqItem(); }
AbstractSeqItemPtr BodyMotionItem_extraSeqItem(BodyMotionItem& self, int index) { return self.extraSeqItem(index); }
BodyMotion& BodyMotionItem_motion(BodyMotionItem& self) { return *self.motion(); }


BodyItemPtr SimulationBody_bodyItem(SimulationBody& self) { return self.bodyItem(); }
BodyPtr SimulationBody_body(SimulationBody& self) { return self.body(); }

SimulatorItemPtr SimulatorItem_findActiveSimulatorItemFor(Item* item)
{
    return SimulatorItem::findActiveSimulatorItemFor(item);
}

BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(SimulatorItem_startSimulation_overloads, startSimulation, 0, 1)
BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(SimulatorItem_setExternalForce_overloads, setExternalForce, 4, 5)

void AISTSimulatorItem_setFriction1(AISTSimulatorItem& self, double staticFriction, double slipFriction)
{
    self.setFriction(staticFriction, slipFriction);
}

void AISTSimulatorItem_setFriction2(AISTSimulatorItem& self, Link* link1, Link* link2, double staticFriction, double slipFriction)
{
    self.setFriction(link1, link2, staticFriction, slipFriction);
}


}

void exportItems()
{
    def("loadBodyItem", loadBodyItem);
    
    {
        scope bodyItemScope = 
            class_< BodyItem, BodyItemPtr, bases<Item, SceneProvider> >("BodyItem")
            .def("loadModelFile", &BodyItem::loadModelFile)
            .def("setName", &BodyItem::setName)
            .def("body", BodyItem_body)
            .def("isEditable", &BodyItem::isEditable)
            .def("moveToOrigin", &BodyItem::moveToOrigin)
            .def("setPresetPose", &BodyItem::setPresetPose)
            .def("currentBaseLink", &BodyItem::currentBaseLink, return_internal_reference<>())
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
            .def("notifyKinematicStateChange", BodyItem_notifyKinematicStateChange1, BodyItem_notifyKinematicStateChange1_overloads())
            .def("notifyKinematicStateChange", BodyItem_notifyKinematicStateChange2, BodyItem_notifyKinematicStateChange2_overloads())
            .def("enableCollisionDetection", &BodyItem::enableCollisionDetection)
            .def("isCollisionDetectionEnabled", &BodyItem::isCollisionDetectionEnabled)
            .def("enableSelfCollisionDetection", &BodyItem::enableSelfCollisionDetection)
            .def("isSelfCollisionDetectionEnabled", &BodyItem::isSelfCollisionDetectionEnabled)
            .def("clearCollisions", &BodyItem::clearCollisions)
            .def("centerOfMass", &BodyItem::centerOfMass, return_value_policy<copy_const_reference>())
            .def("doLegIkToMoveCm", &BodyItem::doLegIkToMoveCm)
            .def("zmp", &BodyItem::zmp, return_value_policy<copy_const_reference>())
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

    class_< WorldItem, WorldItemPtr, bases<Item, SceneProvider> >("WorldItem")
        .def("selectCollisionDetector", &WorldItem::selectCollisionDetector)
        .def("enableCollisionDetection", &WorldItem::enableCollisionDetection)
        .def("isCollisionDetectionEnabled", &WorldItem::isCollisionDetectionEnabled)
        .def("updateCollisionDetectorLater", &WorldItem::updateCollisionDetectorLater)
        .def("updateCollisionDetector", &WorldItem::updateCollisionDetector)
        .def("updateCollisions", &WorldItem::updateCollisions)
        .def("sigCollisionsUpdated", &WorldItem::sigCollisionsUpdated)
        ;

    implicitly_convertible<WorldItemPtr, ItemPtr>();
    implicitly_convertible<WorldItemPtr, SceneProvider*>();
    PyItemList<WorldItem>("WorldItemList");
    
    class_< BodyMotionItem, BodyMotionItemPtr, bases<AbstractMultiSeqItem> >("BodyMotionItem")
        .def("motion", BodyMotionItem_motion, return_value_policy<reference_existing_object>())
        .def("jointPosSeqItem", BodyMotionItem_jointPosSeqItem)
        .def("linkPosSeqItem", BodyMotionItem_linkPosSeqItem)
        .def("numExtraSeqItems", &BodyMotionItem::numExtraSeqItems, return_value_policy<return_by_value>())
        .def("extraSeqKey", &BodyMotionItem::extraSeqKey, return_value_policy<copy_const_reference>())
        .def("extraSeqItem", BodyMotionItem_extraSeqItem)
        .def("updateExtraSeqItems", &BodyMotionItem::updateExtraSeqItems)
        ;

    implicitly_convertible<BodyMotionItemPtr, AbstractMultiSeqItemPtr>();
    PyItemList<BodyMotionItem>("BodyMotionItemList");
    
    class_<SimulationBody, SimulationBodyPtr, bases<Referenced>, boost::noncopyable>("SimulationBody", no_init)
        .def("bodyItem", SimulationBody_bodyItem)
        .def("body", SimulationBody_body);

    implicitly_convertible<SimulationBodyPtr, ReferencedPtr>();

    class_<SimulatorItem, SimulatorItemPtr, bases<Item>, boost::noncopyable>
        simulatorItemClass("SimulatorItem", no_init);

    simulatorItemClass
        .def("findActiveSimulatorItemFor", SimulatorItem_findActiveSimulatorItemFor).staticmethod("findActiveSimulatorItemFor")
        .def("worldTimeStep", &SimulatorItem::worldTimeStep)
        .def("startSimulation", &SimulatorItem::startSimulation, SimulatorItem_startSimulation_overloads())
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
        .def("setExternalForce", &SimulatorItem::setExternalForce, SimulatorItem_setExternalForce_overloads())
        .def("clearExternalForces", &SimulatorItem::clearExternalForces)
        ;
    {
        scope simulatorItemScope = simulatorItemClass;

        enum_<SimulatorItem::RecordingMode>("RecordingMode")
            .value("REC_FULL", SimulatorItem::REC_FULL) 
            .value("REC_TAIL", SimulatorItem::REC_TAIL)
            .value("REC_NONE", SimulatorItem::REC_NONE)
            .value("N_RECORDING_MODES", SimulatorItem::N_RECORDING_MODES);
        
        enum_<SimulatorItem::TimeRangeMode>("TimeRangeMode")
            .value("TR_UNLIMITED", SimulatorItem::TR_UNLIMITED)
            .value("TR_ACTIVE_CONTROL", SimulatorItem::TR_ACTIVE_CONTROL)
            .value("TR_SPECIFIC", SimulatorItem::TR_SPECIFIC)
            .value("TR_TIMEBAR", SimulatorItem::TR_TIMEBAR) 
            .value("N_TIME_RANGE_MODES", SimulatorItem::N_TIME_RANGE_MODES);
    }

    implicitly_convertible<SimulatorItemPtr, ItemPtr>();
    PyItemList<SimulatorItem>("SimulatorItemList", simulatorItemClass);

    {
        scope aistSimulatorItemScope = 
            class_< AISTSimulatorItem, AISTSimulatorItemPtr, bases<SimulatorItem> >("AISTSimulatorItem")
            .def("setDynamicsMode", &AISTSimulatorItem::setDynamicsMode)
            .def("setIntegrationMode", &AISTSimulatorItem::setIntegrationMode)
            .def("setGravity", &AISTSimulatorItem::setGravity)
            .def("setFriction", AISTSimulatorItem_setFriction1)
            .def("setFriction", AISTSimulatorItem_setFriction2)
            .def("setContactCullingDistance", &AISTSimulatorItem::setContactCullingDistance)
            .def("setContactCullingDepth", &AISTSimulatorItem::setContactCullingDepth)
            .def("setErrorCriterion", &AISTSimulatorItem::setErrorCriterion)
            .def("setMaxNumIterations", &AISTSimulatorItem::setMaxNumIterations)
            .def("setContactCorrectionDepth", &AISTSimulatorItem::setContactCorrectionDepth)
            .def("setContactCorrectionVelocityRatio", &AISTSimulatorItem::setContactCorrectionVelocityRatio)
            .def("setEpsilon", &AISTSimulatorItem::setEpsilon)
            .def("set2Dmode", &AISTSimulatorItem::set2Dmode)
            .def("setKinematicWalkingEnabled", &AISTSimulatorItem::setKinematicWalkingEnabled)
            ;

        enum_<AISTSimulatorItem::DynamicsMode>("DynamicsMode")
            .value("FORWARD_DYNAMICS", AISTSimulatorItem::FORWARD_DYNAMICS) 
            .value("HG_DYNAMICS", AISTSimulatorItem::HG_DYNAMICS) 
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
        .def("setThreadEnabled", &GLVisionSimulatorItem::setThreadEnabled)
        .def("setDedicatedSensorThreadsEnabled", &GLVisionSimulatorItem::setDedicatedSensorThreadsEnabled)
        .def("setBestEffortMode", &GLVisionSimulatorItem::setBestEffortMode)
        .def("setRangeSensorPrecisionRatio", &GLVisionSimulatorItem::setRangeSensorPrecisionRatio)
        .def("setAllSceneObjectsEnabled", &GLVisionSimulatorItem::setAllSceneObjectsEnabled)
        .def("setHeadLightEnabled", &GLVisionSimulatorItem::setHeadLightEnabled)
        .def("setAdditionalLightsEnabled", &GLVisionSimulatorItem::setAdditionalLightsEnabled)
        ;

    implicitly_convertible<GLVisionSimulatorItemPtr, SubSimulatorItemPtr>();
    PyItemList<GLVisionSimulatorItem>("GLVisionSimulatorItemList");

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
}
