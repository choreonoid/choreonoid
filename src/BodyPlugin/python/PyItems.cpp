/*!
  @author Shin'ichiro Nakaoka
*/

#include "../BodyItem.h"
#include "../BodyMotionItem.h"
#include "../WorldItem.h"
#include "../SimulatorItem.h"
#include "../AISTSimulatorItem.h"
#include <cnoid/BodyState>
#include <cnoid/PyUtil>

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

BodyItemPtr SimulationBody_bodyItem(SimulationBody& self) { return self.bodyItem(); }
BodyPtr SimulationBody_body(SimulationBody& self) { return self.body(); }

BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(SimulatorItem_startSimulation_overloads, startSimulation, 0, 1)

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
    
    class_< BodyMotionItem, BodyMotionItemPtr, bases<AbstractMultiSeqItem> >("BodyMotionItem")
        .def("motion", &BodyMotionItem::motion, return_value_policy<copy_const_reference>())
        .def("jointPosSeqItem", BodyMotionItem_jointPosSeqItem)
        .def("jointPosSeq", &BodyMotionItem::jointPosSeq, return_value_policy<copy_const_reference>())
        .def("linkPosSeqItem", BodyMotionItem_linkPosSeqItem)
        .def("linkPosSeq", &BodyMotionItem::linkPosSeq, return_value_policy<copy_const_reference>())
        .def("numExtraSeqItems", &BodyMotionItem::numExtraSeqItems, return_value_policy<return_by_value>())
        .def("extraSeqKey", &BodyMotionItem::extraSeqKey, return_value_policy<copy_const_reference>())
        .def("extraSeqItem", BodyMotionItem_extraSeqItem)
        .def("updateExtraSeqItems", &BodyMotionItem::updateExtraSeqItems)
        ;

    implicitly_convertible<BodyMotionItemPtr, AbstractMultiSeqItemPtr>();
    
    class_<SimulationBody, SimulationBodyPtr, bases<Referenced>, boost::noncopyable>("SimulationBody", no_init)
        .def("bodyItem", SimulationBody_bodyItem)
        .def("body", SimulationBody_body);

    implicitly_convertible<SimulationBodyPtr, ReferencedPtr>();

    {
        scope simulatorItemScope = 
            class_<SimulatorItem, SimulatorItemPtr, bases<Item>, boost::noncopyable>("SimulatorItem", no_init)
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
            .def("setActiveControlPeriodOnlyMode", &SimulatorItem::setActiveControlPeriodOnlyMode)
            .def("isRecordingEnabled", &SimulatorItem::isRecordingEnabled)
            .def("isDeviceStateOutputEnabled", &SimulatorItem::isDeviceStateOutputEnabled)
            .def("isAllLinkPositionOutputMode", &SimulatorItem::isAllLinkPositionOutputMode)
            .def("setAllLinkPositionOutputMode", &SimulatorItem::setAllLinkPositionOutputMode)
            .def("selectMotionItems", &SimulatorItem::selectMotionItems)
            ;

            enum_<SimulatorItem::RecordingMode>("RecordingMode")
                .value("RECORD_FULL", SimulatorItem::RECORD_FULL) 
                .value("RECORD_TAIL", SimulatorItem::RECORD_TAIL)
                .value("RECORD_NONE", SimulatorItem::RECORD_NONE)
                .value("N_RECORDING_MODES", SimulatorItem::N_RECORDING_MODES);

            enum_<SimulatorItem::TimeRangeMode>("TimeRangeMode")
                .value("TIMEBAR_RANGE", SimulatorItem::TIMEBAR_RANGE) 
                .value("SPECIFIED_PERIOD", SimulatorItem::SPECIFIED_PERIOD)
                .value("UNLIMITED", SimulatorItem::UNLIMITED)
                .value("N_TIME_RANGE_MODES", SimulatorItem::N_TIME_RANGE_MODES);
    }

    implicitly_convertible<SimulatorItemPtr, ItemPtr>();

    {
        scope aistSimulatorItemScope = 
            class_< AISTSimulatorItem, AISTSimulatorItemPtr, bases<SimulatorItem> >("AISTSimulatorItem")
            .def("setDynamicsMode", &AISTSimulatorItem::setDynamicsMode)
            .def("setIntegrationMode", &AISTSimulatorItem::setIntegrationMode)
            .def("setGravity", &AISTSimulatorItem::setGravity)
            .def("setStaticFriction", &AISTSimulatorItem::setStaticFriction)
            .def("setSlipFriction", &AISTSimulatorItem::setSlipFriction)
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
}
