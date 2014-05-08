/*!
 * @author Hisashi Ikari
 */
#include <boost/python.hpp>
#include <cnoid/Body>
#include <cnoid/Item>
#include <cnoid/BodyItem>
#include <cnoid/BodyState>
#include <cnoid/BodyMotionItem>
#include <cnoid/RootItem>
#include <cnoid/WorldItem>
#include <cnoid/SimulatorItem>
#include <cnoid/FileUtil>
#include <cnoid/Link>
#include <cnoid/ItemTreeView>
#include <cnoid/SimulationBar>
#include <cnoid/ItemManager>
#include <cnoid/EigenTypes>

#include <cnoid/AbstractSeq>
#include <cnoid/MultiSeq>
#include <cnoid/MultiValueSeq>
#include <cnoid/MultiSE3Seq>
#include <cnoid/MultiAffine3Seq>
#include <cnoid/Array2D>

#include "../../Base/python/PyBase.h"
#include "../AISTSimulatorItem.h"

// We recommend the use of minieigen.
using namespace boost::python;
using namespace boost::filesystem;
using namespace cnoid;

namespace cnoid
{
/*!
 * @brief Reference types are explicitly declare a function pointer. 
 *        Reference types share a reference to the original(in C++).
 *        But, it does not share the type of destination(in python).
 *        (Reason-1. Primitive types can not be a reference type.)
 *        (Reason-2. Minieigen create an object always.)
 *        Therefore, we must use the setter if you want to use the python.
 *        We will define the settings of the following variables.       
 */
BOOST_PYTHON_MODULE(BodyPlugin)
{
    /*!
     * @brief Definition for getting of variable BodyItem.
     * @note  Item must be created "cnoid.Base FIRST".
     *        Warning appears when you run the same statement in the other modules.
     *        And dependency was created in the module. It must be loaded at the beginning of the "cnoid.Base" always.
     *        (e.g.) on the python-script.
     *        import cnoid.Base as base # MUST BE FIRST
     *        import cnoid.Body as body
     *        import cnoid.BodyPlugin as bp
     */
    void (BodyItem::*notifyKinematicStateChange)(bool, bool, bool) = &BodyItem::notifyKinematicStateChange; // ...(*1)
    
    // (*1)... The function of override must show signature of function pointer explicitly.
    //         And BOOST_PYTHON_FUNCTION_OVERLOADS is available only if the first argument matches.
    //         Otherwise, we must define the wrapped manually.
    
    
    /*!
     * @brief Define the interface for WorldItem.
     */
    class_ < BodyItem, boost::shared_ptr<BodyItem>, bases<Item>, boost::noncopyable>("BodyItem", init<>())
        .def("__init__", boost::python::make_constructor(&createInstance<BodyItem>))
        .def("loadModelFile", &BodyItem::loadModelFile, return_value_policy<return_by_value>())
        .def("setName", &BodyItem::setName)
        .def("body", &BodyItem::body, return_internal_reference<>())
        .def("moveToOrigin", &BodyItem::moveToOrigin)
        .def("setPresetPose", &BodyItem::setPresetPose)
        .def("currentBaseLink", &BodyItem::currentBaseLink, return_internal_reference<>())
        .def("setCurrentBaseLink", &BodyItem::setCurrentBaseLink)
        .def("calcForwardKinematics", &BodyItem::calcForwardKinematics)
        .def("copyKinematicState", &BodyItem::copyKinematicState)
        .def("pasteKinematicState", &BodyItem::pasteKinematicState)
        .def("storeKinematicState", &BodyItem::storeKinematicState)
        .def("restoreKinematicState", &BodyItem::restoreKinematicState)
        .def("storeInitialState", &BodyItem::storeInitialState)
        .def("restoreInitialState", &BodyItem::restoreInitialState)
        .def("notifyKinematicStateChange", notifyKinematicStateChange)
        .def("getInitialState", &BodyItem::getInitialState)
        .def("beginKinematicStateEdit", &BodyItem::beginKinematicStateEdit)
        .def("acceptKinematicStateEdit", &BodyItem::acceptKinematicStateEdit)
        .def("undoKinematicState", &BodyItem::undoKinematicState)
        .def("enableSelfCollisionDetection", &BodyItem::enableSelfCollisionDetection)
        .def("isSelfCollisionDetectionEnabled", &BodyItem::isSelfCollisionDetectionEnabled, return_value_policy<return_by_value>())
        .def("clearCollisions", &BodyItem::clearCollisions)
        .def("centerOfMass", &BodyItem::centerOfMass, return_value_policy<return_by_value>())
        .def("doLegIkToMoveCm", &BodyItem::doLegIkToMoveCm)
        .def("zmp", &BodyItem::zmp, return_value_policy<return_by_value>())
        .def("setZmp", &BodyItem::setZmp)
        .def("setStance", &BodyItem::setStance);
    
    
    /*!
     * @brief Define the interface for WorldItem.
     */
    class_ < WorldItem, boost::shared_ptr<WorldItem>, bases<Item>, boost::noncopyable >("WorldItem", init<>())
        .def("__init__", boost::python::make_constructor(&createInstance<WorldItem>))
        .def("selectCollisionDetector", &WorldItem::selectCollisionDetector, return_value_policy<return_by_value>())
        .def("enableCollisionDetection", &WorldItem::enableCollisionDetection)
        .def("isCollisionDetectionEnabled", &WorldItem::isCollisionDetectionEnabled, return_value_policy<return_by_value>())
        .def("updateCollisionDetector", &WorldItem::updateCollisionDetector)
        .def("updateCollisions", &WorldItem::updateCollisions);


    /*!
     * @brief Define the interface for BodyMotionItem.
     */
    const MultiValueSeqPtr& (BodyMotionItem::*jointPosSeq)() = &BodyMotionItem::jointPosSeq;
    const MultiSE3SeqPtr& (BodyMotionItem::*linkPosSeq)() = & BodyMotionItem::linkPosSeq;

    class_ < BodyMotionItem, boost::shared_ptr<BodyMotionItem>, bases<Item>, boost::noncopyable >("BodyMotionItem", init<>())
        .def("__init__", boost::python::make_constructor(&createInstance<BodyMotionItem>))
        .def("numExtraSeqItems", &BodyMotionItem::numExtraSeqItems, return_value_policy<return_by_value>())
        .def("extraSeqKey", &BodyMotionItem::extraSeqKey, return_value_policy<return_by_value>())
        .def("updateExtraSeqItems", &BodyMotionItem::updateExtraSeqItems); 


    /*!
     * @brief Definition for getting of variable running state and currentFrame.
     *        Python script knows the current time by the SimuratorItem.
     *        This is a case that python is running in the "BACKGROUND".
     *        This process is the process of "PULL" type of python.
     *        Addition, boost-python to use only "CONCRETE METHOD".
     */
    class_ < SimulatorItem, boost::shared_ptr<SimulatorItem>, bases<Item>, boost::noncopyable >("SimulatorItem", no_init)
        .def("isRunning", &SimulatorItem::isRunning, return_value_policy<return_by_value>())
        .def("currentFrame", &SimulatorItem::currentFrame, return_value_policy<return_by_value>())
        .def("worldTimeStep", &SimulatorItem::worldTimeStep, return_value_policy<return_by_value>())
        .def("setRealtimeSyncMode", &SimulatorItem::setRealtimeSyncMode)
        .def("setTimeRangeMode", &SimulatorItem::setTimeRangeMode)
        .def("setAllLinkPositionOutputMode", &SimulatorItem::setAllLinkPositionOutputMode)
        .def("setRecordingEnabled", &SimulatorItem::setRecordingEnabled)
        .def("setDeviceStateOutputEnabled", &SimulatorItem::setDeviceStateOutputEnabled)
        .def("setActiveControlPeriodOnlyMode", &SimulatorItem::setActiveControlPeriodOnlyMode)
        .def("selectMotionItems", &SimulatorItem::selectMotionItems);


    /*!
     * @brief Define the interface for AISTSimulationItem.
     */ 
    class_ < AISTSimulatorItem, boost::shared_ptr<AISTSimulatorItem>, bases<SimulatorItem>, boost::noncopyable >
        ("AISTSimulatorItem", init<>())
        .def("__init__", boost::python::make_constructor(&createInstance<AISTSimulatorItem>))
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
        .def("setKinematicWalkingEnabled", &AISTSimulatorItem::setKinematicWalkingEnabled);


    /*!
     * @brief Define the interface for SimulationBar.
     */
    class_ < SimulationBar, boost::noncopyable >("SimulationBar", no_init)
        .def("instance", &SimulationBar::instance, return_value_policy<reference_existing_object>()).staticmethod("instance")
        .def("startSimulation",
             static_cast<void(SimulationBar::*)(SimulatorItem*, bool)>(&SimulationBar::startSimulation),
             (args("simulator"), args("doRest") = false))
        .def("stopSimulation", &SimulationBar::stopSimulation);


    /*!
     * @brief Provides following enum value of the BodyItem.
     */
    enum_ <BodyItem::PresetPoseID>("PresetPoseID")
        .value("INITIAL_POSE", BodyItem::INITIAL_POSE) 
        .value("STANDARD_POSE", BodyItem::STANDARD_POSE);


    /*!
     * @brief Provides following enum value of the SimulatorItem.
     */
    enum_ <SimulatorItem::TimeRangeMode>("TimeRangeMode")
        .value("TIMEBAR_RANGE", SimulatorItem::TIMEBAR_RANGE) 
        .value("SPECIFIED_PERIOD", SimulatorItem::SPECIFIED_PERIOD)
        .value("UNLIMITED", SimulatorItem::UNLIMITED)
        .value("N_TIME_RANGE_MODES", SimulatorItem::N_TIME_RANGE_MODES);   


    /*!
     * @brief Provides following enum value of the BodyItem.
     */
    enum_ <BodyItem::PositionType>("PositionType")
        .value("CM_PROJECTION", BodyItem::CM_PROJECTION)
        .value("HOME_COP", BodyItem::HOME_COP)
        .value("RIGHT_HOME_COP", BodyItem::RIGHT_HOME_COP)
        .value("LEFT_HOME_COP", BodyItem::LEFT_HOME_COP)
        .value("ZERO_MOMENT_POINT", BodyItem::ZERO_MOMENT_POINT);

}

}; // end of namespace
