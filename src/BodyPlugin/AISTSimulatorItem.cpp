/*!
  @file
  @author Shin'ichiro Nakaoka
*/
#include "AISTSimulatorItem.h"
#include "WorldItem.h"
#include "BodyItem.h"
#include "ControllerItem.h"
#include <cnoid/ItemManager>
#include <cnoid/Archive>
#include <cnoid/EigenArchive>
#include <cnoid/DyWorld>
#include <cnoid/DyBody>
#include <cnoid/ForwardDynamicsCBM>
#include <cnoid/ConstraintForceSolver>
#include <cnoid/LeggedBodyHelper>
#include <cnoid/FloatingNumberString>
#include <cnoid/EigenUtil>
#include <cnoid/MessageView>
#include <cnoid/IdPair>
#include <fmt/format.h>
#include <mutex>
#include <iomanip>
#include <fstream>
#include "gettext.h"

using namespace std;
using namespace cnoid;
using fmt::format;

// for Windows
#undef min
#undef max

namespace {

const bool TRACE_FUNCTIONS = false;
const bool ENABLE_DEBUG_OUTPUT = false;
const double DEFAULT_GRAVITY_ACCELERATION = 9.80665;

class AISTSimBody : public SimulationBody
{
public:
    AISTSimBody(DyBody* body) : SimulationBody(body) { }
};
    

class KinematicWalkBody : public AISTSimBody
{
public:
    KinematicWalkBody(DyBody* body, LeggedBodyHelper* legged)
        : AISTSimBody(body),
          legged(legged) {
        supportFootIndex = 0;
        for(int i=1; i < legged->numFeet(); ++i){
            if(legged->footLink(i)->p().z() < legged->footLink(supportFootIndex)->p().z()){
                supportFootIndex = i;
            }
        }
        traverse.find(legged->footLink(supportFootIndex), true, true);
    }
    LeggedBodyHelper* legged;
    int supportFootIndex;
    LinkTraverse traverse;
};

}


namespace cnoid {
  
class AISTSimulatorItemImpl
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

    AISTSimulatorItem* self;

    World<ConstraintForceSolver> world;

    vector<shared_ptr<ForwardDynamicsCBM>> highGainDynamicsList;
        
    Selection dynamicsMode;
    Selection integrationMode;
    Vector3 gravity;
    double staticFriction;
    double dynamicFriction;
    FloatingNumberString contactCullingDistance;
    FloatingNumberString contactCullingDepth;
    FloatingNumberString errorCriterion;
    int maxNumIterations;
    FloatingNumberString contactCorrectionDepth;
    FloatingNumberString contactCorrectionVelocityRatio;
    double epsilon;
    bool is2Dmode;
    bool isKinematicWalkingEnabled;
    bool isOldAccelSensorMode;

    typedef std::map<Body*, int> BodyIndexMap;
    BodyIndexMap bodyIndexMap;

    typedef std::map<Link*, Link*> LinkMap;
    LinkMap orgLinkToInternalLinkMap;

    stdx::optional<int> forcedBodyPositionFunctionId;
    std::mutex forcedBodyPositionMutex;
    DyBody* forcedPositionBody;
    Position forcedBodyPosition;

    AISTSimulatorItemImpl(AISTSimulatorItem* self);
    AISTSimulatorItemImpl(AISTSimulatorItem* self, const AISTSimulatorItemImpl& org);
    bool initializeSimulation(const std::vector<SimulationBody*>& simBodies);
    void addBody(AISTSimBody* simBody);
    void clearExternalForces();
    void stepKinematicsSimulation(const std::vector<SimulationBody*>& activeSimBodies);
    void setForcedPosition(BodyItem* bodyItem, const Position& T);
    void doSetForcedPosition();
    void doPutProperties(PutPropertyFunction& putProperty);
    void addExtraJoint(ExtraJoint& extrajoint);
    void clearExtraJoint();
    bool store(Archive& archive);
    bool restore(const Archive& archive);

    // for debug
    ofstream os;
};

}


void AISTSimulatorItem::initializeClass(ExtensionManager* ext)
{
    ext->itemManager().registerClass<AISTSimulatorItem>(N_("AISTSimulatorItem"));
    ext->itemManager().addCreationPanel<AISTSimulatorItem>();
}


AISTSimulatorItem::AISTSimulatorItem()
{
    impl = new AISTSimulatorItemImpl(this);
    setName("AISTSimulator");
}


AISTSimulatorItemImpl::AISTSimulatorItemImpl(AISTSimulatorItem* self)
    : self(self),
      dynamicsMode(AISTSimulatorItem::N_DYNAMICS_MODES, CNOID_GETTEXT_DOMAIN_NAME),
      integrationMode(AISTSimulatorItem::N_INTEGRATION_MODES, CNOID_GETTEXT_DOMAIN_NAME)
{
    dynamicsMode.setSymbol(AISTSimulatorItem::FORWARD_DYNAMICS,  N_("Forward dynamics"));
    dynamicsMode.setSymbol(AISTSimulatorItem::KINEMATICS,        N_("Kinematics"));

    integrationMode.setSymbol(AISTSimulatorItem::EULER_INTEGRATION,  N_("Euler"));
    integrationMode.setSymbol(AISTSimulatorItem::RUNGE_KUTTA_INTEGRATION,  N_("Runge Kutta"));
    integrationMode.select(AISTSimulatorItem::RUNGE_KUTTA_INTEGRATION);
    
    gravity << 0.0, 0.0, -DEFAULT_GRAVITY_ACCELERATION;

    ConstraintForceSolver& cfs = world.constraintForceSolver;
    staticFriction = cfs.staticFriction();
    dynamicFriction = cfs.slipFriction();
    contactCullingDistance = cfs.contactCullingDistance();
    contactCullingDepth = cfs.contactCullingDepth();
    epsilon = cfs.coefficientOfRestitution();
    
    errorCriterion = cfs.gaussSeidelErrorCriterion();
    maxNumIterations = cfs.gaussSeidelMaxNumIterations();
    contactCorrectionDepth = cfs.contactCorrectionDepth();
    contactCorrectionVelocityRatio = cfs.contactCorrectionVelocityRatio();

    isKinematicWalkingEnabled = false;
    is2Dmode = false;
    isOldAccelSensorMode = false;
}


AISTSimulatorItem::AISTSimulatorItem(const AISTSimulatorItem& org)
    : SimulatorItem(org),
      impl(new AISTSimulatorItemImpl(this, *org.impl))
{

}


AISTSimulatorItemImpl::AISTSimulatorItemImpl(AISTSimulatorItem* self, const AISTSimulatorItemImpl& org)
    : self(self),
      dynamicsMode(org.dynamicsMode),
      integrationMode(org.integrationMode)
{
    gravity = org.gravity;
    staticFriction = org.staticFriction;
    dynamicFriction = org.dynamicFriction;
    contactCullingDistance = org.contactCullingDistance;
    contactCullingDepth = org.contactCullingDepth;
    errorCriterion = org.errorCriterion;
    maxNumIterations = org.maxNumIterations;
    contactCorrectionDepth = org.contactCorrectionDepth;
    contactCorrectionVelocityRatio = org.contactCorrectionVelocityRatio;
    epsilon = org.epsilon;
    isKinematicWalkingEnabled = org.isKinematicWalkingEnabled;
    is2Dmode = org.is2Dmode;
    isOldAccelSensorMode = org.isOldAccelSensorMode;
}


AISTSimulatorItem::~AISTSimulatorItem()
{
    delete impl;
}


void AISTSimulatorItem::setDynamicsMode(int mode)
{
    impl->dynamicsMode.select(mode);
}


void AISTSimulatorItem::setIntegrationMode(int mode)
{
    impl->integrationMode.select(mode);
}


void AISTSimulatorItem::setGravity(const Vector3& gravity)
{
    impl->gravity = gravity;
}


const Vector3& AISTSimulatorItem::gravity() const
{
    return impl->gravity;
}


void AISTSimulatorItem::setFriction(double staticFriction, double dynamicFriction)
{
    impl->staticFriction = staticFriction;
    impl->dynamicFriction = dynamicFriction;
}


double AISTSimulatorItem::staticFriction() const
{
    return impl->staticFriction;
}


double AISTSimulatorItem::dynamicFriction() const
{
    return impl->dynamicFriction;
}


void AISTSimulatorItem::setFriction(Link* link1, Link* link2, double staticFriction, double dynamicFriction)
{
    MessageView::instance()->putln(
        MessageView::WARNING,
        _("AISTSimulatorItem::setFriction(Link* link1, Link* link2, double staticFriction, double dynamicFriction) "
          "is not supported in this version.\n"
          "Please use the material table instead of it."));
}


void AISTSimulatorItem::registerCollisionHandler(const std::string& name, CollisionHandler handler)
{
    impl->world.constraintForceSolver.registerCollisionHandler(name, handler);
}


bool AISTSimulatorItem::unregisterCollisionHandler(const std::string& name)
{
    return impl->world.constraintForceSolver.unregisterCollisionHandler(name);
}


void AISTSimulatorItem::setContactCullingDistance(double value)    
{
    impl->contactCullingDistance = value;
}


void AISTSimulatorItem::setContactCullingDepth(double value)    
{
    impl->contactCullingDepth = value;
}

    
void AISTSimulatorItem::setErrorCriterion(double value)    
{
    impl->errorCriterion = value;
}

    
void AISTSimulatorItem::setMaxNumIterations(int value)
{
    impl->maxNumIterations = value;   
}


void AISTSimulatorItem::setContactCorrectionDepth(double value)
{
    impl->contactCorrectionDepth = value;
}


void AISTSimulatorItem::setContactCorrectionVelocityRatio(double value)
{
    impl->contactCorrectionVelocityRatio = value;
}


void AISTSimulatorItem::setEpsilon(double epsilon)
{
    impl->epsilon = epsilon;
}


void AISTSimulatorItem::set2Dmode(bool on)
{
    impl->is2Dmode = on;
}


void AISTSimulatorItem::setKinematicWalkingEnabled(bool on)
{
    impl->isKinematicWalkingEnabled = on;
}


void AISTSimulatorItem::setConstraintForceOutputEnabled(bool on)
{
    impl->world.constraintForceSolver.enableConstraintForceOutput(on);
}


Item* AISTSimulatorItem::doDuplicate() const
{
    return new AISTSimulatorItem(*this);
}


bool AISTSimulatorItem::startSimulation(bool doReset)
{
    impl->orgLinkToInternalLinkMap.clear();
    return SimulatorItem::startSimulation(doReset);
}


SimulationBody* AISTSimulatorItem::createSimulationBody(Body* orgBody)
{
    SimulationBody* simBody = 0;
    DyBody* body = new DyBody(*orgBody);

    const int n = orgBody->numLinks();
    for(int i=0; i < n; ++i){
        impl->orgLinkToInternalLinkMap[orgBody->link(i)] = body->link(i);

        auto link = body->link(i);
        if(link->isFreeJoint() && !link->isRoot()){
            MessageView::instance()->putln(
                format(_("The joint {0} of {1} is a free joint. AISTSimulator does not allow for a free joint except for the root link."),
                       link->name(), body->name(), MessageView::WARNING));
            link->setJointType(Link::FIXED_JOINT);
        }
    }
    
    if(impl->dynamicsMode.is(KINEMATICS) && impl->isKinematicWalkingEnabled){
        LeggedBodyHelper* legged = getLeggedBodyHelper(body);
        if(legged->isValid()){
            simBody = new KinematicWalkBody(body, legged);
        }
    }
    if(!simBody){
        simBody = new AISTSimBody(body);
    }

    return simBody;
}


bool AISTSimulatorItem::initializeSimulation(const std::vector<SimulationBody*>& simBodies)
{
    return impl->initializeSimulation(simBodies);
}


bool AISTSimulatorItemImpl::initializeSimulation(const std::vector<SimulationBody*>& simBodies)
{
    if(ENABLE_DEBUG_OUTPUT){
        static int ntest = 0;
        os.open((string("test-log-") + std::to_string(ntest++) + ".log").c_str());
        os << setprecision(30);
    }

    if(integrationMode.is(AISTSimulatorItem::EULER_INTEGRATION)){
        world.setEulerMethod();
    } else if(integrationMode.is(AISTSimulatorItem::RUNGE_KUTTA_INTEGRATION)){
        world.setRungeKuttaMethod();
    }
    world.setGravityAcceleration(gravity);
    world.enableSensors(true);
    world.setOldAccelSensorCalcMode(isOldAccelSensorMode);
    world.setTimeStep(self->worldTimeStep());
    world.setCurrentTime(0.0);

    ConstraintForceSolver& cfs = world.constraintForceSolver;
    cfs.setMaterialTable(self->worldItem()->materialTable());
    cfs.setGaussSeidelErrorCriterion(errorCriterion.value());
    cfs.setGaussSeidelMaxNumIterations(maxNumIterations);
    cfs.setContactDepthCorrection(contactCorrectionDepth.value(), contactCorrectionVelocityRatio.value());
    
    self->addPreDynamicsFunction([&](){ clearExternalForces(); });

    world.clearBodies();
    bodyIndexMap.clear();
    highGainDynamicsList.clear();

    for(size_t i=0; i < simBodies.size(); ++i){
        addBody(static_cast<AISTSimBody*>(simBodies[i]));
    }

    if(!highGainDynamicsList.empty()){
        mvout() << format(_("{} uses the ForwardDynamicsCBM module to perform the high-gain control."),
                          self->name()) << endl;
    }

    cfs.setFriction(staticFriction, dynamicFriction);
    cfs.setContactCullingDistance(contactCullingDistance.value());
    cfs.setContactCullingDepth(contactCullingDepth.value());
    cfs.setCoefficientOfRestitution(epsilon);
    cfs.setCollisionDetector(self->getOrCreateCollisionDetector());

    if(is2Dmode){
        cfs.set2Dmode(true);
    }

    world.initialize();

    return true;
}


void AISTSimulatorItemImpl::addBody(AISTSimBody* simBody)
{
    DyBody* body = static_cast<DyBody*>(simBody->body());

    bool hasHighgainJoints = false;
    for(auto& link : body->links()){
        if(link->actuationMode() == Link::JOINT_DISPLACEMENT ||
           link->actuationMode() == Link::JOINT_VELOCITY ||
           link->actuationMode() == Link::LINK_POSITION){
            hasHighgainJoints = true;
        }
    }

    int bodyIndex;
    if(hasHighgainJoints){
        auto dynamics = make_shared_aligned<ForwardDynamicsCBM>(body);
        highGainDynamicsList.push_back(dynamics);
        bodyIndex = world.addBody(body, dynamics);
    } else {
        bodyIndex = world.addBody(body);
    }
    bodyIndexMap[body] = bodyIndex;

    world.constraintForceSolver.setSelfCollisionDetectionEnabled(
        bodyIndex, simBody->bodyItem()->isSelfCollisionDetectionEnabled());
}


void AISTSimulatorItemImpl::clearExternalForces()
{
    world.constraintForceSolver.clearExternalForces();
}


bool AISTSimulatorItem::stepSimulation(const std::vector<SimulationBody*>& activeSimBodies)
{
    switch(impl->dynamicsMode.which()){
    case FORWARD_DYNAMICS:
        for(auto&& dynamics : impl->highGainDynamicsList){
            dynamics->complementHighGainModeCommandValues();
        }
        impl->world.calcNextState();
        break;
    case KINEMATICS:
        impl->stepKinematicsSimulation(activeSimBodies);
        break;
    }
    return true;
}


void AISTSimulatorItemImpl::stepKinematicsSimulation(const std::vector<SimulationBody*>& activeSimBodies)
{
    for(size_t i=0; i < activeSimBodies.size(); ++i){
        SimulationBody* simBody = activeSimBodies[i];
        Body* body = simBody->body();

        for(auto& joint : body->allJoints()){
            joint->q() = joint->q_target();
            joint->dq() = joint->dq_target();
        }
        
        if(!isKinematicWalkingEnabled){
            body->calcForwardKinematics(true, true);
        } else {
            KinematicWalkBody* walkBody = dynamic_cast<KinematicWalkBody*>(simBody);
            if(!walkBody){
                body->calcForwardKinematics(true, true);
            } else {
                walkBody->traverse.calcForwardKinematics(true, true);
                
                LeggedBodyHelper* legged = walkBody->legged;
                const int supportFootIndex = walkBody->supportFootIndex;
                int nextSupportFootIndex = supportFootIndex;
                Link* supportFoot = legged->footLink(supportFootIndex);
                Link* nextSupportFoot = supportFoot;
                const int n = legged->numFeet();
                for(int i=0; i < n; ++i){
                    if(i != supportFootIndex){
                        Link* foot = legged->footLink(i);
                        if(foot->p().z() < nextSupportFoot->p().z()){
                            nextSupportFootIndex = i;
                            nextSupportFoot = foot;
                        }
                    }
                }
                if(nextSupportFoot != supportFoot){
                    nextSupportFoot->p().z() = supportFoot->p().z();
                    walkBody->supportFootIndex = nextSupportFootIndex;
                    supportFoot = nextSupportFoot;
                    walkBody->traverse.find(supportFoot, true, true);
                    walkBody->traverse.calcForwardKinematics(true, true);
                }
            }
        }
    }
}


void AISTSimulatorItem::finalizeSimulation()
{
    if(ENABLE_DEBUG_OUTPUT){
        impl->os.close();
    }
}


std::shared_ptr<CollisionLinkPairList> AISTSimulatorItem::getCollisions()
{
    return impl->world.constraintForceSolver.getCollisions();
}


Vector3 AISTSimulatorItem::getGravity() const
{
    return impl->gravity;
}


void AISTSimulatorItem::setForcedPosition(BodyItem* bodyItem, const Position& T)
{
    impl->setForcedPosition(bodyItem, T);
}


void AISTSimulatorItemImpl::setForcedPosition(BodyItem* bodyItem, const Position& T)
{
    if(SimulationBody* simBody = self->findSimulationBody(bodyItem)){
        {
            std::lock_guard<std::mutex> lock(forcedBodyPositionMutex);
            forcedPositionBody = static_cast<DyBody*>(simBody->body());
            forcedBodyPosition = T;
        }
        if(!forcedBodyPositionFunctionId){
            forcedBodyPositionFunctionId =
                self->addPostDynamicsFunction([&](){ doSetForcedPosition(); });
        }
    }
}


bool AISTSimulatorItem::isForcedPositionActiveFor(BodyItem* bodyItem) const
{
    bool isActive = false;
    if(impl->forcedBodyPositionFunctionId){
        SimulationBody* simBody = const_cast<AISTSimulatorItem*>(this)->findSimulationBody(bodyItem);
        {
            std::lock_guard<std::mutex> lock(impl->forcedBodyPositionMutex);
            if(impl->forcedPositionBody == static_cast<DyBody*>(simBody->body())){
                isActive = true;
            }
        }
    }
    return isActive;
}


void AISTSimulatorItem::clearForcedPositions()
{
    if(impl->forcedBodyPositionFunctionId){
        removePostDynamicsFunction(*impl->forcedBodyPositionFunctionId);
        impl->forcedBodyPositionFunctionId = stdx::nullopt;
    }
}
    

void AISTSimulatorItemImpl::doSetForcedPosition()
{
    std::lock_guard<std::mutex> lock(forcedBodyPositionMutex);
    DyLink* rootLink = forcedPositionBody->rootLink();
    rootLink->setPosition(forcedBodyPosition);
    rootLink->v().setZero();
    rootLink->w().setZero();
    rootLink->vo().setZero();
    forcedPositionBody->calcSpatialForwardKinematics();
}


void AISTSimulatorItem::doPutProperties(PutPropertyFunction& putProperty)
{
    SimulatorItem::doPutProperties(putProperty);
    impl->doPutProperties(putProperty);
}


void AISTSimulatorItem::clearExtraJoint()
{
    impl->clearExtraJoint();
}


void AISTSimulatorItem::addExtraJoint(ExtraJoint& extrajoint)
{
    impl->addExtraJoint(extrajoint);
}


void AISTSimulatorItemImpl::clearExtraJoint()
{
    world.extrajoints.clear();
}


void AISTSimulatorItemImpl::addExtraJoint(ExtraJoint& extrajoint)
{
    world.extrajoints.push_back(extrajoint);
}


void AISTSimulatorItemImpl::doPutProperties(PutPropertyFunction& putProperty)
{
    putProperty(_("Dynamics mode"), dynamicsMode,
                [&](int index){ return dynamicsMode.selectIndex(index); });
    putProperty(_("Integration mode"), integrationMode,
                [&](int index){ return integrationMode.selectIndex(index); });
    putProperty(_("Gravity"), str(gravity), [&](const string& v){ return toVector3(v, gravity); });
    putProperty.decimals(3).min(0.0);
    putProperty(_("Static friction"), staticFriction, changeProperty(staticFriction));
    putProperty(_("Slip friction"), dynamicFriction, changeProperty(dynamicFriction));
    putProperty(_("Contact culling distance"), contactCullingDistance,
                [&](const string& v){ return contactCullingDistance.setNonNegativeValue(v); });
    putProperty(_("Contact culling depth"), contactCullingDepth,
                [&](const string& v){ return contactCullingDepth.setNonNegativeValue(v); });
    putProperty(_("Error criterion"), errorCriterion,
                [&](const string& v){ return errorCriterion.setPositiveValue(v); });
    putProperty.min(1.0)(_("Max iterations"), maxNumIterations, changeProperty(maxNumIterations));
    putProperty(_("CC depth"), contactCorrectionDepth,
                [&](const string& v){ return contactCorrectionDepth.setNonNegativeValue(v); });
    putProperty(_("CC v-ratio"), contactCorrectionVelocityRatio,
                [&](const string& v){ return contactCorrectionVelocityRatio.setNonNegativeValue(v); });
    putProperty(_("Kinematic walking"), isKinematicWalkingEnabled,
                changeProperty(isKinematicWalkingEnabled));
    putProperty(_("2D mode"), is2Dmode, changeProperty(is2Dmode));
    putProperty(_("Old accel sensor mode"), isOldAccelSensorMode, changeProperty(isOldAccelSensorMode));
}


bool AISTSimulatorItem::store(Archive& archive)
{
    SimulatorItem::store(archive);
    return impl->store(archive);
}


bool AISTSimulatorItemImpl::store(Archive& archive)
{
    archive.write("dynamicsMode", dynamicsMode.selectedSymbol(), DOUBLE_QUOTED);
    archive.write("integrationMode", integrationMode.selectedSymbol(), DOUBLE_QUOTED);
    write(archive, "gravity", gravity);
    archive.write("staticFriction", staticFriction);
    archive.write("dynamicFriction", dynamicFriction);
    archive.write("cullingThresh", contactCullingDistance);
    archive.write("contactCullingDepth", contactCullingDepth);
    archive.write("errorCriterion", errorCriterion);
    archive.write("maxNumIterations", maxNumIterations);
    archive.write("contactCorrectionDepth", contactCorrectionDepth);
    archive.write("contactCorrectionVelocityRatio", contactCorrectionVelocityRatio);
    archive.write("kinematicWalking", isKinematicWalkingEnabled);
    archive.write("2Dmode", is2Dmode);
    archive.write("oldAccelSensorMode", isOldAccelSensorMode);
    return true;
}


bool AISTSimulatorItem::restore(const Archive& archive)
{
    SimulatorItem::restore(archive);
    return impl->restore(archive);
}


bool AISTSimulatorItemImpl::restore(const Archive& archive)
{
    string symbol;
    if(archive.read("dynamicsMode", symbol)){
        dynamicsMode.select(symbol);
    }
    if(archive.read("integrationMode", symbol)){
        integrationMode.select(symbol);
    }
    read(archive, "gravity", gravity);
    archive.read("staticFriction", staticFriction);
    if(!archive.read("dynamicFriction", dynamicFriction)){
        archive.read("slipFriction", dynamicFriction);
    }
    contactCullingDistance = archive.get("cullingThresh", contactCullingDistance.string());
    contactCullingDepth = archive.get("contactCullingDepth", contactCullingDepth.string());
    errorCriterion = archive.get("errorCriterion", errorCriterion.string());
    archive.read("maxNumIterations", maxNumIterations);
    contactCorrectionDepth = archive.get("contactCorrectionDepth", contactCorrectionDepth.string());
    contactCorrectionVelocityRatio = archive.get("contactCorrectionVelocityRatio", contactCorrectionVelocityRatio.string());
    archive.read("kinematicWalking", isKinematicWalkingEnabled);
    archive.read("2Dmode", is2Dmode);
    archive.read("oldAccelSensorMode", isOldAccelSensorMode);
    return true;
}

#ifdef ENABLE_SIMULATION_PROFILING
void AISTSimulatorItem::getProfilingNames(vector<string>& profilingNames)
{
    profilingNames.push_back("Collision detection time");
    profilingNames.push_back("Constraint force calculation time");
    profilingNames.push_back("Forward dynamics calculation time");
    profilingNames.push_back("Customizer calculation time");
}


void AISTSimulatorItem::getProfilingTimes(vector<double>& profilingToimes)
{
    double collisionTime = impl->world.constraintForceSolver.getCollisionTime();
    profilingToimes.push_back(collisionTime);
    profilingToimes.push_back(impl->world.forceSolveTime - collisionTime);
    profilingToimes.push_back(impl->world.forwardDynamicsTime);
    profilingToimes.push_back(impl->world.customizerTime);
}
#endif
