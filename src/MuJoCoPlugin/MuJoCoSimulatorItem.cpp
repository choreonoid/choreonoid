#include "MuJoCoSimulatorItem.h"
#include <cnoid/ItemManager>
#include <cnoid/WorldItem>
#include <cnoid/BodyItem>
#include <cnoid/PutPropertyFunction>
#include <cnoid/Body>
#include <cnoid/Link>
#include <cnoid/Material>
#include <cnoid/MaterialTable>
#include <cnoid/BasicSensorSimulationHelper>
#include <cnoid/ForceSensor>
#include <cnoid/SceneDrawables>
#include <cnoid/MeshExtractor>
#include <cnoid/Selection>
#include <cnoid/EigenUtil>
#include <cnoid/EigenArchive>
#include <cnoid/CloneMap>
#include <cnoid/Archive>
#include <cnoid/Format>
#include <cnoid/MessageOut>
#include <mujoco/mujoco.h>
#include <vector>
#include <string>
#include <map>
#include <unordered_map>
#include <unordered_set>
#include <array>
#include <cmath>
#include <algorithm>
#include <utility>
#include "gettext.h"

using namespace std;
using namespace cnoid;

namespace {

constexpr double DefaultGravityAcceleration = 9.80665;

// Sequential indices for the integrator selection (RK4 is not offered).
enum IntegratorIndex { IntegratorEuler, IntegratorImplicit, IntegratorImplicitFast };

// Map the selection index to the MuJoCo integrator value.
int mujocoIntegrator(int index)
{
    switch(index){
    case IntegratorEuler:        return mjINT_EULER;
    case IntegratorImplicit:     return mjINT_IMPLICIT;
    case IntegratorImplicitFast: return mjINT_IMPLICITFAST;
    default:                     return mjINT_IMPLICITFAST;
    }
}

void setQuat(double* q, const Matrix3& R)
{
    Quaternion quat(R);
    quat.normalize();
    q[0] = quat.w();
    q[1] = quat.x();
    q[2] = quat.y();
    q[3] = quat.z();
}

Matrix3 toMatrix3(const double* quat) // MuJoCo quat order: w, x, y, z
{
    Quaternion q(quat[0], quat[1], quat[2], quat[3]);
    return q.toRotationMatrix();
}

string meshLabel(SgShape* shape)
{
    if(shape){
        auto& name = shape->name();
        if(!name.empty()){
            return formatR(_("Mesh \"{0}\""), name);
        }
    }
    return _("A mesh");
}

mjfCollision originalBoxBoxCollisionFunc = nullptr;

int limitedBoxBoxCollision(
    const mjModel* model, mjData* data, mjPreContact* contacts,
    int geom1, int geom2, mjtNum margin)
{
    if(!originalBoxBoxCollisionFunc){
        return 0;
    }

    // Work around a MuJoCo 3.9 narrowphase issue where the built-in box-box
    // collider can return one extra contact for degenerate contacts. The engine
    // treats a value above mj_maxContact as fatal. Once MuJoCo fixes this or is
    // updated to a fixed version, this wrapper should no longer be needed.
    int numContacts = originalBoxBoxCollisionFunc(model, data, contacts, geom1, geom2, margin);
    int maxContacts = mj_maxContact(model, geom1, geom2, margin > 0.0 ? 1 : 0);
    return maxContacts >= 0 ? std::min(numContacts, maxContacts) : numContacts;
}

void installBoxBoxCollisionLimit()
{
    mjfCollision& collisionFunc = mjCOLLISIONFUNC[mjGEOM_BOX][mjGEOM_BOX];
    if(collisionFunc != limitedBoxBoxCollision){
        originalBoxBoxCollisionFunc = collisionFunc;
        collisionFunc = limitedBoxBoxCollision;
    }
}

}

namespace cnoid {

// Force sensor bridge: maps a Choreonoid ForceSensor to the MuJoCo body whose
// incoming joint (constraint) wrench is read via cfrc_int.
struct MuJoCoForceSensorInfo
{
    ForceSensorPtr sensor;
    Link* link;
    int bodyId; // MuJoCo body id of the link (resolved after compile)

    MuJoCoForceSensorInfo() : link(nullptr), bodyId(-1) { }
};


// Resolved joint control law, derived from the link's actuation mode bitmask.
enum ControlType { NoControl, MotorControl, PositionControl, VelocityControl };

// Choose the control law from the actuation mode bits. The mode is a bitmask
// (e.g. JointDisplacement | JointVelocity means position control with a velocity
// feedforward), so it must be tested by bit, not by exact equality. Position
// takes precedence over velocity over effort.
inline ControlType controlTypeFromActuationMode(short mode)
{
    if(mode & Link::JointDisplacement){
        return PositionControl;
    } else if(mode & Link::JointVelocity){
        return VelocityControl;
    } else if(mode & Link::JointEffort){
        return MotorControl;
    }
    return NoControl;
}


// Per-link bridge information between a Choreonoid Link and the MuJoCo model.
struct MuJoCoLinkInfo
{
    Link* link;
    mjsBody* mjBody;   // valid during model building
    mjsJoint* mjJoint; // valid during model building (nullptr if welded)
    string bodyName;
    int bodyId;        // MuJoCo body id (after compile)
    int jointType;     // mjtJoint, or -1 if welded
    int qposadr;       // start address in qpos (after compile)
    int dofadr;        // start address in qvel (after compile)
    int actuatorId;    // MuJoCo actuator id (after compile), or -1 if none
    int controlType;   // ControlType resolved from the actuation mode at build time
    double velocityGain; // kv of this joint's actuator (used by the feedforward)
    bool velocityFeedforward; // position control also has the JointVelocity bit
    bool hasActualJoint; // hinge or slide
    bool isFreeJoint;
    bool useSurfaceVelocity; // pseudo continuous track joint

    MuJoCoLinkInfo()
        : link(nullptr), mjBody(nullptr), mjJoint(nullptr),
          bodyId(-1), jointType(-1), qposadr(-1), dofadr(-1),
          actuatorId(-1), controlType(NoControl), velocityGain(0.0),
          velocityFeedforward(false),
          hasActualJoint(false), isFreeJoint(false), useSurfaceVelocity(false)
    { }
};


class MuJoCoBody : public SimulationBody
{
public:
    MuJoCoSimulatorItem::Impl* simImpl;
    int bodyIndex;
    string namePrefix;
    vector<MuJoCoLinkInfo> linkInfos; // indexed by link->index()
    BasicSensorSimulationHelper sensorHelper;
    vector<MuJoCoForceSensorInfo> forceSensorInfos;
    bool hasForceSensors;
    bool isStatic;
    mjsBody* worldBody;

    MuJoCoBody(Body* body, MuJoCoSimulatorItem::Impl* simImpl);
    void createModelElements(mjsBody* worldBody);
    void buildLink(Link* link, mjsBody* parentMjBody);
    void resolveIndices();
    void addSelfCollisionExcludes();
    void setInitialState();
    void inputControl();
    void addExternalForces();
    void getState();
    void updateForceSensors();
};


class MuJoCoSimulatorItem::Impl
{
public:
    MuJoCoSimulatorItem* self;

    mjSpec* spec;
    mjModel* model;
    mjData* data;
    mjThreadPool* threadPool;

    Vector3 gravity;
    double timeStep;
    MaterialTable* materialTable;
    MeshExtractor meshExtractor;
    int meshCounter;
    int extraJointElementCounter;
    std::unordered_map<Body*, MuJoCoBody*> mujocoBodyMap;

    // Mesh asset deduplication. The same SgMesh instance is often shared by
    // multiple geoms (e.g. identical wheels); register one MuJoCo mesh asset
    // per (mesh, scale) combination and let the geoms reference it. The scale
    // is part of the key because it is baked into the asset vertices.
    std::map<std::pair<SgMesh*, std::array<double, 3>>, std::string> meshAssetMap;

    // Pseudo continuous track (surface velocity) support
    bool hasSurfaceVelocityLinks;
    std::vector<Link*> bodySurfaceVelocityLink; // MuJoCo body id -> surface velocity link (nullptr if none)

    // Contact material support. Contact friction and restitution are resolved per
    // material-id pair from the MaterialTable and written into each mjContact
    // before the constraints are built (lazy per-contact resolution, like the AIST
    // ConstraintForceSolver). This avoids the geom-pair explosion of mjsPair.
    bool hasContactMaterials;
    std::vector<int> geomMaterialId;  // MuJoCo geom id -> Choreonoid material id (-1 if none)
    struct ResolvedContactParam {
        double friction;     // slide friction coefficient
        double solref[2];    // normal-direction solref in direct (negative) form when bouncy
        bool bouncy;         // restitution > 0
        bool resolved;
        ResolvedContactParam() : friction(1.0), solref{0.0, 0.0}, bouncy(false), resolved(false) { }
    };
    std::vector<ResolvedContactParam> contactParamCache; // indexed by id1*numMaterials + id2
    int numMaterialIds;

    Selection integratorType;
    Selection solverType;
    int numIterations;
    int numThreads;
    double positionGain;
    double velocityGain;
    double minFriction; // lower bound for contact friction (see overrideContactParams)
    double maxFriction; // upper bound for contact friction (see overrideContactParams)
    double impratio;    // MuJoCo friction-to-normal contact impedance ratio
    bool isVelocityOutputEnabled;
    bool isAccelerationOutputEnabled;
    bool isSelfCollisionEnabledByDefault;

    MessageOut* mout;

    Impl(MuJoCoSimulatorItem* self);
    Impl(MuJoCoSimulatorItem* self, const Impl& org);
    ~Impl();
    void initialize();
    void clear();
    bool initializeSimulation(const vector<SimulationBody*>& simBodies);
    bool stepSimulation(const vector<SimulationBody*>& activeSimBodies);
    void manualForwardWithContactOverrides();
    void injectSurfaceVelocities();
    void setupContactMaterials(const vector<SimulationBody*>& simBodies);
    const ResolvedContactParam& resolveContactParam(int materialId1, int materialId2);
    void overrideContactParams();
    void addExtraJoints(const vector<SimulationBody*>& simBodies);
    MuJoCoLinkInfo* findLinkInfo(Link* link);
    void addGeoms(mjsBody* mjBody, Link* link);
    void readMeshNode(mjsBody* mjBody, Link* link);
    bool addPrimitiveGeom(mjsBody* mjBody, SgMesh* mesh, const Vector3& scale, const Isometry3& T);
    void addMeshGeom(mjsBody* mjBody, SgMesh* mesh, const Vector3& scale, const Isometry3& T);
    void doPutProperties(PutPropertyFunction& putProperty);
    void store(Archive& archive);
    void restore(const Archive& archive);
};

}


void MuJoCoSimulatorItem::initialize(ExtensionManager* ext)
{
    installBoxBoxCollisionLimit();

    ext->itemManager().registerClass<MuJoCoSimulatorItem, SimulatorItem>(N_("MuJoCoSimulatorItem"));
    ext->itemManager().addCreationPanel<MuJoCoSimulatorItem>();
}


MuJoCoSimulatorItem::MuJoCoSimulatorItem()
{
    impl = new Impl(this);
    setAllLinkPositionOutputMode(false);
}


MuJoCoSimulatorItem::Impl::Impl(MuJoCoSimulatorItem* self)
    : self(self)
{
    initialize();

    gravity << 0.0, 0.0, -DefaultGravityAcceleration;

    // RK4 is intentionally not offered: it evaluates the dynamics several times
    // per step, which makes the per-contact friction override (applied once per
    // step) only approximate, and RK4 has little benefit for contact-rich rigid
    // body simulation. The selection uses sequential indices; the MuJoCo
    // integrator value is resolved from the symbol (see mujocoIntegrator()).
    integratorType.setSymbol(IntegratorEuler, N_("Euler"));
    integratorType.setSymbol(IntegratorImplicit, N_("Implicit"));
    integratorType.setSymbol(IntegratorImplicitFast, N_("ImplicitFast"));
    integratorType.select(IntegratorImplicitFast);

    solverType.setSymbol(mjSOL_PGS, N_("PGS"));
    solverType.setSymbol(mjSOL_CG, N_("CG"));
    solverType.setSymbol(mjSOL_NEWTON, N_("Newton"));
    solverType.select(mjSOL_NEWTON);

    numIterations = 100;
    numThreads = 1;
    positionGain = 1.0e4;
    velocityGain = 1.0e2;
    minFriction = 1.0e-3;
    maxFriction = 1.0;
    impratio = 1.0; // MuJoCo default
    isVelocityOutputEnabled = false;
    isAccelerationOutputEnabled = false;
    isSelfCollisionEnabledByDefault = false;
}


MuJoCoSimulatorItem::MuJoCoSimulatorItem(const MuJoCoSimulatorItem& org)
    : SimulatorItem(org)
{
    impl = new Impl(this, *org.impl);
}


MuJoCoSimulatorItem::Impl::Impl(MuJoCoSimulatorItem* self, const Impl& org)
    : self(self)
{
    initialize();

    gravity = org.gravity;
    integratorType = org.integratorType;
    solverType = org.solverType;
    numIterations = org.numIterations;
    numThreads = org.numThreads;
    positionGain = org.positionGain;
    velocityGain = org.velocityGain;
    minFriction = org.minFriction;
    maxFriction = org.maxFriction;
    impratio = org.impratio;
    isVelocityOutputEnabled = org.isVelocityOutputEnabled;
    isAccelerationOutputEnabled = org.isAccelerationOutputEnabled;
    isSelfCollisionEnabledByDefault = org.isSelfCollisionEnabledByDefault;
}


void MuJoCoSimulatorItem::Impl::initialize()
{
    spec = nullptr;
    model = nullptr;
    data = nullptr;
    threadPool = nullptr;
    materialTable = nullptr;
    meshCounter = 0;
    extraJointElementCounter = 0;
    mujocoBodyMap.clear();
    meshAssetMap.clear();
    hasSurfaceVelocityLinks = false;
    bodySurfaceVelocityLink.clear();
    hasContactMaterials = false;
    geomMaterialId.clear();
    contactParamCache.clear();
    numMaterialIds = 0;
    mout = MessageOut::master();
}


MuJoCoSimulatorItem::~MuJoCoSimulatorItem()
{
    delete impl;
}


MuJoCoSimulatorItem::Impl::~Impl()
{
    clear();
}


void MuJoCoSimulatorItem::Impl::clear()
{
    if(data){
        mj_deleteData(data);
        data = nullptr;
    }
    if(threadPool){
        mju_threadPoolDestroy(threadPool);
        threadPool = nullptr;
    }
    if(model){
        mj_deleteModel(model);
        model = nullptr;
    }
    if(spec){
        mj_deleteSpec(spec);
        spec = nullptr;
    }
    materialTable = nullptr;
    meshCounter = 0;
    extraJointElementCounter = 0;
    mujocoBodyMap.clear();
    meshAssetMap.clear();
    hasSurfaceVelocityLinks = false;
    bodySurfaceVelocityLink.clear();
    hasContactMaterials = false;
    geomMaterialId.clear();
    contactParamCache.clear();
    numMaterialIds = 0;
}


void MuJoCoSimulatorItem::setGravity(const Vector3& gravity)
{
    impl->gravity = gravity;
}


const Vector3& MuJoCoSimulatorItem::gravity() const
{
    return impl->gravity;
}


Vector3 MuJoCoSimulatorItem::getGravity() const
{
    return impl->gravity;
}


Item* MuJoCoSimulatorItem::doDuplicate() const
{
    return new MuJoCoSimulatorItem(*this);
}


SimulationBody* MuJoCoSimulatorItem::createSimulationBody(Body* orgBody, CloneMap& cloneMap)
{
    return new MuJoCoBody(cloneMap.getClone(orgBody), impl);
}


bool MuJoCoSimulatorItem::initializeSimulation(const std::vector<SimulationBody*>& simBodies)
{
    return impl->initializeSimulation(simBodies);
}


bool MuJoCoSimulatorItem::Impl::initializeSimulation(const std::vector<SimulationBody*>& simBodies)
{
    clear();

    timeStep = self->worldTimeStep();

    if(WorldItem* worldItem = self->worldItem()){
        materialTable = worldItem->materialTable();
    }

    spec = mj_makeSpec();
    if(!spec){
        mout->putErrorln(_("MuJoCo: mj_makeSpec failed."));
        return false;
    }

    // Global compiler and physics options
    spec->compiler.degree = 0; // angles in radians
    spec->compiler.balanceinertia = 1; // tolerate slightly invalid inertia
    spec->option.timestep = timeStep;
    spec->option.gravity[0] = gravity.x();
    spec->option.gravity[1] = gravity.y();
    spec->option.gravity[2] = gravity.z();
    spec->option.integrator = mujocoIntegrator(integratorType.selectedIndex());
    spec->option.solver = solverType.selectedIndex();
    spec->option.iterations = numIterations;
    spec->option.impratio = impratio;

    mjsBody* worldBody = mjs_findBody(spec, "world");
    if(!worldBody){
        mout->putErrorln(_("MuJoCo: failed to obtain the world body."));
        return false;
    }

    mujocoBodyMap.clear();
    for(size_t i = 0; i < simBodies.size(); ++i){
        auto mjcBody = static_cast<MuJoCoBody*>(simBodies[i]);
        mjcBody->bodyIndex = static_cast<int>(i);
        mujocoBodyMap[mjcBody->body()] = mjcBody;
    }
    for(size_t i = 0; i < simBodies.size(); ++i){
        auto mjcBody = static_cast<MuJoCoBody*>(simBodies[i]);
        mjcBody->createModelElements(worldBody);
    }
    addExtraJoints(simBodies);

    // Pseudo continuous tracks impose a surface velocity on contact friction.
    // The elliptic friction cone maps the two friction constraints directly to
    // the contact tangent directions, which makes the velocity injection exact.
    if(hasSurfaceVelocityLinks){
        spec->option.cone = mjCONE_ELLIPTIC;
    }

    model = mj_compile(spec, nullptr);
    if(!model){
        const char* msg = mjs_getError(spec);
        mout->putErrorln(formatR(_("MuJoCo: model compilation failed: {0}"),
                                 msg ? msg : "unknown error"));
        return false;
    }

    data = mj_makeData(model);
    if(!data){
        mout->putErrorln(_("MuJoCo: mj_makeData failed."));
        return false;
    }

    if(numThreads >= 2){
        threadPool = mju_threadPoolCreate(static_cast<size_t>(numThreads));
        if(threadPool){
            mju_bindThreadPool(data, threadPool);
        }
    }

    if(hasSurfaceVelocityLinks){
        bodySurfaceVelocityLink.assign(model->nbody, nullptr);
    }
    for(auto& simBody : simBodies){
        auto mjcBody = static_cast<MuJoCoBody*>(simBody);
        mjcBody->resolveIndices();
    }

    setupContactMaterials(simBodies);

    mj_resetData(model, data);

    for(auto& simBody : simBodies){
        auto mjcBody = static_cast<MuJoCoBody*>(simBody);
        mjcBody->setInitialState();
    }

    mj_forward(model, data);

    return true;
}


bool MuJoCoSimulatorItem::stepSimulation(const std::vector<SimulationBody*>& activeSimBodies)
{
    return impl->stepSimulation(activeSimBodies);
}


void MuJoCoSimulatorItem::finalizeSimulation()
{
    // The MuJoCo thread pool keeps its workers spinning to keep task dispatch
    // latency low, which would otherwise burn CPU after the simulation stops.
    // Detach it from mjData and destroy it here. mjModel/mjData are kept alive
    // so that post-simulation queries (e.g. getCollisions) still work.
    if(impl->threadPool){
        if(impl->data){
            impl->data->threadpool = 0;
        }
        mju_threadPoolDestroy(impl->threadPool);
        impl->threadPool = nullptr;
    }
}


bool MuJoCoSimulatorItem::Impl::stepSimulation(const std::vector<SimulationBody*>& activeSimBodies)
{
    mju_zero(data->qfrc_applied, model->nv);
    mju_zero(data->xfrc_applied, 6 * model->nbody);

    for(auto& simBody : activeSimBodies){
        auto mjcBody = static_cast<MuJoCoBody*>(simBody);
        mjcBody->body()->setVirtualJointForces();
        mjcBody->inputControl();
        mjcBody->addExternalForces();
    }

    if(hasContactMaterials){
        // Contact friction/restitution must be written into each mjContact between
        // collision detection and constraint construction, which mj_step1 has
        // already passed. So the forward pipeline is expanded manually here. The
        // surface-velocity injection (which edits the efc_aref values computed
        // from the contact constraints) is folded into the same manual pipeline.
        manualForwardWithContactOverrides();
    } else if(hasSurfaceVelocityLinks){
        // Split the step so that the surface velocities can be injected into the
        // contact constraints after they have been set up.
        mj_step1(model, data);
        injectSurfaceVelocities();
        mj_step2(model, data);
    } else {
        mj_step(model, data);
    }

    // cfrc_int (the body interaction wrench read by force sensors) is not computed
    // by mj_step, so derive it on demand only when some body has a force sensor.
    bool needForceSensors = false;
    for(auto& simBody : activeSimBodies){
        if(static_cast<MuJoCoBody*>(simBody)->hasForceSensors){
            needForceSensors = true;
            break;
        }
    }
    if(needForceSensors){
        mj_rnePostConstraint(model, data);
    }

    for(auto& simBody : activeSimBodies){
        auto mjcBody = static_cast<MuJoCoBody*>(simBody);
        mjcBody->getState();
        if(mjcBody->hasForceSensors){
            mjcBody->updateForceSensors();
        }
        if(mjcBody->sensorHelper.hasGyroOrAccelerationSensors()){
            mjcBody->sensorHelper.updateGyroAndAccelerationSensors();
        }
    }

    return true;
}


void MuJoCoSimulatorItem::Impl::manualForwardWithContactOverrides()
{
    // This reproduces mj_step (forward + integrate) by expanding mj_step1 +
    // mj_step2 verbatim from the MuJoCo source (engine_forward.c), so that the
    // contact parameters can be overridden after mj_collision but before
    // mj_makeConstraint. The expansion must match the source exactly: in
    // particular mj_checkPos/mj_checkVel (which sanitize qpos/qvel each step and
    // are essential for stability), mj_makeM (the mass matrix), and the order of
    // mj_transmission are all required. Omitting any of them makes the simulation
    // diverge in degenerate situations such as zero friction.
    //
    // Verified against plain mj_step for the offered integrators (Euler, Implicit,
    // ImplicitFast). RK4 is not offered, so a single override per step is exact.
    // Sleep/wake handling (mj_wakeCollision etc.) is omitted as that feature is off.

    // --- mj_step1 ---
    mj_checkPos(model, data);
    mj_checkVel(model, data);

    // mj_fwdPosition, with the override inserted between collision and makeConstraint
    mj_fwdKinematics(model, data);
    mj_makeM(model, data);
    mj_factorM(model, data);
    mj_collision(model, data);

    overrideContactParams();

    mj_makeConstraint(model, data);
    mj_island(model, data);
    mj_projectConstraint(model, data);
    mj_transmission(model, data);

    mj_sensorPos(model, data);
    mj_fwdVelocity(model, data);

    // The surface-velocity injection edits efc_aref. The contact rows and their
    // KBIP parameters are built by makeConstraint, while efc_aref itself is
    // computed by mj_fwdVelocity/mj_referenceConstraint. Apply the injection here
    // after efc_aref is computed and before mj_fwdConstraint consumes it.
    if(hasSurfaceVelocityLinks){
        injectSurfaceVelocities();
    }

    mj_sensorVel(model, data);

    // --- mj_step2 ---
    mj_fwdActuation(model, data);
    mj_fwdAcceleration(model, data);
    mj_fwdConstraint(model, data);
    mj_sensorAcc(model, data);
    mj_checkAcc(model, data);

    // Integration (RK4 is not offered; defaults to Euler in MuJoCo's mj_step2)
    if(model->opt.integrator == mjINT_IMPLICIT || model->opt.integrator == mjINT_IMPLICITFAST){
        mj_implicit(model, data);
    } else {
        mj_Euler(model, data);
    }
}


void MuJoCoSimulatorItem::Impl::overrideContactParams()
{
    for(int c = 0; c < data->ncon; ++c){
        mjContact& con = data->contact[c];
        if(con.exclude){
            continue;
        }
        int g0 = con.geom[0];
        int g1 = con.geom[1];
        if(g0 < 0 || g1 < 0){
            continue; // flex contacts are not handled
        }
        int m0 = geomMaterialId[g0];
        int m1 = geomMaterialId[g1];
        if(m0 < 0 || m1 < 0 || m0 >= numMaterialIds || m1 >= numMaterialIds){
            continue; // a geom without a valid Choreonoid material keeps the defaults
        }
        const ResolvedContactParam& p = resolveContactParam(m0, m1);
        if(!p.resolved){
            continue;
        }
        // Slide friction (both tangential directions). Spin/roll keep their values.
        // Clamp to [minFriction, maxFriction]. Both bounds work around limitations
        // of MuJoCo's contact solver (these are properties of plain mj_step too, not
        // of this override):
        //  - Near zero friction the solver becomes ill-conditioned and bodies bounce
        //    or vibrate. The min default (1e-3) has margin above the empirical onset
        //    of bouncing for a walking humanoid (around 6e-4).
        //  - With the default impratio, friction much above 1 weakens the normal
        //    contact (the normal impedance is relatively reduced), so heavy bodies
        //    sink into the floor. The max default (1.0) avoids that. Raising
        //    impratio is the proper fix for genuinely high friction.
        double friction = std::min(std::max(p.friction, minFriction), maxFriction);
        con.friction[0] = friction;
        con.friction[1] = friction;
        // Restitution is realized through the normal-direction solref in its
        // direct (negative) form [-stiffness, -damping]; see resolveContactParam
        // for the mapping. The friction rows must keep the original (positive
        // form) solref: with the elliptic friction cone they would otherwise
        // pick up the bounce damping (pyramidal cones ignore solreffriction).
        if(p.bouncy){
            if(con.solreffriction[0] == 0.0 && con.solreffriction[1] == 0.0){
                con.solreffriction[0] = con.solref[0];
                con.solreffriction[1] = con.solref[1];
            }
            con.solref[0] = p.solref[0];
            con.solref[1] = p.solref[1];
        }
    }
}


const MuJoCoSimulatorItem::Impl::ResolvedContactParam&
MuJoCoSimulatorItem::Impl::resolveContactParam(int materialId1, int materialId2)
{
    int i1 = materialId1;
    int i2 = materialId2;
    if(i1 > i2){
        std::swap(i1, i2);
    }
    ResolvedContactParam& cache = contactParamCache[i1 * numMaterialIds + i2];
    if(cache.resolved){
        return cache;
    }

    double friction = 1.0;
    double restitution = 0.0;

    // Prefer an explicitly defined contact material pair; otherwise derive the
    // values from the two single materials, matching the AIST ConstraintForceSolver
    // (friction = sqrt(r1*r2), restitution = sqrt((1-v1)*(1-v2))).
    ContactMaterial* cm = materialTable ? materialTable->contactMaterial(i1, i2) : nullptr;
    if(cm){
        friction = cm->friction();
        restitution = cm->restitution();
    } else if(materialTable){
        Material* mat1 = materialTable->material(i1);
        Material* mat2 = materialTable->material(i2);
        if(mat1 && mat2){
            friction = std::sqrt(mat1->roughness() * mat2->roughness());
            restitution = std::sqrt(std::max(0.0, 1.0 - mat1->viscosity())
                                    * std::max(0.0, 1.0 - mat2->viscosity()));
        }
    }

    cache.friction = friction;

    // Map the restitution coefficient e to the normal-direction solref in its
    // direct (negative) form [-stiffness, -damping]. The damping ratio comes
    // from the classical linear spring-damper relation
    // e = exp(-pi*z / sqrt(1 - z^2))  =>  z = -ln(e) / sqrt(pi^2 + ln(e)^2),
    // and the stiffness time scale is tied to the timestep (omega = 1/(2*dt)),
    // matching MuJoCo's recommended minimum solref time constant of 2*dt.
    // Verified with standalone ball-drop tests: the measured restitution tracks
    // the target within a few percent over e in [0.2, 0.9], independently of
    // mass, integrator, and impact speed. The standard (positive) solref form
    // cannot express restitution: its stiffness grows as the damping ratio
    // shrinks, which adds energy and drives the measured restitution above 1.
    restitution = std::min(std::max(restitution, 0.0), 0.99);
    cache.bouncy = restitution > 0.0;
    if(cache.bouncy){
        double L = std::log(restitution);
        double z = -L / std::sqrt(M_PI * M_PI + L * L);
        double omega = 1.0 / (2.0 * model->opt.timestep);
        cache.solref[0] = -omega * omega;
        cache.solref[1] = -2.0 * z * omega;
    }

    cache.resolved = true;
    return cache;
}


void MuJoCoSimulatorItem::Impl::setupContactMaterials(const vector<SimulationBody*>& simBodies)
{
    hasContactMaterials = false;
    geomMaterialId.clear();
    contactParamCache.clear();
    numMaterialIds = 0;

    if(!materialTable || materialTable->numMaterials() <= 0){
        return;
    }

    // Map every MuJoCo body id to its Choreonoid link, then map every geom to the
    // material id of the link that owns it.
    std::vector<Link*> bodyLink(model->nbody, nullptr);
    for(auto& simBody : simBodies){
        auto mjcBody = static_cast<MuJoCoBody*>(simBody);
        for(auto& info : mjcBody->linkInfos){
            if(info.link && info.bodyId >= 0){
                bodyLink[info.bodyId] = info.link;
            }
        }
    }

    geomMaterialId.assign(model->ngeom, -1);
    bool anyMaterial = false;
    for(int g = 0; g < model->ngeom; ++g){
        Link* link = bodyLink[model->geom_bodyid[g]];
        if(link){
            int id = link->materialId();
            geomMaterialId[g] = id;
            if(id >= 0){
                anyMaterial = true;
            }
        }
    }
    if(!anyMaterial){
        geomMaterialId.clear();
        return;
    }

    // Use the material id array size (maxMaterialId + 1) as the index range. The
    // ids may be sparse, so numMaterials() (the count of non-null entries) is not
    // a valid upper bound.
    numMaterialIds = materialTable->maxMaterialId() + 1;
    if(numMaterialIds <= 0){
        geomMaterialId.clear();
        return;
    }
    contactParamCache.assign(numMaterialIds * numMaterialIds, ResolvedContactParam());
    hasContactMaterials = true;
}


void MuJoCoSimulatorItem::Impl::injectSurfaceVelocities()
{
    // For each contact involving a surface velocity (pseudo continuous track)
    // link, set the target tangential velocity of the contact friction
    // constraints to the commanded surface (belt) velocity. With the elliptic
    // cone the two friction rows that follow the normal row correspond to the
    // contact frame tangent directions, and their reference acceleration is
    // (-damping * velocity), so adding (damping * v_target) shifts the target
    // velocity from 0 to v_target while the friction cone (mu * normal force)
    // is still respected by the solver.
    for(int c = 0; c < data->ncon; ++c){
        mjContact& con = data->contact[c];
        if(con.efc_address < 0 || con.dim < 3){
            continue;
        }
        Link* link0 = bodySurfaceVelocityLink[model->geom_bodyid[con.geom[0]]];
        Link* link1 = bodySurfaceVelocityLink[model->geom_bodyid[con.geom[1]]];
        if(!link0 && !link1){
            continue;
        }

        Vector3 normal(con.frame[0], con.frame[1], con.frame[2]); // points geom0 -> geom1
        Vector3 targetRel = Vector3::Zero();

        // The target relative tangential velocity (geom1 relative to geom0) that
        // makes a surface move at its commanded speed is
        //   dq * normalize(axisWorld x normal)
        // regardless of which geom slot it occupies. This matches the sign
        // convention of Choreonoid's own constraint force solver.
        auto accumulate = [&](int bodyId, Link* link){
            const mjtNum* R = &data->xmat[bodyId * 9]; // row-major body orientation
            const Vector3& a = link->a();
            Vector3 axisWorld(
                R[0] * a.x() + R[1] * a.y() + R[2] * a.z(),
                R[3] * a.x() + R[4] * a.y() + R[5] * a.z(),
                R[6] * a.x() + R[7] * a.y() + R[8] * a.z());
            Vector3 dir = axisWorld.cross(normal);
            double dl = dir.norm();
            if(dl > 1.0e-9){
                targetRel += (link->dq_target() / dl) * dir;
            }
        };
        if(link0){
            accumulate(model->geom_bodyid[con.geom[0]], link0);
        }
        if(link1){
            accumulate(model->geom_bodyid[con.geom[1]], link1);
        }

        Vector3 t1(con.frame[3], con.frame[4], con.frame[5]);
        Vector3 t2(con.frame[6], con.frame[7], con.frame[8]);
        int a = con.efc_address;
        double damping1 = data->efc_KBIP[(a + 1) * 4 + 1];
        double damping2 = data->efc_KBIP[(a + 2) * 4 + 1];
        data->efc_aref[a + 1] += damping1 * targetRel.dot(t1);
        data->efc_aref[a + 2] += damping2 * targetRel.dot(t2);
    }
}


//---------------------------------------------------------------------------
// Model building
//---------------------------------------------------------------------------

MuJoCoBody::MuJoCoBody(Body* body, MuJoCoSimulatorItem::Impl* simImpl)
    : SimulationBody(body),
      simImpl(simImpl)
{
    bodyIndex = 0;
    hasForceSensors = false;
    isStatic = false;
    worldBody = nullptr;
}


void MuJoCoBody::createModelElements(mjsBody* worldBody)
{
    auto orgBody = body();
    this->worldBody = worldBody;
    namePrefix = formatC("b{}_", bodyIndex);
    linkInfos.clear();
    linkInfos.resize(orgBody->numLinks());

    buildLink(orgBody->rootLink(), worldBody);

    sensorHelper.initialize(orgBody, simImpl->timeStep, simImpl->gravity);

    forceSensorInfos.clear();
    for(auto& sensor : sensorHelper.forceSensors()){
        MuJoCoForceSensorInfo info;
        info.sensor = sensor;
        info.link = sensor->link();
        forceSensorInfos.push_back(info);
        hasForceSensors = true;
    }

    bool selfCollision = simImpl->isSelfCollisionEnabledByDefault;
    if(bodyItem()){
        selfCollision = bodyItem()->isSelfCollisionDetectionEnabled();
    }
    if(!selfCollision){
        addSelfCollisionExcludes();
    }
}


void MuJoCoBody::buildLink(Link* link, mjsBody* parentMjBody)
{
    bool isTopLevelFreeLink = link->isFreeJoint() && !link->isRoot();
    mjsBody* mjParentBody = isTopLevelFreeLink ? worldBody : parentMjBody;
    mjsBody* mjBody = mjs_addBody(mjParentBody, nullptr);
    string bodyName = namePrefix + std::to_string(link->index());
    mjs_setName(mjBody->element, bodyName.c_str());

    auto& info = linkInfos[link->index()];
    info.link = link;
    info.mjBody = mjBody;
    info.bodyName = bodyName;

    // Body frame relative to parent
    if(link->isRoot() || isTopLevelFreeLink){
        const Isometry3& T = link->T();
        mjBody->pos[0] = T.translation().x();
        mjBody->pos[1] = T.translation().y();
        mjBody->pos[2] = T.translation().z();
        setQuat(mjBody->quat, T.linear());
    } else {
        const Isometry3& Tb = link->Tb();
        mjBody->pos[0] = Tb.translation().x();
        mjBody->pos[1] = Tb.translation().y();
        mjBody->pos[2] = Tb.translation().z();
        setQuat(mjBody->quat, Tb.linear());
    }

    // Inertial properties (explicit inertial frame at the center of mass)
    mjBody->mass = link->m();
    const Vector3& c = link->c();
    mjBody->ipos[0] = c.x();
    mjBody->ipos[1] = c.y();
    mjBody->ipos[2] = c.z();
    const Matrix3& I = link->I();
    mjBody->fullinertia[0] = I(0, 0);
    mjBody->fullinertia[1] = I(1, 1);
    mjBody->fullinertia[2] = I(2, 2);
    mjBody->fullinertia[3] = I(0, 1);
    mjBody->fullinertia[4] = I(0, 2);
    mjBody->fullinertia[5] = I(1, 2);

    // Joint
    string jointName = namePrefix + "j" + std::to_string(link->index());
    if(link->isFreeJoint()){
        mjsJoint* joint = mjs_addFreeJoint(mjBody);
        mjs_setName(joint->element, jointName.c_str());
        info.mjJoint = joint;
        info.jointType = mjJNT_FREE;
        info.isFreeJoint = true;

    } else if(link->isRevoluteJoint() || link->isPrismaticJoint()){
        mjsJoint* joint = mjs_addJoint(mjBody, nullptr);
        mjs_setName(joint->element, jointName.c_str());
        joint->type = link->isRevoluteJoint() ? mjJNT_HINGE : mjJNT_SLIDE;
        const Vector3& a = link->jointAxis();
        joint->axis[0] = a.x();
        joint->axis[1] = a.y();
        joint->axis[2] = a.z();
        joint->pos[0] = joint->pos[1] = joint->pos[2] = 0.0;
        joint->ref = 0.0;
        joint->armature = link->Jm2();
        if(link->hasJointDisplacementLimits()){
            joint->limited = mjLIMITED_TRUE;
            joint->range[0] = link->q_lower();
            joint->range[1] = link->q_upper();
        } else {
            joint->limited = mjLIMITED_FALSE;
        }
        info.mjJoint = joint;
        info.jointType = joint->type;
        info.hasActualJoint = true;

        // Create a native MuJoCo actuator matching the joint's actuation mode.
        // The mode is already set by the controller at this point (controllers
        // are initialized before SimulatorItem::initializeSimulation).
        ControlType controlType = controlTypeFromActuationMode(link->actuationMode());
        if(controlType != NoControl){
            mjsActuator* act = mjs_addActuator(simImpl->spec, nullptr);
            string actName = jointName + "_act";
            mjs_setName(act->element, actName.c_str());
            act->trntype = mjTRN_JOINT;
            mjs_setString(act->target, jointName.c_str());
            // The gains can be overridden per joint in the body file with the
            // link info keys drive_stiffness (kp) and drive_damping (kv),
            // which are shared with the PhysX plugin.
            double kp = simImpl->positionGain;
            double kv = simImpl->velocityGain;
            link->info()->read("drive_stiffness", kp);
            link->info()->read("drive_damping", kv);
            if(controlType == MotorControl){
                mjs_setToMotor(act);
            } else if(controlType == PositionControl){
                mjs_setToPosition(act, kp, &kv, nullptr, nullptr, 0.0);
            } else { // VelocityControl
                mjs_setToVelocity(act, kv);
            }
            info.velocityGain = kv;
            // Limit the actuator output force to the joint effort range.
            if(link->hasJointEffortLimits()){
                act->forcelimited = mjLIMITED_TRUE;
                act->forcerange[0] = link->u_lower();
                act->forcerange[1] = link->u_upper();
            }
            info.controlType = controlType;
            // The velocity feedforward is only meaningful when the controller
            // also declares the JointVelocity channel (so dq_target is supplied).
            info.velocityFeedforward =
                (controlType == PositionControl) && (link->actuationMode() & Link::JointVelocity);
        }
    }
    // Fixed joint and pseudo continuous track -> no joint element (welded).
    // A pseudo continuous track is a rigid (welded) link whose contact surface
    // is given a commanded velocity; it is handled at contact time instead.
    if(link->jointType() == Link::PseudoContinuousTrackJoint){
        info.useSurfaceVelocity = true;
        simImpl->hasSurfaceVelocityLinks = true;
    }

    simImpl->addGeoms(mjBody, link);

    for(Link* child = link->child(); child; child = child->sibling()){
        buildLink(child, mjBody);
    }
}


MuJoCoLinkInfo* MuJoCoSimulatorItem::Impl::findLinkInfo(Link* link)
{
    if(!link){
        return nullptr;
    }
    auto p = mujocoBodyMap.find(link->body());
    if(p == mujocoBodyMap.end()){
        return nullptr;
    }
    auto mjcBody = p->second;
    const int linkIndex = link->index();
    if(linkIndex < 0 || linkIndex >= static_cast<int>(mjcBody->linkInfos.size())){
        return nullptr;
    }
    auto& info = mjcBody->linkInfos[linkIndex];
    if(info.link != link || !info.mjBody){
        return nullptr;
    }
    return &info;
}


void MuJoCoSimulatorItem::Impl::addExtraJoints(const vector<SimulationBody*>& simBodies)
{
    std::unordered_set<ExtraJoint*> initializedExtraJoints;

    auto addSite = [this](MuJoCoLinkInfo* info, const Isometry3& T){
        mjsSite* site = mjs_addSite(info->mjBody, nullptr);
        string name = formatC("ej_site{}", extraJointElementCounter++);
        mjs_setName(site->element, name.c_str());
        site->pos[0] = T.translation().x();
        site->pos[1] = T.translation().y();
        site->pos[2] = T.translation().z();
        setQuat(site->quat, T.linear());
        return name;
    };

    auto addEquality = [this](mjtEq type, const string& name0, const string& name1){
        mjsEquality* eq = mjs_addEquality(spec, nullptr);
        eq->type = type;
        eq->objtype = mjOBJ_SITE;
        mjs_setString(eq->name1, name0.c_str());
        mjs_setString(eq->name2, name1.c_str());
        if(type == mjEQ_WELD){
            eq->data[10] = 1.0; // torquescale
        }
        eq->active = 1;
    };

    auto makePosition = [](const Vector3& p){
        Isometry3 T;
        T.setIdentity();
        T.translation() = p;
        return T;
    };

    for(auto simBody : simBodies){
        auto mjcBody = static_cast<MuJoCoBody*>(simBody);
        auto body = mjcBody->body();
        int numExtraJoints = body->numExtraJoints();

        for(int i = 0; i < numExtraJoints; ++i){
            ExtraJoint* extraJoint = body->extraJoint(i);
            if(!initializedExtraJoints.insert(extraJoint).second){
                continue;
            }

            Link* links[2] = {
                extraJoint->link(0),
                extraJoint->link(1)
            };
            MuJoCoLinkInfo* linkInfos[2] = {
                findLinkInfo(links[0]),
                findLinkInfo(links[1])
            };
            if(!links[0] || !links[1]){
                continue;
            }
            if(!linkInfos[0] || !linkInfos[1]){
                mout->putWarningln(
                    formatR(_("MuJoCo: the extra joint between {0} and {1} is ignored "
                              "because one of the links is not included in the simulation."),
                            links[0]->name(), links[1]->name()));
                continue;
            }

            // MuJoCo expresses loop-closing constraints as equality constraints.
            // The equality is attached to sites so both link-local frames of the
            // ExtraJoint are honored, including extra joints across Body objects.
            int jointType = extraJoint->type();

            if(jointType == ExtraJoint::Piston){
                mout->putWarningln(
                    formatR(_("MuJoCo: the piston extra joint between {0} and {1} is not "
                              "supported and is ignored."),
                            links[0]->name(), links[1]->name()));
                continue;
            }

            if(jointType == ExtraJoint::Fixed){
                string site0 = addSite(linkInfos[0], extraJoint->localPosition(0));
                string site1 = addSite(linkInfos[1], extraJoint->localPosition(1));
                addEquality(mjEQ_WELD, site0, site1);

            } else if(jointType == ExtraJoint::Ball){
                string site0 = addSite(linkInfos[0], extraJoint->localPosition(0));
                string site1 = addSite(linkInfos[1], extraJoint->localPosition(1));
                addEquality(mjEQ_CONNECT, site0, site1);

            } else if(jointType == ExtraJoint::Hinge){
                // Pin two points along the hinge axis. Connecting both pairs
                // makes the axis lines coincide while leaving rotation about it.
                const Vector3 axis0 =
                    (extraJoint->localRotation(0) * extraJoint->axis()).normalized();
                const Vector3 axis1 =
                    (extraJoint->localRotation(1) * extraJoint->axis()).normalized();
                const double halfSpan = 0.05;
                for(int s = -1; s <= 1; s += 2){
                    Vector3 p0 = extraJoint->localTranslation(0) + (s * halfSpan) * axis0;
                    Vector3 p1 = extraJoint->localTranslation(1) + (s * halfSpan) * axis1;
                    string site0 = addSite(linkInfos[0], makePosition(p0));
                    string site1 = addSite(linkInfos[1], makePosition(p1));
                    addEquality(mjEQ_CONNECT, site0, site1);
                }
            }
        }
    }
}


void MuJoCoBody::addSelfCollisionExcludes()
{
    // Exclude collisions among all link pairs of this body. Parent-child pairs
    // are already excluded by MuJoCo, but adding them again is harmless.
    int n = static_cast<int>(linkInfos.size());
    for(int i = 0; i < n; ++i){
        if(!linkInfos[i].mjBody){
            continue;
        }
        const string& name1 = linkInfos[i].bodyName;
        for(int j = i + 1; j < n; ++j){
            if(!linkInfos[j].mjBody){
                continue;
            }
            const string& name2 = linkInfos[j].bodyName;
            mjsExclude* exclude = mjs_addExclude(simImpl->spec);
            mjs_setString(exclude->bodyname1, name1.c_str());
            mjs_setString(exclude->bodyname2, name2.c_str());
        }
    }
}


void MuJoCoSimulatorItem::Impl::addGeoms(mjsBody* mjBody, Link* link)
{
    auto shape = link->collisionShape();
    if(!shape){
        return;
    }
    meshExtractor.extract(shape, [this, mjBody, link](){ readMeshNode(mjBody, link); });
}


void MuJoCoSimulatorItem::Impl::readMeshNode(mjsBody* mjBody, Link* link)
{
    SgMesh* mesh = meshExtractor.currentMesh();
    if(!mesh){
        return;
    }

    Vector3 scale = Vector3::Ones();
    if(meshExtractor.isCurrentScaled()){
        Affine3 S = meshExtractor.currentTransformWithoutScaling().inverse()
                  * meshExtractor.currentTransform();
        if(S.linear().isDiagonal()){
            scale = S.linear().diagonal();
        } else {
            mout->putWarningln(
                formatR(_("{0} has a non-axis-aligned scaling which is not supported "
                          "in the MuJoCo plugin. The mesh is skipped."),
                        meshLabel(meshExtractor.currentShape())));
            return;
        }
    }

    Isometry3 T = meshExtractor.currentTransformWithoutScaling();

    bool done = false;
    if(mesh->primitiveType() != SgMesh::MeshType){
        done = addPrimitiveGeom(mjBody, mesh, scale, T);
    }
    if(!done){
        addMeshGeom(mjBody, mesh, scale, T);
    }
}


bool MuJoCoSimulatorItem::Impl::addPrimitiveGeom(
    mjsBody* mjBody, SgMesh* mesh, const Vector3& scale, const Isometry3& T)
{
    mjsGeom* geom = nullptr;
    bool needAxisRotation = false; // MuJoCo cylinder/capsule axis is local Z; Choreonoid is local Y

    switch(mesh->primitiveType()){
    case SgMesh::BoxType: {
        auto box = mesh->primitive<SgMesh::Box>();
        geom = mjs_addGeom(mjBody, nullptr);
        geom->type = mjGEOM_BOX;
        geom->size[0] = box.size.x() * scale.x() / 2.0;
        geom->size[1] = box.size.y() * scale.y() / 2.0;
        geom->size[2] = box.size.z() * scale.z() / 2.0;
        break;
    }
    case SgMesh::SphereType: {
        if(!(scale.x() == scale.y() && scale.y() == scale.z())){
            return false;
        }
        auto sphere = mesh->primitive<SgMesh::Sphere>();
        geom = mjs_addGeom(mjBody, nullptr);
        geom->type = mjGEOM_SPHERE;
        geom->size[0] = sphere.radius * scale.x();
        break;
    }
    case SgMesh::CapsuleType: {
        if(scale.x() != scale.z()){
            return false;
        }
        auto capsule = mesh->primitive<SgMesh::Capsule>();
        geom = mjs_addGeom(mjBody, nullptr);
        geom->type = mjGEOM_CAPSULE;
        geom->size[0] = capsule.radius * scale.x();
        geom->size[1] = capsule.height * scale.y() / 2.0;
        needAxisRotation = true;
        break;
    }
    case SgMesh::CylinderType: {
        if(scale.x() != scale.z()){
            return false;
        }
        auto cylinder = mesh->primitive<SgMesh::Cylinder>();
        geom = mjs_addGeom(mjBody, nullptr);
        geom->type = mjGEOM_CYLINDER;
        geom->size[0] = cylinder.radius * scale.x();
        geom->size[1] = cylinder.height * scale.y() / 2.0;
        needAxisRotation = true;
        break;
    }
    default:
        return false;
    }

    Isometry3 frame = T;
    if(needAxisRotation){
        // Map the geom's local Z (MuJoCo primitive axis) to local Y (Choreonoid axis)
        frame.linear() = frame.linear() * AngleAxis(-M_PI / 2.0, Vector3::UnitX()).toRotationMatrix();
    }
    geom->pos[0] = frame.translation().x();
    geom->pos[1] = frame.translation().y();
    geom->pos[2] = frame.translation().z();
    setQuat(geom->quat, frame.linear());

    return true;
}


void MuJoCoSimulatorItem::Impl::addMeshGeom(
    mjsBody* mjBody, SgMesh* mesh, const Vector3& scale, const Isometry3& T)
{
    auto vertices = mesh->vertices();
    if(!vertices || vertices->empty() || !mesh->hasTriangles()){
        return;
    }

    // One mesh asset is shared by all the geoms that use the same mesh with
    // the same scale (the scale is baked into the asset vertices).
    const auto assetKey = std::make_pair(
        mesh, std::array<double, 3>{ scale.x(), scale.y(), scale.z() });
    string meshName;
    auto it = meshAssetMap.find(assetKey);
    if(it != meshAssetMap.end()){
        meshName = it->second;
    } else {
        meshName = formatC("mesh{}", meshCounter++);
        meshAssetMap[assetKey] = meshName;

        mjsMesh* mjMesh = mjs_addMesh(spec, nullptr);
        mjs_setName(mjMesh->element, meshName.c_str());

        int numVertices = static_cast<int>(vertices->size());
        vector<float> vertData(numVertices * 3);
        for(int i = 0; i < numVertices; ++i){
            const Vector3f& v = (*vertices)[i];
            vertData[i * 3]     = v.x() * scale.x();
            vertData[i * 3 + 1] = v.y() * scale.y();
            vertData[i * 3 + 2] = v.z() * scale.z();
        }
        mjs_setFloat(mjMesh->uservert, vertData.data(), static_cast<int>(vertData.size()));

        int numTriangles = mesh->numTriangles();
        auto& triangleVertices = mesh->triangleVertices();
        vector<int> faceData(triangleVertices.begin(), triangleVertices.end());
        mjs_setInt(mjMesh->userface, faceData.data(), numTriangles * 3);
    }

    mjsGeom* geom = mjs_addGeom(mjBody, nullptr);
    geom->type = mjGEOM_MESH;
    mjs_setString(geom->meshname, meshName.c_str());
    geom->pos[0] = T.translation().x();
    geom->pos[1] = T.translation().y();
    geom->pos[2] = T.translation().z();
    setQuat(geom->quat, T.linear());
}


//---------------------------------------------------------------------------
// Index resolution and state transfer
//---------------------------------------------------------------------------

void MuJoCoBody::resolveIndices()
{
    mjModel* model = simImpl->model;
    for(auto& info : linkInfos){
        if(!info.link){
            continue;
        }
        info.bodyId = mj_name2id(model, mjOBJ_BODY, info.bodyName.c_str());

        if(info.useSurfaceVelocity && info.bodyId >= 0){
            simImpl->bodySurfaceVelocityLink[info.bodyId] = info.link;
        }

        if(info.mjJoint){
            string jointName = namePrefix + "j" + std::to_string(info.link->index());
            int jid = mj_name2id(model, mjOBJ_JOINT, jointName.c_str());
            if(jid >= 0){
                info.qposadr = model->jnt_qposadr[jid];
                info.dofadr = model->jnt_dofadr[jid];
            }
            if(info.controlType != NoControl){
                string actName = jointName + "_act";
                info.actuatorId = mj_name2id(model, mjOBJ_ACTUATOR, actName.c_str());
            }
        }
    }

    for(auto& fsInfo : forceSensorInfos){
        fsInfo.bodyId = linkInfos[fsInfo.link->index()].bodyId;
    }
}


void MuJoCoBody::setInitialState()
{
    mjData* d = simImpl->data;
    for(auto& info : linkInfos){
        if(!info.link){
            continue;
        }
        Link* link = info.link;
        if(info.isFreeJoint){
            int a = info.qposadr;
            if(a >= 0){
                const Vector3& p = link->p();
                d->qpos[a]     = p.x();
                d->qpos[a + 1] = p.y();
                d->qpos[a + 2] = p.z();
                setQuat(&d->qpos[a + 3], link->R());
            }
        } else if(info.hasActualJoint && info.qposadr >= 0){
            d->qpos[info.qposadr] = link->q();
            d->qvel[info.dofadr] = link->dq();
        }
    }
}


void MuJoCoBody::inputControl()
{
    mjData* d = simImpl->data;

    for(auto& info : linkInfos){
        if(info.actuatorId < 0){
            continue;
        }
        Link* link = info.link;

        if(info.controlType == MotorControl){
            // Motor actuator: ctrl is the joint torque. The joint effort limit is
            // applied by the actuator's forcerange (set at build time), which is
            // MuJoCo's native mechanism for clamping the output force.
            d->ctrl[info.actuatorId] = link->u();

        } else if(info.controlType == PositionControl){
            // Position actuator applies kp*(q_target - q) - kv*qvel implicitly.
            // When the JointVelocity bit is also set, add the kv*dq_target
            // feedforward so the total damping term becomes kv*(dq_target - qvel),
            // i.e. a full PD with velocity feedforward. Without that bit dq_target
            // is not a supplied command, so no feedforward is applied.
            d->ctrl[info.actuatorId] = link->q_target();
            d->qfrc_applied[info.dofadr] =
                info.velocityFeedforward ? info.velocityGain * link->dq_target() : 0.0;

        } else if(info.controlType == VelocityControl){
            // Velocity actuator applies kv*(dq_target - qvel) implicitly.
            d->ctrl[info.actuatorId] = link->dq_target();
        }
    }
}


void MuJoCoBody::addExternalForces()
{
    mjModel* model = simImpl->model;
    mjData* d = simImpl->data;
    for(auto& info : linkInfos){
        if(info.bodyId < 0){
            continue;
        }
        Link* link = info.link;
        const Vector3& f = link->f_ext();
        const Vector3& tau = link->tau_ext();
        if(f.isZero() && tau.isZero()){
            continue;
        }
        // f_ext/tau_ext follow Choreonoid's convention: force applied at the
        // global origin and torque about the global origin.
        mjtNum force[3] = { f.x(), f.y(), f.z() };
        mjtNum torque[3] = { tau.x(), tau.y(), tau.z() };
        mjtNum point[3] = { 0.0, 0.0, 0.0 };
        mj_applyFT(model, d, force, torque, point, info.bodyId, d->qfrc_applied);
    }
    body()->clearExternalForces();
}


void MuJoCoBody::getState()
{
    mjModel* model = simImpl->model;
    mjData* d = simImpl->data;
    bool velOut = simImpl->isVelocityOutputEnabled || simImpl->isAccelerationOutputEnabled;
    bool accOut = simImpl->isAccelerationOutputEnabled;

    for(auto& info : linkInfos){
        if(info.bodyId < 0){
            continue;
        }
        Link* link = info.link;
        int b = info.bodyId;
        link->p() << d->xpos[b * 3], d->xpos[b * 3 + 1], d->xpos[b * 3 + 2];
        link->R() = toMatrix3(&d->xquat[b * 4]);

        if(info.hasActualJoint && info.qposadr >= 0){
            link->q() = d->qpos[info.qposadr];
            if(velOut){
                link->dq() = d->qvel[info.dofadr];
            }
        }

        if(velOut){
            mjtNum vel[6];
            mj_objectVelocity(model, d, mjOBJ_BODY, b, vel, 0); // global frame
            link->w() << vel[0], vel[1], vel[2];
            link->v() << vel[3], vel[4], vel[5];
        }

        if(accOut){
            mjtNum acc[6];
            mj_objectAcceleration(model, d, mjOBJ_BODY, b, acc, 0); // global frame
            link->dw() << acc[0], acc[1], acc[2];
            link->dv() << acc[3], acc[4], acc[5];
        }
    }
}


void MuJoCoBody::updateForceSensors()
{
    mjModel* model = simImpl->model;
    mjData* d = simImpl->data;

    for(auto& fsInfo : forceSensorInfos){
        int b = fsInfo.bodyId;
        if(b < 0){
            continue;
        }
        Link* link = fsInfo.link;
        ForceSensor* sensor = fsInfo.sensor;

        // cfrc_int[body] is the constraint wrench the parent applies to this body,
        // expressed about the subtree center of mass in the global frame as
        // [torque(3), force(3)] (MuJoCo spatial order rot:lin). This matches the
        // sign convention of Choreonoid's own force sensor (the wrench acting on
        // the sensing link from the proximal side).
        const mjtNum* c = &d->cfrc_int[b * 6];
        Vector3 tauCom(c[0], c[1], c[2]);
        Vector3 f(c[3], c[4], c[5]);

        int rootId = model->body_rootid[b];
        Vector3 com(d->subtree_com[rootId * 3],
                    d->subtree_com[rootId * 3 + 1],
                    d->subtree_com[rootId * 3 + 2]);

        // Sensor frame in the world.
        const Matrix3 Rsensor = link->R() * sensor->R_local();
        const Vector3 pSensor = link->p() + link->R() * sensor->p_local();

        // Move the moment from the subtree COM to the sensor position, then rotate
        // both force and moment into the sensor frame.
        Vector3 tauSensorOrigin = tauCom + (com - pSensor).cross(f);
        sensor->f() = Rsensor.transpose() * f;
        sensor->tau() = Rsensor.transpose() * tauSensorOrigin;
        sensor->notifyStateChange();
    }
}


//---------------------------------------------------------------------------
// Properties and serialization
//---------------------------------------------------------------------------

void MuJoCoSimulatorItem::doPutProperties(PutPropertyFunction& putProperty)
{
    SimulatorItem::doPutProperties(putProperty);
    impl->doPutProperties(putProperty);
}


void MuJoCoSimulatorItem::Impl::doPutProperties(PutPropertyFunction& putProperty)
{
    putProperty(_("Gravity"), str(gravity), [&](const string& v){ return toVector3(v, gravity); });
    putProperty(_("Integrator"), integratorType, changeProperty(integratorType));
    putProperty(_("Solver"), solverType, changeProperty(solverType));
    putProperty.min(1)(_("Solver iterations"), numIterations, changeProperty(numIterations));
    putProperty.min(1)(_("Number of threads"), numThreads, changeProperty(numThreads));
    putProperty.decimals(1).min(0.0)(_("Position gain"), positionGain, changeProperty(positionGain));
    putProperty.decimals(1).min(0.0)(_("Velocity gain"), velocityGain, changeProperty(velocityGain));
    putProperty.decimals(6).min(0.0)(_("Min friction"), minFriction, changeProperty(minFriction));
    putProperty.decimals(3).min(0.0)(_("Max friction"), maxFriction, changeProperty(maxFriction));
    putProperty.decimals(2).min(0.0)(_("Friction impedance ratio"), impratio, changeProperty(impratio));
    putProperty(_("Self collision"), isSelfCollisionEnabledByDefault,
                changeProperty(isSelfCollisionEnabledByDefault));
    putProperty(_("Velocity output"), isVelocityOutputEnabled, changeProperty(isVelocityOutputEnabled));
    putProperty(_("Acceleration output"), isAccelerationOutputEnabled, changeProperty(isAccelerationOutputEnabled));
}


bool MuJoCoSimulatorItem::store(Archive& archive)
{
    SimulatorItem::store(archive);
    impl->store(archive);
    return true;
}


void MuJoCoSimulatorItem::Impl::store(Archive& archive)
{
    write(archive, "gravity", gravity);
    archive.write("integrator", integratorType.selectedSymbol());
    archive.write("solver", solverType.selectedSymbol());
    archive.write("solver_iterations", numIterations);
    archive.write("num_threads", numThreads);
    archive.write("position_gain", positionGain);
    archive.write("velocity_gain", velocityGain);
    archive.write("min_friction", minFriction);
    archive.write("max_friction", maxFriction);
    archive.write("impratio", impratio);
    archive.write("self_collision", isSelfCollisionEnabledByDefault);
    archive.write("velocity_output", isVelocityOutputEnabled);
    archive.write("acceleration_output", isAccelerationOutputEnabled);
}


bool MuJoCoSimulatorItem::restore(const Archive& archive)
{
    SimulatorItem::restore(archive);
    impl->restore(archive);
    return true;
}


void MuJoCoSimulatorItem::Impl::restore(const Archive& archive)
{
    read(archive, "gravity", gravity);
    string symbol;
    if(archive.read("integrator", symbol)){
        integratorType.select(symbol);
    }
    if(archive.read("solver", symbol)){
        solverType.select(symbol);
    }
    archive.read("solver_iterations", numIterations);
    archive.read("num_threads", numThreads);
    archive.read("position_gain", positionGain);
    archive.read("velocity_gain", velocityGain);
    archive.read("min_friction", minFriction);
    archive.read("max_friction", maxFriction);
    archive.read("impratio", impratio);
    archive.read("self_collision", isSelfCollisionEnabledByDefault);
    archive.read("velocity_output", isVelocityOutputEnabled);
    archive.read("acceleration_output", isAccelerationOutputEnabled);
}
