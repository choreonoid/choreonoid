#include "PhysXSimulatorItemImpl.h"
#include "PhysXPlugin.h"
#include "PxContinuousTrackSimulator.h"
#include <cnoid/ItemManager>
#include <cnoid/WorldItem>
#include <cnoid/BodyItem>
#include <cnoid/PutPropertyFunction>
#include <cnoid/Body>
#include <cnoid/Link>
#include <cnoid/Material>
#include <cnoid/EigenUtil>
#include <cnoid/EigenArchive>
#include <cnoid/Archive>
#include <cnoid/Format>
#include <cnoid/MessageOut>
#include <cnoid/MessageView>
#include <geometry/PxConvexCoreGeometry.h>
#include <extensions/PxDefaultSimulationFilterShader.h>
#include <cmath>
#include "gettext.h"

using namespace std;
using namespace cnoid;
using namespace physx;

namespace {

constexpr double DefaultGravityAcceleration = 9.80665;
constexpr double DefaultDriveStiffness = 1.0e8;
constexpr double DefaultDriveDamping = 1.0e5;

PxTransform getPxTransform(const Isometry3& T)
{
    auto M = T.matrix();
    return PxTransform(
        PxMat44(
            PxVec3(M(0,0), M(1,0), M(2,0)),
            PxVec3(M(0,1), M(1,1), M(2,1)),
            PxVec3(M(0,2), M(1,2), M(2,2)),
            PxVec3(M(0,3), M(1,3), M(2,3))));
}

PxMat33 getPxMat33(const Matrix3& M)
{
    return PxMat33(
        PxVec3(M(0,0), M(1,0), M(2,0)),
        PxVec3(M(0,1), M(1,1), M(2,1)),
        PxVec3(M(0,2), M(1,2), M(2,2)));
}

PxVec3 getPxVec3(const Vector3& v)
{
    return PxVec3(v.x(), v.y(), v.z());
}

string meshLabel(SgShape* shape)
{
    auto& name = shape->name();
    if(!name.empty()){
        return formatR(_("Mesh \"{0}\""), name);
    }
    if(auto mesh = shape->mesh()){
        auto& meshName = mesh->name();
        if(!meshName.empty()){
            return formatR(_("Mesh \"{0}\""), meshName);
        }
    }
    return _("A mesh");
}

}

namespace {

bool checkIfArticulationRoot(Link* link)
{
    for(Link* child = link->child(); child; child = child->sibling()){
        if(!child->isFreeJoint()){
            if(!child->isStatic() || checkIfArticulationRoot(child)){
                return true;
            }
        }
    }
    return false;
}

PxFilterFlags customFilterShader(
    PxFilterObjectAttributes attributes0, PxFilterData filterData0,
    PxFilterObjectAttributes attributes1, PxFilterData filterData1,
    PxPairFlags& pairFlags, const void* constantBlock, PxU32 constantBlockSize)
{
    pairFlags = PxPairFlag::eCONTACT_DEFAULT;

    if(filterData0.word0 || filterData1.word0){
        pairFlags |= PxPairFlag::eMODIFY_CONTACTS;
    }

    // Same body pair: need callback for self-collision filtering
    if(filterData0.word1 != 0 && filterData0.word1 == filterData1.word1){
        return PxFilterFlag::eCALLBACK;
    }

    return PxFilterFlags();
}

} // anonymous namespace


PxFilterFlags CustomFilterCallback::pairFound(
    PxU64 pairID,
    PxFilterObjectAttributes attributes0, PxFilterData filterData0, const PxActor* a0, const PxShape* s0,
    PxFilterObjectAttributes attributes1, PxFilterData filterData1, const PxActor* a1, const PxShape* s1,
    PxPairFlags& pairFlags)
{
    // This callback is now only called for same-body pairs
    PhysxLink* physxLink0 = static_cast<PhysxLink*>(a0->userData);
    PhysxLink* physxLink1 = static_cast<PhysxLink*>(a1->userData);

    // If either actor has no userData, it is a linear segment (PxContinuousTrack).
    // Kill all same-body collisions involving linear segments to prevent
    // segment-wheel and segment-segment interference within the same articulation.
    if(!physxLink0 || !physxLink1){
        return PxFilterFlag::eKILL;
    }

    PhysxBody* physxBody0 = physxLink0->physxBody;
    if(!physxBody0->bodyCollisionLinkFilter.checkIfEnabledLinkPair(
           physxLink0->link->index(), physxLink1->link->index())){
        return PxFilterFlag::eKILL;
    }

    // pairFlags is already set by the shader, including eMODIFY_CONTACTS if needed
    return PxFilterFlags();
}


void CrawlerContactModifyCallback::onContactModify(PxContactModifyPair* const pairs, PxU32 count)
{
    for(PxU32 p = 0; p < count; ++p){
        auto& pair = pairs[p];
        PhysxLink* physxLinks[2] = {
            static_cast<PhysxLink*>(pair.actor[0]->userData),
            static_cast<PhysxLink*>(pair.actor[1]->userData)
        };
        if(!physxLinks[0] || !physxLinks[1]){
            continue;
        }
        if(!physxLinks[0]->useSurfaceVelocity && !physxLinks[1]->useSurfaceVelocity){
            continue;
        }

        PxU32 numContacts = pair.contacts.size();
        for(PxU32 i = 0; i < numContacts; ++i){
            PxVec3 normal = pair.contacts.getNormal(i);
            PxVec3 targetVel(0.0f, 0.0f, 0.0f);

            for(int k = 0; k < 2; ++k){
                if(!physxLinks[k]->useSurfaceVelocity){
                    continue;
                }
                Link* link = physxLinks[k]->link;
                Vector3 axis = link->R() * link->a();
                PxVec3 ax(axis.x(), axis.y(), axis.z());
                // The contact normal direction and setTargetVelocity sign convention
                // are both tied to the actor ordering, and their effects cancel out.
                // Therefore the same formula works regardless of which actor index
                // the crawler is assigned to.
                PxVec3 dir = ax.cross(normal);
                if(dir.magnitude() > 1e-5f){
                    dir.normalize();
                    targetVel += dir * static_cast<float>(link->dq_target());
                }
            }

            pair.contacts.setTargetVelocity(i, targetVel);
        }
    }
}

void PhysXSimulatorItem::initialize(ExtensionManager* ext)
{
    ext->itemManager().registerClass<PhysXSimulatorItem, SimulatorItem>(N_("PhysXSimulatorItem"));
    ext->itemManager().addCreationPanel<PhysXSimulatorItem>();
}


PhysXSimulatorItem::PhysXSimulatorItem()
{
    impl = new Impl(this);
    setAllLinkPositionOutputMode(false);
}


PhysXSimulatorItem::Impl::Impl(PhysXSimulatorItem* self)
    : self(self)

{
    initialize();

    gravity << 0.0, 0.0, -DefaultGravityAcceleration;
    sceneOriginShift = Vector3::Zero();
    useTightBounds = false;
    skipConvexMeshCleanup = false;

    isVelocityOutputEnabled = false;
    isAccelerationOutputEnabled = false;
    isErrorOutputEnabled = true;

    isTriangleMeshEnabledForDynamicObjects = false;
    isTriangleMeshPreprocessingEnabled = true;
    sdfSpacing = 0.01;
    sdfSubgridSize = 6;
    sdfBitsPerSubgridPixel = 16;
    numThreadsForSdfConstruction = 4;

    solverType.setSymbol(PxSolverType::ePGS, N_("PGS"));
    solverType.setSymbol(PxSolverType::eTGS, N_("TGS"));
    solverType.select(PxSolverType::eTGS);
    positionIterations = 8;
    velocityIterations = 2;
    linearDamping = 0.0;
    angularDamping = 0.0;
    driveStiffness = "1.0e8";
    driveDamping = "1.0e5";
}


PhysXSimulatorItem::PhysXSimulatorItem(const PhysXSimulatorItem& org)
    : SimulatorItem(org)
{
    impl = new Impl(this, *org.impl);
}


PhysXSimulatorItem::Impl::Impl(PhysXSimulatorItem* self, const Impl& org)
    : self(self)
{
    initialize();

    gravity = org.gravity;
    sceneOriginShift = org.sceneOriginShift;
    numThreads = org.numThreads;
    tolerancesScale = org.tolerancesScale;
    useTightBounds = org.useTightBounds;
    skipConvexMeshCleanup = org.skipConvexMeshCleanup;

    isVelocityOutputEnabled = org.isVelocityOutputEnabled;
    isAccelerationOutputEnabled = org.isAccelerationOutputEnabled;
    isErrorOutputEnabled = org.isErrorOutputEnabled;

    isTriangleMeshEnabledForDynamicObjects = org.isTriangleMeshEnabledForDynamicObjects;
    isTriangleMeshPreprocessingEnabled = org.isTriangleMeshPreprocessingEnabled;
    sdfSpacing = org.sdfSpacing;
    sdfSubgridSize = org.sdfSubgridSize;
    sdfBitsPerSubgridPixel = org.sdfBitsPerSubgridPixel;
    numThreadsForSdfConstruction = org.numThreadsForSdfConstruction;

    solverType = org.solverType;
    positionIterations = org.positionIterations;
    velocityIterations = org.velocityIterations;
    linearDamping = org.linearDamping;
    angularDamping = org.angularDamping;
    driveStiffness = org.driveStiffness;
    driveDamping = org.driveDamping;
}


void PhysXSimulatorItem::Impl::initialize()
{
    physics = nullptr;
    scene = nullptr;
    cpuDispatcher = nullptr;
    filterCallback = nullptr;
    contactModifyCallback = nullptr;
    materialTable = nullptr;
    numThreads = 0;
    mout = MessageOut::master();
}


PhysXSimulatorItem::~PhysXSimulatorItem()
{
    delete impl;
}


PhysXSimulatorItem::Impl::~Impl()
{
    clear();

    if(physics){
        physics->release();
    }
}


void PhysXSimulatorItem::setNumThreads(int n)
{
    if(n >= 0){
        impl->numThreads = n;
    }
}


void PhysXSimulatorItem::setGravity(const Vector3& gravity)
{
    impl->gravity = gravity;
}


const Vector3& PhysXSimulatorItem::gravity() const
{
    return impl->gravity;
}


Vector3 PhysXSimulatorItem::getGravity() const
{
    return impl->gravity;
}


void PhysXSimulatorItem::Impl::clear()
{
    trackSimulator.reset();

    for(auto& pair : materialCache){
        if(pair.second){
            pair.second->release();
        }
    }
    materialCache.clear();
    materialTable = nullptr;

    for(auto& [sgMesh, triMesh] : triangleMeshMap){
        triMesh->release();
    }
    triangleMeshMap.clear();
    for(auto& [sgMesh, triMesh] : triangleMeshWithSdfMap){
        triMesh->release();
    }
    triangleMeshWithSdfMap.clear();
    for(auto& [sgMesh, cMesh] : convexMeshMap){
        cMesh->release();
    }
    convexMeshMap.clear();

    if(scene){
        scene->release();
        scene = nullptr;
    }
    if(cpuDispatcher){
        cpuDispatcher->release();
        cpuDispatcher = nullptr;
    }
    if(filterCallback){
        delete filterCallback;
        filterCallback = nullptr;
    }
    if(contactModifyCallback){
        delete contactModifyCallback;
        contactModifyCallback = nullptr;
    }
}    


Item* PhysXSimulatorItem::doDuplicate() const
{
    return new PhysXSimulatorItem(*this);
}


SimulationBody* PhysXSimulatorItem::createSimulationBody(Body* orgBody)
{
    return new PhysxBody(orgBody->clone(), impl);
}


bool PhysXSimulatorItem::initializeSimulation(const std::vector<SimulationBody*>& simBodies)
{
    return impl->initializeSimulation(simBodies);
}


bool PhysXSimulatorItem::Impl::initializeSimulation(const std::vector<SimulationBody*>& simBodies)
{
    PhysXPlugin::setErrorOutputEnabled(isErrorOutputEnabled);
    mout = isErrorOutputEnabled ? MessageOut::master() : MessageOut::nullout();

    if(!physics){
        physics = PxCreatePhysics(PX_PHYSICS_VERSION, *PhysXPlugin::foundation(), tolerancesScale, false);
        if(!physics){
            mout->putErrorln(_("PxCreatePhysics failed!"));
            return false;
        }
    }

    clear();

    PxSceneDesc sceneDesc(physics->getTolerancesScale());
    sceneDesc.gravity = getPxVec3(gravity);
    sceneDesc.solverType = static_cast<PxSolverType::Enum>(solverType.selectedIndex());
    if(solverType.selectedIndex() == PxSolverType::eTGS){
        sceneDesc.flags |= PxSceneFlag::eENABLE_EXTERNAL_FORCES_EVERY_ITERATION_TGS;
    }
    if(isAccelerationOutputEnabled){
        sceneDesc.flags |= PxSceneFlag::eENABLE_BODY_ACCELERATIONS;
    }
    cpuDispatcher = PxDefaultCpuDispatcherCreate(numThreads);
    sceneDesc.cpuDispatcher = cpuDispatcher;
    filterCallback = new CustomFilterCallback();
    sceneDesc.filterShader = customFilterShader;
    sceneDesc.filterCallback = filterCallback;
    contactModifyCallback = new CrawlerContactModifyCallback();
    sceneDesc.contactModifyCallback = contactModifyCallback;

    scene = physics->createScene(sceneDesc);
    if(!scene){
        mout->putErrorln(_("createScene failed!"));
        return false;
    }

    if(!sceneOriginShift.isZero()){
        scene->shiftOrigin(getPxVec3(sceneOriginShift));
    }

    materialTable = nullptr;
    if(WorldItem* worldItem = self->worldItem()){
        materialTable = worldItem->materialTable();
    }

    timeStep = self->worldTimeStep();

    bool hasNonRootFreeJoints = false;
    for(size_t i = 0; i < simBodies.size(); ++i){
        auto physxBody = static_cast<PhysxBody*>(simBodies[i]);
        physxBody->bodyIndex = static_cast<int>(i) + 1; // 0 means no body (static actor)
        physxBody->createPhysxObjects();
        if(!hasNonRootFreeJoints){
            for(auto& link : physxBody->body()->links()){
                if(link->isFreeJoint() && !link->isRoot()){
                    hasNonRootFreeJoints = true;
                    break;
                }
            }
        }
    }
    if(hasNonRootFreeJoints && !self->isAllLinkPositionOutputMode()){
        bool confirmed = showConfirmDialog(
            _("Confirmation of all link position recording mode"),
            formatR(_("{0}: There is a model that has free-type joints other than the root link "
                      "and all the link positions should be recorded in this case. "
                      "Do you enable the mode to do it?"), self->displayName()));
        if(confirmed){
            self->setAllLinkPositionOutputMode(true);
            self->notifyUpdate();
        }
    }

    referencedVertexIndices.clear();
    referencedVertexIndices.rehash(0);
    referencedVertices.clear();
    referencedVertices.shrink_to_fit();

    // Generate initial track states so that frame 0 has valid track data
    if(trackSimulator){
        trackSimulator->initializeTrackStates();
    }

    return true;
}


bool PhysXSimulatorItem::stepSimulation(const std::vector<SimulationBody*>& activeSimBodies)
{
    return impl->stepSimulation(activeSimBodies);
}


bool PhysXSimulatorItem::Impl::stepSimulation(const std::vector<SimulationBody*>& activeSimBodies)
{
    for(auto& simBody : activeSimBodies){
        auto physxBody = static_cast<PhysxBody*>(simBody);
        physxBody->body()->setVirtualJointForces();
        physxBody->inputControlCommandToPhysx();
        physxBody->addExternalForceToPhysx();
    }

    if(trackSimulator){
        trackSimulator->updateSimulation();
    }

    scene->simulate(timeStep);
    scene->fetchResults(true);

    for(auto& simBody : activeSimBodies){
        auto physxBody = static_cast<PhysxBody*>(simBody);
        physxBody->getKinematicStateFromPhysx();
        if(!physxBody->isSleeping_){
            if(physxBody->hasForceSensors){
                physxBody->updateForceSensors();
            }
            if(physxBody->sensorHelper.hasGyroOrAccelerationSensors()){
                physxBody->sensorHelper.updateGyroAndAccelerationSensors();
            }
        }
    }

    if(trackSimulator){
        trackSimulator->updateTrackStates();
    }

    return true;
}


void PhysXSimulatorItem::doPutProperties(PutPropertyFunction& putProperty)
{
    SimulatorItem::doPutProperties(putProperty);
    impl->doPutProperties(putProperty);
}


void PhysXSimulatorItem::Impl::doPutProperties(PutPropertyFunction& putProperty)
{
    putProperty(_("Gravity"), str(gravity), [&](const string& v){ return toVector3(v, gravity); });
    putProperty(_("Scene origin shift"), str(sceneOriginShift),
                [&](const string& v){ return toVector3(v, sceneOriginShift); });
    putProperty(_("Num threads"), numThreads, changeProperty(numThreads));
    putProperty.decimals(3).min(0.001)
        (_("Tolerances scale length"), tolerancesScale.length, changeProperty(tolerancesScale.length));
    putProperty.decimals(2).min(0.01)
        (_("Tolerances scale speed"), tolerancesScale.speed, changeProperty(tolerancesScale.speed));
    putProperty(_("Convex mesh tight bounds"), useTightBounds, changeProperty(useTightBounds));
    putProperty(_("Skip convex mesh cleanup"), skipConvexMeshCleanup,
                changeProperty(skipConvexMeshCleanup));

    // Dynamic object TriangleMesh option
    putProperty(_("Triangle mesh for dynamic objects"), isTriangleMeshEnabledForDynamicObjects,
                changeProperty(isTriangleMeshEnabledForDynamicObjects));

    // TriangleMesh preprocessing option
    putProperty(_("Triangle mesh preprocessing"), isTriangleMeshPreprocessingEnabled,
                changeProperty(isTriangleMeshPreprocessingEnabled));

    // SDF options (for dynamic objects)
    putProperty.decimals(4)(_("SDF spacing"), sdfSpacing, changeProperty(sdfSpacing));
    putProperty(_("SDF subgrid size"), sdfSubgridSize, changeProperty(sdfSubgridSize));
    putProperty(_("SDF bits per pixel"), sdfBitsPerSubgridPixel,
                changeProperty(sdfBitsPerSubgridPixel));
    putProperty(_("SDF construction threads"), numThreadsForSdfConstruction,
                changeProperty(numThreadsForSdfConstruction));

    putProperty(_("Solver type"), solverType, changeProperty(solverType));
    putProperty.min(1).max(255)(_("Position iterations"), positionIterations, changeProperty(positionIterations));
    putProperty.min(0).max(255)(_("Velocity iterations"), velocityIterations, changeProperty(velocityIterations));
    putProperty.decimals(4).min(0.0)(_("Linear damping"), linearDamping, changeProperty(linearDamping));
    putProperty.decimals(4).min(0.0)(_("Angular damping"), angularDamping, changeProperty(angularDamping));
    putProperty(_("Drive stiffness"), driveStiffness,
                [&](const string& v){ return driveStiffness.setNonNegativeValue(v); });
    putProperty(_("Drive damping"), driveDamping,
                [&](const string& v){ return driveDamping.setNonNegativeValue(v); });

    putProperty(_("Enable velocity output"), isVelocityOutputEnabled, changeProperty(isVelocityOutputEnabled));
    putProperty(_("Enable acceleration output"), isAccelerationOutputEnabled, changeProperty(isAccelerationOutputEnabled));
    putProperty(_("Error output"), isErrorOutputEnabled, changeProperty(isErrorOutputEnabled));
}


bool PhysXSimulatorItem::store(Archive& archive)
{
    SimulatorItem::store(archive);
    impl->store(archive);
    return true;
}


void PhysXSimulatorItem::Impl::store(Archive& archive)
{
    write(archive, "gravity", gravity);
    if(!sceneOriginShift.isZero()){
        write(archive, "scene_origin_shift", sceneOriginShift);
    }
    archive.write("num_threads", numThreads);
    if(tolerancesScale.length != PxTolerancesScale().length){
        archive.write("tolerances_scale_length", tolerancesScale.length);
    }
    if(tolerancesScale.speed != PxTolerancesScale().speed){
        archive.write("tolerances_scale_speed", tolerancesScale.speed);
    }
    if(useTightBounds){
        archive.write("convex_mesh_tight_bounds", useTightBounds);
    }
    if(skipConvexMeshCleanup){
        archive.write("skip_convex_mesh_cleanup", true);
    }
    if(isTriangleMeshEnabledForDynamicObjects){
        archive.write("triangle_mesh_for_dynamic_objects", isTriangleMeshEnabledForDynamicObjects);
    }
    if(!isTriangleMeshPreprocessingEnabled){
        archive.write("triangle_mesh_preprocessing", isTriangleMeshPreprocessingEnabled);
    }
    archive.write("sdf_spacing", sdfSpacing);
    archive.write("sdf_subgrid_size", sdfSubgridSize);
    archive.write("sdf_bits_per_pixel", sdfBitsPerSubgridPixel);
    archive.write("sdf_construction_threads", numThreadsForSdfConstruction);
    if(solverType.selectedIndex() != PxSolverType::eTGS){
        archive.write("solver_type", solverType.selectedSymbol());
    }
    archive.write("position_iterations", positionIterations);
    archive.write("velocity_iterations", velocityIterations);
    if(linearDamping != 0.0){
        archive.write("linear_damping", linearDamping);
    }
    if(angularDamping != 0.0){
        archive.write("angular_damping", angularDamping);
    }
    if(driveStiffness.value() != DefaultDriveStiffness){
        archive.write("drive_stiffness", driveStiffness);
    }
    if(driveDamping.value() != DefaultDriveDamping){
        archive.write("drive_damping", driveDamping);
    }
    if(isVelocityOutputEnabled){
        archive.write("enable_velocity_output", isVelocityOutputEnabled);
    }
    if(isAccelerationOutputEnabled){
        archive.write("enable_acceleration_output", isAccelerationOutputEnabled);
    }
    if(!isErrorOutputEnabled){
        archive.write("error_output", false);
    }
}


bool PhysXSimulatorItem::restore(const Archive& archive)
{
    SimulatorItem::restore(archive);
    impl->restore(archive);
    return true;
}


void PhysXSimulatorItem::Impl::restore(const Archive& archive)
{
    read(archive, "gravity", gravity);
    read(archive, "scene_origin_shift", sceneOriginShift);
    archive.read("num_threads", numThreads);
    archive.read("tolerances_scale_length", tolerancesScale.length);
    archive.read("tolerances_scale_speed", tolerancesScale.speed);
    archive.read("convex_mesh_tight_bounds", useTightBounds);
    archive.read("skip_convex_mesh_cleanup", skipConvexMeshCleanup);
    archive.read("triangle_mesh_for_dynamic_objects", isTriangleMeshEnabledForDynamicObjects);
    archive.read("triangle_mesh_preprocessing", isTriangleMeshPreprocessingEnabled);
    archive.read("sdf_spacing", sdfSpacing);
    archive.read("sdf_subgrid_size", sdfSubgridSize);
    archive.read("sdf_bits_per_pixel", sdfBitsPerSubgridPixel);
    archive.read("sdf_construction_threads", numThreadsForSdfConstruction);
    string solverTypeStr;
    if(archive.read("solver_type", solverTypeStr)){
        solverType.select(solverTypeStr);
    }
    archive.read("position_iterations", positionIterations);
    archive.read("velocity_iterations", velocityIterations);
    archive.read("linear_damping", linearDamping);
    archive.read("angular_damping", angularDamping);
    driveStiffness = archive.get("drive_stiffness", driveStiffness.string());
    driveDamping = archive.get("drive_damping", driveDamping.string());
    archive.read("enable_velocity_output", isVelocityOutputEnabled);
    archive.read("enable_acceleration_output", isAccelerationOutputEnabled);
    archive.read("error_output", isErrorOutputEnabled);
}


PxMaterial* PhysXSimulatorItem::Impl::getOrCreatePxMaterial(int materialId)
{
    auto it = materialCache.find(materialId);
    if(it != materialCache.end()){
        return it->second;
    }

    Material* material = nullptr;
    if(materialTable){
        material = materialTable->material(materialId);
    }

    // Convert to PhysX parameters using sqrt for AGX-compatible combine behavior
    // friction = sqrt(roughness) -> with MULTIPLY mode: sqrt(r1)*sqrt(r2) = sqrt(r1*r2)
    // restitution = sqrt(1-viscosity) -> with MULTIPLY mode: sqrt((1-v1)*(1-v2))
    double roughness = material ? material->roughness() : 0.5;
    double viscosity = material ? material->viscosity() : 0.0;
    double stiffness = material ? material->stiffness() : 0.0;
    double damping = material ? material->damping() : 0.0;

    float friction = static_cast<float>(std::sqrt(roughness));

    PxMaterial* pxMat;
    if(stiffness > 0.0){
        pxMat = physics->createMaterial(
            friction, friction, -static_cast<PxReal>(stiffness));
        if(pxMat){
            pxMat->setDamping(static_cast<PxReal>(damping));
        }
    } else {
        float restitution = static_cast<float>(std::sqrt(std::max(0.0, 1.0 - viscosity)));
        pxMat = physics->createMaterial(friction, friction, restitution);
    }

    if(pxMat){
        //pxMat->setFlag(PxMaterialFlag::eDISABLE_STRONG_FRICTION, true);
        pxMat->setFrictionCombineMode(PxCombineMode::eMULTIPLY);
        pxMat->setRestitutionCombineMode(PxCombineMode::eMULTIPLY);
        materialCache[materialId] = pxMat;
    }
    return pxMat;
}


PhysxBody::PhysxBody(Body* body, PhysXSimulatorItem::Impl* simImpl)
    : SimulationBody(body),
      simImpl(simImpl)
{
    bodyIndex = 0;
    selfCollisionDetectionEnabled = false;
    hasForceSensors = false;
}


PhysxBody::~PhysxBody()
{
    for(auto& joint : extraJoints){
        joint->release();
    }
}


// TODO: Define this function as a function overriding the SimulationBody::initialize?
void PhysxBody::createPhysxObjects()
{
    auto body_ = body();
    selfCollisionDetectionEnabled = bodyItem()->isSelfCollisionDetectionEnabled();
    bodyCollisionLinkFilter.setTargetBody(body_, selfCollisionDetectionEnabled);

    auto& physics = simImpl->physics;
    physxLinks.reserve(body_->numLinks());
    PhysxLink::create(body_->rootLink(), this, nullptr, nullptr);

    sensorHelper.initialize(body(), simImpl->timeStep, simImpl->gravity);
    for(auto& sensor : sensorHelper.forceSensors()){
        int linkIndex = sensor->link()->index();
        if(auto physxLink = dynamic_pointer_cast<PhysxArticulationLink>(physxLinks[linkIndex])){
            PxTransform T_cm = physxLink->rigidBody->getCMassLocalPose();
            PxTransform T_sensor = getPxTransform(sensor->T_local());
            PxTransform sensorFrameInCom = T_cm.transformInv(T_sensor);
            physxLink->bodyArticulation->forceSensorInfos.push_back(
                new ForceSensorInfo(physxLink, sensor, sensorFrameInCom));
            hasForceSensors = true;
        }
    }

    // Create continuous track handlers (linear segment links must be added
    // to the articulation BEFORE adding the articulation to the scene)
    if(!body_->devices<PxContinuousTrack>().empty()){
        if(!simImpl->trackSimulator){
            simImpl->trackSimulator = make_unique<PxContinuousTrackSimulator>();
        }
        simImpl->trackSimulator->setupTrackHandlers(this);
    }

    for(auto& bodyArticulation : bodyArticulations){
        simImpl->scene->addArticulation(*bodyArticulation->articulation);
    }
    for(auto& physxLink : physxLinks){
        if(physxLink->rigidActor && !dynamic_pointer_cast<PhysxArticulationLink>(physxLink)){
            simImpl->scene->addActor(*physxLink->rigidActor);
        }
    }

    std::vector<int> linkIndexToDofIndexMap(physxLinks.size(), -1);
    for(auto& bodyArticulation : bodyArticulations){
        auto articulation = bodyArticulation->articulation;
        articulation->setSolverIterationCounts(
            simImpl->positionIterations, simImpl->velocityIterations);
        bodyArticulation->articulationCache = articulation->createCache();
        articulation->zeroCache(*bodyArticulation->articulationCache); // Is this necessary?
        articulation->updateKinematic(
            PxArticulationKinematicFlag::ePOSITION | PxArticulationKinematicFlag::eVELOCITY);
        bodyArticulation->buildLinkIndexToDofIndexMap(linkIndexToDofIndexMap);
    }
    for(auto& bodyArticulation : bodyArticulations){
        for(auto physxLink : bodyArticulation->physxLinks){
            physxLink->initArticulationIndices(linkIndexToDofIndexMap);
        }
    }

    for(auto& bodyArticulation : bodyArticulations){
        for(auto physxLink : bodyArticulation->physxLinks){
            if(physxLink->useDirectTorqueControl){
                bodyArticulation->hasTorqueControlLinks = true;
                break;
            }
        }
    }

    setExtraJoints();
}


void PhysxBodyArticulation::buildLinkIndexToDofIndexMap(std::vector<int>& linkIndexToDofIndexMap)
{
    PxU32 nbLinks = articulation->getNbLinks();
    std::vector<PxArticulationLink*> artLinks(nbLinks);
    articulation->getLinks(artLinks.data(), nbLinks);
    std::vector<int> dofByArtLinkIndex(nbLinks, -1);
    for(PxU32 i = 0; i < nbLinks; ++i){
        dofByArtLinkIndex[artLinks[i]->getLinkIndex()] = artLinks[i]->getInboundJointDof();
    }
    int dofStart = 0;
    for(PxU32 li = 0; li < nbLinks; ++li){
        int dof = dofByArtLinkIndex[li];
        if(dof > 0){
            dofByArtLinkIndex[li] = dofStart;
            dofStart += dof;
        }
    }
    for(PxU32 i = 0; i < nbLinks; ++i){
        auto physxLink = static_cast<PhysxArticulationLink*>(artLinks[i]->userData);
        if(!physxLink){
            continue; // Skip dummy articulation root links
        }
        int artLinkIndex = artLinks[i]->getLinkIndex();
        linkIndexToDofIndexMap[physxLink->link->index()] = dofByArtLinkIndex[artLinkIndex];
    }
}


void PhysxBody::setExtraJoints()
{
    auto body_ = body();
    auto& physics = simImpl->physics;
    int numExtraJoints = body_->numExtraJoints();

    for(int i = 0; i < numExtraJoints; ++i){
        auto extraJoint = body_->extraJoint(i);
        Link* link0 = extraJoint->link(0);
        Link* link1 = extraJoint->link(1);
        if(!link0 || !link1){
            continue;
        }
        auto physxLink0 = physxLinks[link0->index()];
        auto physxLink1 = physxLinks[link1->index()];
        if(!physxLink0->rigidActor || !physxLink1->rigidActor){
            continue;
        }

        int jointType = extraJoint->type();
        PxTransform T0, T1;
        if(jointType == ExtraJoint::Hinge || jointType == ExtraJoint::Piston){
            // PhysX uses X-axis as the joint axis.
            // Compute rotation that maps UnitX to the specified axis direction.
            Matrix3 Rax = Quaternion::FromTwoVectors(Vector3::UnitX(), extraJoint->axis()).toRotationMatrix();
            Isometry3 T0frame;
            T0frame.linear() = extraJoint->localRotation(0) * Rax;
            T0frame.translation() = extraJoint->localTranslation(0);
            T0 = getPxTransform(T0frame);
            Isometry3 T1frame;
            T1frame.linear() = extraJoint->localRotation(1) * Rax;
            T1frame.translation() = extraJoint->localTranslation(1);
            T1 = getPxTransform(T1frame);
        } else {
            T0 = getPxTransform(extraJoint->localPosition(0));
            T1 = getPxTransform(extraJoint->localPosition(1));
        }

        PxJoint* joint = nullptr;
        switch(jointType){
        case ExtraJoint::Fixed:
            joint = PxFixedJointCreate(
                *physics, physxLink0->rigidActor, T0, physxLink1->rigidActor, T1);
            break;
        case ExtraJoint::Hinge:
            joint = PxRevoluteJointCreate(
                *physics, physxLink0->rigidActor, T0, physxLink1->rigidActor, T1);
            break;
        case ExtraJoint::Ball:
            joint = PxSphericalJointCreate(
                *physics, physxLink0->rigidActor, T0, physxLink1->rigidActor, T1);
            break;
        case ExtraJoint::Piston: {
            auto d6Joint = PxD6JointCreate(
                *physics, physxLink0->rigidActor, T0, physxLink1->rigidActor, T1);
            if(d6Joint){
                d6Joint->setMotion(PxD6Axis::eX, PxD6Motion::eFREE);
                d6Joint->setMotion(PxD6Axis::eTWIST, PxD6Motion::eFREE);
            }
            joint = d6Joint;
            break;
        }
        default:
            break;
        }
        if(joint){
            extraJoints.push_back(joint);
        }
    }
}


void PhysxBody::setKinematicStateToPhysx()
{
    for(auto& physxLink : physxLinks){
        physxLink->setKinematicStateToPhysx();
    }
}


void PhysxBody::inputControlCommandToPhysx()
{
    // Execute control input functions
    for(auto& physxLink : controllablePhysxLinks){
        physxLink->controlInputFunc();
    }

    // Apply torque via cache for JointEffort mode
    for(auto& bodyArticulation : bodyArticulations){
        if(bodyArticulation->hasTorqueControlLinks){
            if(bodyArticulation->isSleeping_){
                bodyArticulation->articulation->wakeUp();
            }
            bodyArticulation->articulation->applyCache(
                *bodyArticulation->articulationCache,
                PxArticulationCacheFlag::eFORCE);
        }
    }

}


void PhysxBody::addExternalForceToPhysx()
{
    for(auto& physxLink : physxLinks){
        physxLink->addExternalForce();
    }
    body()->clearExternalForces();
}


void PhysxBody::getKinematicStateFromPhysx()
{
    isSleeping_ = true;

    for(auto& bodyArticulation : bodyArticulations){
        bodyArticulation->isSleeping_ = bodyArticulation->articulation->isSleeping();
        if(bodyArticulation->isSleeping_){
            continue;
        }
        isSleeping_ = false;
        PxArticulationCacheFlags cacheFlags = PxArticulationCacheFlag::ePOSITION;
        if(simImpl->isVelocityOutputEnabled){
            cacheFlags |= PxArticulationCacheFlag::eVELOCITY;
        }
        bodyArticulation->articulation->copyInternalStateToCache(
            *bodyArticulation->articulationCache, cacheFlags);
        for(auto physxLink : bodyArticulation->physxLinks){
            physxLink->getArticulationKinematicStateFromPhysx();
        }
    }

    for(auto physxLink : rigidDynamicPhysxLinks){
        if(physxLink->rigidDynamic->isSleeping()){
            continue;
        }
        isSleeping_ = false;
        physxLink->getKinematicStateFromPhysx();
    }
}


void PhysxBody::updateForceSensors()
{
    for(auto& bodyArticulation : bodyArticulations){
        if(!bodyArticulation->forceSensorInfos.empty()){
            bodyArticulation->updateForceSensors();
        }
    }
}


void PhysxBodyArticulation::updateForceSensors()
{
    articulation->copyInternalStateToCache(
        *articulationCache,
        PxArticulationCacheFlag::eLINK_INCOMING_JOINT_FORCE);

    for(auto& info : forceSensorInfos){
        int linkIndex = info->physxLink->articulationLinkIndex;
        PxSpatialForce sf = articulationCache->linkIncomingJointForce[linkIndex];
        const PxTransform& T = info->sensorFrameInCom;
        PxVec3 f = T.q.rotate(sf.force);
        PxVec3 t = T.q.rotate(sf.torque) + T.p.cross(f);
        info->orgSensor->wrench() << f.x, f.y, f.z, t.x, t.y, t.z;
    }
}


PhysxLink::PhysxLink(Link* link, PhysxBody* physxBody)
    : physxBody(physxBody),
      link(link)
{
    rigidActor = nullptr;
    rigidBody = nullptr;
    rigidDynamic = nullptr;
    useSurfaceVelocity = false;
}


void PhysxLink::setupRigidBodyProperties()
{
    rigidBody->setMass(link->m());
    PxQuat aq;
    PxVec3 t = PxDiagonalize(getPxMat33(link->I()), aq);
    rigidBody->setCMassLocalPose(PxTransform(getPxVec3(link->c()), aq));
    rigidBody->setMassSpaceInertiaTensor(t);
    rigidBody->setLinearDamping(static_cast<PxReal>(physxBody->simImpl->linearDamping));
    rigidBody->setAngularDamping(static_cast<PxReal>(physxBody->simImpl->angularDamping));
    rigidActor = rigidBody;
}


void PhysxLink::finalizeConstruction()
{
    rigidActor->userData = this;

    if(physxBody->bodyCollisionLinkFilter.checkIfEnabledLinkIndex(link->index())){
        createShape();
    }

    PxU32 nbShapes = rigidActor->getNbShapes();
    if(nbShapes > 0){
        std::vector<PxShape*> shapes(nbShapes);
        rigidActor->getShapes(shapes.data(), nbShapes);
        for(PxU32 i = 0; i < nbShapes; ++i){
            PxFilterData filterData = shapes[i]->getSimulationFilterData();
            if(useSurfaceVelocity){
                filterData.word0 = 1;
            }
            filterData.word1 = static_cast<PxU32>(physxBody->bodyIndex);
            shapes[i]->setSimulationFilterData(filterData);
        }
    }

    physxBody->physxLinks.push_back(this);

    if(controlInputFunc){
        physxBody->controllablePhysxLinks.push_back(this);
    }

    setKinematicStateToPhysx();
}


PhysxLink* PhysxLink::create(
    Link* link, PhysxBody* physxBody, PhysxLink* parent,
    PhysxBodyArticulation* bodyArticulation)
{
    auto& simImpl = physxBody->simImpl;
    auto& physics = simImpl->physics;

    // Determine if this link should be an articulation link
    bool shouldBeArticulation = false;
    PhysxBodyArticulation* effectiveBodyArticulation = bodyArticulation;

    if(link->isStatic()){
        // Static link -> always static rigid body regardless of descendants
        shouldBeArticulation = false;
        effectiveBodyArticulation = nullptr;
    } else if(effectiveBodyArticulation){
        // Already in an articulation -> articulation child
        shouldBeArticulation = true;
    } else if(link->isRoot() || !link->isFreeJoint()){
        // Non-static, not yet in articulation, non-free joint
        if(link->isRoot()){
            if(checkIfArticulationRoot(link)){
                shouldBeArticulation = true;
            }
        } else {
            // Non-root, non-free joint, not in articulation
            shouldBeArticulation = true;
        }
    }
    // else: free joint, not root, not in articulation -> standalone rigid body

    PhysxLink* physxLink;

    if(shouldBeArticulation){
        // If the parent is a static rigid body, create a dummy articulation root
        if(!effectiveBodyArticulation && parent && parent->link->isStatic()){
            auto dummyRoot = new PhysxArticulationLink(
                parent->link, physxBody, nullptr, nullptr, true);
            physxBody->dummyArticulationRoots.push_back(dummyRoot);
            effectiveBodyArticulation = dummyRoot->bodyArticulation;
            parent = dummyRoot;
        }
        auto physxArticulationLink = new PhysxArticulationLink(
            link, physxBody, parent, effectiveBodyArticulation);
        effectiveBodyArticulation = physxArticulationLink->bodyArticulation;
        physxLink = physxArticulationLink;
    } else {
        // Standalone rigid body
        physxLink = new PhysxLink(link, physxBody);
        if(link->isStatic()){
            physxLink->rigidActor = physics->createRigidStatic(getPxTransform(link->T()));
        } else {
            physxLink->rigidDynamic = physics->createRigidDynamic(getPxTransform(link->T()));
            if(physxLink->rigidDynamic){
                physxLink->rigidDynamic->setSolverIterationCounts(
                    simImpl->positionIterations, simImpl->velocityIterations);
            }
            physxLink->rigidBody = physxLink->rigidDynamic;
        }
        if(physxLink->rigidBody){
            physxLink->setupRigidBodyProperties();
        }
        physxLink->finalizeConstruction();

        // Actor will be added to scene later in createPhysxObjects()
        if(physxLink->rigidDynamic){
            physxBody->rigidDynamicPhysxLinks.push_back(physxLink);
        }
    }

    for(Link* child = link->child(); child; child = child->sibling()){
        create(child, physxBody, physxLink, effectiveBodyArticulation);
    }

    return physxLink;
}


PhysxLink::~PhysxLink()
{
    if(rigidActor){
        rigidActor->release();
    }
}


PhysxArticulationLink::PhysxArticulationLink(
    Link* link, PhysxBody* physxBody, PhysxLink* parent,
    PhysxBodyArticulation* bodyArticulationArg,
    bool isDummyRoot)
    : PhysxLink(link, physxBody)
{
    articulationLink = nullptr;
    articulationAxis = PxArticulationAxis::eTWIST;
    joint = nullptr;
    articulationDofIndex = -1;
    articulationLinkIndex = -1;
    useDirectTorqueControl = false;

    auto& simImpl = physxBody->simImpl;
    auto& physics = simImpl->physics;

    if(!bodyArticulationArg){
        // Root of a new articulation
        auto newBodyArticulation = new PhysxBodyArticulation();
        auto articulation = physics->createArticulationReducedCoordinate();
        newBodyArticulation->articulation = articulation;

        if(isDummyRoot){
            newBodyArticulation->hasDummyRoot = true;
            articulation->setArticulationFlag(PxArticulationFlag::eFIX_BASE, true);
        } else if(link->isFixedJoint()){
            articulation->setArticulationFlag(PxArticulationFlag::eFIX_BASE, true);
        }

        articulation->setArticulationFlag(
            PxArticulationFlag::eDISABLE_SELF_COLLISION,
            !physxBody->selfCollisionDetectionEnabled);
        articulation->setArticulationFlag(
            PxArticulationFlag::eDRIVE_LIMITS_ARE_FORCES, true);
        articulationLink = articulation->createLink(nullptr, getPxTransform(link->T()));
        articulation->setRootGlobalPose(getPxTransform(link->T()), false);

        if(isDummyRoot){
            // Minimal mass and inertia (PhysX requires non-zero values)
            articulationLink->setMass(1.0e-6f);
            articulationLink->setCMassLocalPose(PxTransform(PxIdentity));
            articulationLink->setMassSpaceInertiaTensor(PxVec3(1.0e-6f, 1.0e-6f, 1.0e-6f));
            // No shape, no userData (not a real Choreonoid link)
            articulationLink->userData = nullptr;
        }

        bodyArticulation = newBodyArticulation;
        physxBody->bodyArticulations.push_back(newBodyArticulation);
    } else {
        // Child link in an existing articulation
        bodyArticulation = bodyArticulationArg;
        auto parentArtLink = static_cast<PhysxArticulationLink*>(parent);
        articulationLink = bodyArticulation->articulation->createLink(
            parentArtLink->articulationLink, getPxTransform(link->T()));
        initializeArticulationJoint();
    }

    if(!isDummyRoot){
        bodyArticulation->physxLinks.push_back(this);
        rigidBody = articulationLink;
        setupRigidBodyProperties();
        finalizeConstruction();
    } else {
        rigidBody = articulationLink;
    }
}



PhysxArticulationLink::~PhysxArticulationLink()
{
    // Prevent base destructor from releasing the actor;
    // articulation links are owned by the articulation.
    rigidActor = nullptr;
}



void PhysxArticulationLink::initArticulationIndices(const std::vector<int>& linkIndexToDofIndexMap)
{
    articulationLinkIndex = articulationLink->getLinkIndex();
    articulationDofIndex = linkIndexToDofIndexMap[link->index()];
}


void PhysxArticulationLink::initializeArticulationJoint()
{
    joint = articulationLink->getInboundJoint();

    auto jointFrameRot = Quaternion::FromTwoVectors(Vector3::UnitX(), link->jointAxis());
    Isometry3 T0 = link->Tb() * jointFrameRot;
    Isometry3 T1;
    T1.linear() = jointFrameRot.toRotationMatrix();
    T1.translation().setZero();
    joint->setParentPose(getPxTransform(T0));
    joint->setChildPose(getPxTransform(T1));

    if(link->isRevoluteJoint()){
        articulationAxis = PxArticulationAxis::eTWIST;
        if(link->hasJointDisplacementLimits()){
            joint->setJointType(PxArticulationJointType::eREVOLUTE);
            joint->setMotion(articulationAxis, PxArticulationMotion::eLIMITED);
        } else {
            joint->setJointType(PxArticulationJointType::eREVOLUTE_UNWRAPPED);
            joint->setMotion(articulationAxis, PxArticulationMotion::eFREE);
        }

    } else if(link->isPrismaticJoint()){
        joint->setJointType(PxArticulationJointType::ePRISMATIC);
        articulationAxis = PxArticulationAxis::eX;
        if(link->hasJointDisplacementLimits()){
            joint->setMotion(articulationAxis, PxArticulationMotion::eLIMITED);
        } else {
            joint->setMotion(articulationAxis, PxArticulationMotion::eFREE);
        }

    } else if(link->jointType() == Link::PseudoContinuousTrackJoint){
        joint->setJointType(PxArticulationJointType::eFIX);
        useSurfaceVelocity = true;
        controlInputFunc =
            [this]{
                if(link->dq_target() != 0.0 && bodyArticulation->isSleeping_){
                    bodyArticulation->articulation->wakeUp();
                }
            };

    } else { // Fixed joint and others
        joint->setJointType(PxArticulationJointType::eFIX);
    }

    if(link->hasActualJoint()){
        joint->setFrictionCoefficient(0.0f); // Old model. This should always be disabled.
        joint->setFrictionParams(articulationAxis, PxJointFrictionParams(0.0f, 0.0f, 0.0f));
        joint->setArmature(articulationAxis, link->Jm2());
        if(link->hasJointDisplacementLimits()){
            PxArticulationLimit limits;
            limits.low = link->q_lower();
            limits.high = link->q_upper();
            joint->setLimitParams(articulationAxis, limits);
        }

        drive.driveType = PxArticulationDriveType::eNONE;
        drive.stiffness = 0.0f;
        drive.damping = 0.0f;
        drive.maxForce = 0.0f;

        if(link->actuationMode() == Link::JointEffort){
            // Use direct torque control via articulation cache
            drive.driveType = PxArticulationDriveType::eNONE;
            //drive.maxForce = PX_MAX_F32;
            useDirectTorqueControl = true;
            controlInputFunc =
                [this] {
                    double u = link->u();
                    // TODO: cache this property for performance
                    if(link->hasJointEffortLimits()){
                        if(u < link->u_lower()) u = link->u_lower();
                        if(u > link->u_upper()) u = link->u_upper();
                    }
                    bodyArticulation->articulationCache->jointForce[articulationDofIndex] = static_cast<PxReal>(u);
                };

        } else if(link->actuationMode() == Link::JointVelocity){
            drive.driveType = PxArticulationDriveType::eFORCE;
            drive.stiffness = 0.0f;
            drive.damping = static_cast<PxReal>(physxBody->simImpl->driveDamping.value());
            drive.maxForce = link->hasJointEffortLimits()
                ? static_cast<PxReal>(std::max(fabs(link->u_upper()), fabs(link->u_lower())))
                : PX_MAX_F32;
            controlInputFunc =
                [this] {
                    joint->setDriveVelocity(articulationAxis, link->dq_target());
                    bodyArticulation->articulationCache->jointForce[articulationDofIndex] = 0.0f;
                };

        } else if(link->actuationMode() == Link::JointDisplacement){
            drive.driveType = PxArticulationDriveType::eFORCE;
            drive.stiffness = static_cast<PxReal>(physxBody->simImpl->driveStiffness.value());
            drive.damping = static_cast<PxReal>(physxBody->simImpl->driveDamping.value());
            drive.maxForce = link->hasJointEffortLimits()
                ? static_cast<PxReal>(std::max(fabs(link->u_upper()), fabs(link->u_lower())))
                : PX_MAX_F32;
            controlInputFunc =
                [this] {
                    joint->setDriveVelocity(articulationAxis, link->dq_target());
                    joint->setDriveTarget(articulationAxis, link->q_target());
                    bodyArticulation->articulationCache->jointForce[articulationDofIndex] = 0.0f;
                };
        }

        // Read optional drive parameter overrides from Body file
        if(drive.driveType != PxArticulationDriveType::eNONE){
            const Mapping* info = link->info();
            double val;
            if(info->read("drive_stiffness", val)){
                drive.stiffness = static_cast<PxReal>(val);
            }
            if(info->read("drive_damping", val)){
                drive.damping = static_cast<PxReal>(val);
            }
        }

        joint->setDriveParams(articulationAxis, drive);
    }

}


void PhysxLink::createShape()
{
    auto shape = link->collisionShape();
    if(!shape){
        return;
    }

    auto simImpl = physxBody->simImpl;
    PxMaterial* material = simImpl->getOrCreatePxMaterial(link->materialId());
    MeshExtractor* pMeshExtractor = &simImpl->meshExtractor;
    pMeshExtractor->extract(shape, [this, pMeshExtractor, material](){ readMeshNode(pMeshExtractor, material); });
}


void PhysxLink::readMeshNode(MeshExtractor* meshExtractor, PxMaterial* material)
{
    SgMesh* mesh = meshExtractor->currentMesh();
    PxShape* shape = nullptr;

    Vector3 scale = Vector3::Ones();
    std::optional<Vector3> translation;

    if(meshExtractor->isCurrentScaled()){
        Affine3 S = meshExtractor->currentTransformWithoutScaling().inverse()
                  * meshExtractor->currentTransform();
        if(S.linear().isDiagonal()){
            scale = S.linear().diagonal();
            if(!S.translation().isZero()){
                translation = S.translation();
            }
        } else {
            physxBody->simImpl->mout->putWarningln(
                formatR(_("{0} has a non-axis-aligned scaling which is not supported "
                          "in the PhysX plugin. The mesh is skipped."),
                        meshLabel(meshExtractor->currentShape())));
            return;
        }
    }

    if(mesh->primitiveType() != SgMesh::MeshType){
        bool doAddPrimitive = false;
        if(!meshExtractor->isCurrentScaled()){
            doAddPrimitive = true;
        } else if(mesh->primitiveType() == SgMesh::BoxType){
            doAddPrimitive = true;
        } else if(mesh->primitiveType() == SgMesh::SphereType){
            if(scale.x() == scale.y() && scale.x() == scale.z()){
                doAddPrimitive = true;
            }
        } else if(mesh->primitiveType() == SgMesh::CylinderType ||
                  mesh->primitiveType() == SgMesh::CapsuleType){
            if(scale.x() == scale.z()){
                doAddPrimitive = true;
            }
        } else if(mesh->primitiveType() == SgMesh::ConeType){
            if(scale.x() == scale.z()){
                doAddPrimitive = true;
            }
        }
        if(doAddPrimitive){
            shape = createPrimitiveShape(mesh, material, scale, translation);
        }
    }
    if(!shape){
        if(rigidBody){ // Dynamic object
            auto simImpl = physxBody->simImpl;
            if(simImpl->isTriangleMeshEnabledForDynamicObjects){
                shape = createTriangleMeshShapeWithSDF(mesh, material, scale, translation);
            } else {
                shape = createConvexMeshShape(mesh, material, scale, translation);
            }
        } else { // Static object
            shape = createTriangleMeshShape(mesh, material, scale, translation);
        }
    }
    if(shape){
        shape->setFlag(PxShapeFlag::eSCENE_QUERY_SHAPE, false);
        rigidActor->attachShape(*shape);
        shape->release();
    }
}


PxShape* PhysxLink::createPrimitiveShape(
    SgMesh* mesh, PxMaterial* material,
    const Vector3& scale, const std::optional<Vector3>& translation)
{
    auto& simImpl = physxBody->simImpl;
    auto& physics = simImpl->physics;
    PxShape* shape = nullptr;
    bool needsAxisRotation = false;

    switch(mesh->primitiveType()){
    case SgMesh::BoxType: {
        auto box = mesh->primitive<SgMesh::Box>();
        PxVec3 halfExtents(
            box.size.x() * scale.x() / 2.0f,
            box.size.y() * scale.y() / 2.0f,
            box.size.z() * scale.z() / 2.0f);
        shape = physics->createShape(PxBoxGeometry(halfExtents), *material, true);
        break;
    }
    case SgMesh::SphereType: {
        auto sphere = mesh->primitive<SgMesh::Sphere>();
        shape = physics->createShape(
            PxSphereGeometry(sphere.radius * scale.x()), *material, true);
        break;
    }
    case SgMesh::CapsuleType: {
        auto capsule = mesh->primitive<SgMesh::Capsule>();
        shape = physics->createShape(
            PxCapsuleGeometry(capsule.radius * scale.x(), capsule.height / 2.0f * scale.y()),
            *material, true);
        needsAxisRotation = true;
        break;
    }
    case SgMesh::CylinderType: {
        auto cylinder = mesh->primitive<SgMesh::Cylinder>();
        float scaledRadius = cylinder.radius * scale.x();
        float scaledHeight = cylinder.height * scale.y();
        float margin = scaledRadius * 0.0001f;
        PxConvexCore::Cylinder core(
            scaledHeight - margin * 2.0f,
            scaledRadius - margin);
        shape = physics->createShape(PxConvexCoreGeometry(core, margin), *material, true);
        needsAxisRotation = true;
        break;
    }
    case SgMesh::ConeType: {
        auto cone = mesh->primitive<SgMesh::Cone>();
        float scaledRadius = cone.radius * scale.x();
        float scaledHeight = cone.height * scale.y();
        float margin = scaledRadius * 0.0001f;
        PxConvexCore::Cone core(
            scaledHeight - margin * 2.0f,
            scaledRadius - margin);
        shape = physics->createShape(PxConvexCoreGeometry(core, margin), *material, true);
        needsAxisRotation = true;
        break;
    }
    default:
        break;
    }

    if(shape){
        auto& meshExtractor = simImpl->meshExtractor;
        Isometry3 T = meshExtractor.currentTransformWithoutScaling();
        if(translation){
            T *= Translation3(*translation);
        }
        if(needsAxisRotation){
            Matrix3 R;
            R << 0,  -1.0, 0,
                 1.0, 0,   0,
                 0,   0,   1.0;
            T.linear() = T.linear() * R;
        }
        shape->setLocalPose(getPxTransform(T));
    }

    return shape;
}


PxShape* PhysxLink::createTriangleMeshShape(
    SgMesh* srcMesh, PxMaterial* material,
    const Vector3& scale, const std::optional<Vector3>& translation)
{
    PxShape* shape = nullptr;
    auto simImpl = physxBody->simImpl;

    auto it = simImpl->triangleMeshMap.find(srcMesh);
    if(it != simImpl->triangleMeshMap.end()){
        PxTriangleMesh* triMesh = it->second;
        PxMeshScale meshScale(PxVec3(scale.x(), scale.y(), scale.z()));
        shape = simImpl->physics->createShape(
            PxTriangleMeshGeometry(triMesh, meshScale), *material, true);
        Isometry3 T = simImpl->meshExtractor.currentTransformWithoutScaling();
        if(translation){
            T *= Translation3(*translation);
        }
        shape->setLocalPose(getPxTransform(T));
        return shape;
    }

    PxCookingParams params(simImpl->tolerancesScale);

    // When preprocessing is disabled, skip clean mesh and active edges precompute
    if(!simImpl->isTriangleMeshPreprocessingEnabled){
        params.meshPreprocessParams |= PxMeshPreprocessingFlag::eDISABLE_CLEAN_MESH;
        params.meshPreprocessParams |= PxMeshPreprocessingFlag::eDISABLE_ACTIVE_EDGES_PRECOMPUTE;
    }

    PxTriangleMeshDesc meshDesc;
    auto vertices = srcMesh->vertices();
    meshDesc.points.count = vertices->size();
    meshDesc.points.stride = sizeof(PxVec3);
    meshDesc.points.data = vertices->data();
    meshDesc.triangles.count = srcMesh->numTriangles();
    meshDesc.triangles.stride = 3 * sizeof(PxU32);
    meshDesc.triangles.data = srcMesh->triangleVertices().data();

    if(!PxValidateTriangleMesh(params, meshDesc)){
        simImpl->mout->putWarningln(
            formatR(_("{0} is not validated."),
                    meshLabel(simImpl->meshExtractor.currentShape())));
    } else {
        PxTriangleMesh* triMesh = PxCreateTriangleMesh(params, meshDesc, simImpl->physics->getPhysicsInsertionCallback());
        if(triMesh){
            simImpl->triangleMeshMap[srcMesh] = triMesh;
            PxMeshScale meshScale(PxVec3(scale.x(), scale.y(), scale.z()));
            shape = simImpl->physics->createShape(
                PxTriangleMeshGeometry(triMesh, meshScale), *material, true);
            Isometry3 T = simImpl->meshExtractor.currentTransformWithoutScaling();
            if(translation){
                T *= Translation3(*translation);
            }
            shape->setLocalPose(getPxTransform(T));
        }
    }

    return shape;
}


PxShape* PhysxLink::createTriangleMeshShapeWithSDF(
    SgMesh* srcMesh, PxMaterial* material,
    const Vector3& scale, const std::optional<Vector3>& translation)
{
    PxShape* shape = nullptr;
    auto simImpl = physxBody->simImpl;

    auto it = simImpl->triangleMeshWithSdfMap.find(srcMesh);
    if(it != simImpl->triangleMeshWithSdfMap.end()){
        PxTriangleMesh* triMesh = it->second;
        PxMeshScale meshScale(PxVec3(scale.x(), scale.y(), scale.z()));
        shape = simImpl->physics->createShape(
            PxTriangleMeshGeometry(triMesh, meshScale), *material, true);
        Isometry3 T = simImpl->meshExtractor.currentTransformWithoutScaling();
        if(translation){
            T *= Translation3(*translation);
        }
        shape->setLocalPose(getPxTransform(T));
        return shape;
    }

    PxCookingParams params(simImpl->tolerancesScale);

    PxSDFDesc sdfDesc;
    sdfDesc.spacing = static_cast<PxReal>(simImpl->sdfSpacing);
    sdfDesc.subgridSize = simImpl->sdfSubgridSize;
    sdfDesc.bitsPerSubgridPixel =
        static_cast<PxSdfBitsPerSubgridPixel::Enum>(simImpl->sdfBitsPerSubgridPixel);
    sdfDesc.numThreadsForSdfConstruction = simImpl->numThreadsForSdfConstruction;

    PxTriangleMeshDesc meshDesc;
    auto vertices = srcMesh->vertices();
    meshDesc.points.count = vertices->size();
    meshDesc.points.stride = sizeof(PxVec3);
    meshDesc.points.data = vertices->data();
    meshDesc.triangles.count = srcMesh->numTriangles();
    meshDesc.triangles.stride = 3 * sizeof(PxU32);
    meshDesc.triangles.data = srcMesh->triangleVertices().data();
    meshDesc.sdfDesc = &sdfDesc;

    if(!PxValidateTriangleMesh(params, meshDesc)){
        simImpl->mout->putWarningln(
            formatR(_("{0} is not validated."),
                    meshLabel(simImpl->meshExtractor.currentShape())));
    } else {
        PxTriangleMesh* triMesh = PxCreateTriangleMesh(
            params, meshDesc, simImpl->physics->getPhysicsInsertionCallback());
        if(triMesh){
            simImpl->triangleMeshWithSdfMap[srcMesh] = triMesh;
            PxMeshScale meshScale(PxVec3(scale.x(), scale.y(), scale.z()));
            shape = simImpl->physics->createShape(
                PxTriangleMeshGeometry(triMesh, meshScale), *material, true);
            Isometry3 T = simImpl->meshExtractor.currentTransformWithoutScaling();
            if(translation){
                T *= Translation3(*translation);
            }
            shape->setLocalPose(getPxTransform(T));
        }
    }

    return shape;
}


PxShape* PhysxLink::createConvexMeshShape(
    SgMesh* srcMesh, PxMaterial* material,
    const Vector3& scale, const std::optional<Vector3>& translation)
{
    PxShape* shape = nullptr;
    auto simImpl = physxBody->simImpl;

    auto it = simImpl->convexMeshMap.find(srcMesh);
    if(it != simImpl->convexMeshMap.end()){
        PxConvexMesh* cMesh = it->second;
        PxConvexMeshGeometryFlags flags;
        if(simImpl->useTightBounds){
            flags |= PxConvexMeshGeometryFlag::eTIGHT_BOUNDS;
        }
        PxMeshScale meshScale(PxVec3(scale.x(), scale.y(), scale.z()));
        shape = simImpl->physics->createShape(
            PxConvexMeshGeometry(cMesh, meshScale, flags), *material, true);
        Isometry3 T = simImpl->meshExtractor.currentTransformWithoutScaling();
        if(translation){
            T *= Translation3(*translation);
        }
        shape->setLocalPose(getPxTransform(T));
        return shape;
    }

    auto vertices = srcMesh->vertices();

    const void* pointsData;
    PxU32 pointsCount;

    if(!simImpl->skipConvexMeshCleanup && srcMesh->hasTriangles()){
        auto& indices = srcMesh->triangleVertices();
        auto& refIndices = simImpl->referencedVertexIndices;
        auto& refVertices = simImpl->referencedVertices;
        refIndices.clear();
        // Collect vertex indices from valid (non-degenerate) triangles only
        int numTriangles = srcMesh->numTriangles();
        for(int i = 0; i < numTriangles; ++i){
            int i0 = indices[i * 3];
            int i1 = indices[i * 3 + 1];
            int i2 = indices[i * 3 + 2];
            if(i0 != i1 && i1 != i2 && i2 != i0){
                refIndices.insert(i0);
                refIndices.insert(i1);
                refIndices.insert(i2);
            }
        }
        refVertices.clear();
        refVertices.reserve(refIndices.size());
        for(int idx : refIndices){
            refVertices.push_back((*vertices)[idx]);
        }
        pointsCount = refVertices.size();
        pointsData = pointsCount > 0 ? refVertices.front().data() : nullptr;
    } else {
        pointsData = vertices->data();
        pointsCount = vertices->size();
    }

    if(pointsCount == 0){
        simImpl->mout->putWarningln(
            formatR(_("{0} has no valid vertices for creating a convex mesh."),
                    meshLabel(simImpl->meshExtractor.currentShape())));
        return nullptr;
    }

    PxCookingParams params(simImpl->tolerancesScale);
    PxConvexMeshDesc convexDesc;
    convexDesc.points.count = pointsCount;
    convexDesc.points.stride = sizeof(PxVec3);
    convexDesc.points.data = pointsData;
    convexDesc.flags = PxConvexFlag::eCOMPUTE_CONVEX;

    PxDefaultMemoryOutputStream buf;
    if(!PxCookConvexMesh(params, convexDesc, buf)){
        simImpl->mout->putWarningln(
            formatR(_("{0} cannot be converted to a convex mesh."),
                    meshLabel(simImpl->meshExtractor.currentShape())));
    } else {
        PxDefaultMemoryInputData input(buf.getData(), buf.getSize());
        PxConvexMesh* cMesh = simImpl->physics->createConvexMesh(input);
        if(cMesh){
            simImpl->convexMeshMap[srcMesh] = cMesh;
            PxConvexMeshGeometryFlags flags;
            if(simImpl->useTightBounds){
                flags |= PxConvexMeshGeometryFlag::eTIGHT_BOUNDS;
            }
            PxMeshScale meshScale(PxVec3(scale.x(), scale.y(), scale.z()));
            shape = simImpl->physics->createShape(
                PxConvexMeshGeometry(cMesh, meshScale, flags), *material, true);
            Isometry3 T = simImpl->meshExtractor.currentTransformWithoutScaling();
            if(translation){
                T *= Translation3(*translation);
            }
            shape->setLocalPose(getPxTransform(T));
        }
    }

    return shape;
}


void PhysxLink::setKinematicStateToPhysx()
{
    rigidActor->setGlobalPose(getPxTransform(link->T()));

    if(rigidDynamic){
        // Linear velocity of the center of mass
        Vector3 vc = link->v() + link->w().cross(Vector3(link->R() * link->c()));
        rigidDynamic->setLinearVelocity(getPxVec3(vc));
        rigidDynamic->setAngularVelocity(getPxVec3(link->w()));
    }
}


void PhysxArticulationLink::setKinematicStateToPhysx()
{
    // Root link of the articulation (skip if the articulation has a dummy root,
    // since the dummy root is fixed and its pose should not be updated)
    if(!bodyArticulation->hasDummyRoot && bodyArticulation->physxLinks.front() == this){
        auto articulation = bodyArticulation->articulation;
        articulation->setRootGlobalPose(getPxTransform(link->T()), false);
        Vector3 vc = link->v() + link->w().cross(Vector3(link->R() * link->c()));
        articulation->setRootLinearVelocity(getPxVec3(vc), false);
        articulation->setRootAngularVelocity(getPxVec3(link->w()), false);
    }
    if(link->hasActualJoint()){
        joint->setJointPosition(articulationAxis, link->q());
        joint->setJointVelocity(articulationAxis, link->dq());
    }
}


void PhysxLink::getKinematicStateFromPhysx()
{
    if(rigidBody){
        PxTransform T = rigidBody->getGlobalPose();
        PxVec3& p = T.p;
        link->p() << p.x, p.y, p.z;
        PxMat33 R(T.q);
        link->R() <<
            R(0,0), R(0,1), R(0,2),
            R(1,0), R(1,1), R(1,2),
            R(2,0), R(2,1), R(2,2);

        if(physxBody->simImpl->isVelocityOutputEnabled){
            PxVec3 w = rigidBody->getAngularVelocity();
            link->w() << w.x, w.y, w.z;
            PxVec3 vc = rigidBody->getLinearVelocity();
            Vector3 c = link->R() * link->c();
            link->v() = Vector3(vc.x, vc.y, vc.z) - link->w().cross(c);
        }

        if(physxBody->simImpl->isAccelerationOutputEnabled){
            PxVec3 dw = rigidBody->getAngularAcceleration();
            link->dw() << dw.x, dw.y, dw.z;
            PxVec3 dv = rigidBody->getLinearAcceleration();
            Vector3 c = link->R() * link->c();
            link->dv() = Vector3(dv.x, dv.y, dv.z) - link->dw().cross(c);
        }
    }
}


void PhysxArticulationLink::getArticulationKinematicStateFromPhysx()
{
    getKinematicStateFromPhysx();

    if(articulationDofIndex >= 0){
        auto articulationCache = bodyArticulation->articulationCache;
        link->q() = articulationCache->jointPosition[articulationDofIndex];
        if(physxBody->simImpl->isVelocityOutputEnabled){
            link->dq() = articulationCache->jointVelocity[articulationDofIndex];
        }
    }
}


void PhysxLink::addExternalForce()
{
    if(!rigidBody){
        return;
    }

    const Vector3& f = link->f_ext();
    const Vector3& tau = link->tau_ext();

    if(f.isZero() && tau.isZero()){
        return;
    }

    // Apply force at global origin (same approach as AGX)
    PxRigidBodyExt::addForceAtPos(
        *rigidBody,
        getPxVec3(f),
        PxVec3(0.0f, 0.0f, 0.0f),
        PxForceMode::eFORCE);

    // Apply torque directly
    rigidBody->addTorque(getPxVec3(tau), PxForceMode::eFORCE);
}
