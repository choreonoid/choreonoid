#ifndef CNOID_PHYSX_PLUGIN_PHYSX_SIMULATOR_ITEM_IMPL_H
#define CNOID_PHYSX_PLUGIN_PHYSX_SIMULATOR_ITEM_IMPL_H

#include "PhysXSimulatorItem.h"
#include <cnoid/BasicSensorSimulationHelper>
#include <cnoid/BodyCollisionLinkFilter>
#include <cnoid/MaterialTable>
#include <cnoid/ForceSensor>
#include <cnoid/SceneDrawables>
#include <cnoid/MeshExtractor>
#include <cnoid/FloatingNumberString>
#include <cnoid/Selection>
#include <PxPhysicsAPI.h>
#include <vector>
#include <map>
#include <unordered_map>
#include <unordered_set>
#include <functional>
#include <memory>
#include <optional>

namespace cnoid {

class PhysxLink;
class PhysxArticulationLink;
class PhysxBody;
class PhysxBodyArticulation;
class PxContinuousTrackSimulator;

typedef ref_ptr<PhysxLink> PhysxLinkPtr;
typedef ref_ptr<PhysxBodyArticulation> PhysxBodyArticulationPtr;
typedef ref_ptr<PhysxBody> PhysxBodyPtr;

class CrawlerContactModifyCallback : public physx::PxContactModifyCallback
{
public:
    void onContactModify(physx::PxContactModifyPair* const pairs, physx::PxU32 count) override;
};

class CustomFilterCallback : public physx::PxSimulationFilterCallback
{
public:
    virtual physx::PxFilterFlags pairFound(
        physx::PxU64 pairID,
        physx::PxFilterObjectAttributes attributes0, physx::PxFilterData filterData0,
        const physx::PxActor* a0, const physx::PxShape* s0,
        physx::PxFilterObjectAttributes attributes1, physx::PxFilterData filterData1,
        const physx::PxActor* a1, const physx::PxShape* s1,
        physx::PxPairFlags& pairFlags) override;

    virtual void pairLost(
        physx::PxU64 pairID,
        physx::PxFilterObjectAttributes attributes0, physx::PxFilterData filterData0,
        physx::PxFilterObjectAttributes attributes1, physx::PxFilterData filterData1,
        bool objectRemoved) override
    {
    }

    virtual bool statusChange(physx::PxU64& pairID, physx::PxPairFlags& pairFlags,
                              physx::PxFilterFlags& filterFlags) override
    {
        return false;
    }
};


struct ForceSensorInfo : public Referenced
{
    PhysxArticulationLink* physxLink;
    ForceSensorPtr orgSensor;
    physx::PxTransform sensorFrameInCom;
    ForceSensorInfo(PhysxArticulationLink* physxLink, ForceSensor* orgSensor,
                    const physx::PxTransform& sensorFrameInCom)
        : physxLink(physxLink), orgSensor(orgSensor), sensorFrameInCom(sensorFrameInCom)
    { }
};


class PhysxBodyArticulation : public Referenced
{
public:
    physx::PxArticulationReducedCoordinate* articulation;
    physx::PxArticulationCache* articulationCache;
    std::vector<PhysxArticulationLink*> physxLinks;
    std::vector<ref_ptr<ForceSensorInfo>> forceSensorInfos;
    bool hasDummyRoot;
    bool hasTorqueControlLinks;
    bool isSleeping_;

    PhysxBodyArticulation()
        : articulation(nullptr),
          articulationCache(nullptr),
          hasDummyRoot(false),
          hasTorqueControlLinks(false),
          isSleeping_(false)
    { }

    ~PhysxBodyArticulation()
    {
        if(articulationCache){
            articulationCache->release();
        }
        if(articulation){
            articulation->release();
        }
    }

    bool isSleeping() const
    {
        return articulation->isSleeping();
    }

    void buildLinkIndexToDofIndexMap(std::vector<int>& linkIndexToDofIndexMap);
    void updateForceSensors();
};


class PhysxBody : public SimulationBody
{
public:
    PhysXSimulatorItem::Impl* simImpl;
    std::vector<PhysxLinkPtr> physxLinks;
    std::vector<PhysxLink*> controllablePhysxLinks;
    std::vector<PhysxLink*> rigidDynamicPhysxLinks;
    std::vector<PhysxLinkPtr> dummyArticulationRoots;
    // bodyArticulations must be declared after physxLinks and dummyArticulationRoots
    // so that links are destroyed before articulations during destruction (reverse order).
    std::vector<PhysxBodyArticulationPtr> bodyArticulations;
    int bodyIndex;
    bool selfCollisionDetectionEnabled;
    bool hasForceSensors;
    bool isSleeping_;
    BasicSensorSimulationHelper sensorHelper;
    std::vector<physx::PxJoint*> extraJoints;
    BodyCollisionLinkFilter bodyCollisionLinkFilter;

    PhysxBody(Body* body, PhysXSimulatorItem::Impl* simImpl);
    ~PhysxBody();
    void createPhysxObjects();
    void setExtraJoints();
    void setKinematicStateToPhysx();
    void inputControlCommandToPhysx();
    void addExternalForceToPhysx();
    void getKinematicStateFromPhysx();
    void updateForceSensors();
};


class PhysxLink : public Referenced
{
public:
    PhysxBody* physxBody;
    Link* link;
    physx::PxRigidActor* rigidActor;
    physx::PxRigidBody* rigidBody;
    physx::PxRigidDynamic* rigidDynamic;
    std::function<void()> controlInputFunc;
    bool useSurfaceVelocity;

    static PhysxLink* create(
        Link* link, PhysxBody* physxBody, PhysxLink* parent,
        PhysxBodyArticulation* bodyArticulation);

    virtual ~PhysxLink();
    virtual void setKinematicStateToPhysx();
    void getKinematicStateFromPhysx();
    void addExternalForce();
    void createShape();
    void readMeshNode(MeshExtractor* meshExtractor, physx::PxMaterial* material);
    physx::PxShape* createPrimitiveShape(
        SgMesh* mesh, physx::PxMaterial* material,
        const Vector3& scale, const std::optional<Vector3>& translation);
    physx::PxShape* createTriangleMeshShape(
        SgMesh* srcMesh, physx::PxMaterial* material,
        const Vector3& scale, const std::optional<Vector3>& translation);
    physx::PxShape* createTriangleMeshShapeWithSDF(
        SgMesh* srcMesh, physx::PxMaterial* material,
        const Vector3& scale, const std::optional<Vector3>& translation);
    physx::PxShape* createConvexMeshShape(
        SgMesh* srcMesh, physx::PxMaterial* material,
        const Vector3& scale, const std::optional<Vector3>& translation);

protected:
    PhysxLink(Link* link, PhysxBody* physxBody);
    void setupRigidBodyProperties();
    void finalizeConstruction();
};


class PhysxArticulationLink : public PhysxLink
{
public:
    PhysxBodyArticulation* bodyArticulation;
    physx::PxArticulationLink* articulationLink;
    physx::PxArticulationAxis::Enum articulationAxis;
    physx::PxArticulationJointReducedCoordinate* joint;
    physx::PxArticulationDrive drive;
    int articulationDofIndex;
    int articulationLinkIndex;
    bool useDirectTorqueControl;

    PhysxArticulationLink(
        Link* link, PhysxBody* physxBody, PhysxLink* parent,
        PhysxBodyArticulation* bodyArticulation,
        bool isDummyRoot = false);
    ~PhysxArticulationLink() override;

    virtual void setKinematicStateToPhysx() override;
    void getArticulationKinematicStateFromPhysx();

    void initializeArticulationJoint();
    void initArticulationIndices(const std::vector<int>& linkIndexToDofIndexMap);
};


class PhysXSimulatorItem::Impl
{
public:
    PhysXSimulatorItem* self;

    physx::PxPhysics* physics;
    physx::PxScene* scene;
    physx::PxDefaultCpuDispatcher* cpuDispatcher;
    CustomFilterCallback* filterCallback;
    CrawlerContactModifyCallback* contactModifyCallback;

    MaterialTable* materialTable;
    std::map<int, physx::PxMaterial*> materialCache;
    std::unordered_map<SgMesh*, physx::PxTriangleMesh*> triangleMeshMap;
    std::unordered_map<SgMesh*, physx::PxTriangleMesh*> triangleMeshWithSdfMap;
    std::unordered_map<SgMesh*, physx::PxConvexMesh*> convexMeshMap;

    Vector3 gravity;
    Vector3 sceneOriginShift;
    double timeStep;
    int numThreads;

    physx::PxTolerancesScale tolerancesScale;
    bool useTightBounds;
    bool skipConvexMeshCleanup;

    // Working buffers for createConvexMeshShape (reused across calls)
    std::unordered_set<int> referencedVertexIndices;
    std::vector<Vector3f> referencedVertices;

    // Enable TriangleMesh with SDF for dynamic objects
    bool isTriangleMeshEnabledForDynamicObjects;

    // TriangleMesh cooking preprocessing option
    bool isTriangleMeshPreprocessingEnabled;

    // SDF options (for dynamic objects when enabled)
    double sdfSpacing;
    int sdfSubgridSize;
    int sdfBitsPerSubgridPixel;
    int numThreadsForSdfConstruction;

    bool isVelocityOutputEnabled;
    bool isAccelerationOutputEnabled;
    bool isErrorOutputEnabled;

    // Solver options
    Selection solverType;
    int positionIterations;
    int velocityIterations;
    double linearDamping;
    double angularDamping;
    FloatingNumberString driveStiffness;
    FloatingNumberString driveDamping;

    MeshExtractor meshExtractor;
    MessageOut* mout;

    std::unique_ptr<PxContinuousTrackSimulator> trackSimulator;

    Impl(PhysXSimulatorItem* self);
    Impl(PhysXSimulatorItem* self, const Impl& org);
    ~Impl();
    void initialize();
    void clear();
    bool initializeSimulation(const std::vector<SimulationBody*>& simBodies);
    bool stepSimulation(const std::vector<SimulationBody*>& activeSimBodies);
    void doPutProperties(PutPropertyFunction& putProperty);
    void store(Archive& archive);
    void restore(const Archive& archive);
    physx::PxMaterial* getOrCreatePxMaterial(int materialId);
};

}

#endif
