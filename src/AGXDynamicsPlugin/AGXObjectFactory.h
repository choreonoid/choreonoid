#ifndef CNOID_AGXDYNAMICS_PLUGIN_AGX_OBJECT_FACTORY_H
#define CNOID_AGXDYNAMICS_PLUGIN_AGX_OBJECT_FACTORY_H

#include "AGXInclude.h"
#include "exportdecl.h"

namespace cnoid{

const double DEFAULT_GRAVITY_ACCELERATION = 9.80665;

struct AGXSimulationDesc
{
    AGXSimulationDesc(){
        numThreads = 1;
        timeStep = 0.0167;
        gravity = agx::Vec3(0.0, 0.0, -DEFAULT_GRAVITY_ACCELERATION);
        enableContactReduction = true;
        contactReductionBinResolution = 3;
        contactReductionThreshhold = 12;
        enableAutoSleep = false;
    }
    agx::Int   numThreads;
    agx::Real  timeStep;
    agx::Vec3  gravity;
    agx::Bool  enableContactReduction;
    agx::UInt8 contactReductionBinResolution;
    agx::UInt  contactReductionThreshhold;
    agx::Bool  enableAutoSleep;
};

struct AGXMaterialDesc
{
    AGXMaterialDesc(){
        name = default_name();
        density = 1000;
        youngsModulus = 4.0E8;
        poissonRatio = 0.3;
        viscosity = 0.5;
        damping = 0.075;
        roughness = 0.416667;
        surfaceViscosity = 5E-09;
        adhesionForce = 0.0;
        adhesivOverlap = 0.0;
    }
    static agx::String default_name()
    {
        return "default";
    }
    agx::String name;
    agx::Real density;              // [kg/m^3]
    agx::Real youngsModulus;        // stiffness[Pa]
    agx::Real poissonRatio;

    // Below are overried when ContactMaterials are used.
    agx::Real viscosity;            // relation to restitution. compliace.
    agx::Real damping;              // relax time of penetration
    agx::Real roughness;            // relation to friction
    agx::Real surfaceViscosity;     // wetness
    agx::Real adhesionForce;        // attracive force[N]
    agx::Real adhesivOverlap;       // range[m]
};

enum AGXFrictionModelType
{
    DEFAULT,
    BOX,
    SCALE_BOX,
    ITERATIVE_PROJECTED_CONE
};

struct AGXContactMaterialDesc
{
    AGXContactMaterialDesc(){
        nameA = "default";
        nameB = "default";
        youngsModulus = 2.0E8;
        restitution = 0.5;
        damping = 0.075;
        friction = 0.416667;
        surfaceViscosity = 1.0E-8;
        adhesionForce = 0.0;
        adhesivOverlap = 0.0;
        frictionDirection = agx::ContactMaterial::FrictionDirection::BOTH_PRIMARY_AND_SECONDARY;
        frictionModelType = AGXFrictionModelType::DEFAULT;
        solveType =  agx::FrictionModel::SolveType::SPLIT;
        contactReductionMode = agx::ContactMaterial::ContactReductionMode::REDUCE_GEOMETRY;
        contactReductionBinResolution = 0;  // Use default value from space
    }
    agx::String nameA;
    agx::String nameB;
    agx::Real youngsModulus;        // stiffness[Pa], (m1.ym * m2.ym)/(m1.ym + m2.ym)
    agx::Real restitution;          // 0:perfectly inelastic collision, 1:perfectly elastic collision, sqrt((1-m1.visco) * (1-m2.vico))
    agx::Real damping;              // relax time of penetration(loop count?)
    agx::Real friction;             // sqrt(m1.rough * m2.rough)
    agx::Real surfaceViscosity;     // m1.svisco + m2.svisco
    agx::Real adhesionForce;        // attracive force[N], m1.ad + m2.ad
    agx::Real adhesivOverlap;       // 
    agx::ContactMaterial::FrictionDirection frictionDirection;
    AGXFrictionModelType frictionModelType;
    agx::FrictionModel::SolveType solveType;
    agx::ContactMaterial::ContactReductionMode contactReductionMode;
    agx::UInt8 contactReductionBinResolution;
};

struct AGXRigidBodyDesc
{
    AGXRigidBodyDesc(){}
    agx::RigidBody::MotionControl control = agx::RigidBody::DYNAMICS;
    agx::Vec3 v = agx::Vec3();            // velocity
    agx::Vec3 w = agx::Vec3();            // angular velocity
    agx::Vec3 p = agx::Vec3();            // position(world)
    agx::OrthoMatrix3x3 R = agx::OrthoMatrix3x3(); // rotation(world);
    //agx::Real m = 1.0;                    // mass
    //agx::SPDMatrix3x3 I = agx::SPDMatrix3x3(agx::Vec3(1.0, 1.0, 1.0));    // inertia
    //agx::Real density = -1.0;
    //agx::Vec3 c = agx::Vec3();            // center of mass(local)
    //agx::MassProperties::AutoGenerateFlags genflags = agx::MassProperties::AutoGenerateFlags::AUTO_GENERATE_ALL;        //
    agx::String name;                    // name
    agx::Bool   enableAutoSleep = true;
};

struct AGXGeometryDesc
{
    AGXGeometryDesc(){
        isPseudoContinuousTrack = false;
    };
    bool isPseudoContinuousTrack;
    agx::Vec3f axis;
    agx::Vec3f surfacevel;
    agx::Name selfCollsionGroupName;
};

class AGXPseudoContinuousTrackGeometry : public agxCollide::Geometry
{
public:
    void setAxis(const agx::Vec3f& axis);
    agx::Vec3f getAxis() const;
    virtual agx::Vec3f calculateSurfaceVelocity( const agxCollide::LocalContactPoint& point , size_t index ) const;
private:
    agx::Vec3f m_axis;
};

enum AGXShapeType
{
    NONE = 0,
    AGXCOLLIDE_BOX,
    AGXCOLLIDE_SPHERE,
    AGXCOLLIDE_CAPSULE,
    AGXCOLLIDE_CYLINDER,
    AGXCOLLIDE_PLANE,
    AGXCOLLIDE_LINE,
    AGXCOLLIDE_MESH,
    AGXCOLLIDE_HEIGHTFIELD,
    AGXCOLLIDE_CONVEX,
    AGXCOLLIDE_TRIMESH,
    AGXCOLLIDE_TERRAIN
};

struct AGXShapeDesc
{
    AGXShapeDesc(AGXShapeType st) : shapeType(st){};
    const AGXShapeType shapeType = AGXShapeType::NONE;
};

struct AGXBoxDesc : public AGXShapeDesc
{
    AGXBoxDesc() : AGXShapeDesc(AGXShapeType::AGXCOLLIDE_BOX){}
    agx::Vec3 halfExtents;
};

struct AGXSphereDesc : public AGXShapeDesc
{
    AGXSphereDesc() : AGXShapeDesc(AGXShapeType::AGXCOLLIDE_SPHERE){}
    agx::Real radius;
};

struct AGXCapsuleDesc : public AGXShapeDesc
{
    AGXCapsuleDesc() : AGXShapeDesc(AGXShapeType::AGXCOLLIDE_CAPSULE){}
    agx::Real radius;
    agx::Real height;
};

struct AGXCylinderDesc : public AGXShapeDesc
{
    AGXCylinderDesc() : AGXShapeDesc(AGXShapeType::AGXCOLLIDE_CYLINDER){}
    agx::Real radius;
    agx::Real height;
};

struct AGXTrimeshDesc : public AGXShapeDesc
{
    AGXTrimeshDesc() : AGXShapeDesc(AGXShapeType::AGXCOLLIDE_TRIMESH)
    {
        name = "empty";
        optionsMask = 0;
        bottomMargin = 0;
    }
    agx::Vec3Vector vertices;
    agx::UInt32Vector indices;
    const char* name;
    uint32_t optionsMask;     // agxCollide::Trimesh::TrimeshOptionsFlags
    agx::Real bottomMargin;
};

enum AGXConstraintType
{
    AGXHINGE,
    AGXLOCKJOINT,
    AGXPRISMATIC,
    AGXBALLJOINT,
    AGXPLANEJOINT
};

struct AGXConstraintDesc
{
    AGXConstraintDesc(AGXConstraintType type) : constraintType(type){}
    const AGXConstraintType constraintType;
    agx::RigidBodyRef rigidBodyA;
    agx::RigidBodyRef rigidBodyB;
};

struct AGXElementaryConstraint {
    AGXElementaryConstraint(){
        enable = false;
        compliance = 1e-08;
        damping = 0.0333333;
        forceRange = agx::RangeReal(agx::Infinity);
    }
    agx::Bool enable;
    agx::Real compliance;
    agx::Real damping;
    agx::RangeReal forceRange;
};

struct AGXMotor1DDesc : public AGXElementaryConstraint{
    AGXMotor1DDesc(){
        enableLock = false;
        enableLockAtZeroSpeed = false;
    }
    agx::Bool enableLock;
    agx::Bool enableLockAtZeroSpeed;
};

struct AGXLock1DDesc : public AGXElementaryConstraint{
    AGXLock1DDesc(){}
};

struct AGXRange1DDesc : public AGXElementaryConstraint{
    AGXRange1DDesc() {
        range = agx::RangeReal(agx::Infinity);
    } 
    agx::RangeReal range;
};

struct AGXHingeDesc : public AGXConstraintDesc
{
    AGXHingeDesc(): AGXConstraintDesc(AGXConstraintType::AGXHINGE){
    }
    agx::Vec3 frameAxis;
    agx::Vec3 frameCenter;
    AGXMotor1DDesc motor;
    AGXLock1DDesc  lock;
    AGXRange1DDesc range;
};

struct AGXPrismaticDesc : public AGXConstraintDesc
{
    AGXPrismaticDesc() : AGXConstraintDesc(AGXConstraintType::AGXPRISMATIC){
    }
    agx::Vec3 frameAxis;
    agx::Vec3 framePoint;
    AGXMotor1DDesc motor;
    AGXLock1DDesc  lock;
    AGXRange1DDesc range;
};

struct AGXLockJointDesc : public AGXConstraintDesc
{
    AGXLockJointDesc() : AGXConstraintDesc(AGXConstraintType::AGXLOCKJOINT){}
};

struct AGXBallJointDesc : public AGXConstraintDesc
{
    AGXBallJointDesc() : AGXConstraintDesc(AGXConstraintType::AGXBALLJOINT){}
    agx::Vec3 framePoint;
};

struct AGXPlaneJointDesc : public AGXConstraintDesc
{
    AGXPlaneJointDesc() : AGXConstraintDesc(AGXConstraintType::AGXPLANEJOINT){}
    agx::FrameRef frameA;
    agx::FrameRef frameB;
};

struct AGXVehicleTrackWheelDesc{
    AGXVehicleTrackWheelDesc(){
        model = agxVehicle::TrackWheel::Model::SPROCKET;
        radius = 1.0;
        rigidbody = nullptr;
        rbRelTransform = agx::AffineMatrix4x4();
    }
    agxVehicle::TrackWheel::Model model;
    agx::Real radius;
    agx::RigidBodyRef rigidbody;
    agx::AffineMatrix4x4 rbRelTransform;
};

struct AGXVehicleTrackDesc{
    AGXVehicleTrackDesc() {
        numberOfNodes = 50;
        nodeThickness = 0.075;
        nodeWidth = 0.6;
        nodeDistanceTension = 5.0E-3;
        nodeThickerThickness = 0.09;
        useThickerNodeEvery = 0;
        hingeCompliance = 1.0E-10;
        hingeDamping = 0.0333;
        minStabilizingHingeNormalForce = 100;
        stabilizingHingeFrictionParameter = 1.5;
        enableMerge = false;
        numNodesPerMergeSegment = 3;
        contactReduction = agxVehicle::TrackInternalMergeProperties::ContactReduction::MINIMAL;
        enableLockToReachMergeCondition = true;
        lockToReachMergeConditionCompliance = 1.0E-11;
        lockToReachMergeConditionDamping = 3/ 60;
        maxAngleMergeCondition = 1.0E-5;
        trackWheelRefs.clear();
    }

    agx::UInt numberOfNodes;           // Total number of nodes in the track.
    agx::Real nodeThickness;           // Thickness of each node in the track.
    agx::Real nodeWidth;               // Width of each node in the track.
    agx::Real nodeDistanceTension;      // The calculated node length is close to ideal, meaning close to zero tension
                                        // in the tracks if they were simulated without gravity. This distance is an offset
                                        // how much closer each node will be to each other, resulting in a given initial tension.
    agx::Real nodeThickerThickness;
    agx::UInt useThickerNodeEvery;
    agx::Real hingeCompliance;
    agx::Real hingeDamping;
    agx::Real minStabilizingHingeNormalForce;
    agx::Real stabilizingHingeFrictionParameter;
    agx::Bool enableMerge;
    agx::UInt numNodesPerMergeSegment;
    agxVehicle::TrackInternalMergeProperties::ContactReduction contactReduction;
    agx::Bool enableLockToReachMergeCondition;
    agx::Real lockToReachMergeConditionCompliance;
    agx::Real lockToReachMergeConditionDamping;
    agx::Real maxAngleMergeCondition;
    std::vector<agxVehicle::TrackWheelRef> trackWheelRefs;
};

class CNOID_EXPORT AGXObjectFactory
{
public:
    static bool checkModuleEnalbled(const char* name);
    static agxSDK::SimulationRef createSimulation(const AGXSimulationDesc& desc);
    static agx::MaterialRef createMaterial(const AGXMaterialDesc& desc);
    static agx::ContactMaterialRef createContactMaterial(agx::Material* const matA, agx::Material* const matB, const AGXContactMaterialDesc& desc);
    static agx::ContactMaterialRef createContactMaterial(const AGXContactMaterialDesc& desc, agxSDK::MaterialManager* const mgr);
    static agx::RigidBodyRef createRigidBody(const AGXRigidBodyDesc& desc);
    static agxCollide::GeometryRef createGeometry(const AGXGeometryDesc& desc);
    static agxCollide::ShapeRef createShape(const AGXShapeDesc& desc);
    static agx::ConstraintRef createConstraint(const AGXConstraintDesc& desc);
    static agx::FrameRef createFrame();
    static agxSDK::AssemblyRef createAssembly();
private:
    AGXObjectFactory();
    static agx::Bool setContactMaterialParam(agx::ContactMaterial* const cm, const AGXContactMaterialDesc& desc);
    static agxCollide::BoxRef createShapeBox(const AGXBoxDesc& desc);
    static agxCollide::SphereRef createShapeSphere(const AGXSphereDesc& desc);
    static agxCollide::CapsuleRef createShapeCapsule(const AGXCapsuleDesc& desc);
    static agxCollide::CylinderRef createShapeCylinder(const AGXCylinderDesc& desc);
    static agxCollide::MeshRef createShapeTrimesh(const AGXTrimeshDesc& desc);
public:
    static agx::HingeRef createConstraintHinge(const AGXHingeDesc& desc);
    static agx::LockJointRef createConstraintLockJoint(const AGXLockJointDesc& desc);
    static agx::PrismaticRef createConstraintPrismatic(const AGXPrismaticDesc& desc);
    static agx::BallJointRef createConstraintBallJoint(const AGXBallJointDesc& desc);
    static agx::PlaneJointRef createConstraintPlaneJoint(const AGXPlaneJointDesc& desc);
    static agxVehicle::TrackWheelRef createVehicleTrackWheel(const AGXVehicleTrackWheelDesc& desc);
    static agxVehicle::TrackRef createVehicleTrack(const AGXVehicleTrackDesc& desc);
};

}

#endif