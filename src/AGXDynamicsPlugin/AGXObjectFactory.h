#ifndef CNOID_AGXDYNAMICS_PLUGIN_AGX_OBJECT_FACTORY_H
#define CNOID_AGXDYNAMICS_PLUGIN_AGX_OBJECT_FACTORY_H

#include "AGXInclude.h"

namespace cnoid{
struct AGXRigidBodyDesc
{
    AGXRigidBodyDesc(){}
    agx::RigidBody::MotionControl control = agx::RigidBody::DYNAMICS;
    agx::Vec3 v = agx::Vec3();            // velocity
    agx::Vec3 w = agx::Vec3();            // angular velocity
    agx::Vec3 p = agx::Vec3();            // position(world)
    agx::OrthoMatrix3x3 R = agx::OrthoMatrix3x3(); // rotation(world);
    agx::Real m = 1.0;                    // mass
    agx::SPDMatrix3x3 I = agx::SPDMatrix3x3(agx::Vec3(1.0, 1.0, 1.0));    // inertia
    agx::Vec3 c = agx::Vec3();            // center of mass(local)
    agx::MassProperties::AutoGenerateFlags genflags = agx::MassProperties::AutoGenerateFlags::AUTO_GENERATE_ALL;        //
    agx::String name;                    // name
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
    void setAxis(const agx::Vec3f& a);
    agx::Vec3f getAxis();
    virtual agx::Vec3f calculateSurfaceVelocity( const agxCollide::LocalContactPoint& point , size_t index ) const;
private:
    agx::Vec3f axis;
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
    AGXShapeDesc(){};
    AGXShapeType shapeType = AGXShapeType::NONE;
};

struct AGXBoxDesc : public AGXShapeDesc
{
    AGXBoxDesc()
    {
        shapeType = AGXShapeType::AGXCOLLIDE_BOX;
    }
    agx::Vec3 halfExtents;
};

struct AGXSphereDesc : public AGXShapeDesc
{
    AGXSphereDesc()
    {
        shapeType = AGXShapeType::AGXCOLLIDE_SPHERE;
    }
    agx::Real radius;
};

struct AGXCapsuleDesc : public AGXShapeDesc
{
    AGXCapsuleDesc()
    {
        shapeType = AGXShapeType::AGXCOLLIDE_CAPSULE;
    }
    agx::Real radius;
    agx::Real hegiht;
};

struct AGXCylinderDesc : public AGXShapeDesc
{
    AGXCylinderDesc()
    {
        shapeType = AGXShapeType::AGXCOLLIDE_CYLINDER;
    }
    agx::Real radius;
    agx::Real hegiht;
};

struct AGXTrimeshDesc : public AGXShapeDesc
{
    AGXTrimeshDesc()
    {
        shapeType = AGXShapeType::AGXCOLLIDE_TRIMESH;
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
    AGXBALLJOINT
};

struct AGXConstraintDesc
{
    AGXConstraintDesc(AGXConstraintType type) : constraintType(type){}
    const AGXConstraintType constraintType;
    agx::RigidBodyRef rigidBodyA;
    agx::RigidBodyRef rigidBodyB;
};

struct AGXHingeDesc : public AGXConstraintDesc
{
    AGXHingeDesc(): AGXConstraintDesc(AGXConstraintType::AGXHINGE){
        isMotorOn = false;
    }
    agx::Vec3 frameAxis;
    agx::Vec3 frameCenter;
    agx::Bool isMotorOn;
};

struct AGXPrismaticDesc : public AGXConstraintDesc
{
    AGXPrismaticDesc() : AGXConstraintDesc(AGXConstraintType::AGXPRISMATIC){
        isMotorOn = false;
    }
    agx::Vec3 frameAxis;
    agx::Vec3 framePoint;
    agx::Bool isMotorOn;
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

class AGXObjectFactory
{
public:
    //AGXObjectFactory();
    static agx::RigidBodyRef createRigidBody(const AGXRigidBodyDesc& desc);
    static agxCollide::GeometryRef createGeometry(const AGXGeometryDesc& desc);
    static agxCollide::ShapeRef createShape(const AGXShapeDesc& desc);
    static agx::ConstraintRef createConstraint(const AGXConstraintDesc& desc);
private:
    AGXObjectFactory();
    static agxCollide::BoxRef createShapeBox(const AGXBoxDesc& desc);
    static agxCollide::SphereRef createShapeSphere(const AGXSphereDesc& desc);
    static agxCollide::CapsuleRef createShapeCapsule(const AGXCapsuleDesc& desc);
    static agxCollide::CylinderRef createShapeCylinder(const AGXCylinderDesc& desc);
    static agxCollide::MeshRef createShapeTrimesh(const AGXTrimeshDesc& desc);
    static agx::HingeRef createConstraintHinge(const AGXHingeDesc& desc);
    static agx::LockJointRef createConstraintLockJoint(const AGXLockJointDesc& desc);
    static agx::PrismaticRef createConstraintPrismatic(const AGXPrismaticDesc& desc);
    static agx::BallJointRef createConstraintBallJoint(const AGXBallJointDesc& desc);
};

}

#endif