#include "AGXObjectFactory.h"

namespace cnoid{

    ////////////////////////////////////////////////////////////
// AGXPseudoContinuousTrackGeometry
void AGXPseudoContinuousTrackGeometry::setAxis(const agx::Vec3f& a)
{
    axis = a;
}

agx::Vec3f AGXPseudoContinuousTrackGeometry::getAxis()
{
    return axis;
}

agx::Vec3f AGXPseudoContinuousTrackGeometry::calculateSurfaceVelocity(const agxCollide::LocalContactPoint & point, size_t index) const
{
    agx::Vec3f dir = axis ^ point.normal();
    dir.normalize();
    agx::Vec3f ret = dir * -1.0 * getSurfaceVelocity().x();
    return ret;
}

////////////////////////////////////////////////////////////
// AGXObjectFactory

agx::RigidBodyRef AGXObjectFactory::createRigidBody(const AGXRigidBodyDesc& desc)
{
    agx::RigidBodyRef rigid = new agx::RigidBody();
    rigid->setMotionControl(desc.control);
    rigid->setVelocity(desc.v);
    rigid->setAngularVelocity(desc.w);
    rigid->setPosition(desc.p);
    rigid->setRotation(desc.R);
    rigid->getMassProperties()->setAutoGenerateMask(0);
    //r->getMassProperties()->setAutoGenerateMask(desc.genflags);
    rigid->getMassProperties()->setMass(desc.m, false);
    rigid->getMassProperties()->setInertiaTensor(desc.I, false);
    rigid->setCmLocalTranslate(desc.c);
    rigid->setName(desc.name);
    return rigid;
}

agxCollide::GeometryRef AGXObjectFactory::createGeometry(const AGXGeometryDesc& desc)
{
    agxCollide::GeometryRef geometry;
    if(desc.isPseudoContinuousTrack){
        AGXPseudoContinuousTrackGeometry* pg = new AGXPseudoContinuousTrackGeometry();
        pg->setAxis(desc.axis);
        pg->setSurfaceVelocity(agx::Vec3f());
        geometry = pg;
    }else{
        geometry = new agxCollide::Geometry();
    }
    geometry->addGroup(desc.selfCollsionGroupName);
    return geometry;
}

agxCollide::ShapeRef AGXObjectFactory::createShape(const AGXShapeDesc& desc)
{
    agxCollide::ShapeRef shape = nullptr;
    if(desc.shapeType == AGXShapeType::AGXCOLLIDE_BOX){
        shape = createShapeBox(static_cast<const AGXBoxDesc&>(desc));
    }else if(desc.shapeType == AGXShapeType::AGXCOLLIDE_SPHERE){
        shape = createShapeSphere(static_cast<const AGXSphereDesc&>(desc));
    }else if(desc.shapeType == AGXShapeType::AGXCOLLIDE_CAPSULE){
        shape = createShapeCapsule(static_cast<const AGXCapsuleDesc&>(desc));
    }else if(desc.shapeType == AGXShapeType::AGXCOLLIDE_CYLINDER){
        shape = createShapeCylinder(static_cast<const AGXCylinderDesc&>(desc));
    }else if(desc.shapeType == AGXShapeType::AGXCOLLIDE_TRIMESH){
        shape = createShapeTrimesh(static_cast<const AGXTrimeshDesc&>(desc));
    }else{}
    return shape;
}

agx::ConstraintRef AGXObjectFactory::createConstraint(const AGXConstraintDesc& desc)
{
    switch(desc.constraintType){
        case AGXConstraintType::AGXHINGE :
            return createConstraintHinge(static_cast<const AGXHingeDesc&>(desc));
            break;
        case AGXConstraintType::AGXLOCKJOINT :
            return createConstraintLockJoint(static_cast<const AGXLockJointDesc&>(desc));
            break;
        case AGXConstraintType::AGXBALLJOINT :
            return createConstraintBallJoint(static_cast<const AGXBallJointDesc&>(desc));
            break;
        default :
            break;
    }
    return nullptr;
}

agxCollide::BoxRef AGXObjectFactory::createShapeBox(const AGXBoxDesc& desc)
{
    return new agxCollide::Box(desc.halfExtents);
}

agxCollide::SphereRef AGXObjectFactory::createShapeSphere(const AGXSphereDesc & desc)
{
    return new agxCollide::Sphere(desc.radius);
}

agxCollide::CapsuleRef AGXObjectFactory::createShapeCapsule(const AGXCapsuleDesc & desc)
{
    return new agxCollide::Capsule(desc.radius, desc.hegiht);
}

agxCollide::CylinderRef AGXObjectFactory::createShapeCylinder(const AGXCylinderDesc & desc)
{
    return new agxCollide::Cylinder(desc.radius, desc.hegiht);
}

agxCollide::MeshRef AGXObjectFactory::createShapeTrimesh(const AGXTrimeshDesc& desc)
{
    return new agxCollide::Trimesh(&desc.vertices, &desc.indices, desc.name, desc.optionsMask, desc.bottomMargin);
}

agx::HingeRef AGXObjectFactory::createConstraintHinge(const AGXHingeDesc& desc)
{
    agx::HingeFrame hingeFrame;
    hingeFrame.setAxis(desc.frameAxis);
    hingeFrame.setCenter(desc.frameCenter);
    agx::HingeRef hinge = new agx::Hinge(hingeFrame, desc.rigidBodyA, desc.rigidBodyB);
    hinge->getMotor1D()->setEnable(desc.isMotorOn);
    return hinge;
}

agx::PrismaticRef AGXObjectFactory::createConstraintPrismatic(const AGXPrismaticDesc & desc)
{
    agx::PrismaticFrame prismaticFrame;
    prismaticFrame.setAxis(desc.frameAxis);
    prismaticFrame.setPoint(desc.framePoint);
    agx::PrismaticRef prismatic = new agx::Prismatic(prismaticFrame, desc.rigidBodyA, desc.rigidBodyB);
    prismatic->getMotor1D()->setEnable(desc.isMotorOn);
    return prismatic;
}

agx::BallJointRef AGXObjectFactory::createConstraintBallJoint(const AGXBallJointDesc & desc)
{
    agx::BallJointFrame ballJointFrame;
    ballJointFrame.setCenter(desc.framePoint);
    return new agx::BallJoint(ballJointFrame, desc.rigidBodyA, desc.rigidBodyB);
}

agx::LockJointRef AGXObjectFactory::createConstraintLockJoint(const AGXLockJointDesc & desc)
{
    return new agx::LockJoint(desc.rigidBodyA, desc.rigidBodyB);
}

}