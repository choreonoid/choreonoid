#include "AGXLinkBody.h"

namespace cnoid{

//    ////////////////////////////////////////////////////////////
//// AGXPseudoContinuousTrackGeometry
//void AGXPseudoContinuousTrackGeometry::setAxis(const agx::Vec3f& a)
//{
//    axis = a;
//}
//
//agx::Vec3f AGXPseudoContinuousTrackGeometry::getAxis()
//{
//    return axis;
//}
//
//agx::Vec3f AGXPseudoContinuousTrackGeometry::calculateSurfaceVelocity(const agxCollide::LocalContactPoint & point, size_t index) const
//{
//    agx::Vec3f dir = axis ^ point.normal();
//    dir.normalize();
//    agx::Vec3f ret = dir * -1.0 * getSurfaceVelocity().x();
//    return ret;
//}

////////////////////////////////////////////////////////////
// AGXLinkBody
AGXLinkBody::AGXLinkBody(){}

agx::RigidBodyRef AGXLinkBody::getRigidBody()
{
    return _rigid;
}
agxCollide::GeometryRef AGXLinkBody::getGeometry()
{
    return _geometry;
}
agx::ConstraintRef AGXLinkBody::getConstraint()
{
    return constraint;
}

void AGXLinkBody::createRigidBody(AGXRigidBodyDesc desc)
{
    agx::RigidBodyRef r = new agx::RigidBody();
    r->setMotionControl(desc.control);
    r->setVelocity(desc.v);
    r->setAngularVelocity(desc.w);
    r->setPosition(desc.p);
    r->setRotation(desc.R);
    r->getMassProperties()->setAutoGenerateMask(0);
    //r->getMassProperties()->setAutoGenerateMask(desc.genflags);
    r->getMassProperties()->setMass(desc.m, false);
    r->getMassProperties()->setInertiaTensor(desc.I, false);
    r->setCmLocalTranslate(desc.c);
    r->setName(desc.name);
    _rigid = r;
}
void AGXLinkBody::createGeometry(AGXGeometryDesc desc)
{
    agxCollide::GeometryRef g;
    if(desc.isPseudoContinuousTrack){
        AGXPseudoContinuousTrackGeometry* pg = new AGXPseudoContinuousTrackGeometry();
        pg->setAxis(desc.axis);
        pg->setSurfaceVelocity(agx::Vec3f());
        g = pg;
    }else{
        g = new agxCollide::Geometry();
    }
    g->addGroup(desc.selfCollsionGroupName);
    getRigidBody()->add(g);
    _geometry = g;
}

void AGXLinkBody::createShape(const AGXShapeDesc& desc, const agx::AffineMatrix4x4& af)
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
    getGeometry()->add(shape, af);
}

void AGXLinkBody::createConstraint(const AGXConstraintDesc& desc)
{
    agx::ConstraintRef c = nullptr;
    if(desc.constraintType == AGXConstraintType::AGXHINGE){
        c = createConstraintHinge(static_cast<const AGXHingeDesc&>(desc));
    }else if(desc.constraintType == AGXConstraintType::AGXLOCKJOINT){
        c = createConstraintLockJoint(static_cast<const AGXLockJointDesc&>(desc));
    }
    constraint = c; 
}

agxCollide::BoxRef AGXLinkBody::createShapeBox(const AGXBoxDesc& desc)
{
    return new agxCollide::Box(desc.halfExtents);
}

agxCollide::SphereRef AGXLinkBody::createShapeSphere(const AGXSphereDesc & desc)
{
    return new agxCollide::Sphere(desc.radius);
}

agxCollide::CapsuleRef AGXLinkBody::createShapeCapsule(const AGXCapsuleDesc & desc)
{
    return new agxCollide::Capsule(desc.radius, desc.hegiht);
}

agxCollide::CylinderRef AGXLinkBody::createShapeCylinder(const AGXCylinderDesc & desc)
{
    return new agxCollide::Cylinder(desc.radius, desc.hegiht);
}

agxCollide::MeshRef AGXLinkBody::createShapeTrimesh(const AGXTrimeshDesc& desc)
{
    return new agxCollide::Trimesh(&desc.vertices, &desc.indices, desc.name, desc.optionsMask, desc.bottomMargin);
}

agx::HingeRef AGXLinkBody::createConstraintHinge(const AGXHingeDesc& desc)
{
    agx::HingeFrame hingeFrame;
    hingeFrame.setAxis(desc.frameAxis);
    hingeFrame.setCenter(desc.frameCenter);
    agx::HingeRef hinge = new agx::Hinge(hingeFrame, desc.rigidBodyA, desc.rigidBodyB);
    hinge->getMotor1D()->setEnable(desc.isMotorOn);
    return hinge;
}

agx::PrismaticRef AGXLinkBody::createConstraintPrismatic(const AGXPrismaticDesc & desc)
{
    agx::PrismaticFrame prismaticFrame;
    prismaticFrame.setAxis(desc.frameAxis);
    prismaticFrame.setPoint(desc.framePoint);
    agx::PrismaticRef prismatic = new agx::Prismatic(prismaticFrame, desc.rigidBodyA, desc.rigidBodyB);
    prismatic->getMotor1D()->setEnable(desc.isMotorOn);
    return prismatic;
}

agx::LockJointRef AGXLinkBody::createConstraintLockJoint(const AGXLockJointDesc & desc)
{
    return new agx::LockJoint(desc.rigidBodyA, desc.rigidBodyB);
}

}