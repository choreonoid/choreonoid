#include "AGXObjectFactory.h"

namespace cnoid{

    ////////////////////////////////////////////////////////////
// AGXPseudoContinuousTrackGeometry
void AGXPseudoContinuousTrackGeometry::setAxis(const agx::Vec3f& axis)
{
    _axis = axis;
}

agx::Vec3f AGXPseudoContinuousTrackGeometry::getAxis() const
{
    return _axis;
}

agx::Vec3f AGXPseudoContinuousTrackGeometry::calculateSurfaceVelocity(const agxCollide::LocalContactPoint & point, size_t index) const
{
    agx::Vec3f dir = getAxis() ^ point.normal();
    dir.normalize();
    const agx::Vec3f ret = dir * -1.0 * getSurfaceVelocity().x();
    return ret;
}

////////////////////////////////////////////////////////////
// AGXObjectFactory

agxSDK::SimulationRef AGXObjectFactory::createSimulation(const AGXSimulationDesc & desc)
{
    agxSDK::SimulationRef sim = new agxSDK::Simulation();
    sim->setTimeStep(desc.timeStep);
    return sim;
}

agx::MaterialRef AGXObjectFactory::createMaterial(const AGXMaterialDesc & desc)
{
    agx::MaterialRef m = new agx::Material(desc.name);
    m->getBulkMaterial()->setDensity(desc.density);
    m->getBulkMaterial()->setYoungsModulus(desc.youngsModulus);
    m->getBulkMaterial()->setPoissonsRatio(desc.poissonRatio);

    // Below are overried when ContactMaterials are used.
    m->getBulkMaterial()->setViscosity(desc.viscosity);
    m->getBulkMaterial()->setDamping(desc.damping);
    m->getSurfaceMaterial()->setRoughness(desc.roughness);
    m->getSurfaceMaterial()->setViscosity(desc.surfaceViscosity);
    m->getSurfaceMaterial()->setAdhesion(desc.adhesionForce, desc.adhesivOverlap);
    return m;
}

agx::ContactMaterialRef AGXObjectFactory::createContactMaterial(agx::MaterialRef const matA, agx::MaterialRef const matB, const AGXContactMaterialDesc& desc)
{
    agx::ContactMaterialRef cm = new agx::ContactMaterial(matA, matB);
    setContactMaterialParam(cm, desc);
    return cm;
}

agx::ContactMaterialRef AGXObjectFactory::createContactMaterial(const AGXContactMaterialDesc& desc, agxSDK::MaterialManagerRef const mgr)
{
    if(!mgr) return nullptr;
    agx::MaterialRef mA = mgr->getMaterial(desc.nameA);
    agx::MaterialRef mB = mgr->getMaterial(desc.nameB);
    agx::ContactMaterialRef cm = mgr->getOrCreateContactMaterial(mA, mB);
    if(!cm) return nullptr;
    setContactMaterialParam(cm, desc);
    return cm;
}

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
    switch (desc.shapeType){
        case AGXShapeType::AGXCOLLIDE_BOX :{
            shape = createShapeBox(static_cast<const AGXBoxDesc&>(desc));
            break;
        }
        case AGXShapeType::AGXCOLLIDE_SPHERE :{
            shape = createShapeSphere(static_cast<const AGXSphereDesc&>(desc));
            break;
        }
        case AGXShapeType::AGXCOLLIDE_CAPSULE :{
            shape = createShapeCapsule(static_cast<const AGXCapsuleDesc&>(desc));
            break;
        }
        case AGXShapeType::AGXCOLLIDE_CYLINDER :{
           shape = createShapeCylinder(static_cast<const AGXCylinderDesc&>(desc));
            break;
        }
        case AGXShapeType::AGXCOLLIDE_TRIMESH :{
            shape = createShapeTrimesh(static_cast<const AGXTrimeshDesc&>(desc));
            break;
        }
        default:
            break;
    }
    return shape;
}

agx::ConstraintRef AGXObjectFactory::createConstraint(const AGXConstraintDesc& desc)
{
    agx::ConstraintRef constraint = nullptr;
    switch(desc.constraintType){
        case AGXConstraintType::AGXHINGE :
            constraint = createConstraintHinge(static_cast<const AGXHingeDesc&>(desc));
            break;
        case AGXConstraintType::AGXLOCKJOINT :
            constraint = createConstraintLockJoint(static_cast<const AGXLockJointDesc&>(desc));
            break;
        case AGXConstraintType::AGXBALLJOINT :
            constraint = createConstraintBallJoint(static_cast<const AGXBallJointDesc&>(desc));
            break;
        default :
            break;
    }
    return constraint;
}

agx::Bool AGXObjectFactory::setContactMaterialParam(agx::ContactMaterialRef const cm, const AGXContactMaterialDesc & desc)
{
    if(!cm) return false;
    cm->setYoungsModulus(desc.youngsModulus);
    cm->setRestitution(desc.restitution);
    cm->setDamping(desc.damping);
    cm->setFrictionCoefficient(desc.friction);
    cm->setAdhesion(desc.adhesionForce, desc.adhesivOverlap);
    cm->setSurfaceViscosity(desc.surfaceViscosity, desc.frictionDirection);

    // Create friction model
    if(desc.frictionModelType != AGXFrictionModelType::DEFAULT){
        agx::FrictionModelRef fm = nullptr;
        switch (desc.frictionModelType){
            case AGXFrictionModelType::BOX :
                fm = new agx::BoxFrictionModel();
                break;
            case AGXFrictionModelType::SCALE_BOX :
                fm = new agx::ScaleBoxFrictionModel();
                break;
            case AGXFrictionModelType::ITERATIVE_PROJECTED_CONE :
                fm = new agx::IterativeProjectedConeFriction();
                break;
            case AGXFrictionModelType::DEFAULT:
            default:
                break;
        }
        fm->setSolveType(desc.solveType);
        cm->setFrictionModel(fm);
    }
    return true;
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
    return new agxCollide::Capsule(desc.radius, desc.height);
}

agxCollide::CylinderRef AGXObjectFactory::createShapeCylinder(const AGXCylinderDesc & desc)
{
    return new agxCollide::Cylinder(desc.radius, desc.height);
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