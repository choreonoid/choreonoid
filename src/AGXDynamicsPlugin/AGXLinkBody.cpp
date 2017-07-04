#include "AGXLinkBody.h"

namespace cnoid{

////////////////////////////////////////////////////////////
// AGXLinkBody
AGXLinkBody::AGXLinkBody(){}

agx::RigidBodyRef AGXLinkBody::getRigidBody(){
	return _rigid;
}
agxCollide::GeometryRef AGXLinkBody::getGeometry(){
	return _geometry;
}
agx::ConstraintRef AGXLinkBody::getConstraint()
{
	return constraint;
}

void AGXLinkBody::createRigidBody(AGXRigidBodyDesc desc){
	agx::RigidBodyRef r = new agx::RigidBody();
	r->setMotionControl(desc.control);
	r->setVelocity(desc.v);
	r->setAngularVelocity(desc.w);
	r->setPosition(desc.p);
	r->setRotation(desc.R);
	r->getMassProperties()->setAutoGenerateMask(desc.genflags);
	r->getMassProperties()->setMass(desc.m, false);
	r->getMassProperties()->setInertiaTensor(desc.I, false);
	r->setName(desc.name);
	_rigid = r;
}
void AGXLinkBody::createGeometry(AGXGeometryDesc desc){
	agxCollide::GeometryRef g = new agxCollide::Geometry();
	//g->setSurfaceVelocity(desc.surfacevel);
	getRigidBody()->add(g);
	_geometry = g;
}

void AGXLinkBody::createShape(const AGXShapeDesc& desc, const agx::AffineMatrix4x4& af){
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

void AGXLinkBody::createConstraint(const AGXConstraintDesc& desc){
	agx::ConstraintRef c = nullptr;
	if(desc.constraintType == AGXConstraintType::AGXHIGE){
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

agxCollide::MeshRef AGXLinkBody::createShapeTrimesh(const AGXTrimeshDesc& desc){
	return new agxCollide::Trimesh(&desc.vertices, &desc.indices, desc.name, desc.optionsMask, desc.bottomMargin);
}

agx::HingeRef AGXLinkBody::createConstraintHinge(const AGXHingeDesc& desc)
{
	/*
	agx::FrameRef myFrame = new agx::Frame();
	agx::FrameRef parentFrame = new agx::Frame();
	myFrame->setLocalTranslate(desc.myFramePos);
	myFrame->setLocalRotate(desc.myFrameRot);
	parentFrame->setLocalTranslate(desc.parentFramePos);
	parentFrame->setRotate(desc.parentFrameRot);
	return new agx::Hinge(desc.myRigidBody, myFrame, desc.parentRigidBody, parentFrame);
	*/
	agx::HingeFrame hingeFrame;
	hingeFrame.setAxis(desc.hingeFrameAxis);
	hingeFrame.setCenter(desc.hingeFrameCenter);
	return new agx::Hinge(hingeFrame, desc.RigidBodyA, desc.RigidBodyB);
}

agx::LockJointRef AGXLinkBody::createConstraintLockJoint(const AGXLockJointDesc & desc)
{
	return new agx::LockJoint(desc.RigidBodyA, desc.RigidBodyB);
}


}