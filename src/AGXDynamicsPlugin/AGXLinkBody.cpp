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
void AGXLinkBody::createRigidBody(AGXRigidBodyDesc desc){
	agx::RigidBodyRef r = new agx::RigidBody();
	r->setMotionControl(desc.control);
	r->setVelocity(desc.v);
	r->setPosition(desc.p);
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

void AGXLinkBody::createShape(const AGXShapeDesc& desc){
	agxCollide::ShapeRef shape = nullptr;
	if(desc.shapeType == AGXShapeType::AGXCOLLIDE_BOX){
		shape = createShapeBox(static_cast<const AGXBoxDesc&>(desc));
	}else if(desc.shapeType == AGXShapeType::AGXCOLLIDE_SPHERE){
		//shape = createShapeSphere(static_cast<const AGXBoxDesc&>(desc));
	}else if(desc.shapeType == AGXShapeType::AGXCOLLIDE_TRIMESH){
		shape = createShapeTrimesh(static_cast<const AGXTrimeshDesc&>(desc));
	}else{}
	getGeometry()->add(shape);
}

agxCollide::BoxRef AGXLinkBody::createShapeBox(const AGXBoxDesc& desc){
	return new agxCollide::Box(desc.halfExtents);
}

agxCollide::MeshRef AGXLinkBody::createShapeTrimesh(const AGXTrimeshDesc& desc){
	//return new agxCollide::Trimesh();
	return nullptr;
}


}