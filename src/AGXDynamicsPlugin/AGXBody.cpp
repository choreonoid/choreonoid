#include "AGXBody.h"
#include <cnoid/SceneDrawables>

namespace cnoid{

////////////////////////////////////////////////////////////
// AGXLink
AGXLink::AGXLink(int index, LinkPtr link) : _index(index), orgLink(link){}

void AGXLink::setParentLink(AGXLinkPtr link){
	agxParentLink = link;
}

int AGXLink::getIndex(){	
	return _index;
}

AGXLinkBodyRef AGXLink::getAGXLinkBody(){
	return agxLinkBody;
}

agx::RigidBodyRef AGXLink::getAGXRigidBody(){
	return agxLinkBody->getRigidBody();
}

void AGXLink::createLinkBody(){
	agxLinkBody = new AGXLinkBody();
	createAGXRigidBody();
	createAGXGeometry();
	createAGXShape();
}

void AGXLink::createAGXRigidBody(){
	const Matrix3 I = orgLink->I();
	const Vector3 c = orgLink->c();
	const Vector3 p = orgLink->p();

	AGXRigidBodyDesc desc;
	desc.name = orgLink->name();
	desc.p.set(p(0), p(1), p(2));
	std::cout << desc.p << std::endl;
	desc.m = orgLink->m();
	desc.I.set( I(0,0), I(1,0), I(2,0),
				I(0,1), I(1,1), I(2,1),
				I(0,2), I(1,2), I(2,2));
	desc.c.set(c(0), c(1), c(2));

	Link::JointType jt = orgLink->jointType();
	if(jt == Link::FIXED_JOINT){
		desc.control = agx::RigidBody::MotionControl::STATIC;
	}

	agxLinkBody->createRigidBody(desc);
}

void AGXLink::createAGXGeometry(){
	AGXGeometryDesc gdesc;
	agxLinkBody->createGeometry(gdesc);
}

void AGXLink::createAGXShape(){
	if(!orgLink->collisionShape()) return;
	MeshExtractor* extractor = new MeshExtractor;
	AGXShapeDesc desc;
	std::vector<Vertex> vertices;
	Affine3 T;
	if(extractor->extract(orgLink->collisionShape(), std::bind(&AGXLink::detectPrimitiveShape, this, extractor))){
		// if vertices have values, it will be trimesh 
		if(!vertices.empty()){
			AGXTrimeshDesc desc;
			agxLinkBody->createShape(desc);
		}
	}
	delete extractor;
}

void AGXLink::detectPrimitiveShape(MeshExtractor* extractor){
	SgMesh* mesh = extractor->currentMesh();
	const Affine3& T = extractor->currentTransform();

	bool meshAdded = false;

	if(mesh->primitiveType() != SgMesh::MESH){
		bool doAddPrimitive = false;
		Vector3 scale;
		Vector3 translation;
		if(!extractor->isCurrentScaled()){
			scale.setOnes();
			doAddPrimitive = true;
		} else {
			Affine3 S = extractor->currentTransformWithoutScaling().inverse() * extractor->currentTransform();

			if(S.linear().isDiagonal()){
				if(!S.translation().isZero()){
					translation = S.translation();
				}
				scale = S.linear().diagonal();
				if(mesh->primitiveType() == SgMesh::BOX){
					doAddPrimitive = true;
				} else if(mesh->primitiveType() == SgMesh::SPHERE){
					// check if the sphere is uniformly scaled for all the axes
					if(scale.x() == scale.y() && scale.x() == scale.z()){
						doAddPrimitive = true;
					}
				} else if(mesh->primitiveType() == SgMesh::CYLINDER){
					// check if the bottom circle face is uniformly scaled
					if(scale.x() == scale.z()){
						doAddPrimitive = true;
					}
				} /*else if(mesh->primitiveType() == SgMesh::CAPSUEL){
				}*/
			}
		}
		if(doAddPrimitive){
			bool created = false;
			switch(mesh->primitiveType()){
			case SgMesh::BOX : {
				const Vector3& s = mesh->primitive<SgMesh::Box>().size / 2.0;
				AGXBoxDesc bd;
				bd.halfExtents = agx::Vec3( s.x()*scale.x(), s.y()*scale.y(), s.z()*scale.z());
				getAGXLinkBody()->createShape(bd);
				created = true;
				break;
			}
			case SgMesh::SPHERE : {
				SgMesh::Sphere sphere = mesh->primitive<SgMesh::Sphere>();
				//agxCollide::SphereRef sphere_ = new agxCollide::Sphere( sphere.radius * scale.x() );
				AGXSphereDesc sd;
				//sd.radius = sphere.radius * scale.x();
				//getAGXLinkBody()->createShape(sd);
				created = true;
				break;
			}
			case SgMesh::CYLINDER : {
				SgMesh::Cylinder cylinder = mesh->primitive<SgMesh::Cylinder>();
				//agxCollide::CylinderRef cylinder_ = new agxCollide::Cylinder(cylinder.radius * scale.x(), cylinder.height * scale.y());
				//AGXCylinderDesc cd;
				//cd.radius = cylinder.radius * scale.x();
				//cd.height = cylinder.height * scale.y()
				//getAGXLinkBody()->createShape(cd);
				created = true;
				break;
			}
			default :
				break;
			}
			if(created){
				Affine3 T_ = extractor->currentTransformWithoutScaling();
				if(translation.norm() > 0){
					T_ *= Translation3(translation);
				}
				//agxGeometry->setLocalTransform( agx::AffineMatrix4x4( T_(0,0), T_(1,0), T_(2,0), 0.0,
    //                                                  T_(0,1), T_(1,1), T_(2,1), 0.0,
    //                                                  T_(0,2), T_(1,2), T_(2,2), 0.0,
    //                                                  T_(0,3), T_(1,3), T_(2,3), 1.0) );
				meshAdded = true;
			}
		}
	}

	if(!meshAdded){
		const size_t vertexIndexTop = vertices.size();
		const SgVertexArray& vertices_ = *mesh->vertices();
		const int numVertices = vertices_.size();
		for(int i=0; i < numVertices; ++i){
			const Vector3 v = T * vertices_[i].cast<Position::Scalar>();
			vertices.push_back(Vertex(v.x(), v.y(), v.z()));
		}

		const int numTriangles = mesh->numTriangles();
		for(int i=0; i < numTriangles; ++i){
			SgMesh::TriangleRef src = mesh->triangle(i);
			indices.push_back(vertexIndexTop + src[0]);
			indices.push_back(vertexIndexTop + src[1]);
			indices.push_back(vertexIndexTop + src[2]);
		}
	}
}

void AGXLink::synchronizeLinkStateToCnoid()
{
	agx::RigidBodyRef agxRigidBody = getAGXRigidBody();
	if(!agxRigidBody) return;

	agx::AffineMatrix4x4 t = agxRigidBody->getTransform();
    orgLink->p() = Vector3(t(3,0), t(3,1), t(3,2));
    orgLink->R() << t(0,0), t(1,0), t(2,0),
                 t(0,1), t(1,1), t(2,1),
                 t(0,2), t(1,2), t(2,2);

    agx::Vec3 w = agxRigidBody->getAngularVelocity();
    orgLink->w() = Vector3(w.x(), w.y(), w.z());
    agx::Vec3 v = agxRigidBody->getVelocity();
    Vector3 v0(v.x(), v.y(), v.z());
    const Vector3 c = orgLink->R() * orgLink->c();
    orgLink->v() = v0 - orgLink->w().cross(c);
}

//LinkPtr AGXLink::link(){
//	return orgLink;
//}
//
//
//LinkPtr AGXLink::getLink(){
//	return orgLink;
//}

//AGXLinkPtr AGXLink::getParent(){
//	return agxParentLink;
//}

////////////////////////////////////////////////////////////
// AGXBody
AGXBody::AGXBody(Body & orgBody) : SimulationBody(new Body(orgBody)){}

void AGXBody::initialize(){
	BodyPtr body = this->body();
	if(!body) assert(body);

	// Initialize the status of the root link
	LinkPtr rootLink= body->rootLink();
	if(rootLink){
		rootLink->v().setZero();
		rootLink->dv().setZero();
		rootLink->w().setZero();
		rootLink->dw().setZero();
	}
	// Initialize the status of all joints
	for(int i=0; i < body->numJoints(); ++i){
		Link* joint = body->joint(i);
		joint->u() = 0.0;
		joint->dq() = 0.0;
		joint->ddq() = 0.0;
	}

	body->clearExternalForces();
	body->calcForwardKinematics(true, true);

	return;
}

void AGXBody::createBody(){
	initialize();

	// Create empty AGXLink
	for(int i = 0; i < body()->numLinks(); ++i){
		AGXLinkPtr agxLink = new AGXLink(i, body()->link(i));
		agxLinks.push_back(agxLink);
	}
	// Create rigidbody and geometry of AGX
	// Set parent link to each AGXLink
	for(int i = 0; i < body()->numLinks(); ++i){
		agxLinks[i]->createLinkBody();

		LinkPtr parent = body()->link(i)->parent();
		if(parent){
			agxLinks[i]->setParentLink(agxLinks[parent->index()]);
		}
	}
	// Create constraints
}

void AGXBody::synchronizeLinkStateToCnoid(){
	for(size_t i = 0; i < agxLinks.size(); ++i){
		agxLinks[i]->synchronizeLinkStateToCnoid();
	}	
}

//AGXLinkPtr AGXBody::getAGXLink(int index){
//	return agxLinks[index];
//}

agx::RigidBodyRef AGXBody::getAGXRigidBody(int index)
{
	return agxLinks[index]->getAGXRigidBody();
}

int AGXBody::getNumLinks(){
	return agxLinks.size();
}

//AGXLinkPtr AGXBody::getAGXLinks(){
//	return &_agxLinks;
//}

}
