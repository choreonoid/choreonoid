#include "AGXBody.h"
#include <cnoid/SceneDrawables>
#include <boost/optional/optional.hpp>

namespace cnoid{

////////////////////////////////////////////////////////////
// AGXLink
AGXLink::AGXLink(const LinkPtr link) : _orgLink(link){}

AGXLink::AGXLink(const LinkPtr link, const AGXLinkPtr parent, const Vector3& parentOrigin, const AGXBodyPtr agxBody)
{
    _orgLink = link;
    _agxParentLink = parent;
    agxBody->addAGXLink(this);
    _origin = parentOrigin + getOrgLink()->b();
    _controlMode = ControlMode::VELOCITY;
    //_controlMode = ControlMode::NONE;
    constructAGXLink();
    for(Link* child = link->child(); child; child = child->sibling()){
        new AGXLink(child, this, getOrigin(), agxBody);
    }
}

void AGXLink::setParentLink(const AGXLinkPtr link)
{
    _agxParentLink = link;
}

int AGXLink::getIndex() const
{
    return getOrgLink()->index();
}

//AGXLinkBodyRef AGXLink::getAGXLinkBody()
//{
//    return _agxLinkBody;
//}

agx::RigidBodyRef AGXLink::getAGXRigidBody()
{
    //return getAGXLinkBody()->getRigidBody();
    return _rigid;
}

agx::ConstraintRef AGXLink::getAGXConstraint()
{
    //return getAGXLinkBody()->getConstraint();
    return _constraint;
}
//
//void AGXLink::createLinkBody()
//{
//    _agxLinkBody = new AGXLinkBody();
//    createAGXRigidBody();
//    createAGXGeometry();
//    createAGXShape();
//}

void AGXLink::constructAGXLink()
{
    _rigid = createAGXRigidBody();
    _geometry = createAGXGeometry();
    _rigid->add(_geometry);
    createAGXShape();
    _constraint = createAGXConstraint();
}


void AGXLink::setCollision(const bool bOn)
{
    //getAGXLinkBody()->getGeometry()->setEnableCollisions(bOn);
    getAGXGeometry()->setEnableCollisions(bOn);
}

void AGXLink::setControlInputToAGX()
{
    switch(getJointControlMode()){
        case ControlMode::TORQUE :{
            setTorqueToAGX();
            break;
        }
        case ControlMode::VELOCITY :{
            setVelocityToAGX();
            break;
        }
        case ControlMode::POSITION :{
            //setPositionToAGX();
            break;
        }
        case ControlMode::NONE :
        default :
            break;
    }
}

void AGXLink::setTorqueToAGX()
{
    LinkPtr orgLink = getOrgLink();
    switch(orgLink->jointType()){
        case Link::ROTATIONAL_JOINT:
        case Link::SLIDE_JOINT:{
            agx::Constraint1DOF* joint1DOF = agx::Constraint1DOF::safeCast(getAGXConstraint());
            if(!joint1DOF) break;
#if 0
            joint1DOF->getElectricMotorController()->setEnable(true);
            joint1DOF->getElectricMotorController()->setTorqueConstant(orgLink->u());
#else
            joint1DOF->getMotor1D()->setSpeed( orgLink->u() < 0 ? -1.0e12 : 1.0e12);
            joint1DOF->getMotor1D()->setForceRange( agx::RangeReal(orgLink->u()));
#endif
            break;
        }
        default :
            break;
    }
}

void AGXLink::setVelocityToAGX()
{
    LinkPtr orgLink = getOrgLink();
    switch(orgLink->jointType()){
        case Link::ROTATIONAL_JOINT:
        case Link::SLIDE_JOINT:{
            agx::Constraint1DOF* joint1DOF = agx::Constraint1DOF::safeCast(getAGXConstraint());
            if(!joint1DOF) break;
            joint1DOF->getMotor1D()->setSpeed(orgLink->dq());
            break;
        }
        case Link::PSEUDO_CONTINUOUS_TRACK:{
            // Set speed(scalar) to x value. Direction is automatically calculated at AGXPseudoContinuousTrackGeometry::calculateSurfaceVelocity
            //getAGXLinkBody()->getGeometry()->setSurfaceVelocity(agx::Vec3f(orgLink->dq(), 0.0, 0.0));
            getAGXGeometry()->setSurfaceVelocity(agx::Vec3f(orgLink->dq(), 0.0, 0.0));
            break;
        }
        default :
            break;
    }
}

void AGXLink::setPositionToAGX()
{
    LinkPtr orgLink = getOrgLink();
    switch(orgLink->jointType()){
        case Link::ROTATIONAL_JOINT:
        case Link::SLIDE_JOINT:{
            agx::Constraint1DOF* joint1DOF = agx::Constraint1DOF::safeCast(getAGXConstraint());
            if(!joint1DOF) break;
            joint1DOF->getLock1D()->setPosition(orgLink->q());
            break;
        }
        default :
            break;
    }
}

agx::RigidBodyRef AGXLink::createAGXRigidBody()
{
    LinkPtr orgLink = getOrgLink();
    const Matrix3& I = orgLink->I();
    const Vector3& c = orgLink->c();
    const Vector3& v = orgLink->v(); 
    const Vector3& w = orgLink->w(); 
    const Vector3& p = getOrigin();

    AGXRigidBodyDesc desc;
    desc.name = orgLink->name();
    desc.m = orgLink->m();
    desc.I.set( I(0,0), I(1,0), I(2,0),
                I(0,1), I(1,1), I(2,1),
                I(0,2), I(1,2), I(2,2));
    desc.c.set(c(0), c(1), c(2));
    desc.v.set(v(0), v(1), v(2));
    desc.w.set(w(0), w(1), w(2));
    desc.p.set(p(0), p(1), p(2));
    // First set rotation with default values. Choreonoid uses relative angles to set rotation and viewing models.
    // When set rotation here relative angles shift from correct angles.
    desc.R.set(agx::Quat(0,0,0,1));

    Link::JointType jt = orgLink->jointType();
    if(!orgLink->parent() && jt == Link::FIXED_JOINT ){
        desc.control = agx::RigidBody::MotionControl::STATIC;
    }

    //getAGXLinkBody()->createRigidBody(desc);
    return AGXObjectFactory::createRigidBody(desc);
}

agxCollide::GeometryRef AGXLink::getAGXGeometry()
{
    return _geometry;
}

agxCollide::GeometryRef AGXLink::createAGXGeometry()
{
    LinkPtr orgLink = getOrgLink();
    AGXGeometryDesc gdesc;
    gdesc.selfCollsionGroupName = orgLink->body()->name();
    if(orgLink->jointType() == Link::PSEUDO_CONTINUOUS_TRACK){
        gdesc.isPseudoContinuousTrack = true;
        Vector3 a = orgLink->a();
        gdesc.axis = agx::Vec3f(a(0), a(1), a(2));
    }
    //getAGXLinkBody()->createGeometry(gdesc);
    return AGXObjectFactory::createGeometry(gdesc);
}

void AGXLink::createAGXShape()
{
    LinkPtr orgLink = getOrgLink();
    if(!orgLink->collisionShape()) return;
    MeshExtractor* extractor = new MeshExtractor;
    AGXTrimeshDesc td;
    if(extractor->extract(orgLink->collisionShape(), std::bind(&AGXLink::detectPrimitiveShape, this, extractor, std::ref(td)))){
        // if vertices have values, it will be trimesh 
        if(!td.vertices.empty()){
            //td.name = extractor->currentMesh()->name().c_str();
            //getAGXLinkBody()->createShape(td, agx::AffineMatrix4x4());
            agxCollide::ShapeRef trimesh = AGXObjectFactory::createShape(td);
            getAGXGeometry()->add(trimesh, agx::AffineMatrix4x4());
        //    std::cout << orgLink->name() << std::endl;
        //    std::cout << td.vertices.size() << std::endl;
        //    for(int i = 0; i < td.vertices.size(); ++i){
        //        std::cout << "vertices " << td.vertices[i] << std::endl;
        //    } 
        //    for(int i = 0; i < td.indices.size(); ++i){
        //        std::cout << "vertices " << td.indices[i] << std::endl;
        //    } 
        }
    }
    delete extractor;
}

void AGXLink::detectPrimitiveShape(MeshExtractor* extractor, AGXTrimeshDesc& td)
{
    SgMesh* mesh = extractor->currentMesh();
    const Affine3& T = extractor->currentTransform();

    bool meshAdded = false;

    if(mesh->primitiveType() != SgMesh::MESH){
        bool doAddPrimitive = false;
        Vector3 scale;
        boost::optional<Vector3> translation;
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
            Affine3 T_ = extractor->currentTransformWithoutScaling();
            if(translation){
                T_ *= Translation3(*translation);
            }
            agx::AffineMatrix4x4 af;
            af.set( T_(0,0), T_(1,0), T_(2,0), 0.0,
                    T_(0,1), T_(1,1), T_(2,1), 0.0,
                    T_(0,2), T_(1,2), T_(2,2), 0.0,
                    T_(0,3), T_(1,3), T_(2,3), 1.0);

            bool created = false;
            agxCollide::ShapeRef shape = nullptr;
            switch(mesh->primitiveType()){
                case SgMesh::BOX : {
                    const Vector3 s = mesh->primitive<SgMesh::Box>().size / 2.0;
                    AGXBoxDesc bd;
                    bd.halfExtents = agx::Vec3( s.x()*scale.x(), s.y()*scale.y(), s.z()*scale.z());
                    //getAGXLinkBody()->createShape(bd, af);
                    shape = AGXObjectFactory::createShape(bd);
                    created = true;
                    break;
                }
                case SgMesh::SPHERE : {
                    SgMesh::Sphere sphere = mesh->primitive<SgMesh::Sphere>();
                    AGXSphereDesc sd;
                    sd.radius = sphere.radius * scale.x();
                    //getAGXLinkBody()->createShape(sd, af);
                    shape = AGXObjectFactory::createShape(sd);
                    created = true;
                    break;
                }
                //case SgMesh::CAPSULE : {
                //    //SgMesh::Cylinder cylinder = mesh->primitive<SgMesh::Cylinder>();
                //    AGXCapsuleDesc cd;
                //    //cd.radius = capsule.radius * scale.x();
                //    //cd .hegiht =  capsule.height * scale.y();
                //    //getAGXLinkBody()->createShape(cd, af);
                //    shape = AGXObjectFactory::createShape(cd);
                //    created = true;
                //    break;
                //}
                case SgMesh::CYLINDER : {
                    SgMesh::Cylinder cylinder = mesh->primitive<SgMesh::Cylinder>();
                    AGXCylinderDesc cd;
                    cd.radius = cylinder.radius * scale.x();
                    cd .hegiht =  cylinder.height * scale.y();
                    //getAGXLinkBody()->createShape(cd, af);
                    shape = AGXObjectFactory::createShape(cd);
                    created = true;
                    break;
                }
                default :
                    break;
            }
            if(shape){
                getAGXGeometry()->add(shape, af);
                meshAdded = true;
            }
        }
    }

    if(!meshAdded){
        const size_t vertexIndexTop = td.vertices.size();
        const SgVertexArray& vertices_ = *mesh->vertices();
        const int numVertices = vertices_.size();
        for(int i=0; i < numVertices; ++i){
            const Vector3 v = T * vertices_[i].cast<Position::Scalar>();
            td.vertices.push_back(agx::Vec3(v.x(), v.y(), v.z()));
        }

        const int numTriangles = mesh->numTriangles();
        for(int i=0; i < numTriangles; ++i){
            SgMesh::TriangleRef src = mesh->triangle(i);
            td.indices.push_back(vertexIndexTop + src[0]);
            td.indices.push_back(vertexIndexTop + src[1]);
            td.indices.push_back(vertexIndexTop + src[2]);
        }
    }
}

agx::ConstraintRef AGXLink::createAGXConstraint()
{
    AGXLinkPtr agxParentLink = getAGXParentLink();
    if(!agxParentLink) return nullptr;
    LinkPtr orgLink = getOrgLink();
    agx::ConstraintRef constraint;
    switch(orgLink->jointType()){
        case Link::REVOLUTE_JOINT :{
            AGXHingeDesc desc;
            const Vector3& a = orgLink->a();
            const Vector3& p = getOrigin();
            desc.frameAxis.set(a(0),a(1),a(2));
            desc.frameCenter.set(p(0),p(1),p(2));
            desc.rigidBodyA = getAGXRigidBody();
            desc.rigidBodyB = agxParentLink->getAGXRigidBody();
            if(getJointControlMode() != ControlMode::NONE) desc.isMotorOn = true;
            //getAGXLinkBody()->createConstraint(desc);
            constraint = AGXObjectFactory::createConstraint(desc);
            break;
        }
        case Link::PRISMATIC_JOINT :{
            AGXPrismaticDesc desc;
            const Vector3& a = orgLink->a();
            const Vector3& p = getOrigin();
            desc.frameAxis.set(a(0),a(1),a(2));
            desc.framePoint.set(p(0),p(1),p(2));
            desc.rigidBodyA = getAGXRigidBody();
            desc.rigidBodyB = agxParentLink->getAGXRigidBody();
            if(getJointControlMode() != ControlMode::NONE) desc.isMotorOn = true;
            //getAGXLinkBody()->createConstraint(desc);
            constraint = AGXObjectFactory::createConstraint(desc);
            break;
        }
        case Link::FIXED_JOINT :
        case Link::PSEUDO_CONTINUOUS_TRACK :{
            AGXLockJointDesc desc;
            desc.rigidBodyA = getAGXRigidBody();
            desc.rigidBodyB = agxParentLink->getAGXRigidBody();
            //getAGXLinkBody()->createConstraint(desc);
            constraint = AGXObjectFactory::createConstraint(desc);
            break;
        }
        case Link::FREE_JOINT :
        default:
            break;
    }
    return constraint;
}

Vector3 AGXLink::getOrigin() const
{
    return _origin;
}

LinkPtr AGXLink::getOrgLink() const
{
    return _orgLink;
}

AGXLinkPtr AGXLink::getAGXParentLink() const
{
    return _agxParentLink;
}

void AGXLink::setLinkStateToAGX()
{
    agx::RigidBodyRef agxRigidBody = getAGXRigidBody();
    if(!agxRigidBody) return;
    LinkPtr orgLink = getOrgLink();
    const Vector3& p = orgLink->p();
    const Matrix3& R = orgLink->R();
    agx::Vec3 translation(p(0), p(1), p(2));
    agx::OrthoMatrix3x3 rotation(R(0,0), R(1,0), R(2,0),
                                 R(0,1), R(1,1), R(2,1),
                                 R(0,2), R(1,2), R(2,2));
    agxRigidBody->setTransform( agx::AffineMatrix4x4( rotation, translation) );

    const Vector3 lc = orgLink->R() * orgLink->c();
    const Vector3& w = orgLink->w();
    const Vector3 v = orgLink->v() + w.cross(lc);
    agxRigidBody->setVelocity( agx::Vec3(v(0),v(1),v(2)) );     // the linear velocity of the center of mass
    agxRigidBody->setAngularVelocity( agx::Vec3(w(0),w(1),w(2)) );
}

void AGXLink::setLinkStateToCnoid()
{
    agx::RigidBodyRef agxRigidBody = getAGXRigidBody();
    if(!agxRigidBody) return;

    // constraint
    LinkPtr orgLink = getOrgLink();
    switch(orgLink->jointType()){
        case Link::ROTATIONAL_JOINT:
        case Link::SLIDE_JOINT:{
            agx::Constraint1DOF* joint1DOF = agx::Constraint1DOF::safeCast(getAGXConstraint());
            if(joint1DOF){
                orgLink->q() = joint1DOF->getAngle();
                orgLink->dq() = joint1DOF->getCurrentSpeed();
                //if(inputMode==VEL){
                //    link->u() = joint1DOF->getMotor1D()->getCurrentForce();
                //    //cout << link->name() << " " << link->u() << endl;
                //}
                break;
            }
            //agx::PrismaticUniversalJoint* pujoint = dynamic_cast<agx::PrismaticUniversalJoint*>(joint);
            //if(pujoint){
            //    link->q() = pujoint->getAngle(customConstraintIndex);
            //    link->dq() = pujoint->getCurrentSpeed(customConstraintIndex);
            //    break;
            //}
        }
        default :
            break;
    }    


    // position, rotation
    agx::AffineMatrix4x4 t = agxRigidBody->getTransform();
    orgLink->p() = Vector3(t(3,0), t(3,1), t(3,2));
    orgLink->R() << t(0,0), t(1,0), t(2,0),
                 t(0,1), t(1,1), t(2,1),
                 t(0,2), t(1,2), t(2,2);

    // angular velocity
    agx::Vec3 w = agxRigidBody->getAngularVelocity();
    orgLink->w() = Vector3(w.x(), w.y(), w.z());

    // velocity
    agx::Vec3 v = agxRigidBody->getVelocity();
    Vector3 v0(v.x(), v.y(), v.z());
    const Vector3 c = orgLink->R() * orgLink->c();
    orgLink->v() = v0 - orgLink->w().cross(c);

}

void AGXLink::setJointControlMode(const ControlMode& mode)
{
    _controlMode = mode;
}

AGXLink::ControlMode AGXLink::getJointControlMode() const
{
    return _controlMode;
}

////////////////////////////////////////////////////////////
// AGXBody
AGXBody::AGXBody(Body& orgBody) : SimulationBody(new Body(orgBody)){}

void AGXBody::initialize()
{
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

void AGXBody::createBody()
{
    initialize();

    // Create AGXLink following child.
    new AGXLink(body()->rootLink(), nullptr, Vector3::Zero(), this);
    setLinkStateToAGX();
    setExtraJoints();
}

void AGXBody::createBodyClosedLoop()
{
    initialize();

    // Create empty AGXLink
    new AGXLink(body()->rootLink(), nullptr, Vector3::Zero(), this);

    // Create rigidbody and geometry of AGX
    for(int i = 0; i < body()->numLinks(); ++i){
        //agxLinks[i]->createLinkBody();
        agxLinks[i]->constructAGXLink();
    }
    // Create constraints
    for(int i = 0; i < body()->numLinks(); ++i){
        agxLinks[i]->createAGXConstraint();
    }

    setLinkStateToAGX();
    setExtraJoints();
}


void AGXBody::setExtraJoints()
{
    BodyPtr body = this->body();
    const int n = body->numExtraJoints();
    for(int j=0; j < n; ++j){
        ExtraJoint& extraJoint = body->extraJoint(j);

        AGXLinkPtr agxLinkPair[2];
        agxLinkPair[0] = agxLinks[extraJoint.link[0]->index()];
        agxLinkPair[1] = agxLinks[extraJoint.link[1]->index()];
        if(!agxLinkPair[0] || !agxLinkPair[1]) continue;

        Link* link = extraJoint.link[0];
        Vector3 p = link->attitude() * extraJoint.point[0] + link->p();
        Vector3 a = link->attitude() * extraJoint.axis;
        agx::ConstraintRef constraint;
        switch (extraJoint.type){
            case ExtraJoint::EJ_PISTON :{
                AGXHingeDesc hd;
                hd.frameAxis   = agx::Vec3( a(0), a(1), a(2));
                hd.frameCenter = agx::Vec3( p(0), p(1), p(2));
                hd.rigidBodyA = agxLinkPair[0]->getAGXRigidBody();
                hd.rigidBodyB = agxLinkPair[1]->getAGXRigidBody();
                constraint = AGXObjectFactory::createConstraint(hd);
            }
            case  ExtraJoint::EJ_BALL :{
                AGXBallJointDesc bd;
                bd.framePoint = agx::Vec3( p(0), p(1), p(2));
                bd.rigidBodyA = agxLinkPair[0]->getAGXRigidBody();
                bd.rigidBodyB = agxLinkPair[1]->getAGXRigidBody(); 
                constraint = AGXObjectFactory::createConstraint(bd);
            }
            default:
                break;
        }
        addAGXExtraConstraint(constraint);
    }
}

void AGXBody::setCollision(bool bOn)
{
    for(size_t i = 0; i < agxLinks.size(); ++i){
        agxLinks[i]->setCollision(bOn);
    }
}

void AGXBody::setTorqueToAGX()
{
    // Skip the root link
    for(size_t i=1; i < agxLinks.size(); ++i){
        agxLinks[i]->setTorqueToAGX();
    }
}

void AGXBody::setControlInputToAGX()
{
    // Skip the root link
    for(size_t i=1; i < agxLinks.size(); ++i){
        agxLinks[i]->setControlInputToAGX();
    }
}

void AGXBody::setLinkStateToAGX()
{
    for(size_t i = 0; i < agxLinks.size(); ++i){
        agxLinks[i]->setLinkStateToAGX();
    }
}

void AGXBody::setLinkStateToCnoid()
{
    for(size_t i = 0; i < agxLinks.size(); ++i){
        agxLinks[i]->setLinkStateToCnoid();
    }    
}

agx::RigidBodyRef AGXBody::getAGXRigidBody(int index)
{
    return agxLinks[index]->getAGXRigidBody();
}

agx::ConstraintRef AGXBody::getAGXConstraint(int index)
{
    return agxLinks[index]->getAGXConstraint();
}


agx::ConstraintRef AGXBody::getAGXExtraConstraint(int index)
{
    return _agxExtraConstraints[index];
}

void AGXBody::setAGXMaterial(const int & index, const agx::MaterialRef mat)
{
    //agxLinks[index]->getAGXLinkBody()->getGeometry()->setMaterial(mat);
    agxLinks[index]->getAGXGeometry()->setMaterial(mat);
}

int AGXBody::getNumLinks() const
{
    return agxLinks.size();
}

void AGXBody::addAGXLink(AGXLinkPtr const agxLink)
{
    agxLinks.push_back(agxLink);
}

void AGXBody::addAGXExtraConstraint(agx::ConstraintRef constraint)
{
    _agxExtraConstraints.push_back(constraint);
}


}
