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
    if(parent && _controlMode != ControlMode::NONE) agxBody->addControllableLink(this);
    _selfCollisionGroupName = agxBody->getSelfCollisionGroupName();
    constructAGXLink();
    for(Link* child = link->child(); child; child = child->sibling()){
        new AGXLink(child, this, getOrigin(), agxBody);
    }
}

void AGXLink::constructAGXLink()
{
    _rigid = createAGXRigidBody();
    _geometry = createAGXGeometry();
    _rigid->add(_geometry);
    createAGXShape();
    _constraint = createAGXConstraint();
}

void AGXLink::setCollision(const bool& bOn)
{
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

void AGXLink::setLinkStateToAGX()
{
    agx::RigidBodyRef const agxRigidBody = getAGXRigidBody();
    if(!agxRigidBody) return;
    LinkPtr const orgLink = getOrgLink();
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
    agx::RigidBodyRef const agxRigidBody = getAGXRigidBody();
    if(!agxRigidBody) return;

    // constraint
    LinkPtr const orgLink = getOrgLink();
    switch(orgLink->jointType()){
        case Link::ROTATIONAL_JOINT:
        case Link::SLIDE_JOINT:{
            agx::Constraint1DOF* const joint1DOF = agx::Constraint1DOF::safeCast(getAGXConstraint());
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
    const agx::AffineMatrix4x4& t = agxRigidBody->getTransform();
    orgLink->p() = Vector3(t(3,0), t(3,1), t(3,2));
    orgLink->R() << t(0,0), t(1,0), t(2,0),
                 t(0,1), t(1,1), t(2,1),
                 t(0,2), t(1,2), t(2,2);

    // angular velocity
    const agx::Vec3& w = agxRigidBody->getAngularVelocity();
    orgLink->w() = Vector3(w.x(), w.y(), w.z());

    // velocity
    const agx::Vec3& v = agxRigidBody->getVelocity();
    Vector3 v0(v.x(), v.y(), v.z());
    const Vector3 c = orgLink->R() * orgLink->c();
    orgLink->v() = v0 - orgLink->w().cross(c);

}

void AGXLink::setJointControlMode(const ControlMode& mode)
{
    _controlMode = mode;
}

int AGXLink::getIndex() const
{
    return getOrgLink()->index();
}

agx::RigidBodyRef AGXLink::getAGXRigidBody() const
{
    return _rigid;
}

agxCollide::GeometryRef AGXLink::getAGXGeometry() const
{
    return _geometry;
}

agx::ConstraintRef AGXLink::getAGXConstraint() const
{
    return _constraint;
}

AGXLink::ControlMode AGXLink::getJointControlMode() const
{
    return _controlMode;
}

std::string AGXLink::getSelfCollisionGroupName() const
{
    return _selfCollisionGroupName;
}

LinkPtr AGXLink::getOrgLink() const
{
    return _orgLink;
}

AGXLinkPtr AGXLink::getAGXParentLink() const
{
    return _agxParentLink;
}

Vector3 AGXLink::getOrigin() const
{
    return _origin;
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

    return AGXObjectFactory::createRigidBody(desc);
}

agxCollide::GeometryRef AGXLink::createAGXGeometry()
{
    LinkPtr const orgLink = getOrgLink();
    AGXGeometryDesc gdesc;
    gdesc.selfCollsionGroupName = getSelfCollisionGroupName();
    if(orgLink->jointType() == Link::PSEUDO_CONTINUOUS_TRACK){
        gdesc.isPseudoContinuousTrack = true;
        const Vector3& a = orgLink->a();
        gdesc.axis = agx::Vec3f(a(0), a(1), a(2));
    }
    return AGXObjectFactory::createGeometry(gdesc);
}

void AGXLink::createAGXShape()
{
    LinkPtr const orgLink = getOrgLink();
    if(!orgLink->collisionShape()) return;
    MeshExtractor* extractor = new MeshExtractor;
    AGXTrimeshDesc td;
    if(extractor->extract(orgLink->collisionShape(), std::bind(&AGXLink::detectPrimitiveShape, this, extractor, std::ref(td)))){
        // if vertices have values, it will be trimesh 
        if(!td.vertices.empty()){
            //td.name = extractor->currentMesh()->name().c_str();
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
                    shape = AGXObjectFactory::createShape(bd);
                    created = true;
                    break;
                }
                case SgMesh::SPHERE : {
                    const SgMesh::Sphere& sphere = mesh->primitive<SgMesh::Sphere>();
                    AGXSphereDesc sd;
                    sd.radius = mesh->primitive<SgMesh::Sphere>().radius * scale.x();
                    shape = AGXObjectFactory::createShape(sd);
                    created = true;
                    break;
                }
                case SgMesh::CAPSULE : {
                    SgMesh::Capsule capsule = mesh->primitive<SgMesh::Capsule>();
                    AGXCapsuleDesc cd;
                    cd.radius = capsule.radius * scale.x();
                    cd .height =  capsule.height * scale.y();
                    shape = AGXObjectFactory::createShape(cd);
                    created = true;
                    break;
                }
                case SgMesh::CYLINDER : {
                    const SgMesh::Cylinder& cylinder = mesh->primitive<SgMesh::Cylinder>();
                    AGXCylinderDesc cd;
                    cd.radius = cylinder.radius * scale.x();
                    cd .height =  cylinder.height * scale.y();
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
    AGXLinkPtr const agxParentLink = getAGXParentLink();
    if(!agxParentLink) return nullptr;
    LinkPtr const orgLink = getOrgLink();
    agx::ConstraintRef constraint = nullptr;
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
            constraint = AGXObjectFactory::createConstraint(desc);
            break;
        }
        case Link::FIXED_JOINT :
        case Link::PSEUDO_CONTINUOUS_TRACK :{
            AGXLockJointDesc desc;
            desc.rigidBodyA = getAGXRigidBody();
            desc.rigidBodyB = agxParentLink->getAGXRigidBody();
            constraint = AGXObjectFactory::createConstraint(desc);
            break;
        }
        case Link::FREE_JOINT :
        default:
            break;
    }
    return constraint;
}

void AGXLink::setTorqueToAGX()
{
    LinkPtr orgLink = getOrgLink();
    switch(orgLink->jointType()){
        case Link::ROTATIONAL_JOINT :
        case Link::SLIDE_JOINT :{
            agx::Constraint1DOF* const joint1DOF = agx::Constraint1DOF::safeCast(getAGXConstraint());
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
            agx::Constraint1DOF* const joint1DOF = agx::Constraint1DOF::safeCast(getAGXConstraint());
            if(!joint1DOF) break;
            joint1DOF->getMotor1D()->setSpeed(orgLink->dq());
            break;
        }
        case Link::PSEUDO_CONTINUOUS_TRACK:{
            // Set speed(scalar) to x value. Direction is automatically calculated at AGXPseudoContinuousTrackGeometry::calculateSurfaceVelocity
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
            agx::Constraint1DOFRef const joint1DOF = agx::Constraint1DOF::safeCast(getAGXConstraint());
            if(!joint1DOF) break;
            joint1DOF->getLock1D()->setPosition(orgLink->q());
            break;
        }
        default :
            break;
    }
}


////////////////////////////////////////////////////////////
// AGXBody
AGXBody::AGXBody(Body& orgBody) : SimulationBody(new Body(orgBody)){}

void AGXBody::initialize()
{
    BodyPtr const body = this->body();

    // Initialize the status of the root link
    const LinkPtr rootLink= body->rootLink();
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
    _agxLinks.clear();
    _controllableLinks.clear();
    _agxExtraConstraints.clear();
    std::stringstream ss;
    ss.str("");
    ss << "SelfCollision" << generateUID() << body->name() << std::flush;
    _selfCollisionGroupName = ss.str();

    return;
}

void AGXBody::createBody()
{
    initialize();
    // Create AGXLink following child link.
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
        getAGXLink(i)->constructAGXLink();
    }
    // Create constraints
    for(int i = 0; i < body()->numLinks(); ++i){
        //getAGXLink(i)->createAGXConstraint();
    }

    setLinkStateToAGX();
    setExtraJoints();
}

void AGXBody::setExtraJoints()
{
    BodyPtr const body = this->body();
    const int n = body->numExtraJoints();
    for(int j=0; j < n; ++j){
        ExtraJoint& extraJoint = body->extraJoint(j);

        AGXLinkPtr agxLinkPair[2];
        agxLinkPair[0] = getAGXLink(extraJoint.link[0]->index());
        agxLinkPair[1] = getAGXLink(extraJoint.link[1]->index());
        if(!agxLinkPair[0] || !agxLinkPair[1]) continue;

        Link* const link = extraJoint.link[0];
        const Vector3 p = link->attitude() * extraJoint.point[0] + link->p();
        const Vector3 a = link->attitude() * extraJoint.axis;
        agx::ConstraintRef constraint = nullptr;
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

std::string AGXBody::getSelfCollisionGroupName() const
{
    return _selfCollisionGroupName;
}

void AGXBody::setCollision(const bool& bOn)
{
    for(int i = 0; i < numAGXLinks(); ++i){
        getAGXLink(i)->setCollision(bOn);
    }
}

void AGXBody::setAGXMaterial(const int & index, const agx::MaterialRef& mat)
{
    getAGXLink(index)->getAGXGeometry()->setMaterial(mat);
}

void AGXBody::setControlInputToAGX()
{
    for(int i = 0; i < numControllableLinks(); ++i){
        getControllableLink(i)->setControlInputToAGX();
    }
    //// Skip the root link
    //for(int i = 1; i < numAGXLinks(); ++i){
    //    getAGXLink(i)->setControlInputToAGX();
    //}
}

void AGXBody::setLinkStateToAGX()
{
    for(int i = 0; i < numAGXLinks(); ++i){
        getAGXLink(i)->setLinkStateToAGX();
    }
}

void AGXBody::setLinkStateToCnoid()
{
    for(int i = 0; i < numAGXLinks(); ++i){
        getAGXLink(i)->setLinkStateToCnoid();
    }
}

int AGXBody::numAGXLinks() const
{
    return _agxLinks.size();
}

void AGXBody::addAGXLink(AGXLinkPtr const agxLink)
{
    _agxLinks.push_back(agxLink);
}

AGXLinkPtr AGXBody::getAGXLink(const int& index) const
{
    return _agxLinks[index];
}

int AGXBody::numControllableLinks() const
{
    return _controllableLinks.size();
}

void AGXBody::addControllableLink(AGXLinkPtr const agxLink)
{
    _controllableLinks.push_back(agxLink);
}

AGXLinkPtr AGXBody::getControllableLink(const int & index) const
{
    return _controllableLinks[index];
}

agx::RigidBodyRef AGXBody::getAGXRigidBody(const int& index) const
{
    return getAGXLink(index)->getAGXRigidBody();
}

agx::ConstraintRef AGXBody::getAGXConstraint(const int& index) const
{
    return getAGXLink(index)->getAGXConstraint();
}

int AGXBody::numAGXExtraConstraints() const
{
    return _agxExtraConstraints.size();
}

agx::ConstraintRef AGXBody::getAGXExtraConstraint(const int& index) const
{
    return _agxExtraConstraints[index];
}

void AGXBody::addAGXExtraConstraint(agx::ConstraintRef constraint)
{
    _agxExtraConstraints.push_back(constraint);
}

unsigned int AGXBody::generateUID()
{
    static unsigned int i = 0;
    i++;
    return i;
}

}
