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
    _origin = parentOrigin + link->b();
    const Link::ActuationMode& actuationMode = link->actuationMode();
    if(parent && actuationMode != Link::ActuationMode::NO_ACTUATION) agxBody->addControllableLink(this);
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
    switch(getOrgLink()->actuationMode()){
        case Link::ActuationMode::JOINT_TORQUE :{
            setTorqueToAGX();
            break;
        }
        case Link::ActuationMode::JOINT_VELOCITY :{
            setVelocityToAGX();
            break;
        }
        case Link::ActuationMode::JOINT_DISPLACEMENT :{
            //setPositionToAGX();
            break;
        }
        case Link::ActuationMode::NO_ACTUATION :
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
                orgLink->u() = joint1DOF->getMotor1D()->getCurrentForce();
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

    //const agx::RigidBody::AutoSleepProperties& p = agxRigidBody->getAutoSleepProperties();
    //std::cout << agxRigidBody->getName() << " : " << p.getEnable() << " " << p.getState() << std::endl;
}

int AGXLink::getIndex() const
{
    return getOrgLink()->index();
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

agx::RigidBodyRef AGXLink::getAGXRigidBody() const
{
    return _rigid;
}

agxCollide::GeometryRef AGXLink::getAGXGeometry() const
{
    return _geometry;
}

void AGXLink::setAGXConstraint(agx::ConstraintRef const constraint)
{
    _constraint = constraint;
}

agx::ConstraintRef AGXLink::getAGXConstraint() const
{
    return _constraint;
}

std::string AGXLink::getSelfCollisionGroupName() const
{
    return _selfCollisionGroupName;
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
                } else if(mesh->primitiveType() == SgMesh::CAPSULE){
                    if(scale.x() == scale.z()){
                        doAddPrimitive = true;
                    }
                }
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
            if(orgLink->actuationMode() != Link::ActuationMode::NO_ACTUATION) desc.isMotorOn = true;
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
            if(orgLink->actuationMode() != Link::ActuationMode::NO_ACTUATION) desc.isMotorOn = true;
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
    _agxBodyParts.clear();
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
    createContinuousTrack();
    createExtraJoint();
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
    //setExtraJoints();
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

bool AGXBody::hasForceSensors() const
{
    if(sensorHelper.forceSensors().empty()) return false;
    return true;
}

bool AGXBody::hasGyroOrAccelerationSensors() const {
    return sensorHelper.hasGyroOrAccelerationSensors();
}

void AGXBody::setSensor(const double& timeStep, const Vector3 &gravity)
{
    sensorHelper.initialize(body(), timeStep, gravity);
    const DeviceList<ForceSensor> &forceSensors = sensorHelper.forceSensors();
    for (size_t i = 0; i < forceSensors.size(); ++i) {
        AGXLinkPtr agxLink = getAGXLink(forceSensors[i]->link()->index());
        agxLink->getAGXConstraint()->setEnableComputeForces(true);
    }
}

void AGXBody::updateForceSensors()
{
    const DeviceList<ForceSensor>& forceSensors = sensorHelper.forceSensors();
    for(size_t i=0; i < forceSensors.size(); ++i) {
        ForceSensor *sensor = forceSensors[i];
        AGXLinkPtr agxLink = getAGXLink(sensor->link()->index());
        if (agxLink && agxLink->getAGXParentLink() && agxLink->getAGXConstraint()) {
            agx::Vec3 force, torque;
            agxLink->getAGXConstraint()->getLastForce(agxLink->getAGXParentLink()->getAGXRigidBody(), force, torque, false);
            Vector3 f(force[0], force[1], force[2]);
            Vector3 tau(torque[0], torque[1], torque[2]);
            const Matrix3 R = sensor->link()->R() * sensor->R_local();
            const Vector3 p = sensor->link()->R() * sensor->p_local();
            sensor->f() = R.transpose() * f;
            sensor->tau() = R.transpose() * (tau - p.cross(f));
            sensor->notifyStateChange();
        }
    }
}

void AGXBody::updateGyroAndAccelerationSensors()
{
    sensorHelper.updateGyroAndAccelerationSensors();
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

int AGXBody::numAGXBodyParts() const
{
    return _agxBodyParts.size();
}

void AGXBody::addAGXBodyPart(AGXBodyPartPtr const bp)
{
    _agxBodyParts.push_back(bp);
}

AGXBodyPartPtr AGXBody::getAGXBodyPart(const int & index) const
{
    return _agxBodyParts[index];
}


bool AGXBody::findAGXLinksFromInfo(const std::string& key, const bool& defaultValue, AGXLinkPtrs& agxLinks) const
{
    agxLinks.clear();
    for(int i = 0; i < numAGXLinks(); ++i){
        AGXLinkPtr agxLink =  getAGXLink(i);
        if(agxLink->getOrgLink()->info(key, defaultValue)) agxLinks.push_back(agxLink);
    }
    if(agxLinks.size() > 0) return true;
    return false;
}

void AGXBody::createExtraJoint()
{
    if(this->body()->numExtraJoints() > 0) 
        addAGXBodyPart(new AGXExtraJoint(this));
}

void AGXBody::createContinuousTrack()
{
    AGXLinkPtrs agxLinks;
    if(!findAGXLinksFromInfo("isContinuousTrack", false, agxLinks)) return;
    for(int i = 0; i < agxLinks.size(); ++i){
        addAGXBodyPart(new AGXContinousTrack(agxLinks[i], this));
    }
}

////////////////////////////////////////////////////////////
// AGXBodyPart
AGXBodyPart::AGXBodyPart()
{
    _hasSelfCollisionGroupName = false;
    _constraints.clear();
}

bool AGXBodyPart::hasSelfCollisionGroupName() const
{
    return _hasSelfCollisionGroupName;
}

std::string AGXBodyPart::getSelfCollisionGroupName() const 
{
    return _selfCollisionGroupName; 
}

int AGXBodyPart::numAGXConstraints() const 
{ 
    return _constraints.size(); 
}

agx::ConstraintRef AGXBodyPart::getAGXConstraint(const int& index) const 
{
    return  _constraints[index]; 
}

void AGXBodyPart::setSelfCollsionGroupName(const std::string & name)
{
    _selfCollisionGroupName = name;
    _hasSelfCollisionGroupName = true;
}

void AGXBodyPart::addAGXConstraint(agx::ConstraintRef const constraint)
{
    _constraints.push_back(constraint);
}

////////////////////////////////////////////////////////////
// AGXExtraJoint
AGXExtraJoint::AGXExtraJoint(AGXBodyPtr agxBody)
{
    createJoints(agxBody);
}

void AGXExtraJoint::createJoints(AGXBodyPtr agxBody)
{
    BodyPtr const body = agxBody->body();
    const int n = body->numExtraJoints();
    for(int j=0; j < n; ++j){
        ExtraJoint& extraJoint = body->extraJoint(j);

        AGXLinkPtr agxLinkPair[2];
        agxLinkPair[0] = agxBody->getAGXLink(extraJoint.link[0]->index());
        agxLinkPair[1] = agxBody->getAGXLink(extraJoint.link[1]->index());
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
        addAGXConstraint(constraint);
    }
}


////////////////////////////////////////////////////////////
// AGXContinousTrack
AGXContinousTrack::AGXContinousTrack(AGXLinkPtr footLinkStart, AGXBodyPtr body)
{
    std::stringstream ss;
    ss.str("");
    ss << "SelfCollisionContinousTrack" << generateUID() << body->bodyItem()->name() << std::flush;
    setSelfCollsionGroupName(ss.str());
    _feet.clear();
    _chassisLink = footLinkStart->getAGXParentLink();
    _feet.push_back(footLinkStart);
    addFoot(footLinkStart->getOrgLink(), body);
    createTrackConstraint();
}

void AGXContinousTrack::addFoot(LinkPtr link, AGXBodyPtr body)
{
    for(Link* child = link->child(); child; child = child->sibling()){
        _feet.push_back(body->getAGXLink(child->index()));
        addFoot(child, body);
    }
}

void AGXContinousTrack::createTrackConstraint()
{
        AGXLinkPtr agxFootLinkStart = _feet[0];
        AGXLinkPtr agxFootLinkEnd = _feet[_feet.size()-1];

        // Connect footLinkStart and footLinkEnd with Hinge
        // Create Hinge with world coordination
        LinkPtr link = agxFootLinkStart->getOrgLink();
        const Vector3& a = link->a();                       // local
        const Vector3 aw = link->attitude() * a;            // world
        const Vector3& p = link->p();                       // world
        AGXHingeDesc hd;
        hd.frameAxis   = agx::Vec3(aw(0), aw(1), aw(2));
        hd.frameCenter = agx::Vec3(p(0), p(1), p(2));
        hd.rigidBodyA = agxFootLinkStart->getAGXRigidBody();
        hd.rigidBodyB = agxFootLinkEnd->getAGXRigidBody();
        hd.isMotorOn = false;
        agx::ConstraintRef constraint = AGXObjectFactory::createConstraint(hd);   // needs addsimulation
        link->setJointType(Link::ROTATIONAL_JOINT);
        agxFootLinkStart->setAGXConstraint(constraint);
        addAGXConstraint(constraint);

        // Create PlaneJoint to prvent the track falling off
        // Create joint with parent(example:chasis) coordination 
        const Vector3& b = link->b();
        const agx::Vec3 a0 = agx::Vec3(a(0), a(1), a(2));   // Rotate axis of track. local
        const agx::Vec3& b0 = agx::Vec3(b(0), b(1), b(2));  // Vector from parent to footLinkStart
        const agx::Vec3& p0 = ( a0 * b0 ) * a0;             // Middle position of track(projection vector of b0 to a0). local
        agx::Vec3 nx = agx::Vec3::Z_AXIS().cross(a0);       // Check a0 is parallel to Z. X-Y consists plane joint.
        agx::OrthoMatrix3x3 rotation;
        if(nx.normalize() > 1.0e-6){
            nx.normal();
            rotation.setColumn(0, nx);
            agx::Vec3 ny = a0.cross(nx);
            ny.normal();
            rotation.setColumn(1, ny);
            rotation.setColumn(2, a0);
        }
        agx::AffineMatrix4x4 af( rotation, p0 );
        AGXPlaneJointDesc pd;
        pd.frameB = AGXObjectFactory::createFrame();
        pd.frameB->setMatrix(af);
        pd.rigidBodyB = agxFootLinkStart->getAGXParentLink()->getAGXRigidBody();

        for(int i = 0;  i < _feet.size(); ++i){
            AGXLinkPtr agxLink = _feet[i];
            // Add plane joint
            pd.frameA = AGXObjectFactory::createFrame();
            pd.rigidBodyA = agxLink->getAGXRigidBody();
            agx::PlaneJointRef pj = AGXObjectFactory::createConstraintPlaneJoint(pd);
            addAGXConstraint((agx::ConstraintRef)pj);

            // Enable collision between tracks and the others. Need to contact with wheels.
            agxLink->getAGXGeometry()->removeGroup(agxLink->getSelfCollisionGroupName());
            agxLink->getAGXGeometry()->addGroup(getSelfCollisionGroupName());
        }
}




}

