#include "AGXBody.h"
#include "AGXScene.h"
#include <cnoid/SceneDrawables>
#include "AGXVehicleContinuousTrack.h"

namespace {
std::mutex agxBodyExtensionAdditionalFuncsMutex;
AGXBodyExtensionFuncMap agxBodyExtensionAdditionalFuncs;
};

namespace cnoid{
////////////////////////////////////////////////////////////
// AGXLink
AGXLink::AGXLink(const LinkPtr link) : _orgLink(link){}
AGXLink::AGXLink(const LinkPtr link, const AGXLinkPtr parent, const Vector3& parentOrigin, const AGXBodyPtr agxBody)
{
    _agxBody = agxBody;
    _orgLink = link;
    _agxParentLink = parent;
    agxBody->addAGXLink(this);
    _origin = parentOrigin + link->b();
    const Link::ActuationMode& actuationMode = link->actuationMode();
    if(parent && actuationMode != Link::ActuationMode::NO_ACTUATION) agxBody->addControllableLink(this);
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
    setAGXMaterial();
    _constraint = createAGXConstraint();
    //printDebugInfo();
}

void AGXLink::setAGXMaterial(){
    string matName = "";
    auto matNameNode = getOrgLink()->info()->find("materialName");
    if(matNameNode->isValid()) matName = matNameNode->toString();
    if(setAGXMaterialFromName(matName)){
        /* success to set material from material name */
    }else if(setAGXMaterialFromLinkInfo()){
        /* success to set material from yaml */
    }

    // set center of mass, mass, inertia
    double density = 0.0;
    if(getOrgLink()->info()->read("density", density)){
        /* if density is set, we use density for center of mass, mass, inertia */
    }else{
        setCenterOfMassFromLinkInfo();
        setMassFromLinkInfo();
        setInertiaFromLinkInfo();
    }
}

bool AGXLink::setAGXMaterialFromName(const std::string& materialName)
{
    agxSDK::SimulationRef simulation = getAGXBody()->getAGXScene()->getSimulation();
    if(!simulation) return false;
    agx::MaterialRef mat = simulation->getMaterial(materialName);
    if(!mat) return false;
    getAGXGeometry()->setMaterial(mat);
    getAGXRigidBody()->updateMassProperties(agx::MassProperties::AUTO_GENERATE_ALL);
    return true;
}

#define SET_AGXMATERIAL_FIELD(field) desc.field = getOrgLink()->info<double>(#field, desc.field)
bool AGXLink::setAGXMaterialFromLinkInfo()
{
    AGXMaterialDesc desc;
    std::stringstream ss;
    ss << "AGXMaterial" << generateUID() << std::endl;
    desc.name = ss.str();
    SET_AGXMATERIAL_FIELD(density);
    SET_AGXMATERIAL_FIELD(youngsModulus);
    SET_AGXMATERIAL_FIELD(poissonRatio);
    SET_AGXMATERIAL_FIELD(viscosity);
    SET_AGXMATERIAL_FIELD(damping);
    SET_AGXMATERIAL_FIELD(surfaceViscosity);
    SET_AGXMATERIAL_FIELD(adhesionForce);
    SET_AGXMATERIAL_FIELD(adhesivOverlap);
    desc.roughness = getOrgLink()->info<double>("friction", desc.roughness);
    agx::MaterialRef mat = AGXObjectFactory::createMaterial(desc);
    getAGXGeometry()->setMaterial(mat);
    getAGXRigidBody()->updateMassProperties(agx::MassProperties::AUTO_GENERATE_ALL);
    return true;
}
#undef SET_AGXMATERIAL_FIELD

bool AGXLink::setCenterOfMassFromLinkInfo()
{
    const Vector3& c(getOrgLink()->c());
    const agx::Vec3 ca(c(0), c(1), c(2));
    getAGXRigidBody()->setCmLocalTranslate(ca);
    return true;
}

bool AGXLink::setMassFromLinkInfo()
{
    const double& m = getOrgLink()->m();
    if(m <= 0.0) return false;
    getAGXRigidBody()->getMassProperties()->setMass(m, false);
    return true;
}

bool AGXLink::setInertiaFromLinkInfo()
{
    const Matrix3& I = getOrgLink()->I();
    if(I.isZero()) return false;
    agx::SPDMatrix3x3 Ia;
    Ia.set( I(0,0), I(1,0), I(2,0),
            I(0,1), I(1,1), I(2,1),
            I(0,2), I(1,2), I(2,2));
    getAGXRigidBody()->getMassProperties()->setInertiaTensor(Ia, false);
    return true;
}

void AGXLink::enableExternalCollision(const bool & bOn)
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
        case Link::ActuationMode::JOINT_VELOCITY :
        case Link::ActuationMode::JOINT_SURFACE_VELOCITY :{
            setVelocityToAGX();
            break;
        }
        case Link::ActuationMode::JOINT_ANGLE :{
            setPositionToAGX();
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

AGXBody * AGXLink::getAGXBody()
{
    return _agxBody;
}

agx::RigidBodyRef AGXLink::createAGXRigidBody()
{
    LinkPtr orgLink = getOrgLink();
    const Vector3& v = orgLink->v(); 
    const Vector3& w = orgLink->w(); 
    const Vector3& p = getOrigin();

    AGXRigidBodyDesc desc;
    desc.name = orgLink->name();
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
    gdesc.selfCollsionGroupName = getAGXBody()->getCollisionGroupName();
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
                    cd.height =  cylinder.height * scale.y();
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
            if(orgLink->actuationMode() != Link::ActuationMode::NO_ACTUATION) desc.motor.enable = true;
            if(orgLink->actuationMode() == Link::ActuationMode::JOINT_ANGLE) desc.lock.enable = true;
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
            if(orgLink->actuationMode() != Link::ActuationMode::NO_ACTUATION) desc.motor.enable = true;
            if(orgLink->actuationMode() == Link::ActuationMode::JOINT_ANGLE) desc.lock.enable = true;
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

#define PRINT_DEBUGINFO(FIELD1, FIELD2) std::cout << #FIELD1 << " " << FIELD2 << std::endl;
void AGXLink::printDebugInfo()
{
    PRINT_DEBUGINFO("name", getOrgLink()->name());
    PRINT_DEBUGINFO("agxcenterofmass", getAGXRigidBody()->getCmLocalTranslate());
    PRINT_DEBUGINFO("agxmass", getAGXRigidBody()->getMassProperties()->getMass());
    PRINT_DEBUGINFO("agxinertia", getAGXRigidBody()->getMassProperties()->getInertiaTensor());
    PRINT_DEBUGINFO("cnoidcenterofmass", getOrgLink()->c());
    PRINT_DEBUGINFO("cnoidmass", getOrgLink()->m());
    PRINT_DEBUGINFO("cnoidinertia", getOrgLink()->I());
}
#undef  PRINT_DEBUGINFO

////////////////////////////////////////////////////////////
// AGXBody
AGXBody::AGXBody(Body& orgBody) : SimulationBody(new Body(orgBody)) {}

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
	_agxBodyExtensions.clear();
    _collisionGroupNamesToDisableCollision.clear();
    std::stringstream ss;
    ss.str("");
    ss << generateUID() << body->name() << std::flush;
    _bodyCollisionGroupName = ss.str();
    return;
}

void AGXBody::createBody(AGXScene* agxScene)
{
    initialize();
    _agxScene = agxScene;
    // Create AGXLink following child link.
    new AGXLink(body()->rootLink(), nullptr, Vector3::Zero(), this);
    setLinkStateToAGX();
    createExtraJoint();
    callExtensionFuncs();
}

std::string AGXBody::getCollisionGroupName() const
{
    return _bodyCollisionGroupName;
}

void AGXBody::enableExternalCollision(const bool& bOn)
{
    for(int i = 0; i < numAGXLinks(); ++i){
        getAGXLink(i)->enableExternalCollision(bOn);
    }
}

void AGXBody::addCollisionGroupNameToDisableCollision(const std::string & name)
{
    return _collisionGroupNamesToDisableCollision.push_back(name);
}

const VectorString& AGXBody::getCollisionGroupNamesToDisableCollision() const
{
    return _collisionGroupNamesToDisableCollision;
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

AGXScene* AGXBody::getAGXScene() const
{
    return _agxScene;
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

AGXLinkPtr AGXBody::getAGXLink(const std::string & name) const
{
    Link* link = body()->link(name);
    if(!link) return nullptr;
    return getAGXLink(link->index());
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

bool AGXBody::addAGXBodyExtension(AGXBodyExtension* const extension)
{
    _agxBodyExtensions.push_back(extension);
    return false;
}

const AGXBodyExtensionPtrs& AGXBody::getAGXBodyExtensions() const
{
    return _agxBodyExtensions;
}

void AGXBody::callExtensionFuncs(){
    // update func list
    updateAGXBodyExtensionFuncs();
    //agxBodyExtensionFuncs["hoge"] = [](AGXBody* agxBody){ std::cout << "hoge" << std::endl; return false;};
    agxBodyExtensionFuncs["ContinousTrack"] = [&](AGXBody* agxBody){ return createContinuousTrack(agxBody); };
    agxBodyExtensionFuncs["AGXVehicleContinousTrack"] = [&](AGXBody* agxBody){ return createAGXVehicleContinousTrack(this); };

    // call
    //agxBodyExtensionFuncs["AGXBodyExtensionSample"](this);
    for(auto it = agxBodyExtensionFuncs.begin(); it !=agxBodyExtensionFuncs.end(); ++it){
        (*it).second(this);
    }
}

void AGXBody::addAGXBodyExtensionAdditionalFunc(const std::string& typeName,
    std::function<bool(AGXBody* agxBody)> func){
    std::lock_guard<std::mutex> guard(agxBodyExtensionAdditionalFuncsMutex);
    agxBodyExtensionAdditionalFuncs[typeName] = func;
}

void AGXBody::updateAGXBodyExtensionFuncs(){
    std::lock_guard<std::mutex> guard(agxBodyExtensionAdditionalFuncsMutex);
    if(agxBodyExtensionAdditionalFuncs.size() > agxBodyExtensionFuncs.size()){
        for(auto& p : agxBodyExtensionAdditionalFuncs){
            AGXBodyExtensionFunc& func = p.second;
            agxBodyExtensionFuncs[p.first] = func;
        }
    }
}

bool AGXBody::getAGXLinksFromInfo(const std::string& key, const bool& defaultValue, AGXLinkPtrs& agxLinks) const
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
        addAGXBodyExtension(new AGXExtraJoint(this));
}

bool AGXBody::createContinuousTrack(AGXBody* agxBody)
{
    AGXLinkPtrs agxLinks;
    if(!getAGXLinksFromInfo("isContinuousTrack", false, agxLinks)) return false;
    for(int i = 0; i < agxLinks.size(); ++i){
        addAGXBodyExtension(new AGXContinousTrack(agxLinks[i], this));
    }
    return true;
}

bool AGXBody::createAGXVehicleContinousTrack(AGXBody* agxBody)
{
    DeviceList<> devices = this->body()->devices();
    DeviceList<AGXVehicleContinuousTrackDevice> conTrackDevices;
    conTrackDevices.extractFrom(devices);
    for(int i = 0; i < conTrackDevices.size(); ++i){
        addAGXBodyExtension(new AGXVehicleContinuousTrack(conTrackDevices[i], this));
    }
    return true;
}


}

