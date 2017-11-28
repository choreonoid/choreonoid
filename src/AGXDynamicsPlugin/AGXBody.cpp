#include "AGXBody.h"
#include "AGXScene.h"
#include <cnoid/SceneDrawables>
#include "AGXVehicleContinuousTrack.h"

using namespace std;

namespace {
std::mutex agxBodyExtensionAdditionalFuncsMutex;
AGXBodyExtensionFuncMap agxBodyExtensionAdditionalFuncs;
};

namespace cnoid{
////////////////////////////////////////////////////////////
// AGXLink
AGXLink::AGXLink(Link* const link) : _orgLink(link){}
AGXLink::AGXLink(Link* const link, AGXLink* const parent, const Vector3& parentOrigin, AGXBody* const agxBody) :
    _agxBody(agxBody),
    _orgLink(link),
    _agxParentLink(parent),
    _origin(parentOrigin + link->b())
{
    agxBody->addAGXLink(this);
    const Link::ActuationMode& actuationMode = link->actuationMode();
    if(actuationMode == Link::ActuationMode::NO_ACTUATION){
    }else if(actuationMode == Link::ActuationMode::LINK_POSITION){
        agxBody->addControllableLink(this);
    }else if(parent){
        agxBody->addControllableLink(this);
    }

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

    agxSDK::SimulationRef sim =  getAGXBody()->getAGXScene()->getSimulation();
    sim->add(_rigid);
    sim->add(_constraint);
    //printDebugInfo();
}

void AGXLink::setAGXMaterial(){
    // Check density is written in body file
    double density = 0.0;
    bool bDensity = getOrgLink()->info()->read("density", density);

    // Set material
    string matName = "";
    auto matNameNode = getOrgLink()->info()->find("materialName");
    if(matNameNode->isValid()) matName = matNameNode->toString();
    if(setAGXMaterialFromName(matName)){
    }else{
        setAGXMaterialFromLinkInfo();
        if(!bDensity){
            setCenterOfMassFromLinkInfo();
            setMassFromLinkInfo();
            setInertiaFromLinkInfo();
        }
    }

    // if density is written in body file, we use this density
    if(bDensity) getAGXGeometry()->getMaterial()->getBulkMaterial()->setDensity(density);
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
void AGXLink::setAGXMaterialFromLinkInfo()
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
        case Link::ActuationMode::LINK_POSITION :{
            setLinkPositionToAGX();
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

Link* AGXLink::getOrgLink() const
{
    return _orgLink;
}

AGXLink* AGXLink::getAGXParentLink() const
{
    return _agxParentLink;
}

agx::RigidBody* AGXLink::getAGXRigidBody() const
{
    return _rigid;
}

agxCollide::Geometry* AGXLink::getAGXGeometry() const
{
    return _geometry;
}

void AGXLink::setAGXConstraint(agx::Constraint* const constraint)
{
    _constraint = constraint;
}

agx::Constraint* AGXLink::getAGXConstraint() const
{
    return _constraint;
}

AGXBody* AGXLink::getAGXBody()
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
    if(orgLink->isRoot() && jt == Link::FIXED_JOINT){
        desc.control = agx::RigidBody::MotionControl::STATIC;
    }

    string agxMotion = "";
    auto agxMotionNode = getOrgLink()->info()->find("AGXMotion");
    if(agxMotionNode->isValid()){
        agxMotion = agxMotionNode->toString();
        if(agxMotion == "kinematics") desc.control = agx::RigidBody::MotionControl::KINEMATICS;
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
    AGXLink* const agxParentLink = getAGXParentLink();
    if(!agxParentLink) return nullptr;
    Link* const orgLink = getOrgLink();
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
            // motor
            if(orgLink->actuationMode() != Link::ActuationMode::NO_ACTUATION) desc.motor.enable = true;
            // lock
            if(orgLink->actuationMode() == Link::ActuationMode::JOINT_ANGLE) desc.lock.enable = true;
            // range
            desc.range.enable = true;
            desc.range.range = agx::RangeReal(orgLink->q_lower(), orgLink->q_upper());
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
            // range
            desc.range.enable = true;
            desc.range.range = agx::RangeReal(orgLink->q_lower(), orgLink->q_upper());
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

void AGXLink::setLinkPositionToAGX()
{
    LinkPtr orgLink = getOrgLink();
    const Vector3& p = orgLink->p();
    getAGXRigidBody()->setPosition(p(0), p(1), p(2));
}

#define PRINT_DEBUGINFO(FIELD1, FIELD2) std::cout << #FIELD1 << " " << FIELD2 << std::endl;
void AGXLink::printDebugInfo()
{
    PRINT_DEBUGINFO("DEBUG", "---------------------------")
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
    setCollision();
}

void AGXBody::setCollision()
{
    AGXSceneRef agxScene = getAGXScene();
    // Disable collision
    for(const auto& name : getCollisionGroupNamesToDisableCollision()){
        agxScene->setCollision(name, false);
    }
    // Set self collision
    agxScene->setCollision(getCollisionGroupName(), bodyItem()->isSelfCollisionDetectionEnabled());
    // Set external collision
    enableExternalCollision(bodyItem()->isCollisionDetectionEnabled());
}

std::string AGXBody::getCollisionGroupName() const
{
    return _bodyCollisionGroupName;
}

void AGXBody::enableExternalCollision(const bool& bOn)
{
    for(const auto& agxLink : getAGXLinks()){
        agxLink->enableExternalCollision(bOn);
    }
}

void AGXBody::addCollisionGroupNameToDisableCollision(const std::string & name)
{
    return _collisionGroupNamesToDisableCollision.push_back(name);
}

const std::vector<std::string>& AGXBody::getCollisionGroupNamesToDisableCollision() const
{
    return _collisionGroupNamesToDisableCollision;
}

void AGXBody::addCollisionGroupNameToAllLink(const std::string& name)
{
    for(const auto& agxLink : getAGXLinks()){
        agxLink->getAGXGeometry()->addGroup(name);
    }
}

void AGXBody::setAGXMaterial(const int & index, agx::Material* const mat)
{
    getAGXLink(index)->getAGXGeometry()->setMaterial(mat);
}

void AGXBody::setControlInputToAGX()
{
    for(const auto& clink : getControllableLinks()){
        clink->setControlInputToAGX();
    }
}

void AGXBody::setLinkStateToAGX()
{
    for(const auto& agxLink : getAGXLinks()){
        agxLink->setLinkStateToAGX();
    }
}

void AGXBody::setLinkStateToCnoid()
{
    for(const auto& agxLink : getAGXLinks()){
        agxLink->setLinkStateToCnoid();
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
    for(const auto& fs : forceSensors){
        AGXLink* agxLink = getAGXLink(fs->link()->index());
        agxLink->getAGXConstraint()->setEnableComputeForces(true);
    }
}

void AGXBody::updateForceSensors()
{
    const DeviceList<ForceSensor>& forceSensors = sensorHelper.forceSensors();
    for(const auto& fs : forceSensors){
        AGXLink* agxLink = getAGXLink(fs->link()->index());
        if (agxLink && agxLink->getAGXParentLink() && agxLink->getAGXConstraint()) {
            agx::Vec3 force, torque;
            agxLink->getAGXConstraint()->getLastForce(agxLink->getAGXParentLink()->getAGXRigidBody(), force, torque, false);
            Vector3 f(force[0], force[1], force[2]);
            Vector3 tau(torque[0], torque[1], torque[2]);
            const Matrix3 R = fs->link()->R() * fs->R_local();
            const Vector3 p = fs->link()->R() * fs->p_local();
            fs->f() = R.transpose() * f;
            fs->tau() = R.transpose() * (tau - p.cross(f));
            fs->notifyStateChange();
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

void AGXBody::addAGXLink(AGXLink* const agxLink)
{
    _agxLinks.push_back(agxLink);
}

AGXLink* AGXBody::getAGXLink(const int& index) const
{
    return _agxLinks[index];
}

AGXLink* AGXBody::getAGXLink(const std::string & name) const
{
    Link* link = body()->link(name);
    if(!link) return nullptr;
    return getAGXLink(link->index());
}

const AGXLinkPtrs& AGXBody::getAGXLinks() const
{
    return _agxLinks;
}

int AGXBody::numControllableLinks() const
{
    return _controllableLinks.size();
}

void AGXBody::addControllableLink(AGXLink* const agxLink)
{
    _controllableLinks.push_back(agxLink);
}

AGXLink* AGXBody::getControllableLink(const int & index) const
{
    return _controllableLinks[index];
}

const AGXLinkPtrs& AGXBody::getControllableLinks() const
{
    return _controllableLinks;
}

agx::RigidBodyRef AGXBody::getAGXRigidBody(const int& index) const
{
    return getAGXLink(index)->getAGXRigidBody();
}

agx::RigidBody* AGXBody::getAGXRigidBody(const std::string& linkName) const
{
    return getAGXLink(linkName)->getAGXRigidBody();
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
    for(const auto& func : agxBodyExtensionFuncs){
        func.second(this);
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
    for(const auto& agxLink : getAGXLinks()){
        if(agxLink->getOrgLink()->info(key, defaultValue)) agxLinks.push_back(agxLink);
    }
    if(agxLinks.empty()) return false;
    return true;
}

void AGXBody::createExtraJoint()
{
    if(this->body()->numExtraJoints() > 0) 
        addAGXBodyExtension(new AGXExtraJoint(this));
}

bool AGXBody::createContinuousTrack(AGXBody* agxBody)
{
    AGXLinkPtrs myAgxLinks;
    if(!getAGXLinksFromInfo("isContinuousTrack", false, myAgxLinks)) return false;
    for(const auto& agxLink : myAgxLinks){
        addAGXBodyExtension(new AGXContinousTrack(agxLink, this));
    }
    return true;
}

bool AGXBody::createAGXVehicleContinousTrack(AGXBody* agxBody)
{
    DeviceList<> devices = this->body()->devices();
    DeviceList<AGXVehicleContinuousTrackDevice> conTrackDevices;
    conTrackDevices.extractFrom(devices);
    for(const auto& ctd : conTrackDevices){
        addAGXBodyExtension(new AGXVehicleContinuousTrack(ctd, this));
    }
    return true;
}


}

