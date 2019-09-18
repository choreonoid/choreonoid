#include "AGXBody.h"
#include "AGXScene.h"
#include <cnoid/MeshExtractor>
#include <cnoid/SceneDrawables>
#include <cnoid/ForceSensor>
#include <mutex>
#include "AGXVehicleContinuousTrack.h"
#include "AGXConvexDecomposition.h"
#include "AGXConvert.h"
#include "AGXObjectFactory.h"

using namespace std;

namespace {
std::mutex agxBodyExtensionAdditionalFuncsMutex;
AGXBodyExtensionFuncMap agxBodyExtensionAdditionalFuncs;
}

namespace cnoid{
////////////////////////////////////////////////////////////
// AGXBodyExtension
bool createContinuousTrack(AGXBody* agxBody)
{
    AGXLinkPtrs myAgxLinks;
    if(!agxBody->getAGXLinksFromInfo("isContinuousTrack", false, myAgxLinks)) return false;
    for(const auto& agxLink : myAgxLinks){
        agxBody->addAGXBodyExtension(new AGXContinuousTrack(agxLink, agxBody));
    }
    return true;
}

bool createAGXVehicleContinousTrack(AGXBody* agxBody)
{
    DeviceList<> devices = agxBody->body()->devices();
    DeviceList<AGXVehicleContinuousTrackDevice> conTrackDevices;
    conTrackDevices.extractFrom(devices);
    for(const auto& ctd : conTrackDevices){
        agxBody->addAGXBodyExtension(new AGXVehicleContinuousTrack(ctd, agxBody));
    }
    return true;
}

////////////////////////////////////////////////////////////
// AGXLink
AGXLink::AGXLink(Link* const link) : _orgLink(link){}
AGXLink::AGXLink(Link* const link, AGXLink* const parent, const Vector3& parentOrigin, AGXBody* const agxBody, std::set<Link*>& forceSensorLinks, bool makeStatic) :
    _agxBody(agxBody),
    _orgLink(link),
    _agxParentLink(parent),
    _origin(parentOrigin + link->b())
{
    agxBody->addAGXLink(this);
    std::stringstream ss;
    ss << agx::UuidGenerator().generate().str() << link->name() << std::flush;
    _collisionGroupName = ss.str();

    const Link::ActuationMode& actuationMode = link->actuationMode();
    if(actuationMode == Link::ActuationMode::NO_ACTUATION){
    }else if(actuationMode == Link::ActuationMode::LINK_POSITION){
        agxBody->addControllableLink(this);
    }else if(parent){
        agxBody->addControllableLink(this);
    }

    if(link->jointType() != Link::FIXED_JOINT || forceSensorLinks.find(link) != forceSensorLinks.end()){
        makeStatic = false;
    }

    constructAGXLink(makeStatic);
    for(Link* child = link->child(); child; child = child->sibling()){
        new AGXLink(child, this, getOrigin(), agxBody, forceSensorLinks, makeStatic);
    }
}

void AGXLink::constructAGXLink(const bool& makeStatic)
{
    _rigid = createAGXRigidBody();
    _geometry = createAGXGeometry();
    _rigid->add(_geometry);
    _geometry->addGroup(getCollisionGroupName());
    createAGXShape();
    setAGXMaterial();
    if(makeStatic){
        _rigid->setMotionControl(agx::RigidBody::STATIC);
    }else{
        _constraint = createAGXConstraint();
    }

    agxSDK::SimulationRef sim =  getAGXBody()->getAGXScene()->getSimulation();
    sim->add(_rigid);
    bool enableAMOR = false;
    enableAMOR = getOrgLink()->info("AMOR", enableAMOR);
    sim->getMergeSplitHandler()->getOrCreateProperties(_rigid)->setEnableMergeSplit(enableAMOR);
    sim->add(_constraint);
    //printDebugInfo();
}

void AGXLink::setAGXMaterial(){
    Mapping* mapping = getOrgLink()->info();
    // Set material
    string materialName = mapping->get("material", "default");
    if(materialName == "useLinkInfo"){
        setAGXMaterialFromLinkInfo();
    }else{
        setAGXMaterialFromName(getOrgLink()->materialName());
    }

    // Set density or mass
    string massType = mapping->get("massType", "mass");
    if(massType == "density"){
        double density;
        if(mapping->read("density", density)){
            getAGXGeometry()->getMaterial()->getBulkMaterial()->setDensity(density);
        }
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
    if(!mat){
        std::cout << "AGXDynamicsPlugin:Warning " << "Failed to set material " << materialName << std::endl;
        return false;
    }
    getAGXGeometry()->setMaterial(mat);
    getAGXRigidBody()->updateMassProperties(agx::MassProperties::AUTO_GENERATE_ALL);
    return true;
}

void AGXLink::setAGXMaterialFromLinkInfo()
{
    agxSDK::Simulation* sim = getAGXBody()->getAGXScene()->getSimulation();
    std::stringstream ss;
    ss << "AGXMaterial" << agx::UuidGenerator().generate().str() << std::endl;
    agx::Material* dmat = sim->getMaterial(getOrgLink()->materialName());
    agx::MaterialRef mat = new agx::Material(ss.str(), dmat);
    Mapping* mapping = getOrgLink()->info();
    double density;
    if(mapping->read("density", density))
        mat->getBulkMaterial()->setDensity(density);
    double youngsModulus;
    if(mapping->read("youngsModulus", youngsModulus))
        mat->getBulkMaterial()->setYoungsModulus(youngsModulus);
    double poissonRatio;
    if(mapping->read("poissonRatio", poissonRatio))
        mat->getBulkMaterial()->setPoissonsRatio(poissonRatio);
    double viscosity;
    if(mapping->read("viscosity", viscosity))
        mat->getBulkMaterial()->setViscosity(viscosity);
    double roughness;
    if(mapping->read("roughness", roughness))
        mat->getSurfaceMaterial()->setRoughness(roughness);
    double surfaceViscosity;
    if(mapping->read("surfaceViscosity", surfaceViscosity))
        mat->getSurfaceMaterial()->setViscosity(surfaceViscosity);
    double adhesionForce;
    if(mapping->read("adhesionForce", adhesionForce)){
        double adhesivOverlap = mat->getSurfaceMaterial()->getAdhesiveOverlap();
        mapping->read("adhesivOverlap", adhesivOverlap);
        mat->getSurfaceMaterial()->setAdhesion(adhesionForce, adhesivOverlap);
    }
    sim->getMaterialManager()->add(mat);
    getAGXGeometry()->setMaterial(mat);
    getAGXRigidBody()->updateMassProperties(agx::MassProperties::AUTO_GENERATE_ALL);
}

bool AGXLink::setCenterOfMassFromLinkInfo()
{
    const Vector3& c(getOrgLink()->c());
    const agx::Vec3 ca(c(0), c(1), c(2));
    getAGXRigidBody()->setCmLocalTranslate(ca);
    return true;
}

bool AGXLink::setMassFromLinkInfo()
{
    double m = getOrgLink()->m();
    if(m <= 0.0) m = 1.0;
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

void AGXLink::addForceTorqueToAGX()
{
    getAGXRigidBody()->addForceAtPosition(agxConvert::toAGX(getOrgLink()->f_ext()), agx::Vec3());
    getAGXRigidBody()->addTorque(agxConvert::toAGX(getOrgLink()->tau_ext()));
}

void AGXLink::setLinkStateToAGX()
{
    agx::RigidBodyRef const agxRigidBody = getAGXRigidBody();
    if(!agxRigidBody) return;
    setLinkPositionToAGX();
    LinkPtr const orgLink = getOrgLink();
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

    // angular acceleration
    const agx::Vec3& dw = agxRigidBody->getAngularAcceleration();
    orgLink->dw() = Vector3(dw.x(), dw.y(), dw.z());

    // angular velocity
    const agx::Vec3& w = agxRigidBody->getAngularVelocity();
    orgLink->w() = Vector3(w.x(), w.y(), w.z());

    // center of mass
    const Vector3 c = orgLink->R() * orgLink->c();

    // acceleration
    const agx::Vec3& dv = agxRigidBody->getAcceleration();
    const Vector3 dv0(dv.x(), dv.y(), dv.z());
    orgLink->dv() = dv0 - orgLink->dw().cross(c);

    // velocity
    const agx::Vec3& v = agxRigidBody->getVelocity();
    const Vector3 v0(v.x(), v.y(), v.z());
    orgLink->v() = v0 - orgLink->w().cross(c);
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

agx::Name AGXLink::getCollisionGroupName() const
{
    return _collisionGroupName;
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

    if(orgLink->actuationMode() == Link::LINK_POSITION){
        desc.control = agx::RigidBody::MotionControl::KINEMATICS;
    }

    desc.enableAutoSleep = orgLink->info("autoSleep", desc.enableAutoSleep);

    return AGXObjectFactory::createRigidBody(desc);
}

agxCollide::GeometryRef AGXLink::createAGXGeometry()
{
    LinkPtr const orgLink = getOrgLink();
    AGXGeometryDesc gdesc;
    gdesc.selfCollisionGroupName = getAGXBody()->getCollisionGroupName();
    if(orgLink->actuationMode() == Link::JOINT_SURFACE_VELOCITY){
        gdesc.isPseudoContinuousTrack = true;
        const Vector3& a = orgLink->a();
        gdesc.axis = agx::Vec3(a(0), a(1), a(2));
    }
    return AGXObjectFactory::createGeometry(gdesc);
}

void AGXLink::createAGXShape()
{
    LinkPtr const orgLink = getOrgLink();
    if(!orgLink->collisionShape()) return;
    MeshExtractor* extractor = new MeshExtractor;
    AGXTrimeshDesc td;
    if(extractor->extract(orgLink->collisionShape(), [&](){ detectPrimitiveShape(extractor, td); })){
        if(td.vertices.empty()) goto EXIT;
        // if vertices have values, it will be trimesh
        size_t numConvex = 0;
        auto bConvexDecomposition = getOrgLink()->info()->find("convexDecomposition");
        if(bConvexDecomposition->isValid()){
            AGXConvexDecompositionPtr conDec = new AGXConvexDecomposition();
            numConvex = conDec->getConvexBuilder()->build(td.vertices, td.indices, td.triangles);
            if(numConvex > 0){
                LOGGER_INFO() << orgLink->name() << " convex decomposition succeed." << LOGGER_ENDL();
                LOGGER_INFO() << "Divided to " << numConvex << std::endl;
                for(auto shape : conDec->getConvexBuilder()->getConvexShapes()){
                    getAGXGeometry()->add(shape, agx::AffineMatrix4x4());
                }
            }else{
                LOGGER_WARNING() << orgLink->name() << " convex decomposition failed." << LOGGER_ENDL();
            }
        }

        if(numConvex == 0){
            agxCollide::ShapeRef trimesh = AGXObjectFactory::createShape(td);
            getAGXGeometry()->add(trimesh, agx::AffineMatrix4x4());
        }
    }
EXIT:
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
        stdx::optional<Vector3> translation;
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

            agxCollide::ShapeRef shape = nullptr;
            switch(mesh->primitiveType()){
                case SgMesh::BOX : {
                    const Vector3 s = mesh->primitive<SgMesh::Box>().size / 2.0;
                    AGXBoxDesc bd;
                    bd.halfExtents = agx::Vec3( s.x()*scale.x(), s.y()*scale.y(), s.z()*scale.z());
                    shape = AGXObjectFactory::createShape(bd);
                    break;
                }
                case SgMesh::SPHERE : {
                    const auto& sphere = mesh->primitive<SgMesh::Sphere>();
                    AGXSphereDesc sd;
                    sd.radius = sphere.radius * scale.x();
                    shape = AGXObjectFactory::createShape(sd);
                    break;
                }
                case SgMesh::CAPSULE : {
                    const auto& capsule = mesh->primitive<SgMesh::Capsule>();
                    AGXCapsuleDesc cd;
                    cd.radius = capsule.radius * scale.x();
                    cd .height =  capsule.height * scale.y();
                    shape = AGXObjectFactory::createShape(cd);
                    break;
                }
                case SgMesh::CYLINDER : {
                    const auto& cylinder = mesh->primitive<SgMesh::Cylinder>();
                    AGXCylinderDesc cd;
                    cd.radius = cylinder.radius * scale.x();
                    cd.height =  cylinder.height * scale.y();
                    shape = AGXObjectFactory::createShape(cd);
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
        for(unsigned int i=0; i < vertices_.size(); ++i){
            const Vector3 v = T * vertices_[i].cast<Position::Scalar>();
            td.vertices.push_back(agx::Vec3(v.x(), v.y(), v.z()));
        }

        const unsigned int numTriangles = (unsigned int)mesh->numTriangles();
        td.triangles = numTriangles;
        for(unsigned int i=0; i < numTriangles; ++i){
            SgMesh::TriangleRef src = mesh->triangle(i);
            td.indices.push_back((agx::UInt32)vertexIndexTop + src[0]);
            td.indices.push_back((agx::UInt32)vertexIndexTop + src[1]);
            td.indices.push_back((agx::UInt32)vertexIndexTop + src[2]);
        }
    }
}

agx::ConstraintRef AGXLink::createAGXConstraint()
{
    AGXLink* const agxParentLink = getAGXParentLink();
    if(!agxParentLink) return nullptr;
    Link* const orgLink = getOrgLink();
    Mapping* map = orgLink->info();

    AGXElementaryConstraint base, motor, range, lock;
    map->read("jointCompliance", base.compliance);
    map->read("jointSpookDamping", base.spookDamping);
    map->read("jointMotor", motor.enable);
    map->read("jointMotorCompliance", motor.compliance);
    map->read("jointMotorSpookDamping", motor.spookDamping);
    map->read("jointRangeCompliance", range.compliance);
    map->read("jointRangeSpookDamping", range.spookDamping);
    map->read("jointLock", lock.enable);
    map->read("jointLockCompliance",  lock.compliance);
    map->read("jointLockSpookDamping",  lock.spookDamping);

    Vector2 baseForceRange, motorForceRange, rangeForceRange, lockForceRange;
    if(agxConvert::setVector(map->find("jointForceRange"), baseForceRange)){
        base.forceRange = agx::RangeReal(baseForceRange(0), baseForceRange(1));
    }
    if(agxConvert::setVector(map->find("jointMotorForceRange"), motorForceRange)){
        motor.forceRange = agx::RangeReal(motorForceRange(0), motorForceRange(1));
    }
    if(agxConvert::setVector(map->find("jointRangeForceRange"), rangeForceRange)){
        range.forceRange = agx::RangeReal(rangeForceRange(0), rangeForceRange(1));
    }
    if(agxConvert::setVector(map->find("jointLockForceRange"), lockForceRange)){
        lock.forceRange = agx::RangeReal(lockForceRange(0),lockForceRange(1));
    }

    // rotor inertia
    double rotorInertia = orgLink->Jm2();
    if(!rotorInertia)
        rotorInertia = orgLink->info("rotorInertia", 0.0);

    agx::ConstraintRef constraint = nullptr;
    switch(orgLink->jointType()){
        case Link::REVOLUTE_JOINT :{
            AGXHingeDesc desc;
            const Vector3& a = orgLink->a();
            const Vector3& p = getOrigin();
            desc.set(base);
            desc.frameAxis.set(a(0),a(1),a(2));
            desc.frameCenter.set(p(0),p(1),p(2));
            desc.rigidBodyA = getAGXRigidBody();
            desc.rigidBodyB = agxParentLink->getAGXRigidBody();

            // motor
            desc.motor.set(motor);
            // lock
            desc.lock.set(lock);
            // range
            desc.range.set(range);
            desc.range.enable = true;  // range.enable;
            desc.range.range = agx::RangeReal(orgLink->q_lower(), orgLink->q_upper());
            constraint = AGXObjectFactory::createConstraint(desc);

            // Set from Link::ActuationMode
            // motor
            if(orgLink->actuationMode() != Link::ActuationMode::NO_ACTUATION){
                desc.motor.enable = true;
            }
            // lock
            if(orgLink->actuationMode() == Link::ActuationMode::JOINT_ANGLE){
                desc.motor.enable = false;
                desc.lock.enable = true;
            }
            constraint = AGXObjectFactory::createConstraint(desc);
            // add rotor inertia
            getAGXBody()->getAGXScene()->getSimulation()->add(
                AGXObjectFactory::createVirtualConstraintInertia(constraint,
                0.0, rotorInertia,
                0.0, rotorInertia)
            );
            break;
        }
        case Link::PRISMATIC_JOINT :{
            AGXPrismaticDesc desc;
            const Vector3& a = orgLink->a();
            const Vector3& p = getOrigin();
            desc.set(base);
            desc.frameAxis.set(a(0),a(1),a(2));
            desc.framePoint.set(p(0),p(1),p(2));
            desc.rigidBodyA = getAGXRigidBody();
            desc.rigidBodyB = agxParentLink->getAGXRigidBody();

            // motor
            desc.motor.set(motor);

            // lock
            desc.lock.set(lock);
            // range
            desc.range.set(range);
            desc.range.enable = true;  // range.enable;
            desc.range.range = agx::RangeReal(orgLink->q_lower(), orgLink->q_upper());

            // Set from Link::ActuationMode
            // motor
            if(orgLink->actuationMode() != Link::ActuationMode::NO_ACTUATION){
                desc.motor.enable = true;
            }
            // lock
            if(orgLink->actuationMode() == Link::ActuationMode::JOINT_ANGLE){
                desc.motor.enable = false;
                desc.lock.enable = true;
            }

            constraint = AGXObjectFactory::createConstraint(desc);
            // add rotor inertia
            getAGXBody()->getAGXScene()->getSimulation()->add(
                AGXObjectFactory::createVirtualConstraintInertia(constraint,
                rotorInertia, 0.0,
                rotorInertia, 0.0)
            );
            break;
        }
        case Link::FIXED_JOINT :
        case Link::PSEUDO_CONTINUOUS_TRACK :    // deprecated
        {
            AGXLockJointDesc desc;
            desc.set(base);
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
            double dq_l = std::max(-1.0E12, orgLink->dq_lower());
            double dq_u = std::min(1.0E12, orgLink->dq_upper());
            joint1DOF->getMotor1D()->setSpeed( orgLink->u() < 0 ? dq_l : dq_u);
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
            joint1DOF->getMotor1D()->setSpeed(orgLink->dq_target());
            return;
            break;
        }
        default :
            break;
    }

    if(orgLink->actuationMode() == Link::JOINT_SURFACE_VELOCITY){
        // Set speed(scalar) to x value. Direction is automatically calculated at AGXPseudoContinuousTrackGeometry::calculateSurfaceVelocity
        agx::Vec3f vel((float)orgLink->dq_target(), 0.0, 0.0);
        getAGXGeometry()->setSurfaceVelocity(vel);
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
            joint1DOF->getLock1D()->setPosition(orgLink->q_target());
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
    const Matrix3& R = orgLink->R();
    agx::Vec3 translation(p(0), p(1), p(2));
    agx::OrthoMatrix3x3 rotation(R(0,0), R(1,0), R(2,0),
    R(0,1), R(1,1), R(2,1),
    R(0,2), R(1,2), R(2,2));
    getAGXRigidBody()->setTransform( agx::AffineMatrix4x4( rotation, translation) );
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
AGXBody::AGXBody(Body* body) : SimulationBody(body) {}

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
    ss << agx::UuidGenerator().generate().str() << body->name() << std::flush;
    _bodyCollisionGroupName = ss.str();
    return;
}

void AGXBody::createBody(AGXScene* agxScene)
{
    initialize();
    _agxScene = agxScene;
    // Create AGXLink following child link.
    bool makeStatic = true;
    if(body()->rootLink()->jointType() != Link::FIXED_JOINT){
        makeStatic = false;
    }
    std::set<Link*> forceSensorLinks;
    for(auto& sensor : body()->devices<ForceSensor>()){
        forceSensorLinks.insert(sensor->link());
    }
    new AGXLink(body()->rootLink(), nullptr, Vector3::Zero(), this, forceSensorLinks, makeStatic);
    setLinkStateToAGX();
    createExtraJoint();
    callExtensionFuncs();
    setCollision();
}

void AGXBody::setCollision()
{
    AGXSceneRef agxScene = getAGXScene();
    // Set self collision
    agxScene->setCollision(getCollisionGroupName(), bodyItem()->isSelfCollisionDetectionEnabled());
    // Set external collision
    enableExternalCollision(bodyItem()->isCollisionDetectionEnabled());
    // set collision from body file
    setCollisionExclude();
    // Disable collision
    for(const auto& name : getCollisionGroupNamesToDisableCollision()){
        agxScene->setCollision(name, false);
    }
}

void AGXBody::setCollisionExclude(){
    const Mapping& cdMapping = *body()->info()->findMapping("collisionDetection");
    if(!cdMapping.isValid()) return;
    setCollisionExcludeLinks(cdMapping);
    setCollisionExcludeLinksDynamic(cdMapping);
    setCollisionExcludeTreeDepth(cdMapping);
    setCollisionExcludeLinkGroups(cdMapping);
    setCollisionExcludeSelfCollisionLinks(cdMapping);
    setCollisionExcludeLinksWireCollision(cdMapping);
}

void AGXBody::setCollisionExcludeLinks(const Mapping& cdMapping){
    const Listing& excludeLinks = *cdMapping.findListing("excludeLinks");
    for(auto linkName : excludeLinks){
        getAGXLink(linkName->toString())->enableExternalCollision(false);
    }
}

void AGXBody::setCollisionExcludeLinksDynamic(const Mapping& cdMapping){
    const Listing& excludeLinksDynamic = *cdMapping.findListing("excludeLinksDynamic");
    for(auto linkName : excludeLinksDynamic){
        if(AGXLink* agxLink = getAGXLink(linkName->toString())){
            agxLink->getAGXGeometry()->removeGroup(AGXGeometryDesc::globalCollisionGroupName);
            getAGXScene()->setCollisionPair(agxLink->getCollisionGroupName(), AGXGeometryDesc::globalCollisionGroupName, false);
        }
    }
}

void AGXBody::setCollisionExcludeTreeDepth(const Mapping& cdMapping){
    const ValueNodePtr& excludeTreeDepthNode = cdMapping.find("excludeTreeDepth");
    if(!excludeTreeDepthNode->isValid()) return;
    if(!excludeTreeDepthNode->isScalar()) return;
    const int& excludeTreeDepth = excludeTreeDepthNode->toInt();
    for(int i = 0; i < numAGXLinks(); ++i){
        AGXLink* agxLink1 = getAGXLink(i);
        for(int j = i+1; j < numAGXLinks(); ++j){
            AGXLink* agxLink2 = getAGXLink(j);
            AGXLink* parent1 = agxLink1;
            AGXLink* parent2 = agxLink2;
            for(int k = 0; k < excludeTreeDepth; ++k){
                stringstream ss;
                ss << "AGXExcludeTreeDepth_" << agx::UuidGenerator().generate().str() << std::endl;
                addCollisionGroupNameToDisableCollision(ss.str());
                if(parent1){
                    parent1 = parent1->getAGXParentLink();
                }
                if(parent2){
                    parent2 = parent2->getAGXParentLink();
                }
                if(!parent1 && !parent2){
                    break;
                }
                if(parent1 == agxLink2 || parent2 == agxLink1){
                    agxLink1->getAGXGeometry()->addGroup(ss.str());
                    agxLink2->getAGXGeometry()->addGroup(ss.str());
                }
            }
        }
    }
}

void AGXBody::setCollisionExcludeLinkGroups(const Mapping& cdMapping){
    const ValueNodePtr& excludeLinkGroupsNode = cdMapping.find("excludeLinkGroups");
    if(!excludeLinkGroupsNode->isValid())   return;
    if(!excludeLinkGroupsNode->isListing()) return;
    const Listing& groupList = *excludeLinkGroupsNode->toListing();
    for(auto group : groupList){
        if(!group->isMapping()) continue;
        const Mapping&  groupInfo = *group->toMapping();
        // get group name and add name to agx to disable collision
        stringstream ss;
        auto nameNode = groupInfo.find("name");
        if(nameNode->isValid()){
            ss << "AGXExcludeLinkGroups_" << nameNode->toString() << agx::UuidGenerator().generate().str() << std::endl;
        }else{
            ss << "AGXExcludeLinkGroups_" << agx::UuidGenerator().generate().str() << std::endl;
        }
        addCollisionGroupNameToDisableCollision(ss.str());

        // get link name and set group name to agx geometry
        const ValueNodePtr& linkNode = groupInfo.find("links");
        if(!linkNode->isValid())   continue;
        if(!linkNode->isListing()) continue;
        vector<string> excludeLinkNames;
        if(!agxConvert::setVector(linkNode->toListing(), excludeLinkNames)) continue;
        for(auto linkName : excludeLinkNames){
            if(AGXLink*agxLink = getAGXLink(linkName)){
                if(agxCollide::Geometry*geometry = agxLink->getAGXGeometry())
                    geometry->addGroup(ss.str());
            }
        }
    }
}

void AGXBody::setCollisionExcludeSelfCollisionLinks(const Mapping& cdMapping)
{
    const ValueNodePtr& excludeSCLinksNode = cdMapping.find("excludeSelfCollisionLinks");
    if(!excludeSCLinksNode->isValid())   return;
    if(!excludeSCLinksNode->isListing()) return;
    stringstream ss;
    ss << "AGXExcludeSelfCollisionLinks_" << agx::UuidGenerator().generate().str() << std::endl;
    addCollisionGroupNameToDisableCollision(ss.str());
    getAGXScene()->setCollisionPair(ss.str(), getCollisionGroupName(), false);

    vector<string> linkNames;
    if(!agxConvert::setVector(excludeSCLinksNode->toListing(), linkNames)) return;
    for(auto linkName : linkNames){
        if(AGXLink* agxLink = getAGXLink(linkName)){
            if(agxCollide::Geometry* geometry = agxLink->getAGXGeometry())
                geometry->addGroup(ss.str());
        }
    }
}

void AGXBody::setCollisionExcludeLinksWireCollision(const Mapping& cdMapping)
{
    const ValueNodePtr& excludeLinksWireNode = cdMapping.find("excludeLinksWireCollision");
    if(!excludeLinksWireNode->isValid())   return;
    if(!excludeLinksWireNode->isListing()) return;

    vector<string> linkNames;
    if(!agxConvert::setVector(excludeLinksWireNode->toListing(), linkNames)) return;
    for(auto linkName : linkNames){
        if(AGXLink* agxLink = getAGXLink(linkName)){
            getAGXScene()->setCollisionPair(agxLink->getCollisionGroupName(), AGXWireDesc::globalCollisionGroupName, false);
        }
    }
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

void AGXBody::enableAGXWireContact(const bool& bOn)
{
    //getAGXScene()->setCollisionPair(getCollisionGroupName(), AGXWireDesc::, bOn);
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

void AGXBody::addForceTorqueToAGX()
{
    for(const auto& link : getAGXLinks()){
        link->addForceTorqueToAGX();
    }
    body()->clearExternalForces();
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
    return !sensorHelper.forceSensors().empty();
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
    return (int)_agxLinks.size();
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

bool AGXBody::getAGXLinksFromInfo(const std::string& key, const bool& defaultValue, AGXLinkPtrs& agxLinks) const
{
    agxLinks.clear();
    for(const auto& agxLink : getAGXLinks()){
        if(agxLink->getOrgLink()->info(key, defaultValue)) agxLinks.push_back(agxLink);
    }
    if(agxLinks.empty()) return false;
    return true;
}

int AGXBody::numControllableLinks() const
{
    return (int)_controllableLinks.size();
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
    if(AGXLink* agxLink = getAGXLink(index)) return agxLink->getAGXRigidBody();
    return nullptr;
}

agx::RigidBody* AGXBody::getAGXRigidBody(const std::string& linkName) const
{
    if(AGXLink* agxLink = getAGXLink(linkName)) return agxLink->getAGXRigidBody();
    return nullptr;
}

agx::ConstraintRef AGXBody::getAGXConstraint(const int& index) const
{
    if(AGXLink* agxLink = getAGXLink(index)) return agxLink->getAGXConstraint();
    return nullptr;
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
    //agxBodyExtensionFuncs["test"] = [](AGXBody* agxBody){ std::cout << "test" << std::endl; return false;};
    agxBodyExtensionFuncs["ContinuousTrack"] = [&](AGXBody* agxBody){ return createContinuousTrack(agxBody); };
    agxBodyExtensionFuncs["AGXVehicleContinousTrack"] = [&](AGXBody* agxBody){ return createAGXVehicleContinousTrack(this); };

    // call
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

void AGXBody::createExtraJoint()
{
    if(this->body()->numExtraJoints() > 0) 
        addAGXBodyExtension(new AGXExtraJoint(this));
}


}

