/*!
  @file
  @author Shizuko Hattori
*/

#include "PhysXSimulatorItem.h"
#include <cnoid/MessageView>
#include <cnoid/ItemManager>
#include <cnoid/Archive>
#include <cnoid/EigenUtil>
#include <cnoid/MeshExtractor>
#include <cnoid/SceneDrawables>
#include <cnoid/Body>
#include <cnoid/Link>
#include <cnoid/BasicSensorSimulationHelper>
#include <cnoid/BodyItem>
#include <cnoid/stdx/optional>
#include "gettext.h"
#include <iostream>

#ifndef _WIN32
using std::isfinite;
#endif
#ifndef NDEBUG
#define _DEBUG 1
#endif
#include <PxPhysicsAPI.h>

#if PX_PHYSICS_VERSION_MAJOR >=3 && PX_PHYSICS_VERSION_MINOR >=3
#define VERSION_3_3_LATER
#if PX_PHYSICS_VERSION_MAJOR ==3 && PX_PHYSICS_VERSION_MINOR ==3
#define PX_FOUNDATION_VERSION PX_PHYSICS_VERSION
#endif
#endif

using namespace std;
using namespace cnoid;
using namespace physx;

namespace {

const bool DEBUG_COLLISION = false;
const double DEFAULT_GRAVITY_ACCELERATION = 9.80665;
const bool meshOnly = false;     

class PhysXBody;

class PhysXLink : public Referenced
{
public:
    PhysXSimulatorItemImpl* simImpl;
    Link* link;
    PhysXLink* parent;
    PxRigidActor* pxRigidActor;
    PxJoint* pxJoint;
    vector<PxVec3> vertices;
    vector<int> triangles;
    double q_offset;

    PhysXLink(PhysXSimulatorItemImpl* simImpl, PhysXBody* physXBody, PhysXLink* parent,
              const Vector3& parentOrigin, Link* link);
    ~PhysXLink();
    void createLinkBody(bool isStatic, PhysXLink* parent, const Vector3& origin);
    void createGeometry(PhysXBody* physXBody);
    PxTriangleMesh* createTriangleMesh();
    PxConvexMesh* createConvexMeshSafe();
    void addMesh(MeshExtractor* extractor, PhysXBody* physXBody);
    void setKinematicStateToPhysX();
    void getKinematicStateFromPhysX();
    void setTorqueToPhysX();
    void setVelocityToPhysX();
};
typedef ref_ptr<PhysXLink> PhysXLinkPtr;

class PhysXBody : public SimulationBody
{
public:
    PhysXSimulatorItemImpl* simImpl;
    bool isStatic;
    PxAggregate* pxAggregate;
    vector<PhysXLinkPtr> physXLinks;
    BasicSensorSimulationHelper sensorHelper;
    vector<PxJoint*> extraJoints;

    PhysXBody(const Body& orgBody);
    ~PhysXBody();
    void createBody(PhysXSimulatorItemImpl* simImpl);
    void setKinematicStateToPhysX();
    void getKinematicStateFromPhysX();
    void updateForceSensors();
    void setExtraJoints();
    void setControlValToPhysX();
};
    
    PxDefaultErrorCallback pxDefaultErrorCallback;
    PxDefaultAllocator pxDefaultAllocatorCallback;
    int numOfinstance = 0;
    PxFoundation* pxFoundation = 0;
#ifndef VERSION_3_3_LATER
    PxProfileZoneManager* pxProfileZoneManager = 0;
#endif
    PxCooking* pxCooking = 0;
}


namespace cnoid {
  
class PhysXSimulatorItemImpl : public PxSimulationEventCallback, PxContactModifyCallback
{
public:
    PhysXSimulatorItem* self;
    MessageView* mv;

    PxPhysics* pxPhysics;
    PxDefaultCpuDispatcher* pxDispatcher;
    PxScene* pxScene;
    PxMaterial* pxMaterial;

    Vector3 gravity;
    double timeStep;

    double staticFriction;
    double dynamicFriction;
    double restitution;
    bool isJointLimitMode;

    PhysXSimulatorItemImpl(PhysXSimulatorItem* self);
    PhysXSimulatorItemImpl(PhysXSimulatorItem* self, const PhysXSimulatorItemImpl& org);
    void initialize();
    void finalize();
    ~PhysXSimulatorItemImpl();
    void clear();
    bool initializeSimulation(const std::vector<SimulationBody*>& simBodies);
    bool stepSimulation(const std::vector<SimulationBody*>& activeSimBodies);
    void addBody(PhysXBody* physXBody);
    void doPutProperties(PutPropertyFunction& putProperty);
    void store(Archive& archive);
    void restore(const Archive& archive);
    virtual void onContact(const PxContactPairHeader& pairHeader, const PxContactPair* pairs, PxU32 nbPairs);
    virtual void onTrigger(PxTriggerPair* pairs, PxU32 count){}
    virtual void onConstraintBreak(PxConstraintInfo*, PxU32) {}
    virtual void onWake(PxActor** , PxU32 ) {}
    virtual void onSleep(PxActor** , PxU32 ){}
    virtual void onContactModify(PxContactModifyPair* const pairs, PxU32 count);
#ifdef VERSION_3_3_LATER
    virtual void onAdvance(const PxRigidBody* const *  bodyBuffer, const PxTransform* poseBuffer, const PxU32 count ) {}
#endif
};

PxFilterFlags customFilterShader(
    PxFilterObjectAttributes attributes0, PxFilterData filterData0,
    PxFilterObjectAttributes attributes1, PxFilterData filterData1,
    PxPairFlags& pairFlags, const void* constantBlock, PxU32 constantBlockSize)
{
    pairFlags = PxPairFlag::eCONTACT_DEFAULT;

    if(filterData0.word0 || filterData1.word0)
        pairFlags |= PxPairFlag::eMODIFY_CONTACTS;

    if(DEBUG_COLLISION)
        pairFlags |= PxPairFlag::eNOTIFY_TOUCH_PERSISTS
            | PxPairFlag::eNOTIFY_TOUCH_FOUND
            | PxPairFlag::eNOTIFY_CONTACT_POINTS;

    return PxFilterFlag::eDEFAULT;
}
}


PhysXLink::PhysXLink
(PhysXSimulatorItemImpl* simImpl, PhysXBody* physXBody, PhysXLink* parent, const Vector3& parentOrigin, Link* link) :
    simImpl(simImpl),
    link(link),
    parent(parent)
{
    physXBody->physXLinks.push_back(this);

    pxJoint = 0;
    q_offset = 0;

    Vector3 o = parentOrigin + link->b();
    
    createLinkBody(physXBody->isStatic, parent, o);
    if(physXBody->pxAggregate)
        physXBody->pxAggregate->addActor(*pxRigidActor);

    for(Link* child = link->child(); child; child = child->sibling()){
        new PhysXLink(simImpl, physXBody, this, o, child);
    }

    createGeometry(physXBody);

    if(link->actuationMode() == Link::JOINT_SURFACE_VELOCITY){
        PxFilterData simFilterData;
        simFilterData.word0 = 1;
        int nbShapes = pxRigidActor->getNbShapes();
        PxShape** shapes = (PxShape**) ::alloca(nbShapes * sizeof(PxShape*));
        pxRigidActor->getShapes(shapes, nbShapes);
        for(int i = 0; i < nbShapes; i++)
            shapes[i]->setSimulationFilterData(simFilterData);
    }
}


void PhysXLink::createLinkBody(bool isStatic, PhysXLink* parent, const Vector3& origin)
{
    if(isStatic){
        pxRigidActor = simImpl->pxPhysics->createRigidStatic(PxTransform(PxVec3(origin.x(), origin.y(), origin.z())));
    }else{
        PxRigidDynamic* pxRigidDynamic = simImpl->pxPhysics->createRigidDynamic(PxTransform(PxVec3(origin.x(), origin.y(), origin.z())));
        const Matrix3& I = link->I();
        const Vector3& c = link->c();
        PxMat33 inertia(PxVec3(I(0,0), I(1,0), I(2,0)),
                        PxVec3(I(0,1), I(1,1), I(2,1)),
                        PxVec3(I(0,2), I(1,2), I(2,2)));
        PxQuat orient;
        PxVec3 diagTensor = PxDiagonalize(inertia, orient);
        pxRigidDynamic->setCMassLocalPose(PxTransform(PxVec3(c.x(), c.y(), c.z()), orient));
        pxRigidDynamic->setMass(link->m());
        pxRigidDynamic->setMassSpaceInertiaTensor(diagTensor);
        pxRigidDynamic->setSleepThreshold(0.0);      // don't sleep
        pxRigidActor = pxRigidDynamic;
    }
    pxRigidActor->userData = link;

    const Vector3& a = link->a();
    const Vector3& b = link->b();
    const Vector3& d = link->d();

    switch(link->jointType()){
        
    case Link::ROTATIONAL_JOINT:{
        Vector3 u(1,0,0);
        Vector3 ty = a.cross(u);
        PxMat33 R0;
        if(ty.norm() < 1.0e-8){
           	if(a.x() == -1.0){
           		R0.column0 = PxVec3(-1.0, 0.0, 0.0);
                R0.column1 = PxVec3(0.0, -1.0, 0.0);
                R0.column2 = PxVec3(0.0, 0.0, 1.0);
            }else
#ifdef VERSION_3_3_LATER
                R0 = PxMat33(PxIdentity);
#else
            	R0 = PxMat33::createIdentity();
#endif
        } else {
            ty.normalized();
            Vector3 tz = a.cross(ty).normalized();
            R0.column0 = PxVec3(a.x(), a.y(), a.z());
            R0.column1 = PxVec3(ty.x(), ty.y(), ty.z());
            R0.column2 = PxVec3(tz.x(), tz.y(), tz.z());
        }
        PxTransform Tp(PxVec3(b(0), b(1), b(2)), PxQuat(R0));
        PxTransform Tc(PxVec3(0,0,0), PxQuat(R0));

        //  The axis along which the two bodies may rotate is specified by the common origin of the joint frames and their common x-axis
        PxRevoluteJoint* joint = PxRevoluteJointCreate(*simImpl->pxPhysics, 
                                                       parent->pxRigidActor, Tp, pxRigidActor, Tc);

        if(simImpl->isJointLimitMode){
            if(link->q_upper() < numeric_limits<double>::max() && link->q_lower() > -numeric_limits<double>::max()){
                joint->setLimit(PxJointAngularLimitPair(link->q_lower(), link->q_upper(), 0.01f));
                joint->setRevoluteJointFlag(PxRevoluteJointFlag::eLIMIT_ENABLED, true);
            }
        }
        if(link->actuationMode() == Link::JOINT_VELOCITY){
        	joint->setDriveForceLimit(numeric_limits<double>::max());
        	joint->setRevoluteJointFlag(PxRevoluteJointFlag::eDRIVE_ENABLED, true);
        }
        q_offset = joint->getAngle();
        pxJoint = joint;
    }
        break;
       
    case Link::SLIDE_JOINT:{
        Vector3 u(1,0,0);
        Vector3 ty = d.cross(u);
        PxMat33 R0;
        if(ty.norm() < 1.0e-8){
        	if(a.x() == -1.0){
        		R0.column0 = PxVec3(-1.0, 0.0, 0.0);
        		R0.column1 = PxVec3(0.0, -1.0, 0.0);
        		R0.column2 = PxVec3(0.0, 0.0, 1.0);
        	}else
#ifdef VERSION_3_3_LATER
                R0 = PxMat33(PxIdentity);
#else
                R0 = PxMat33::createIdentity();
#endif
        } else {
            ty.normalized();
            Vector3 tz = d.cross(ty).normalized();
            R0.column0 = PxVec3(d.x(), d.y(), d.z());
            R0.column1 = PxVec3(ty.x(), ty.y(), ty.z());
            R0.column2 = PxVec3(tz.x(), tz.y(), tz.z());
        }
        PxTransform Tp(PxVec3(b(0), b(1), b(2)), PxQuat(R0));
        PxTransform Tc(PxVec3(0,0,0), PxQuat(R0));
        PxPrismaticJoint* joint = PxPrismaticJointCreate(*simImpl->pxPhysics, 
                                                         parent->pxRigidActor, Tp, pxRigidActor, Tc);

        if(simImpl->isJointLimitMode){
            if(link->q_upper() < numeric_limits<double>::max() && link->q_lower() > -numeric_limits<double>::max()){
                PxTolerancesScale scale;
                joint->setLimit(PxJointLinearLimitPair(scale, link->q_lower(), link->q_upper(), 0.01f));
                joint->setPrismaticJointFlag(PxPrismaticJointFlag::eLIMIT_ENABLED, true);
            }
        }
        if(link->actuationMode() == Link::JOINT_VELOCITY){

        }
        q_offset = joint->getPosition();
        pxJoint = joint;
    }
        break;

    case Link::FREE_JOINT:
        break;

    case Link::FIXED_JOINT:
    default:
        if(parent){
            PxTransform Tp(PxVec3(b(0), b(1), b(2)));
            PxTransform Tc(PxVec3(0,0,0));
            PxFixedJoint* joint = PxFixedJointCreate(*simImpl->pxPhysics,
                                                     parent->pxRigidActor, Tp, pxRigidActor, Tc);
            pxJoint = joint;
        }
        break;
    }
}


void PhysXLink::createGeometry(PhysXBody* physXBody)
{
    if(link->collisionShape()){
        MeshExtractor* extractor = new MeshExtractor;
        if(extractor->extract(link->collisionShape(), std::bind(&PhysXLink::addMesh, this, extractor, physXBody))){
            if(!vertices.empty()){
#ifdef VERSION_3_3_LATER
                if(pxRigidActor->getType() == PxActorType::eRIGID_STATIC){
#else
                if(pxRigidActor->isRigidStatic()){
#endif
                    PxTriangleMesh* triangleMesh = createTriangleMesh();
                    if(triangleMesh){
                        PxShape* pxShape = pxRigidActor->createShape(PxTriangleMeshGeometry(triangleMesh), *simImpl->pxMaterial);
                        triangleMesh->release();
                    }
                }else{
                    PxConvexMesh* convexMesh = createConvexMeshSafe();
                    if(convexMesh){
                        PxShape* pxShape = pxRigidActor->createShape(PxConvexMeshGeometry(convexMesh), *simImpl->pxMaterial);
                        convexMesh->release();
                    }
                }
            }
        }
        delete extractor;
    }
}


void PhysXLink::addMesh(MeshExtractor* extractor, PhysXBody* physXBody)
{
    SgMesh* mesh = extractor->currentMesh();
    const Affine3& T = extractor->currentTransform();
    
    bool meshAdded = false;

    if(!meshOnly){
        if(mesh->primitiveType() != SgMesh::MESH){
            bool doAddPrimitive = false;
            Vector3 scale;
            stdx::optional<Vector3> translation;
            if(!extractor->isCurrentScaled()){
                scale.setOnes();
                doAddPrimitive = true;
            } else {
                Affine3 S = extractor->currentTransformWithoutScaling().inverse() *
                    extractor->currentTransform();

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
                    }
                }
            }
            if(doAddPrimitive){
                bool created = false;
                PxShape* shape;
                switch(mesh->primitiveType()){
                case SgMesh::BOX : {
                    const Vector3& s = mesh->primitive<SgMesh::Box>().size / 2.0;
                    shape = pxRigidActor->createShape(PxBoxGeometry(s.x()*scale.x(), s.y()*scale.y(), s.z()*scale.z()),
                                                      *simImpl->pxMaterial);
                    created = true;
                    break; }
                case SgMesh::SPHERE : {
                    SgMesh::Sphere sphere = mesh->primitive<SgMesh::Sphere>();
                    shape = pxRigidActor->createShape(PxSphereGeometry(sphere.radius * scale.x()),
                                                      *simImpl->pxMaterial);
                    created = true;
                    break; }
                default :
                    break;
                }
                if(created){
                    Affine3 T_ = extractor->currentTransformWithoutScaling();
                    if(translation){
                        T_ *= Translation3(*translation);
                    }
                    PxVec3 p = PxVec3(T_(0,3), T_(1,3), T_(2,3));
                    PxMat33 R(PxVec3(T_(0,0), T_(1,0), T_(2,0)),
                              PxVec3(T_(0,1), T_(1,1), T_(2,1)),
                              PxVec3(T_(0,2), T_(1,2), T_(2,2)));
                    PxTransform Tp(p, PxQuat(R));
                    shape->setLocalPose(Tp);
                    meshAdded = true;
                }
            }
        }
    }

    if(!meshAdded){
        const int vertexIndexTop = vertices.size();

        const SgVertexArray& vertices_ = *mesh->vertices();
        const int numVertices = vertices_.size();
        for(int i=0; i < numVertices; ++i){
            const Vector3 v = T * vertices_[i].cast<Position::Scalar>();
            vertices.push_back(PxVec3(v.x(), v.y(), v.z()));
        }

        const int numTriangles = mesh->numTriangles();
        for(int i=0; i < numTriangles; ++i){
            SgMesh::TriangleRef tri = mesh->triangle(i);
            triangles.push_back(vertexIndexTop + tri[0]);
            triangles.push_back(vertexIndexTop + tri[1]);
            triangles.push_back(vertexIndexTop + tri[2]);
        }
    }
}


PxTriangleMesh* PhysXLink::createTriangleMesh()
{
    PxTriangleMeshDesc meshDesc;
    meshDesc.points.count           = vertices.size();
    meshDesc.points.stride          = sizeof(PxVec3);
    meshDesc.points.data            = &vertices[0];
    meshDesc.triangles.count        = triangles.size()/3;
    meshDesc.triangles.stride       = 3*sizeof(int);
    meshDesc.triangles.data         = &triangles[0];

    PxDefaultMemoryOutputStream writeBuffer;
    bool status = pxCooking->cookTriangleMesh(meshDesc, writeBuffer);
    if(!status)
        return 0;
    PxDefaultMemoryInputData readBuffer(writeBuffer.getData(), writeBuffer.getSize());
    return simImpl->pxPhysics->createTriangleMesh(readBuffer);
}


PxConvexMesh* PhysXLink::createConvexMeshSafe()
{
    PxConvexMeshDesc convexDesc;
    convexDesc.points.count         = vertices.size();
    convexDesc.points.stride        = sizeof(PxVec3);
    convexDesc.points.data          = &vertices[0];
    convexDesc.flags                = PxConvexFlag::eCOMPUTE_CONVEX;
    convexDesc.vertexLimit          = 256;

    bool aabbCreated = false;

    PxDefaultMemoryOutputStream buf;
    bool retVal = pxCooking->cookConvexMesh(convexDesc, buf);
    if(!retVal){
        convexDesc.flags |= PxConvexFlag::eINFLATE_CONVEX;
        retVal = pxCooking->cookConvexMesh(convexDesc, buf);
    }
    if(!retVal){
        // create AABB
        PxBounds3 aabb;
        aabb.setEmpty();
        for (PxU32 i = 0; i < vertices.size(); i++)
            aabb.include(vertices[i]);

        PxVec3  aabbVerts[8];
        aabbVerts[0] = PxVec3(aabb.minimum.x,aabb.minimum.y,aabb.minimum.z);
        aabbVerts[1] = PxVec3(aabb.maximum.x,aabb.minimum.y,aabb.minimum.z);
        aabbVerts[2] = PxVec3(aabb.maximum.x,aabb.maximum.y,aabb.minimum.z);
        aabbVerts[3] = PxVec3(aabb.minimum.x,aabb.maximum.y,aabb.minimum.z);

        aabbVerts[4] = PxVec3(aabb.minimum.x,aabb.minimum.y,aabb.maximum.z);
        aabbVerts[5] = PxVec3(aabb.maximum.x,aabb.minimum.y,aabb.maximum.z);
        aabbVerts[6] = PxVec3(aabb.maximum.x,aabb.maximum.y,aabb.maximum.z);
        aabbVerts[7] = PxVec3(aabb.minimum.x,aabb.maximum.y,aabb.maximum.z);

        convexDesc.points.count  = 8;
        convexDesc.points.stride = sizeof(PxVec3);
        convexDesc.points.data   = &aabbVerts[0];
        convexDesc.flags         = PxConvexFlag::eCOMPUTE_CONVEX;

        retVal = pxCooking->cookConvexMesh(convexDesc, buf);

        aabbCreated = true;
    }

    if(!retVal)
        return 0;

    PxDefaultMemoryInputData input(buf.getData(), buf.getSize());
    PxConvexMesh* mesh = simImpl->pxPhysics->createConvexMesh(input);

    // vLimit test
    if(mesh && !aabbCreated && (convexDesc.flags & PxConvexFlag::eINFLATE_CONVEX)){
        bool iterate = true;
        PxU32 limit = 256;
        while(iterate){
            if(mesh->getNbVertices() > 256){
                if(limit - (mesh->getNbVertices() - limit) > 4){
                    convexDesc.vertexLimit = limit - (mesh->getNbVertices() - limit);
                }else{
                    convexDesc.vertexLimit = 4;
                }
                
                limit = convexDesc.vertexLimit;
                mesh->release();
                mesh = 0;

                PxDefaultMemoryOutputStream buf2;
                retVal = pxCooking->cookConvexMesh(convexDesc, buf2);
                if(retVal){
                    PxDefaultMemoryInputData input2(buf2.getData(), buf2.getSize());
                    mesh = simImpl->pxPhysics->createConvexMesh(input2);
                    if(!mesh)
                        return 0;
                }else{
                    return 0;
                }
            }else{
                return mesh;
            }
        }
    }

    return mesh;
}


PhysXLink::~PhysXLink()
{
    if(pxJoint)
        pxJoint->release();
    pxRigidActor->release();
}


void PhysXLink::setKinematicStateToPhysX()
{
    const Position& T = link->T();
    PxMat44 m(PxVec4(T(0,0), T(1,0), T(2,0), T(3,0)),
              PxVec4(T(0,1), T(1,1), T(2,1), T(3,1)),
              PxVec4(T(0,2), T(1,2), T(2,2), T(3,2)),
              PxVec4(T(0,3), T(1,3), T(2,3), T(3,3)));
    PxTransform pose(m);
    pxRigidActor->setGlobalPose(pose);
#ifdef VERSION_3_3_LATER
    if(pxRigidActor->getType()==PxActorType::eRIGID_DYNAMIC){
        PxRigidDynamic* actor = reinterpret_cast<PxRigidDynamic*>(pxRigidActor);
        const Vector3& w = link->w();
        const Vector3& v = link->v();
        actor->setLinearVelocity(PxVec3(v.x(), v.y(), v.z()));
        actor->setAngularVelocity(PxVec3(w.x(), w.y(), w.z()));
    }
#else
    if(pxRigidActor->isRigidDynamic()){
        const Vector3& w = link->w();
        const Vector3& v = link->v();
        pxRigidActor->isRigidDynamic()->setLinearVelocity(PxVec3(v.x(), v.y(), v.z()));
        pxRigidActor->isRigidDynamic()->setAngularVelocity(PxVec3(w.x(), w.y(), w.z()));
    }
#endif
}


void PhysXLink::getKinematicStateFromPhysX()
{
    if(pxJoint){
        if(link->isRotationalJoint()){
            PxRevoluteJoint* joint = pxJoint->is<PxRevoluteJoint>();
            link->q() = joint->getAngle() - q_offset;
            link->dq() = joint->getVelocity();
        } else if(link->isSlideJoint()){
            PxPrismaticJoint* joint = pxJoint->is<PxPrismaticJoint>();
            link->q() = joint->getPosition() - q_offset;
            link->dq() = joint->getVelocity();
        }
    }

    PxTransform T = pxRigidActor->getGlobalPose();
    PxVec3& p = T.p;
    link->p() = Vector3(p[0], p[1], p[2]);
    PxMat33 R(T.q);
    link->R() << R(0,0), R(0,1), R(0,2),
        R(1,0), R(1,1), R(1,2),
        R(2,0), R(2,1), R(2,2);

#ifdef VERSION_3_3_LATER
    if(pxRigidActor->getType()==PxActorType::eRIGID_DYNAMIC){
        PxRigidDynamic* actor = reinterpret_cast<PxRigidDynamic*>(pxRigidActor);
        PxVec3 v = actor->getLinearVelocity();
        PxVec3 w = actor->getAngularVelocity();
#else
    if(pxRigidActor->isRigidDynamic()){
        PxVec3 v = pxRigidActor->isRigidDynamic()->getLinearVelocity();
        PxVec3 w = pxRigidActor->isRigidDynamic()->getAngularVelocity();
#endif
        link->w() = Vector3(w[0], w[1], w[2]);
        link->v() = Vector3(v[0], v[1], v[2]);
    }

}


void PhysXLink::setTorqueToPhysX()
{
#ifdef VERSION_3_3_LATER
    PxRigidDynamic* actor = 0;
    PxRigidDynamic* pactor = 0;
    if(pxRigidActor->getType()==PxActorType::eRIGID_DYNAMIC){
        actor = reinterpret_cast<PxRigidDynamic*>(pxRigidActor);
    }
    if(parent->pxRigidActor->getType()==PxActorType::eRIGID_DYNAMIC){
        pactor = reinterpret_cast<PxRigidDynamic*>(parent->pxRigidActor);
    }
#endif

    if(link->isRotationalJoint()){
        const Vector3 u = link->u() * link->a();
        const Vector3 uu = link->R() * u;
        PxVec3 torque(uu(0), uu(1), uu(2));
#ifdef VERSION_3_3_LATER
        if(actor){
            actor->addTorque(torque);
        }
        if(pactor){
            pactor->addTorque(-torque);
        }
#else
        pxRigidActor->isRigidDynamic()->addTorque(torque);
        PxRigidDynamic* actor = parent->pxRigidActor->isRigidDynamic();
        if(actor)
            actor->addTorque(-torque);
#endif
    } else if(link->isSlideJoint()){
        const Vector3 u = link->u() * link->d();
        const Vector3 uu = link->R() * u;
        const PxVec3 force(uu(0), uu(1), uu(2));
#ifdef VERSION_3_3_LATER
        if(actor){
            PxRigidBodyExt::addForceAtLocalPos(*actor, force, PxVec3(0,0,0));
        }
        if(pactor){
            const Vector3& b = link->b();
            PxVec3 pos(b(0), b(1), b(2));
            PxRigidBodyExt::addForceAtLocalPos(*pactor, -force, pos);
        }
#else
        PxRigidBodyExt::addForceAtLocalPos(*pxRigidActor->isRigidDynamic(), force, PxVec3(0,0,0));
        PxRigidDynamic* actor = parent->pxRigidActor->isRigidDynamic();
        if(actor){
            const Vector3& b = link->b();
            PxVec3 pos(b(0), b(1), b(2));
            PxRigidBodyExt::addForceAtLocalPos(*actor, -force, pos);
        }
#endif
    }
}


void PhysXLink::setVelocityToPhysX()
{
    if(link->isRotationalJoint()){
    	PxRevoluteJoint* joint = pxJoint->is<PxRevoluteJoint>();
    	double v = link->dq_target();
    	joint->setDriveVelocity(v);
    } else if(link->isSlideJoint()){


    }
}


PhysXBody::PhysXBody(const Body& orgBody)
    : SimulationBody(new Body(orgBody))
{
    pxAggregate = 0;
    extraJoints.clear();
}


PhysXBody::~PhysXBody()
{
    if(pxAggregate)
        pxAggregate->release();
    for(vector<PxJoint*>::iterator it = extraJoints.begin(); it!=extraJoints.end(); it++)
        (*it)->release();
}


void PhysXBody::createBody(PhysXSimulatorItemImpl* _simImpl)
{
    simImpl = _simImpl;
    isStatic = body()->isStaticModel();

    int numLinks = body()->numLinks();
    if(numLinks > 1){
        bool enableSelfCollision = false;
        pxAggregate = simImpl->pxPhysics->createAggregate(numLinks, enableSelfCollision);
    }
    PhysXLink* rootLink = new PhysXLink(simImpl, this, 0, Vector3::Zero(), body()->rootLink());

    setKinematicStateToPhysX();

    if(numLinks > 1)
        simImpl->pxScene->addAggregate(*pxAggregate);
    else
        simImpl->pxScene->addActor(*physXLinks[0]->pxRigidActor);

#ifdef VERSION_3_3_LATER
    if(rootLink->link->jointType() == Link::FIXED_JOINT && rootLink->pxRigidActor->getType()==PxActorType::eRIGID_DYNAMIC){
#else
    if(rootLink->link->jointType() == Link::FIXED_JOINT && rootLink->pxRigidActor->isRigidDynamic()){
#endif
        PxFixedJoint* joint = PxFixedJointCreate(*simImpl->pxPhysics,
                                                 0, PxTransform(PxIdentity), rootLink->pxRigidActor, rootLink->pxRigidActor->getGlobalPose().getInverse());
        rootLink->pxJoint = joint;
    }

    setExtraJoints();

    setControlValToPhysX();

    sensorHelper.initialize(body(), simImpl->timeStep, simImpl->gravity);
    const DeviceList<ForceSensor>& forceSensors = sensorHelper.forceSensors();
#ifndef VERSION_3_3_LATER
    for(size_t i=0; i < forceSensors.size(); ++i)
        physXLinks[forceSensors[i]->link()->index()]->pxJoint->setConstraintFlag(PxConstraintFlag::eREPORTING, true);
#endif

}


void PhysXBody::setExtraJoints()
{
    const int n = body()->numExtraJoints();

    for(int j=0; j < n; ++j){
        ExtraJoint& extraJoint = body()->extraJoint(j);

        PhysXLinkPtr physXLinkPair[2];
        for(int i=0; i < 2; ++i){
            PhysXLinkPtr physXLink;
            Link* link = extraJoint.link[i];
            if(link->index() < (int)physXLinks.size()){
                physXLink = physXLinks[link->index()];
                if(physXLink->link == link){
                    physXLinkPair[i] = physXLink;
                }
            }
            if(!physXLink){
                break;
            }
        }

        if(physXLinkPair[1]){
            Link* link0 = physXLinkPair[0]->link;
            Link* link1 = physXLinkPair[1]->link;
            Vector3 p0 = link0->Rs() * extraJoint.point[0];  // link0 local position
            Vector3 a = link0->Rs() * extraJoint.axis;        // link0 local axis
            Vector3 p1 = link1->Rs() * extraJoint.point[1];  // link1 local position

            if(extraJoint.type == ExtraJoint::EJ_PISTON){
                Vector3 u(1,0,0);
                Vector3 ty = a.cross(u);
                PxMat33 R0;
                if(ty.norm() == 0){
#ifdef VERSION_3_3_LATER
                    R0 = PxMat33(PxIdentity);
#else
                    R0 = PxMat33::createIdentity();
#endif
                } else {
                    ty.normalized();
                    Vector3 tz = a.cross(ty).normalized();
                    R0.column0 = PxVec3(a.x(), a.y(), a.z());
                    R0.column1 = PxVec3(ty.x(), ty.y(), ty.z());
                    R0.column2 = PxVec3(tz.x(), tz.y(), tz.z());
                }
                PxTransform T0(PxVec3(p0(0), p0(1), p0(2)), PxQuat(R0));
                PxTransform T1(PxVec3(p1(0), p1(1), p1(2)), PxQuat(R0));
                PxRevoluteJoint* joint = PxRevoluteJointCreate(*simImpl->pxPhysics, 
                                                               physXLinkPair[0]->pxRigidActor, T0, physXLinkPair[1]->pxRigidActor, T1);
                extraJoints.push_back(joint);
            } else if(extraJoint.type == ExtraJoint::EJ_BALL){
                PxTransform T0(PxVec3(p0(0), p0(1), p0(2)));
                PxTransform T1(PxVec3(p1(0), p1(1), p1(2)));
                PxSphericalJoint* joint = PxSphericalJointCreate(*simImpl->pxPhysics, 
                                                                 physXLinkPair[0]->pxRigidActor, T0, physXLinkPair[1]->pxRigidActor, T1);
                extraJoints.push_back(joint);
            }
        }
    }
}


void PhysXBody::setKinematicStateToPhysX()
{
    for(size_t i=0; i < physXLinks.size(); ++i){
        physXLinks[i]->setKinematicStateToPhysX();
    }
}


void PhysXBody::setControlValToPhysX()
{
    for(size_t i=1; i < physXLinks.size(); ++i){
        switch(physXLinks[i]->link->actuationMode()){
        case Link::NO_ACTUATION :
            break;
        case Link::JOINT_TORQUE :
            physXLinks[i]->setTorqueToPhysX();
            break;
        case Link::JOINT_VELOCITY :
            physXLinks[i]->setVelocityToPhysX();
            break;
        default :
            break;
        }
    }
}


void PhysXBody::getKinematicStateFromPhysX()
{
    for(size_t i=0; i < physXLinks.size(); ++i){
        physXLinks[i]->getKinematicStateFromPhysX();
    }
}


void PhysXBody::updateForceSensors()
{
    const DeviceList<ForceSensor>& forceSensors = sensorHelper.forceSensors();
    for(size_t i=0; i < forceSensors.size(); ++i){
        ForceSensor* sensor = forceSensors[i];
        const Link* link = sensor->link();
        const PxJoint* joint = physXLinks[link->index()]->pxJoint;
        PxVec3 force, torque;
        joint->getConstraint()->getForce(force, torque);        //The force is resolved at the origin of actor1's joint frame.
        Vector3 f(force[0], force[1], force[2]);
        Vector3 tau(torque[0], torque[1], torque[2]);
        const Matrix3 R = sensor->R_local();
        const Vector3 p = sensor->p_local();
        sensor->f()   = R.transpose() * f;
        sensor->tau() = R.transpose() * (tau - p.cross(f));
        sensor->notifyStateChange();
    }
}


void PhysXSimulatorItem::initialize(ExtensionManager* ext)
{
    ext->itemManager().registerClass<PhysXSimulatorItem>(N_("PhysXSimulatorItem"));
    ext->itemManager().addCreationPanel<PhysXSimulatorItem>();
}


PhysXSimulatorItem::PhysXSimulatorItem()
{
    impl = new PhysXSimulatorItemImpl(this);
}


PhysXSimulatorItemImpl::PhysXSimulatorItemImpl(PhysXSimulatorItem* self)
    : self(self)

{
    numOfinstance++;
    initialize();

    gravity << 0.0, 0.0, -DEFAULT_GRAVITY_ACCELERATION;
    
    isJointLimitMode = false;
    staticFriction = 0.5;
    dynamicFriction = 0.5;
    restitution = 0.1;

}


PhysXSimulatorItem::PhysXSimulatorItem(const PhysXSimulatorItem& org)
    : SimulatorItem(org)
{
    impl = new PhysXSimulatorItemImpl(this, *org.impl);
}


PhysXSimulatorItemImpl::PhysXSimulatorItemImpl(PhysXSimulatorItem* self, const PhysXSimulatorItemImpl& org)
    : self(self)
{
    numOfinstance++;
    initialize();

    gravity << 0.0, 0.0, -DEFAULT_GRAVITY_ACCELERATION;

    staticFriction = org.staticFriction;
    dynamicFriction = org.dynamicFriction;
    restitution = org.restitution;
    isJointLimitMode = org.isJointLimitMode;

}


void PhysXSimulatorItemImpl::initialize()
{
    mv = MessageView::instance();

    if(!pxFoundation)
#ifdef VERSION_3_3_LATER
        pxFoundation = PxCreateFoundation(PX_FOUNDATION_VERSION, pxDefaultAllocatorCallback, pxDefaultErrorCallback);
#else
        pxFoundation = PxCreateFoundation(PX_PHYSICS_VERSION, pxDefaultAllocatorCallback, pxDefaultErrorCallback);
#endif
    if(!pxCooking)
        pxCooking = PxCreateCooking(PX_PHYSICS_VERSION, *pxFoundation, PxCookingParams((PxTolerancesScale())));

#ifdef VERSION_3_3_LATER
    pxPhysics = PxCreatePhysics(PX_PHYSICS_VERSION, *pxFoundation, PxTolerancesScale(), true);
#else
    if(!pxProfileZoneManager)
        pxProfileZoneManager = &PxProfileZoneManager::createProfileZoneManager(pxFoundation);
    pxPhysics = PxCreatePhysics(PX_PHYSICS_VERSION, *pxFoundation, PxTolerancesScale(), true, pxProfileZoneManager);
#endif

    if(!pxPhysics)
        mv->putln("PxCreatePhysics failed!");

    pxScene = 0;
    pxDispatcher = 0;
    pxMaterial = 0;
}


void PhysXSimulatorItemImpl::finalize()
{
#ifndef VERSION_3_3_LATER
    if(pxProfileZoneManager)
        pxProfileZoneManager->release();
#endif
    if(pxCooking)
        pxCooking->release();
    if(pxFoundation)
        pxFoundation->release();
}


PhysXSimulatorItem::~PhysXSimulatorItem()
{
    delete impl;
}


PhysXSimulatorItemImpl::~PhysXSimulatorItemImpl()
{
    clear();

    if(pxPhysics){
        pxPhysics->release();
        pxPhysics = 0;
    }
    
    numOfinstance--;
    if(!numOfinstance)
        finalize();

}


void PhysXSimulatorItem::setAllLinkPositionOutputMode(bool on)
{
    // The mode is not changed.
    // This simulator only supports the all link position output
    // because joint positions may be slightly changed
}


void PhysXSimulatorItemImpl::clear()
{
    if(pxMaterial){
        pxMaterial->release();
        pxMaterial = 0;
    }
    if(pxScene){
        pxScene->release();
        pxScene = 0;
    }
    if(pxDispatcher){
        pxDispatcher->release();
        pxDispatcher = 0;
    }

}    


Item* PhysXSimulatorItem::doDuplicate() const
{
    return new PhysXSimulatorItem(*this);
}


SimulationBody* PhysXSimulatorItem::createSimulationBody(Body* orgBody)
{
    return new PhysXBody(*orgBody);
}


bool PhysXSimulatorItem::initializeSimulation(const std::vector<SimulationBody*>& simBodies)
{
    return impl->initializeSimulation(simBodies);
}


bool PhysXSimulatorItemImpl::initializeSimulation(const std::vector<SimulationBody*>& simBodies)
{
    clear();

    PxSceneDesc sceneDesc(pxPhysics->getTolerancesScale());
    sceneDesc.gravity = PxVec3(gravity.x(), gravity.y(), gravity.z());
    //sceneDesc.flags |= PxSceneFlag::eREQUIRE_RW_LOCK;
    pxDispatcher = PxDefaultCpuDispatcherCreate(0);
    sceneDesc.cpuDispatcher = pxDispatcher;
    sceneDesc.filterShader = customFilterShader;
    sceneDesc.contactModifyCallback = this;
    if(DEBUG_COLLISION)
        sceneDesc.simulationEventCallback = this;

    pxScene = pxPhysics->createScene(sceneDesc);
    if (!pxScene)
        mv->putln("createScene failed!");
    pxMaterial = pxPhysics->createMaterial(staticFriction, dynamicFriction, restitution);
    if(!pxMaterial)
        mv->putln("createMaterial failed!");


    timeStep = self->worldTimeStep();

    for(size_t i=0; i < simBodies.size(); ++i){
        addBody(static_cast<PhysXBody*>(simBodies[i]));
    }

    return true;
}


void PhysXSimulatorItemImpl::addBody(PhysXBody* physXBody)
{
    Body& body = *physXBody->body();

    Link* rootLink = body.rootLink();
    rootLink->v().setZero();
    rootLink->dv().setZero();
    rootLink->w().setZero();
    rootLink->dw().setZero();

    for(int i=0; i < body.numJoints(); ++i){
        Link* joint = body.joint(i);
        joint->u() = 0.0;
        joint->dq() = 0.0;
        joint->ddq() = 0.0;
        joint->q_target() = joint->q();
        joint->dq_target() = joint->dq();
    }
    
    body.clearExternalForces();
    body.calcForwardKinematics(true, true);


    physXBody->createBody(this);
}


void PhysXSimulatorItem::initializeSimulationThread()
{

}


void PhysXSimulatorItemImpl::onContact(const PxContactPairHeader& pairHeader, const PxContactPair* pairs, PxU32 nbPairs) 
{
    Link* link0 = (Link*)pairHeader.actors[0]->userData;
    Link* link1 = (Link*)pairHeader.actors[1]->userData;
    std::cout << link0->name() << " " << link1->name() << std::endl;

    std::vector<PxContactPairPoint> contactPoints;
    for(size_t i=0;i<nbPairs; i++){
        int contactCount = pairs[i].contactCount;
        if(contactCount)
            {
                contactPoints.resize(contactCount);
                pairs[i].extractContacts(&contactPoints[0], contactCount);

                for(int j=0;j<contactCount;j++)
                    {
                        PxVec3 point = contactPoints[j].position;
                        std::cout << "point= " << point[0] << " " << point[1] << " " << point[2] << std::endl;
                        PxVec3 normal = contactPoints[j].normal;
                        std::cout << "normal= " << normal[0] << " " << normal[1] << " " << normal[2] << std::endl;
                    }
            }
    }
}


void PhysXSimulatorItemImpl::onContactModify(PxContactModifyPair* const pairs, PxU32 count)
{
    Link* link0 = (Link*)pairs->actor[0]->userData;
    Link* link1 = (Link*)pairs->actor[1]->userData;

    //std::cout << link0->name() << " " << link1->name() << std::endl;

    Link* crawlerlink = 0;
    double sign = 1;
    if(link0->actuationMode() == Link::JOINT_SURFACE_VELOCITY){
        crawlerlink = link0;
    } else if(link1->actuationMode() == Link::JOINT_SURFACE_VELOCITY){
        crawlerlink = link1;
        sign = -1;
    }
    if(!crawlerlink)
        return;

    const Vector3 axis0 = crawlerlink->R() * crawlerlink->a();
    const PxVec3 axis(axis0.x(), axis0.y(), axis0.z()); 
    for(size_t i=0; i<count; i++){
        PxVec3 normal = pairs->contacts.getNormal(i);
        PxVec3 dir = axis.cross(normal);
        if(dir.magnitude() < 1e-5)
            continue;

        dir.normalize();
        dir *= sign * crawlerlink->dq_target();
        pairs->contacts.setTargetVelocity(i, dir);
    }
}


bool PhysXSimulatorItem::stepSimulation(const std::vector<SimulationBody*>& activeSimBodies)
{
    return impl->stepSimulation(activeSimBodies);
}


bool PhysXSimulatorItemImpl::stepSimulation(const std::vector<SimulationBody*>& activeSimBodies)
{
    for(size_t i=0; i < activeSimBodies.size(); ++i){
        PhysXBody* physXBody = static_cast<PhysXBody*>(activeSimBodies[i]);
        physXBody->body()->setVirtualJointForces();
        physXBody->setControlValToPhysX();
    }

    pxScene->simulate(timeStep);
    pxScene->fetchResults(true);

    for(size_t i=0; i < activeSimBodies.size(); ++i){
        PhysXBody* physXBody = static_cast<PhysXBody*>(activeSimBodies[i]);

        physXBody->getKinematicStateFromPhysX();

        if(!physXBody->sensorHelper.forceSensors().empty()){
            physXBody->updateForceSensors();
        }

        if(physXBody->sensorHelper.hasGyroOrAccelerationSensors()){
            physXBody->sensorHelper.updateGyroAndAccelerationSensors();
        }
    }

    return true;
}


void PhysXSimulatorItem::doPutProperties(PutPropertyFunction& putProperty)
{
    SimulatorItem::doPutProperties(putProperty);
    impl->doPutProperties(putProperty);
}


void PhysXSimulatorItemImpl::doPutProperties(PutPropertyFunction& putProperty)
{
    putProperty.decimals(2).min(0.0)
        (_("Static Friction"), staticFriction, changeProperty(staticFriction));

    putProperty.decimals(2).min(0.0)
        (_("Dynamic Friction"), dynamicFriction, changeProperty(dynamicFriction));

    putProperty.decimals(2).min(0.0)
        (_("Restitution"), restitution, changeProperty(restitution));

    putProperty(_("Limit joint range"), isJointLimitMode, changeProperty(isJointLimitMode));

}


bool PhysXSimulatorItem::store(Archive& archive)
{
    SimulatorItem::store(archive);
    impl->store(archive);
    return true;
}


void PhysXSimulatorItemImpl::store(Archive& archive)
{
    archive.write("staticFriction", staticFriction);
    archive.write("dynamicFriction", dynamicFriction);
    archive.write("Restitution", restitution);
    archive.write("jointLimitMode", isJointLimitMode);
}


bool PhysXSimulatorItem::restore(const Archive& archive)
{
    SimulatorItem::restore(archive);
    impl->restore(archive);
    return true;
}


void PhysXSimulatorItemImpl::restore(const Archive& archive)
{
    string symbol;

    archive.read("staticFriction", staticFriction);
    archive.read("dynamicFriction", dynamicFriction);
    archive.read("Restitution", restitution);
    archive.read("jointLimitMode", isJointLimitMode);
}
