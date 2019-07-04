/**
   \file
   \author Shizuko Hattori
*/

#include "BulletCollisionDetector.h"
#include <cnoid/IdPair>
#include <cnoid/MeshExtractor>
#include <cnoid/SceneDrawables>
#include <cnoid/stdx/optional>
#include <btBulletDynamicsCommon.h>
#include <HACD/hacdHACD.h>
#include <BulletCollision/Gimpact/btGImpactShape.h>
#include <BulletCollision/Gimpact/btGImpactCollisionAlgorithm.h>

using namespace std;
using namespace cnoid;

namespace {

typedef CollisionDetector::GeometryHandle GeometryHandle;

const btScalar DEFAULT_COLLISION_MARGIN = 0.0001;
const bool useHACD = true;

struct FactoryRegistration
{
    FactoryRegistration(){
        CollisionDetector::registerFactory(
                "BulletCollisionDetector",
                [](){ return new BulletCollisionDetector; });
    }
} factoryRegistration;

class GeometryInfo : public Referenced
{
public :
    GeometryInfo();
    ~GeometryInfo();
    GeometryHandle geometryHandle;
    ReferencedPtr object;
    btCollisionObject* collisionObject;
    btCollisionShape* collisionShape;
    vector<btScalar> vertices;
    vector<int> triangles;
    btTriangleIndexVertexArray* meshData;
    btTriangleMesh* trimesh;
    bool isStatic;
};
typedef ref_ptr<GeometryInfo> GeometryInfoPtr;

GeometryInfo::GeometryInfo()
{
    geometryHandle = 0;
    collisionObject = 0;
    collisionShape = 0;
    vertices.clear();
    triangles.clear();
    meshData = 0;
    trimesh = 0;
    isStatic = false;
}

GeometryInfo::~GeometryInfo()
{
    btCompoundShape* compoundShape = dynamic_cast<btCompoundShape*>(collisionShape);
    if(compoundShape){
        int num = compoundShape->getNumChildShapes();
        for(int i=0; i<num; i++){
            delete compoundShape->getChildShape(i);
        }
        delete compoundShape;
    }else{
        if(collisionShape)
            delete collisionShape;
    }
    if(meshData)
        delete meshData;
    if(trimesh)
        delete trimesh;
    if(collisionObject)
        delete collisionObject;
}
}


namespace cnoid {

class BulletCollisionDetectorImpl
{
public:
    vector<GeometryInfoPtr> geometryInfos;
    btDefaultCollisionConfiguration* collisionConfiguration;
    btCollisionDispatcher* dispatcher;
    btBroadphaseInterface* broadphase;
    btCollisionWorld* collisionWorld;

    MeshExtractor meshExtractor;
    typedef set<pair<GeometryHandle, GeometryHandle>> GeometryHandlePairSet;
    GeometryHandlePairSet nonInterfarencePairs;
    GeometryHandlePairSet interfarencePairs;
    std::function<void(const CollisionPair&)> callbackOnCollisionDetected;

    BulletCollisionDetectorImpl();
    ~BulletCollisionDetectorImpl();
    stdx::optional<GeometryHandle> addGeometry(SgNode* geometry);
    void addMesh(GeometryInfo* model);
    void setNonInterfarenceGeometyrPair(GeometryHandle geometry1, GeometryHandle geometry2);
    bool makeReady();
    void setGeometryPosition(GeometryInfo* ginfo, const Position& position);
    void detectCollisions();
    void detectObjectCollisions(btCollisionObject* object1, btCollisionObject* object2, CollisionPair& collisionPair);
};
}


BulletCollisionDetector::BulletCollisionDetector()
{
    impl = new BulletCollisionDetectorImpl;
}


BulletCollisionDetector::~BulletCollisionDetector()
{
    delete impl;
}


const char* BulletCollisionDetector::name() const
{
    return "BulletCollisionDetector";
}


CollisionDetector* BulletCollisionDetector::clone() const
{
    return new BulletCollisionDetector;
}


BulletCollisionDetectorImpl::BulletCollisionDetectorImpl()
{
    collisionConfiguration = new btDefaultCollisionConfiguration();
    dispatcher = new btCollisionDispatcher(collisionConfiguration);
    broadphase = new btDbvtBroadphase();
    collisionWorld = new btCollisionWorld(dispatcher,broadphase,collisionConfiguration);
    btGImpactCollisionAlgorithm::registerAlgorithm(dispatcher);
}


BulletCollisionDetectorImpl::~BulletCollisionDetectorImpl()
{
    if(collisionWorld)
        delete collisionWorld;
    if(dispatcher)
        delete dispatcher;
    if(collisionConfiguration)
        delete collisionConfiguration;	
    if(broadphase)
        delete broadphase;
}


void BulletCollisionDetector::clearGeometries()
{
    impl->geometryInfos.clear();
    impl->nonInterfarencePairs.clear();
    impl->interfarencePairs.clear();
}


int BulletCollisionDetector::numGeometries() const
{
    return impl->geometryInfos.size();
}


stx::optional<GeometryHandle> BulletCollisionDetector::addGeometry(SgNode* geometry)
{
    return impl->addGeometry(geometry);
}


stdx::optional<GeometryHandle> BulletCollisionDetectorImpl::addGeometry(SgNode* geometry)
{
    if(geometry){
        GeometryHandle handle = geometryInfos.size();
        GeometryInfoPtr ginfo = new GeometryInfo;
        ginfo->geometryHandle = handle;
        if(meshExtractor.extract(geometry,  [this, ginfo](){ addMesh(ginfo.get()); })){
            if(!useHACD){
                if(!ginfo->vertices.empty()){
                    ginfo->meshData = new btTriangleIndexVertexArray( ginfo->triangles.size()/3, &ginfo->triangles[0], sizeof(int)*3,
                            ginfo->vertices.size()/3, &ginfo->vertices[0], sizeof(btScalar)*3);
                    btGImpactMeshShape* meshShape = new btGImpactMeshShape(ginfo->meshData);
                    meshShape->setLocalScaling(btVector3(1.f,1.f,1.f));
                    meshShape->setMargin(DEFAULT_COLLISION_MARGIN);
                    meshShape->updateBound();
                    btCompoundShape* compoundShape = dynamic_cast<btCompoundShape*>(ginfo->collisionShape);
                    if(compoundShape){
                        btTransform T;
                        T.setIdentity();
                        compoundShape->addChildShape(T, meshShape);
                    }else
                        ginfo->collisionShape = meshShape;
                }
            }else{
                if(!ginfo->vertices.empty()){
                    ginfo->trimesh = new btTriangleMesh();
                    for (size_t i=0; i<ginfo->triangles.size()/3; i++){
                        int index0 = ginfo->triangles[i*3];
                        int index1 = ginfo->triangles[i*3+1];
                        int index2 = ginfo->triangles[i*3+2];
                        vector<btScalar>& vertices = ginfo->vertices;
                        btVector3 vertex0(vertices[index0*3], vertices[index0*3+1], vertices[index0*3+2]);
                        btVector3 vertex1(vertices[index1*3], vertices[index1*3+1], vertices[index1*3+2]);
                        btVector3 vertex2(vertices[index2*3], vertices[index2*3+1], vertices[index2*3+2]);
                        ginfo->trimesh->addTriangle(vertex0,vertex1,vertex2);
                    }
                    btBvhTriangleMeshShape* concaveShape = new btBvhTriangleMeshShape(ginfo->trimesh, true);
                    concaveShape->setMargin(DEFAULT_COLLISION_MARGIN);
                    btCompoundShape* compoundShape = dynamic_cast<btCompoundShape*>(ginfo->collisionShape);
                    if(compoundShape){
                        btTransform T;
                        T.setIdentity();
                        compoundShape->addChildShape(T, concaveShape);
                    }else
                        ginfo->collisionShape = concaveShape;
                }
            }
            ginfo->collisionObject = new btCollisionObject();
            ginfo->collisionObject->setCollisionShape(ginfo->collisionShape);
            geometryInfos.push_back(ginfo);
            return handle;
        }
    }

    return boost::none;
}


void BulletCollisionDetectorImpl::addMesh(GeometryInfo* ginfo)
{
    SgMesh* mesh = meshExtractor.currentMesh();
    const Affine3& T = meshExtractor.currentTransform();

    bool meshAdded = false;
    
    if(mesh->primitiveType() != SgMesh::MESH){
        bool doAddPrimitive = false;
        Vector3 scale;
        stdx::optional<Vector3> translation;
        if(!meshExtractor.isCurrentScaled()){
            scale.setOnes();
            doAddPrimitive = true;
        } else {
            Affine3 S = meshExtractor.currentTransformWithoutScaling().inverse() *
                meshExtractor.currentTransform();

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
                } else if(mesh->primitiveType() == SgMesh::CYLINDER ||
                        mesh->primitiveType() == SgMesh::CAPSULE ){
                    // check if the bottom circle face is uniformly scaled
                    if(scale.x() == scale.z()){
                        doAddPrimitive = true;
                    }
                } else if(mesh->primitiveType() == SgMesh::CONE){
                    if(scale.x() == scale.z()){
                        doAddPrimitive = true;
                    }
                }
            }
        }
        if(doAddPrimitive){
            bool created = false;

            btCollisionShape* primitiveShape;
            switch(mesh->primitiveType()){
            case SgMesh::BOX : {
                const Vector3& s = mesh->primitive<SgMesh::Box>().size;
                primitiveShape = new btBoxShape(btVector3(s.x() * scale.x()/2.0, s.y() * scale.y()/2.0, s.z() * scale.z()/2.0));
                created = true;
                break; }
            case SgMesh::SPHERE : {
                SgMesh::Sphere sphere = mesh->primitive<SgMesh::Sphere>();
                primitiveShape = new btSphereShape(sphere.radius * scale.x());
                created = true;
                break; }
            case SgMesh::CYLINDER : {
                SgMesh::Cylinder cylinder = mesh->primitive<SgMesh::Cylinder>();
                primitiveShape = new btCylinderShape(btVector3(cylinder.radius * scale.x(), cylinder.height * scale.y()/2.0, cylinder.radius * scale.x()));
                created = true;
                break; }
            case SgMesh::CONE : {
                SgMesh::Cone cone = mesh->primitive<SgMesh::Cone>();
                primitiveShape = new btConeShape(cone.radius * scale.x(), cone.height * scale.y());
                created = true;
                break; }
            case SgMesh::CAPSULE : {
                SgMesh::Capsule capsule = mesh->primitive<SgMesh::Capsule>();
                primitiveShape = new btCapsuleShape(capsule.radius * scale.x(), capsule.height * scale.y());
                created = true;
                break; }
            default :
                break;
            }
            if(created){
                primitiveShape->setMargin(DEFAULT_COLLISION_MARGIN);
                btCompoundShape* compoundShape = dynamic_cast<btCompoundShape*>(ginfo->collisionShape);
                if(!compoundShape){
                    ginfo->collisionShape = new btCompoundShape();
                    ginfo->collisionShape->setLocalScaling(btVector3(1.f,1.f,1.f));
                    compoundShape = dynamic_cast<btCompoundShape*>(ginfo->collisionShape);
                }
                Affine3 T_ = meshExtractor.currentTransformWithoutScaling();
                if(translation){
                    T_ *= Translation3(*translation);
                }
                btVector3 p(T_(0,3), T_(1,3), T_(2,3));
                btMatrix3x3 R(T_(0,0), T_(0,1), T_(0,2),
                              T_(1,0), T_(1,1), T_(1,2),
                              T_(2,0), T_(2,1), T_(2,2));
                btTransform btT(R, p);
                compoundShape->addChildShape(btT, primitiveShape);
                meshAdded = true;
            }
        }
    }

    if(!meshAdded){
        if(!useHACD || ginfo->isStatic){
            const int vertexIndexTop = ginfo->vertices.size() / 3;

            const SgVertexArray& vertices_ = *mesh->vertices();
            const int numVertices = vertices_.size();
            for(int i=0; i < numVertices; ++i){
                const Vector3 v = T * vertices_[i].cast<Position::Scalar>();
                ginfo->vertices.push_back((btScalar)v.x());
                ginfo->vertices.push_back((btScalar)v.y());
                ginfo->vertices.push_back((btScalar)v.z());
            }

            const int numTriangles = mesh->numTriangles();
            for(int i=0; i < numTriangles; ++i){
                SgMesh::TriangleRef tri = mesh->triangle(i);
                ginfo->triangles.push_back(vertexIndexTop + tri[0]);
                ginfo->triangles.push_back(vertexIndexTop + tri[1]);
                ginfo->triangles.push_back(vertexIndexTop + tri[2]);
            }
        }else{
            btConvexHullShape* convexHullShape = dynamic_cast<btConvexHullShape*>(ginfo->collisionShape);
            if(convexHullShape){
                btCompoundShape* compoundShape = new btCompoundShape();
                compoundShape->setLocalScaling(btVector3(1.f,1.f,1.f));

                btTransform T;
                T.setIdentity();
                compoundShape->addChildShape(T, convexHullShape);
                ginfo->collisionShape = compoundShape;
            }

            std::vector< HACD::Vec3<HACD::Real> > points;
            std::vector< HACD::Vec3<long> > triangles;

            const SgVertexArray& vertices_ = *mesh->vertices();
            const int numVertices = vertices_.size();
            for(int i=0; i < numVertices; ++i){
                const Vector3 v = T * vertices_[i].cast<Position::Scalar>();
                HACD::Vec3<HACD::Real> vertex(v.x(), v.y(), v.z());
                points.push_back(vertex);
            }

            const int numTriangles = mesh->numTriangles();
            for(int i=0; i < numTriangles; ++i){
                SgMesh::TriangleRef tri = mesh->triangle(i);
                HACD::Vec3<long> triangle(tri[0], tri[1], tri[2]);
                triangles.push_back(triangle);
            }

            HACD::HACD hacd;
            hacd.SetPoints(&points[0]);
            hacd.SetNPoints(points.size());
            hacd.SetTriangles(&triangles[0]);
            hacd.SetNTriangles(triangles.size());
            hacd.SetCompacityWeight(0.1);
            hacd.SetVolumeWeight(0.0);

            size_t nClusters = 1;
            double concavity = 100;
            bool invert = false;
            bool addExtraDistPoints = false;
            bool addNeighboursDistPoints = false;
            bool addFacesPoints = false;       

            hacd.SetNClusters(nClusters);                     // minimum number of clusters
            hacd.SetNVerticesPerCH(100);                      // max of 100 vertices per convex-hull
            hacd.SetConcavity(concavity);                     // maximum concavity
            hacd.SetAddExtraDistPoints(addExtraDistPoints);   
            hacd.SetAddNeighboursDistPoints(addNeighboursDistPoints);   
            hacd.SetAddFacesPoints(addFacesPoints); 
            hacd.Compute();

            btTransform T;
            T.setIdentity();

            nClusters = hacd.GetNClusters();
            if(nClusters>1){
                btCompoundShape* compoundShape = dynamic_cast<btCompoundShape*>(ginfo->collisionShape);
                if(!compoundShape){
                    ginfo->collisionShape = new btCompoundShape();
                    ginfo->collisionShape->setLocalScaling(btVector3(1.f,1.f,1.f));
                }
            }

            for(size_t i=0; i<nClusters; i++){
                size_t nPoints = hacd.GetNPointsCH(i);
                size_t nTriangles = hacd.GetNTrianglesCH(i);

                HACD::Vec3<HACD::Real> * pointsCH = new HACD::Vec3<HACD::Real>[nPoints];
                HACD::Vec3<long> * trianglesCH = new HACD::Vec3<long>[nTriangles];
                hacd.GetCH(i, pointsCH, trianglesCH);
                
                btAlignedObjectArray<btVector3> newVertices_;
                for(size_t j=0; j<nTriangles; j++){
                    long index0 = trianglesCH[j].X();
                    long index1 = trianglesCH[j].Y();
                    long index2 = trianglesCH[j].Z();
                    btVector3 vertex0(pointsCH[index0].X(), pointsCH[index0].Y(), pointsCH[index0].Z());
                    btVector3 vertex1(pointsCH[index1].X(), pointsCH[index1].Y(), pointsCH[index1].Z());
                    btVector3 vertex2(pointsCH[index2].X(), pointsCH[index2].Y(), pointsCH[index2].Z());
                    newVertices_.push_back(vertex0);
                    newVertices_.push_back(vertex1);
                    newVertices_.push_back(vertex2);
                }
                delete [] pointsCH;
                delete [] trianglesCH;

                /*
                  float collisionMargin = 0.01f;
                  btAlignedObjectArray<btVector3> planeEquations;
                  btGeometryUtil::getPlaneEquationsFromVertices(newVertices_, planeEquations);

                  btAlignedObjectArray<btVector3> shiftedPlaneEquations;
                  for (int j=0; j<planeEquations.size(); j++){
                  btVector3 plane = planeEquations[j];
                  plane[3] += collisionMargin;
                  shiftedPlaneEquations.push_back(plane);
                  }
                  btAlignedObjectArray<btVector3> shiftedVertices;
                  btGeometryUtil::getVerticesFromPlaneEquations(shiftedPlaneEquations,shiftedVertices);
                
                  btConvexHullShape* convexHullShape_ = new btConvexHullShape(&(shiftedVertices[0].getX()),shiftedVertices.size());
                */
                btConvexHullShape* convexHullShape_ = new btConvexHullShape(&(newVertices_[0].getX()), newVertices_.size());
                convexHullShape_->setMargin(DEFAULT_COLLISION_MARGIN);
                btCompoundShape* compoundShape = dynamic_cast<btCompoundShape*>(ginfo->collisionShape);
                if(compoundShape)
                    compoundShape->addChildShape(T, convexHullShape_);
                else
                    ginfo->collisionShape = convexHullShape_;
            }
        }
    }
}


void BulletCollisionDetector::setCustomObject(GeometryHandle geometry, Referenced* object)
{
    GeometryInfo* ginfo = impl->geometryInfos[geometry];
    if(ginfo){
        ginfo->object = object;
    }
}

void BulletCollisionDetector::setGeometryStatic(GeometryHandle geometry, bool isStatic)
{
    GeometryInfo* ginfo = impl->geometryInfos[geometry];
    if(ginfo){
        ginfo->isStatic = isStatic;
    }
}


void BulletCollisionDetector::setNonInterfarenceGeometyrPair(GeometryHandle geometry1, GeometryHandle geometry2)
{
    impl->nonInterfarencePairs.insert(make_pair(geometry1, geometry2));
}


bool BulletCollisionDetector::makeReady()
{
    return impl->makeReady();
}


bool BulletCollisionDetectorImpl::makeReady()
{
    interfarencePairs.clear();

    const int n = geometryInfos.size();
    for(int i=0; i < n; ++i){
        GeometryInfo* info1 = geometryInfos[i];
        if(info1){
            for(int j = i+1; j < n; ++j){
                GeometryInfo* info2 = geometryInfos[j];
                if(info2){
                    if(!info1->isStatic || !info2->isStatic){
                        if(nonInterfarencePairs.find(make_pair(i, j)) == nonInterfarencePairs.end()){
                            interfarencePairs.insert(make_pair(i,j));
                        }
                    }
                }
            }
        }
    }
    return true;
}


void BulletCollisionDetector::updatePosition(GeometryHandle geometry, const Position& position)
{
    GeometryInfo* ginfo = impl->geometryInfos[geometry];
    if(ginfo){
        impl->setGeometryPosition(ginfo, position);
    }
}


void BulletCollisionDetector::updatePositions
(std::function<void(Referenced* object, Position*& out_Position)> positionQuery)
{
    for(auto& info : impl->geometryInfos){
        if(info){
            Position* T;
            positionQuery(info->object, T);
            impl->setGeometryPosition(info, *T);
        }
    }
}


void BulletCollisionDetectorImpl::setGeometryPosition(GeometryInfo* ginfo, const Position& position)
{
    if(ginfo){
        btVector3 p(position(0,3), position(1,3), position(2,3));
        btMatrix3x3 R(position(0,0), position(0,1), position(0,2),
                      position(1,0), position(1,1), position(1,2),
                      position(2,0), position(2,1), position(2,2));
        btTransform transform;
        transform.setBasis(R);
        transform.setOrigin(p);
        if(ginfo->collisionObject)
            ginfo->collisionObject->setWorldTransform(transform);
    }
}


void BulletCollisionDetector::detectCollisions(std::function<void(const CollisionPair&)> callback)
{
    impl->callbackOnCollisionDetected = callback;
    impl->detectCollisions();
}


void BulletCollisionDetectorImpl::detectCollisions()
{
    for(auto& interfarencePair : interfarencePairs){
        GeometryInfo* ginfo1 = geometryInfos[interfarencePair.first];
        GeometryInfo* ginfo2 = geometryInfos[interfarencePair.second];

        if(ginfo1->collisionObject && ginfo2->collisionObject){
            CollisionPair collisionPair(
                    ginfo1->geometryHandle, ginfo1->object, ginfo2->geometryHandle, ginfo2->object);
            detectObjectCollisions(ginfo1->collisionObject, ginfo2->collisionObject, collisionPair);

            if(!collisionPair.collisions().empty()){
                callbackOnCollisionDetected(collisionPair);
            }
        }
    }
}


void BulletCollisionDetectorImpl::detectObjectCollisions(btCollisionObject* object1, btCollisionObject* object2, CollisionPair& collisionPair)
{
    CollisionArray& collisions = collisionPair.collisions();
    #ifdef BT_VER_GT_281
    btCollisionObjectWrapper objectWrapper1(0, object1->getCollisionShape(), object1, object1->getWorldTransform(), -1, -1);
    btCollisionObjectWrapper objectWrapper2(0, object2->getCollisionShape(), object2, object2->getWorldTransform(), -1, -1);
    #else
    btCollisionObjectWrapper objectWrapper1(0, object1->getCollisionShape(), object1, object1->getWorldTransform());
    btCollisionObjectWrapper objectWrapper2(0, object2->getCollisionShape(), object2, object2->getWorldTransform());
    #endif

#ifdef BT_VER_GT_286
    btCollisionAlgorithm* algo = collisionWorld->getDispatcher()->findAlgorithm(&objectWrapper1, &objectWrapper2, 0, BT_CONTACT_POINT_ALGORITHMS);
#else
    btCollisionAlgorithm* algo = collisionWorld->getDispatcher()->findAlgorithm(&objectWrapper1, &objectWrapper2);
#endif

    if (algo){
        btManifoldResult contactPointResult(&objectWrapper1, &objectWrapper2);
        algo->processCollision(&objectWrapper1, &objectWrapper2, collisionWorld->getDispatchInfo(), &contactPointResult);

        btManifoldArray manifoldArray;
        algo->getAllContactManifolds(manifoldArray);
        int numManifolds = manifoldArray.size();
        for (int i=0;i<numManifolds;i++)
            {
                btPersistentManifold* contactManifold = manifoldArray[i];
                const btCollisionObject* obA = contactManifold->getBody0();
                int numContacts = contactManifold->getNumContacts();
                bool swap = obA == object1;

                for (int j=0;j<numContacts;j++)
                    {
                        btManifoldPoint& pt = contactManifold->getContactPoint(j);
                        btVector3 ptA = swap? pt.getPositionWorldOnB() : pt.getPositionWorldOnA();
                        btVector3 normal = swap? -pt.m_normalWorldOnB : pt.m_normalWorldOnB;

                        collisions.push_back(Collision());
                        Collision& collision = collisions.back();
                        collision.point = Vector3(ptA.x(), ptA.y(), ptA.z());
                        collision.normal = Vector3(normal.x(), normal.y(), normal.z());
                        collision.depth =  -pt.getDistance();
                    }
            }
    }
}
