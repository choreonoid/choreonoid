/**
   \file
   \author Shizuko Hattori
*/

#include "BulletCollisionDetector.h"
#include <cnoid/IdPair>
#include <cnoid/MeshExtractor>
#include <boost/make_shared.hpp>
#include <boost/bind.hpp>
#include <boost/optional.hpp>
#include <btBulletDynamicsCommon.h>
#include <HACD/hacdHACD.h>
#include <BulletCollision/Gimpact/btGImpactShape.h>
#include <BulletCollision/Gimpact/btGImpactCollisionAlgorithm.h>

using namespace std;
using namespace boost;
using namespace cnoid;

namespace {

const btScalar DEFAULT_COLLISION_MARGIN = 0.0001;
const bool useHACD = true;

CollisionDetectorPtr factory()
{
    return boost::make_shared<BulletCollisionDetector>();
}

struct FactoryRegistration
{
    FactoryRegistration(){
        CollisionDetector::registerFactory("BulletCollisionDetector", factory);
    }
} factoryRegistration;

class GeometryEx
{
public :
    GeometryEx();
    ~GeometryEx();
    btCollisionObject* collisionObject;
    btCollisionShape* collisionShape;
    vector<btScalar> vertices;
    vector<int> triangles;
    btTriangleIndexVertexArray* meshData;
    btTriangleMesh* trimesh;
    bool isStatic;
};
typedef boost::shared_ptr<GeometryEx> GeometryExPtr;

GeometryEx::GeometryEx()
{
    collisionObject = 0;
    collisionShape = 0;
    vertices.clear();
    triangles.clear();
    meshData = 0;
    trimesh = 0;
    isStatic = false;
}

GeometryEx::~GeometryEx()
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
    BulletCollisionDetectorImpl();
    ~BulletCollisionDetectorImpl();

    vector<GeometryExPtr> models;
    btDefaultCollisionConfiguration* collisionConfiguration;
    btCollisionDispatcher* dispatcher;
    btBroadphaseInterface* broadphase;
    btCollisionWorld* collisionWorld;
    MeshExtractor* meshExtractor;

    typedef set< IdPair<> > IdPairSet;
    IdPairSet modelPairs;
    IdPairSet nonInterfarencePairs;

    int addGeometry(SgNode* geometry);
    void addMesh(GeometryEx* model);
    bool makeReady();
    void updatePosition(int geometryId, const Position& _position);
    void detectCollisions(boost::function<void(const CollisionPair&)> callback);
    void detectObjectCollisions(btCollisionObject* object1, btCollisionObject* object2, CollisionPair& collisionPair);
};
}


BulletCollisionDetector::BulletCollisionDetector()
{
    impl = new BulletCollisionDetectorImpl();
}


BulletCollisionDetector::~BulletCollisionDetector()
{
    delete impl;
}


const char* BulletCollisionDetector::name() const
{
    return "BulletCollisionDetector";
}


CollisionDetectorPtr BulletCollisionDetector::clone() const
{
    return boost::make_shared<BulletCollisionDetector>();
}


BulletCollisionDetectorImpl::BulletCollisionDetectorImpl()
{
    collisionConfiguration = new btDefaultCollisionConfiguration();
    dispatcher = new btCollisionDispatcher(collisionConfiguration);
    broadphase = new btDbvtBroadphase();
    collisionWorld = new btCollisionWorld(dispatcher,broadphase,collisionConfiguration);
    btGImpactCollisionAlgorithm::registerAlgorithm(dispatcher);

    meshExtractor = new MeshExtractor;
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

    delete meshExtractor;
}


void BulletCollisionDetector::clearGeometries()
{
    impl->models.clear();
    impl->nonInterfarencePairs.clear();
    impl->modelPairs.clear();
}


int BulletCollisionDetector::numGeometries() const
{
    return impl->models.size();
}


int BulletCollisionDetector::addGeometry(SgNodePtr geometry)
{
    return impl->addGeometry(geometry.get());
}


int BulletCollisionDetectorImpl::addGeometry(SgNode* geometry)
{
    const int index = models.size();

    if(geometry){
        GeometryExPtr model =  boost::make_shared<GeometryEx>();
        if(meshExtractor->extract(geometry, boost::bind(&BulletCollisionDetectorImpl::addMesh, this, model.get()))){
            if(!useHACD){
                if(!model->vertices.empty()){
                    model->meshData = new btTriangleIndexVertexArray( model->triangles.size()/3, &model->triangles[0], sizeof(int)*3,
                                                                      model->vertices.size()/3, &model->vertices[0], sizeof(btScalar)*3);
                    btGImpactMeshShape* meshShape = new btGImpactMeshShape(model->meshData);
                    meshShape->setLocalScaling(btVector3(1.f,1.f,1.f));
                    meshShape->setMargin(DEFAULT_COLLISION_MARGIN);
                    meshShape->updateBound();
                    btCompoundShape* compoundShape = dynamic_cast<btCompoundShape*>(model->collisionShape);
                    if(compoundShape){
                        btTransform T;
                        T.setIdentity();
                        compoundShape->addChildShape(T, meshShape);
                    }else
                        model->collisionShape = meshShape;
                }
            }else{
                if(!model->vertices.empty()){
                    model->trimesh = new btTriangleMesh();
                    for (size_t i=0; i<model->triangles.size()/3; i++){
                        int index0 = model->triangles[i*3];
                        int index1 = model->triangles[i*3+1];
                        int index2 = model->triangles[i*3+2];
                        vector<btScalar>& vertices = model->vertices;
                        btVector3 vertex0(vertices[index0*3], vertices[index0*3+1], vertices[index0*3+2]);
                        btVector3 vertex1(vertices[index1*3], vertices[index1*3+1], vertices[index1*3+2]);
                        btVector3 vertex2(vertices[index2*3], vertices[index2*3+1], vertices[index2*3+2]);
                        model->trimesh->addTriangle(vertex0,vertex1,vertex2);
                    }
                    btBvhTriangleMeshShape* concaveShape = new btBvhTriangleMeshShape(model->trimesh, true);
                    concaveShape->setMargin(DEFAULT_COLLISION_MARGIN);
                    btCompoundShape* compoundShape = dynamic_cast<btCompoundShape*>(model->collisionShape);
                    if(compoundShape){
                        btTransform T;
                        T.setIdentity();
                        compoundShape->addChildShape(T, concaveShape);
                    }else
                        model->collisionShape = concaveShape;
                }
            }
            model->collisionObject = new btCollisionObject();
            model->collisionObject->setCollisionShape(model->collisionShape);
            models.push_back(model);
        }
    }

    return index;
}


void BulletCollisionDetectorImpl::addMesh(GeometryEx* model)
{
    SgMesh* mesh = meshExtractor->currentMesh();
    const Affine3& T = meshExtractor->currentTransform();

    bool meshAdded = false;
    
    if(mesh->primitiveType() != SgMesh::MESH){
        bool doAddPrimitive = false;
        Vector3 scale;
        optional<Vector3> translation;
        if(!meshExtractor->isCurrentScaled()){
            scale.setOnes();
            doAddPrimitive = true;
        } else {
            Affine3 S = meshExtractor->currentTransformWithoutScaling().inverse() *
                meshExtractor->currentTransform();

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
            default :
                break;
            }
            if(created){
                primitiveShape->setMargin(DEFAULT_COLLISION_MARGIN);
                btCompoundShape* compoundShape = dynamic_cast<btCompoundShape*>(model->collisionShape);
                if(!compoundShape){
                    model->collisionShape = new btCompoundShape();
                    model->collisionShape->setLocalScaling(btVector3(1.f,1.f,1.f));
                    compoundShape = dynamic_cast<btCompoundShape*>(model->collisionShape);
                }
                Affine3 T_ = meshExtractor->currentTransformWithoutScaling();
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
        if(!useHACD || model->isStatic){
            const int vertexIndexTop = model->vertices.size() / 3;

            const SgVertexArray& vertices_ = *mesh->vertices();
            const int numVertices = vertices_.size();
            for(int i=0; i < numVertices; ++i){
                const Vector3 v = T * vertices_[i].cast<Position::Scalar>();
                model->vertices.push_back((btScalar)v.x());
                model->vertices.push_back((btScalar)v.y());
                model->vertices.push_back((btScalar)v.z());
            }

            const int numTriangles = mesh->numTriangles();
            for(int i=0; i < numTriangles; ++i){
                SgMesh::TriangleRef tri = mesh->triangle(i);
                model->triangles.push_back(vertexIndexTop + tri[0]);
                model->triangles.push_back(vertexIndexTop + tri[1]);
                model->triangles.push_back(vertexIndexTop + tri[2]);
            }
        }else{
            btConvexHullShape* convexHullShape = dynamic_cast<btConvexHullShape*>(model->collisionShape);
            if(convexHullShape){
                btCompoundShape* compoundShape = new btCompoundShape();
                compoundShape->setLocalScaling(btVector3(1.f,1.f,1.f));

                btTransform T;
                T.setIdentity();
                compoundShape->addChildShape(T, convexHullShape);
                model->collisionShape = compoundShape;
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
                btCompoundShape* compoundShape = dynamic_cast<btCompoundShape*>(model->collisionShape);
                if(!compoundShape){
                    model->collisionShape = new btCompoundShape();
                    model->collisionShape->setLocalScaling(btVector3(1.f,1.f,1.f));
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
                btCompoundShape* compoundShape = dynamic_cast<btCompoundShape*>(model->collisionShape);
                if(compoundShape)
                    compoundShape->addChildShape(T, convexHullShape_);
                else
                    model->collisionShape = convexHullShape_;
            }
        }
    }
}


void BulletCollisionDetector::setGeometryStatic(int geometryId, bool isStatic)
{
    impl->models[geometryId]->isStatic = isStatic;
}


bool BulletCollisionDetector::enableGeometryCache(bool on)
{
    return false;
}


void BulletCollisionDetector::clearGeometryCache(SgNodePtr geometry)
{
    
}


void BulletCollisionDetector::clearAllGeometryCaches()
{

}


void BulletCollisionDetector::setNonInterfarenceGeometyrPair(int geometryId1, int geometryId2)
{
    impl->nonInterfarencePairs.insert(IdPair<>(geometryId1, geometryId2));
}


bool BulletCollisionDetector::makeReady()
{
    return impl->makeReady();
}


bool BulletCollisionDetectorImpl::makeReady()
{
    modelPairs.clear();

    const int n = models.size();
    for(int i=0; i < n; ++i){
        GeometryExPtr& model1 = models[i];
        if(model1){
            for(int j = i+1; j < n; ++j){
                GeometryExPtr& model2 = models[j];
                if(model2){
                    if(!model1->isStatic || !model2->isStatic){
                        if(nonInterfarencePairs.find(IdPair<>(i, j)) == nonInterfarencePairs.end()){
                            modelPairs.insert(IdPair<>(i,j));
                        }
                    }
                }
            }
        }
    }
    return true;
}


void BulletCollisionDetector::updatePosition(int geometryId, const Position& position)
{
    impl->updatePosition(geometryId, position);
}


void BulletCollisionDetectorImpl::updatePosition(int geometryId, const Position& position)
{
    GeometryExPtr& model = models[geometryId];
    if(model){
        btVector3 p(position(0,3), position(1,3), position(2,3));
        btMatrix3x3 R(position(0,0), position(0,1), position(0,2),
                      position(1,0), position(1,1), position(1,2),
                      position(2,0), position(2,1), position(2,2));
        btTransform transform;
        transform.setBasis(R);
        transform.setOrigin(p);
        if(model->collisionObject)
            model->collisionObject->setWorldTransform(transform);
    }
}


void BulletCollisionDetector::detectCollisions(boost::function<void(const CollisionPair&)> callback)
{
    impl->detectCollisions(callback);
}


void BulletCollisionDetectorImpl::detectCollisions(boost::function<void(const CollisionPair&)> callback)
{
    CollisionPair collisionPair;
    vector<Collision>& collisions = collisionPair.collisions;
    
    for(IdPairSet::iterator it = modelPairs.begin(); it!=modelPairs.end(); it++){
        GeometryExPtr& model1 = models[(*it)(0)];
        GeometryExPtr& model2 = models[(*it)(1)];
        collisions.clear();

        if(model1->collisionObject && model2->collisionObject)
            detectObjectCollisions(model1->collisionObject, model2->collisionObject, collisionPair);

        if(!collisions.empty()){
            collisionPair.geometryId[0] = (*it)(0);
            collisionPair.geometryId[1] = (*it)(1);
            callback(collisionPair);
        }
    }

}


void BulletCollisionDetectorImpl::detectObjectCollisions(btCollisionObject* object1, btCollisionObject* object2, CollisionPair& collisionPair)
{
    vector<Collision>& collisions = collisionPair.collisions;
    btCollisionObjectWrapper objectWrapper1(0, object1->getCollisionShape(), object1, object1->getWorldTransform());
    btCollisionObjectWrapper objectWrapper2(0, object2->getCollisionShape(), object2, object2->getWorldTransform());

    btCollisionAlgorithm* algo = collisionWorld->getDispatcher()->findAlgorithm(&objectWrapper1, &objectWrapper2);
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
