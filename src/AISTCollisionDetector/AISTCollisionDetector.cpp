/**
   \file
   \author Shin'ichiro Nakaoka
*/

#include "AISTCollisionDetector.h"
#include "ColdetModelPair.h"
#include <cnoid/IdPair>
#include <cnoid/SceneDrawables>
#include <cnoid/MeshExtractor>
#include <cnoid/ThreadPool>
#include <boost/make_shared.hpp>
#include <boost/scoped_ptr.hpp>
#include <boost/bind.hpp>
#include <boost/random.hpp>
#include <algorithm>

using namespace std;
using namespace cnoid;

namespace {

const bool MULTITHREAD_TYPE = 0;
const bool USE_THREAD_POOL = true;
const bool ENABLE_SHUFFLE = false;

CollisionDetectorPtr factory()
{
    return boost::make_shared<AISTCollisionDetector>();
}

struct FactoryRegistration
{
    FactoryRegistration(){
        CollisionDetector::registerFactory("AISTCollisionDetector", factory);
    }
} factoryRegistration;


class ColdetModelEx : public ColdetModel
{
public:
    ColdetModelEx() { isStatic = false; }
    bool isStatic;
};
typedef boost::shared_ptr<ColdetModelEx> ColdetModelExPtr;
        

class ColdetModelPairEx : public ColdetModelPair
{
    int id1_;
    int id2_;
        
public:
    ColdetModelPairEx(const ColdetModelPtr& model1, int id1, const ColdetModelPtr& model2, int id2)
        : ColdetModelPair(model1, model2)
        {
            id1_ = id1;
            id2_ = id2;
        }
    const int id1() const { return id1_; }
    const int id2() const { return id2_; }
};
typedef boost::shared_ptr<ColdetModelPairEx> ColdetModelPairExPtr;


typedef map<weak_ref_ptr<SgNode>, ColdetModelExPtr>  ModelMap;
ModelMap modelCache;
}

namespace cnoid {

class AISTCollisionDetectorImpl
{
public:
    vector<ColdetModelExPtr> models;

    typedef vector<ColdetModelPairExPtr> ModelPairArray;
    ModelPairArray modelPairs;

    int maxNumThreads;
    int numThreads;
    boost::scoped_ptr<ThreadPool> threadPool;
    boost::thread_group threadGroup;

    vector<int> shuffledPairIndices;
    boost::random::mt19937 randomEngine;
    boost::random_number_generator<boost::random::mt19937>randomNumberGenerator;

    typedef set< IdPair<> > IdPairSet;
    IdPairSet nonInterfarencePairs;

    MeshExtractor* meshExtractor;
        
    AISTCollisionDetectorImpl();
    ~AISTCollisionDetectorImpl();
    int addGeometry(SgNode* geometry);
    void addMesh(ColdetModelEx* model);
    bool makeReady();
    void detectCollisions(boost::function<void(const CollisionPair&)> callback);
    void detectCollisionsInParallel(boost::function<void(const CollisionPair&)> callback);

    // for multithread type 0
    typedef vector<CollisionPair> CollisionPairArray;
    vector<CollisionPairArray> collisionPairArrays;

    void extractCollisionsOfAssignedPairs(
        int pairIndexBegin, int pairIndexEnd, CollisionPairArray& collisionPairs);    
    void dispatchCollisionsInCollisionPairArrays(boost::function<void(const CollisionPair&)> callback);    
    
    // for multithread type 1
    vector< vector<ColdetModelPairEx*> > collidingModelPairArrays;

    void checkCollisionsOfAssignedPairs(
        int pairIndexBegin, int pairIndexEnd, vector<ColdetModelPairEx*>& collidingModelPairs);
    void dispatchCollisionsInCollidingModelPairs(boost::function<void(const CollisionPair&)> callback);    
};

}


AISTCollisionDetector::AISTCollisionDetector()
{
    impl = new AISTCollisionDetectorImpl();
}


AISTCollisionDetectorImpl::AISTCollisionDetectorImpl()
    : randomNumberGenerator(randomEngine)
{
    maxNumThreads = 0;
    numThreads = 0;
    meshExtractor = new MeshExtractor();
}


AISTCollisionDetectorImpl::~AISTCollisionDetectorImpl()
{
    delete meshExtractor;

}


AISTCollisionDetector::~AISTCollisionDetector()
{
    delete impl;
}


const char* AISTCollisionDetector::name() const
{
    return "AISTCollisionDetector";
}


CollisionDetectorPtr AISTCollisionDetector::clone() const
{
    return boost::make_shared<AISTCollisionDetector>();
}


void AISTCollisionDetector::setNumThreads(int n)
{
    impl->maxNumThreads = n;
}

        
void AISTCollisionDetector::clearGeometries()
{
    impl->models.clear();
    impl->modelPairs.clear();
    impl->nonInterfarencePairs.clear();
}


int AISTCollisionDetector::numGeometries() const
{
    return impl->models.size();
}


int AISTCollisionDetector::addGeometry(SgNodePtr geometry)
{
    return impl->addGeometry(geometry.get());
}


int AISTCollisionDetectorImpl::addGeometry(SgNode* geometry)
{
    const int index = models.size();
    bool isValid = false;

    if(geometry){
        ColdetModelExPtr model = boost::make_shared<ColdetModelEx>();
        if(meshExtractor->extract(geometry, boost::bind(&AISTCollisionDetectorImpl::addMesh, this, model.get()))){
            model->setName(geometry->name());
            model->build();
            if(model->isValid()){
                models.push_back(model);
                isValid = true;
            }
        }
    }

    if(!isValid){
        models.push_back(ColdetModelExPtr());
    }

    return index;
}


void AISTCollisionDetectorImpl::addMesh(ColdetModelEx* model)
{
    SgMesh* mesh = meshExtractor->currentMesh();
    const Affine3& T = meshExtractor->currentTransform();
    
    const int vertexIndexTop = model->getNumVertices();
    
    const SgVertexArray& vertices = *mesh->vertices();
    const int numVertices = vertices.size();
    for(int i=0; i < numVertices; ++i){
        const Vector3 v = T * vertices[i].cast<Affine3::Scalar>();
        model->addVertex(v.x(), v.y(), v.z());
    }

    const int numTriangles = mesh->numTriangles();
    for(int i=0; i < numTriangles; ++i){
        SgMesh::TriangleRef tri = mesh->triangle(i);
        const int v0 = vertexIndexTop + tri[0];
        const int v1 = vertexIndexTop + tri[1];
        const int v2 = vertexIndexTop + tri[2];
        model->addTriangle(v0, v1, v2);
    }
}


void AISTCollisionDetector::setGeometryStatic(int geometryId, bool isStatic)
{
    ColdetModelExPtr& model = impl->models[geometryId];
    if(model){
        model->isStatic = isStatic;
    }
}


bool AISTCollisionDetector::enableGeometryCache(bool on)
{
    return false;
}


void AISTCollisionDetector::clearGeometryCache(SgNodePtr geometry)
{
    modelCache.erase(weak_ref_ptr<SgNode>(geometry));
}


void AISTCollisionDetector::clearAllGeometryCaches()
{
    modelCache.clear();
}


void AISTCollisionDetector::setNonInterfarenceGeometyrPair(int geometryId1, int geometryId2)
{
    impl->nonInterfarencePairs.insert(IdPair<>(geometryId1, geometryId2));
}


bool AISTCollisionDetector::makeReady()
{
    return impl->makeReady();
}


bool AISTCollisionDetectorImpl::makeReady()
{
    modelPairs.clear();

    const int n = models.size();
    for(int i=0; i < n; ++i){
        ColdetModelExPtr& model1 = models[i];
        if(model1){
            for(int j = i+1; j < n; ++j){
                ColdetModelExPtr& model2 = models[j];
                if(model2){
                    if(!model1->isStatic || !model2->isStatic){
                        if(nonInterfarencePairs.find(IdPair<>(i, j)) == nonInterfarencePairs.end()){
                            modelPairs.push_back(boost::make_shared<ColdetModelPairEx>(model1, i, model2, j));
                        }
                    }
                }
            }
        }
    }

    const int numPairs = modelPairs.size();

    if(maxNumThreads <= 0){
        numThreads = 0;
        threadPool.reset();
        collisionPairArrays.clear();
        collidingModelPairArrays.clear();

    } else {
        numThreads = (maxNumThreads > numPairs) ? numPairs : maxNumThreads;
        if(USE_THREAD_POOL){
            threadPool.reset(new ThreadPool(numThreads));
        }
        if(ENABLE_SHUFFLE){
            shuffledPairIndices.resize(modelPairs.size());
            for(size_t i=0; i < shuffledPairIndices.size(); ++i){
                shuffledPairIndices[i] = i;
            }
        }
        if(MULTITHREAD_TYPE == 0){
            collisionPairArrays.resize(numThreads);
        } else {
            collidingModelPairArrays.resize(numThreads);
        }
    }

    return true;
}


void AISTCollisionDetector::updatePosition(int geometryId, const Position& position)
{
    ColdetModelExPtr& model = impl->models[geometryId];
    if(model){
        model->setPosition(position);
    }
}


void AISTCollisionDetector::detectCollisions(boost::function<void(const CollisionPair&)> callback)
{
    if(impl->numThreads > 0){
        impl->detectCollisionsInParallel(callback);
    } else {
        impl->detectCollisions(callback);
    }
} 


/**
   \todo Remeber which geometry positions are updated after the last collision detection
   and do the actual collision detection only for the updated geometry pairs.
*/
void AISTCollisionDetectorImpl::detectCollisions(boost::function<void(const CollisionPair&)> callback)
{
    CollisionPair collisionPair;
    vector<Collision>& collisions = collisionPair.collisions;
    
    const int n = modelPairs.size();
    for(int i=0; i < n; ++i){
        ColdetModelPairEx& modelPair = *modelPairs[i];
        const std::vector<collision_data>& cdata = modelPair.detectCollisions();
        if(!cdata.empty()){
            collisionPair.geometryId[0] = modelPair.id1();
            collisionPair.geometryId[1] = modelPair.id2();
            collisions.clear();
            for(size_t j=0; j < cdata.size(); ++j){
                const collision_data& cd = cdata[j];
                for(int k=0; k < cd.num_of_i_points; ++k){
                    if(cd.i_point_new[k]){
                        collisions.push_back(Collision());
                        Collision& collision = collisions.back();
                        collision.point = cd.i_points[k];
                        collision.normal = cd.n_vector;
                        collision.depth = cd.depth;
                    }
                }
            }
            if(!collisions.empty()){
                callback(collisionPair);
            }
        }
    }
}


void AISTCollisionDetectorImpl::detectCollisionsInParallel(boost::function<void(const CollisionPair&)> callback)
{
    if(ENABLE_SHUFFLE){
        std::random_shuffle(shuffledPairIndices.begin(), shuffledPairIndices.end(), randomNumberGenerator);
    }

    const int numPairs = modelPairs.size();
    const int minSize = numPairs / numThreads;
    int remainder = numPairs % numThreads;
    int index = 0;
    for(int i=0; i < numThreads; ++i){
        int size = minSize;
        if(remainder > 0){
            ++size;
            --remainder;
        }
        if(size == 0){
            break;
        }
        if(MULTITHREAD_TYPE == 0){
            if(USE_THREAD_POOL){
                threadPool->start(
                    boost::bind(&AISTCollisionDetectorImpl::extractCollisionsOfAssignedPairs,
                                this, index, index + size, boost::ref(collisionPairArrays[i])));
            } else {
                threadGroup.create_thread(
                    boost::bind(&AISTCollisionDetectorImpl::extractCollisionsOfAssignedPairs,
                                this, index, index + size, boost::ref(collisionPairArrays[i])));
            }
        } else {
            if(USE_THREAD_POOL){
                threadPool->start(
                    boost::bind(&AISTCollisionDetectorImpl::checkCollisionsOfAssignedPairs,
                                this, index, index + size, boost::ref(collidingModelPairArrays[i])));
            } else {
                threadGroup.create_thread(
                    boost::bind(&AISTCollisionDetectorImpl::checkCollisionsOfAssignedPairs,
                                this, index, index + size, boost::ref(collidingModelPairArrays[i])));
            }
        }
        index += size;
    }
    if(USE_THREAD_POOL){
        threadPool->waitLoop();
        //threadPool->wait();
    } else {
        threadGroup.join_all();
    }

    if(MULTITHREAD_TYPE == 0){
        dispatchCollisionsInCollisionPairArrays(callback);
    } else {
        dispatchCollisionsInCollidingModelPairs(callback);
    }
}


void AISTCollisionDetectorImpl::extractCollisionsOfAssignedPairs
(int pairIndexBegin, int pairIndexEnd, CollisionPairArray& collisionPairs)
{
    collisionPairs.clear();

    for(int i=pairIndexBegin; i < pairIndexEnd; ++i){
        ColdetModelPairEx* modelPair;
        if(ENABLE_SHUFFLE){
            modelPair = modelPairs[shuffledPairIndices[i]].get();
        } else {
            modelPair = modelPairs[i].get();
        }
        const std::vector<collision_data>& cdata = modelPair->detectCollisions();
        if(!cdata.empty()){
            collisionPairs.push_back(CollisionPair());
            CollisionPair& collisionPair = collisionPairs.back();
            collisionPair.geometryId[0] = modelPair->id1();
            collisionPair.geometryId[1] = modelPair->id2();
            vector<Collision>& collisions = collisionPair.collisions;
            const int n = cdata.size();
            collisions.reserve(n);
            for(int j=0; j < n; ++j){
                const collision_data& cd = cdata[j];
                for(int k=0; k < cd.num_of_i_points; ++k){
                    if(cd.i_point_new[k]){
                        collisions.push_back(Collision());
                        Collision& collision = collisions.back();
                        collision.point = cd.i_points[k];
                        collision.normal = cd.n_vector;
                        collision.depth = cd.depth;
                    }
                }
            }
            if(collisions.empty()){
                collisionPairs.pop_back();
            }
        }
    }
}

void AISTCollisionDetectorImpl::dispatchCollisionsInCollisionPairArrays
(boost::function<void(const CollisionPair&)> callback)
{
    for(int i=0; i < numThreads; ++i){
        const CollisionPairArray& collisionPairs = collisionPairArrays[i];
        for(size_t j=0; j < collisionPairs.size(); ++j){
            callback(collisionPairs[j]);
        }
    }
}


void AISTCollisionDetectorImpl::checkCollisionsOfAssignedPairs
(int pairIndexBegin, int pairIndexEnd, vector<ColdetModelPairEx*>& collidingModelPairs)
{
    collidingModelPairs.clear();
    for(int i=pairIndexBegin; i < pairIndexEnd; ++i){
        ColdetModelPairEx* modelPair;
        if(ENABLE_SHUFFLE){
            modelPair = modelPairs[shuffledPairIndices[i]].get();
        } else {
            modelPair = modelPairs[i].get();
        }
        if(!modelPair->detectCollisions().empty()){
            collidingModelPairs.push_back(modelPair);
        }
    }
}


void AISTCollisionDetectorImpl::dispatchCollisionsInCollidingModelPairs
(boost::function<void(const CollisionPair&)> callback)
{
    CollisionPair collisionPair;
    vector<Collision>& collisions = collisionPair.collisions;
    for(int i=0; i < numThreads; ++i){
        const vector<ColdetModelPairEx*>& collidingModelPairs = collidingModelPairArrays[i];
        for(size_t j=0; j < collidingModelPairs.size(); ++j){
            ColdetModelPairEx& collidingModelPair = *collidingModelPairs[j];
            const std::vector<collision_data>& cdata = collidingModelPair.collisions();
            if(!cdata.empty()){
                collisionPair.geometryId[0] = collidingModelPair.id1();
                collisionPair.geometryId[1] = collidingModelPair.id2();
                collisions.clear();
                for(size_t k=0; k < cdata.size(); ++k){
                    const collision_data& cd = cdata[k];
                    for(int l=0; l < cd.num_of_i_points; ++l){
                        if(cd.i_point_new[l]){
                            collisions.push_back(Collision());
                            Collision& collision = collisions.back();
                            collision.point = cd.i_points[l];
                            collision.normal = cd.n_vector;
                            collision.depth = cd.depth;
                        }
                    }
                }
                if(!collisions.empty()){
                    callback(collisionPair);
                }
            }
        }
    }
}
