#include "AISTCollisionDetector.h"
#include "ColdetModelPair.h"
#include <cnoid/IdPair>
#include <cnoid/SceneDrawables>
#include <cnoid/MeshExtractor>
#include <cnoid/ThreadPool>
#include <algorithm>
#include <random>
#include <set>

using namespace std;
using namespace cnoid;

namespace {

const bool ENABLE_SHUFFLE = false;

typedef CollisionDetector::GeometryHandle GeometryHandle;

CollisionDetector* factory()
{
    return new AISTCollisionDetector;
}

struct FactoryRegistration
{
    FactoryRegistration(){
        CollisionDetector::registerFactory("AISTCollisionDetector", factory);
    }
} factoryRegistration;

class ColdetModelEx;
typedef ref_ptr<ColdetModelEx> ColdetModelExPtr;

class ColdetModelEx : public ColdetModel
{
public:
    ReferencedPtr object;
    int groupId;
    bool isEnabled;
    bool isStatic;
    stdx::optional<Isometry3> localPosition;
    ColdetModelExPtr sibling;
    
    ColdetModelEx() : groupId(0), isEnabled(true), isStatic(false) { }
};

class ColdetModelPairEx;
typedef ref_ptr<ColdetModelPairEx> ColdetModelPairExPtr;

ColdetModelEx* getColdetModel(GeometryHandle handle)
{
    return reinterpret_cast<ColdetModelEx*>(handle);
}

GeometryHandle getHandle(ColdetModelEx* model)
{
    return reinterpret_cast<GeometryHandle>(model);
}

class ColdetModelPairEx : public ColdetModelPair
{
    ColdetModelPairEx(){ }
    
public:
    ColdetModelPairEx(ColdetModelEx* model1, ColdetModelEx* model2)
        : ColdetModelPair(model1, model2)
    {
        ColdetModelPairEx* last = this;
        for(auto sibling1 = model1->sibling; sibling1; sibling1 = sibling1->sibling){
            for(auto sibling2 = model2->sibling; sibling2; sibling2 = sibling2->sibling){
                last->sibling = new ColdetModelPairEx;
                last->sibling->set(sibling1, sibling2);
                last = last->sibling->sibling;
            }
        }
    }

    ColdetModelEx* model(int which) {
        return static_cast<ColdetModelEx*>(ColdetModelPair::model(which));
    }

    ColdetModelPairExPtr sibling;
};


bool copyCollisionPairCollisions(ColdetModelPairEx* srcPair, CollisionPair& destPair, bool doReserve = false)
{
    vector<Collision>& collisions = destPair.collisions();

    if(collisions.empty()){
        for(int i=0; i < 2; ++i){
            auto model = srcPair->model(i);
            destPair.object(i) = model->object;
            destPair.geometry(i) = getHandle(model);
        }
    }

    const std::vector<collision_data>& cdata = srcPair->collisions();
    const int n = cdata.size();

    if(doReserve){
        collisions.reserve(n);
    }

    for(int j=0; j < n; ++j){
        const collision_data& cd = cdata[j];
        for(int k=0; k < cd.num_of_i_points; ++k){
            if(cd.i_point_new[k]){
                Collision& collision = destPair.newCollision();
                collision.point = cd.i_points[k];
                collision.normal = cd.n_vector;
                collision.depth = cd.depth;
                collision.id1 = cd.id1;
                collision.id2 = cd.id2;
            }
        }
    }

    return !collisions.empty();
}

}

namespace cnoid {

class AISTCollisionDetector::Impl
{
public:
    vector<ColdetModelExPtr> models;
    vector<ColdetModelPairExPtr> modelPairs;
    int maxNumThreads;
    set<IdPair<GeometryHandle>> ignoredPairs;
    set<IdPair<int>> ignoredGroupPairs;
    MeshExtractor* meshExtractor;
    bool isReady;
    bool isDynamicGeometryPairChangeEnabled;
    CollisionPair collisionPair;
        
    Impl();
    Impl(const AISTCollisionDetector::Impl& org);
    ~Impl();
    void initialize();
    stdx::optional<GeometryHandle> addGeometry(SgNode* geometry);
    void addMesh(ColdetModelEx* model);
    void makeReady();
    bool checkIfGroupPairEnabled(int groupId1, int groupId2);
    bool checkIfModelPairEnabled(ColdetModelPairEx* modelPair);
    void detectCollisions(GeometryHandle geometry, const std::function<void(const CollisionPair&)>& callback);
    void detectCollisions(const std::function<void(const CollisionPair&)>& callback);
    void detectCollisionsInParallel(const std::function<void(const CollisionPair&)>& callback);

    // for multithread version
    int numThreads;
    unique_ptr<ThreadPool> threadPool;
    vector<int> shuffledPairIndices;
    vector<vector<CollisionPair>> collisionPairArrays;
    mt19937 randomEngine;
    
    void extractCollisionsOfAssignedPairs(
        int pairIndexBegin, int pairIndexEnd, vector<CollisionPair>& collisionPairs);    
    void dispatchCollisionsInCollisionPairArrays(std::function<void(const CollisionPair&)> callback);    
};

}


AISTCollisionDetector::AISTCollisionDetector()
{
    impl = new Impl;
}


AISTCollisionDetector::Impl::Impl()
{
    isDynamicGeometryPairChangeEnabled = false;
    maxNumThreads = 0;

    initialize();
}


AISTCollisionDetector::AISTCollisionDetector(const AISTCollisionDetector& org)
{
    impl = new Impl(*org.impl);
}


AISTCollisionDetector::Impl::Impl(const AISTCollisionDetector::Impl& org)
{
    isDynamicGeometryPairChangeEnabled = org.isDynamicGeometryPairChangeEnabled;
    maxNumThreads = org.maxNumThreads;

    initialize();
}


void AISTCollisionDetector::Impl::initialize()
{
    isReady = false;
    numThreads = 0;
    meshExtractor = new MeshExtractor;

    if(ENABLE_SHUFFLE){
        random_device seed;
        randomEngine.seed(seed());
    }
}    


AISTCollisionDetector::Impl::~Impl()
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


CollisionDetector* AISTCollisionDetector::clone() const
{
    return new AISTCollisionDetector;
}


void AISTCollisionDetector::setNumThreads(int n)
{
    impl->maxNumThreads = n;
}

        
void AISTCollisionDetector::clearGeometries()
{
    impl->models.clear();
    impl->modelPairs.clear();
    impl->ignoredPairs.clear();
    impl->ignoredGroupPairs.clear();
    impl->isReady = false;
}


int AISTCollisionDetector::numGeometries() const
{
    return impl->models.size();
}


stdx::optional<GeometryHandle> AISTCollisionDetector::addGeometry(SgNode* geometry)
{
    return impl->addGeometry(geometry);
}


stdx::optional<GeometryHandle> AISTCollisionDetector::Impl::addGeometry(SgNode* geometry)
{
    if(geometry){
        ColdetModelExPtr model = new ColdetModelEx;
        if(meshExtractor->extract(geometry, [&]() { addMesh(model); })){
            model->setName(geometry->name());
            model->build();
            if(model->isValid()){
                models.push_back(model);
                isReady = false;
                return getHandle(model);
            }
        }
    }
    return stdx::nullopt;
}


void AISTCollisionDetector::Impl::addMesh(ColdetModelEx* model)
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


void AISTCollisionDetector::setCustomObject(GeometryHandle geometry, Referenced* object)
{
    getColdetModel(geometry)->object = object;
}


void AISTCollisionDetector::setGeometryStatic(GeometryHandle geometry, bool isStatic)
{
    getColdetModel(geometry)->isStatic = isStatic;
    impl->isReady = false;
}


void AISTCollisionDetector::setGroup(GeometryHandle geometry, int groupId)
{
    getColdetModel(geometry)->groupId = groupId;
}


void AISTCollisionDetector::setGroupPairEnabled(int groupId1, int groupId2, bool on)
{
    if(on){
        impl->ignoredGroupPairs.erase(IdPair<int>(groupId1, groupId2));
    } else {
        impl->ignoredGroupPairs.insert(IdPair<int>(groupId1, groupId2));
    }
}


void AISTCollisionDetector::ignoreGeometryPair(GeometryHandle geometry1, GeometryHandle geometry2, bool ignore)
{
    IdPair<GeometryHandle> idPair(geometry1, geometry2);
    if(ignore){
        auto result = impl->ignoredPairs.insert(idPair);
        if(result.second){
            impl->isReady = false;
        }
    } else {
        auto p = impl->ignoredPairs.find(idPair);
        if(p != impl->ignoredPairs.end()){
            impl->ignoredPairs.erase(p);
            impl->isReady = false;
        }
    }
}


void AISTCollisionDetector::setGeometryEnabled(GeometryHandle geometry, bool isEnabled)
{
    getColdetModel(geometry)->isEnabled = isEnabled;
}


void AISTCollisionDetector::setDynamicGeometryPairChangeEnabled(bool on)
{
    if(on != impl->isDynamicGeometryPairChangeEnabled){
        impl->isDynamicGeometryPairChangeEnabled = on;
        impl->isReady = false;
    }
}


bool AISTCollisionDetector::isDynamicGeometryPairChangeEnabled() const
{
    return impl->isDynamicGeometryPairChangeEnabled;
}


bool AISTCollisionDetector::removeGeometry(GeometryHandle geometry)
{
    bool removed = false;
    ColdetModelExPtr model = getColdetModel(geometry);
    
    auto mi = impl->models.rbegin();
    while(mi != impl->models.rend()){
        if(*mi == model){
            impl->models.erase(--(mi.base()));
            removed = true;
            break;
        }
        ++mi;
    }

    if(removed){
        auto pi = impl->modelPairs.begin();
        while(pi != impl->modelPairs.end()){
            auto& modelPair = *pi;
            if(modelPair->model(0) == model || modelPair->model(1) == model){
                pi = impl->modelPairs.erase(pi);
            } else {
                ++pi;
            }
        }
        auto ii = impl->ignoredPairs.begin();
        while(ii != impl->ignoredPairs.end()){
            auto& idPair = *ii;
            if(idPair.hasId(geometry)){
                ii = impl->ignoredPairs.erase(ii);
            } else {
                ++ii;
            }
        }
    }

    return removed;
}


bool AISTCollisionDetector::isGeometryRemovalSupported() const
{
    return true;
}


bool AISTCollisionDetector::makeReady()
{
    impl->makeReady();
    return true;
}


void AISTCollisionDetector::Impl::makeReady()
{
    modelPairs.clear();
    const int n = models.size();
    for(int i=0; i < n; ++i){
        ColdetModelEx* model0 = models[i];
        for(int j = i + 1; j < n; ++j){
            ColdetModelEx* model1 = models[j];
            if(!model0->isStatic || !model1->isStatic){
                bool doRegisterPair = isDynamicGeometryPairChangeEnabled;
                if(!doRegisterPair){
                    if(checkIfGroupPairEnabled(model0->groupId, model1->groupId)){
                        IdPair<GeometryHandle> handlePair(getHandle(model0), getHandle(model1));
                        if(ignoredPairs.find(handlePair) == ignoredPairs.end()){
                            doRegisterPair = true;
                        }
                    }
                }
                if(doRegisterPair){
                    modelPairs.push_back(new ColdetModelPairEx(model0, model1));
                }
            }
        }
    }

    const int numPairs = modelPairs.size();

    if(maxNumThreads <= 0){
        numThreads = 0;
        threadPool.reset();
        collisionPairArrays.clear();
    } else {
        numThreads = (maxNumThreads > numPairs) ? numPairs : maxNumThreads;
        threadPool.reset(new ThreadPool(numThreads));
        if(ENABLE_SHUFFLE){
            shuffledPairIndices.resize(modelPairs.size());
            for(size_t i=0; i < shuffledPairIndices.size(); ++i){
                shuffledPairIndices[i] = i;
            }
        }
        collisionPairArrays.resize(numThreads);
    }

    isReady = true;
}


bool AISTCollisionDetector::Impl::checkIfGroupPairEnabled(int groupId1, int groupId2)
{
    return (ignoredGroupPairs.find(IdPair<>(groupId1, groupId2)) == ignoredGroupPairs.end());
}


/**
   This is only used when the dynamic geometry pair change is enabled.
*/
bool AISTCollisionDetector::Impl::checkIfModelPairEnabled(ColdetModelPairEx* modelPair)
{
    auto model0 = modelPair->model(0);
    auto model1 = modelPair->model(1);
    if(checkIfGroupPairEnabled(model0->groupId, model1->groupId)){
        IdPair<GeometryHandle> handlePair(getHandle(model0), getHandle(model1));
        if(ignoredPairs.find(handlePair) == ignoredPairs.end()){
            return true;
        }
    }
    return false;
}


void AISTCollisionDetector::updatePosition(GeometryHandle geometry, const Isometry3& position)
{
    auto model = getColdetModel(geometry);
    do {
        if(model->localPosition){
            Isometry3 T = position * (*model->localPosition);
            model->setPosition(T);
        } else {
            model->setPosition(position);
        }
        model = model->sibling;
    } while(model);
}


void AISTCollisionDetector::updatePositions
(std::function<void(Referenced* object, Isometry3*& out_Position)> positionQuery)
{
    for(ColdetModelEx* model : impl->models){ // Do not use auto&
        do {
            Isometry3* T;
            positionQuery(model->object, T);
            if(model->localPosition){
                Isometry3 T2 = (*T) * (*model->localPosition);
                model->setPosition(T2);
            } else {
                model->setPosition(*T);
            }
            model = model->sibling; // Elements in models are overridden here if auto& is used
        } while(model);
    }
}


void AISTCollisionDetector::detectCollisions(GeometryHandle geometry, std::function<void(const CollisionPair&)> callback)
{
    if(!impl->isReady){
        impl->makeReady();
    }
    impl->detectCollisions(geometry, callback);
}


void AISTCollisionDetector::Impl::detectCollisions
(GeometryHandle geometry, const std::function<void(const CollisionPair&)>& callback)
{
    auto& collisions = collisionPair.collisions();
    
    for(ColdetModelPairEx* modelPair : modelPairs){ // Do not use auto&
        collisions.clear();
        do {
            auto model0 = modelPair->model(0);
            auto model1 = modelPair->model(1);
            if(getHandle(modelPair->model(0)) == geometry || getHandle(modelPair->model(1)) == geometry){
                if(model0->isEnabled && model1->isEnabled){
                    if(!isDynamicGeometryPairChangeEnabled || checkIfModelPairEnabled(modelPair)){
                        if(!modelPair->detectCollisions().empty()){
                            copyCollisionPairCollisions(modelPair, collisionPair);
                        }
                    }
                }
            }
            modelPair = modelPair->sibling;  // Elements in models are overridden here if auto& is used
        } while(modelPair);

        if(!collisions.empty()){
            callback(collisionPair);
        }
    }
}


void AISTCollisionDetector::detectCollisions(std::function<void(const CollisionPair&)> callback)
{
    if(!impl->isReady){
        impl->makeReady();
    }
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
void AISTCollisionDetector::Impl::detectCollisions(const std::function<void(const CollisionPair&)>& callback)
{
    auto& collisions = collisionPair.collisions();
    
    for(ColdetModelPairEx* modelPair : modelPairs){ // Do not use auto&
        collisions.clear();
        do {
            if(modelPair->model(0)->isEnabled && modelPair->model(1)->isEnabled){
                if(!isDynamicGeometryPairChangeEnabled || checkIfModelPairEnabled(modelPair)){
                    if(!modelPair->detectCollisions().empty()){
                        copyCollisionPairCollisions(modelPair, collisionPair);
                    }
                }
            }
            modelPair = modelPair->sibling;  // Elements in models are overridden here if auto& is used
        } while(modelPair);

        if(!collisions.empty()){
            callback(collisionPair);
        }
    }
}


void AISTCollisionDetector::Impl::detectCollisionsInParallel(const std::function<void(const CollisionPair&)>& callback)
{
    if(ENABLE_SHUFFLE){
        std::shuffle(shuffledPairIndices.begin(), shuffledPairIndices.end(), randomEngine);
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
        threadPool->post([this, i, index, size](){
            extractCollisionsOfAssignedPairs(index, index + size, collisionPairArrays[i]);
        });
        index += size;
    }
    threadPool->waitLoop();
    //threadPool->wait();

    dispatchCollisionsInCollisionPairArrays(callback);
}


void AISTCollisionDetector::Impl::extractCollisionsOfAssignedPairs
(int pairIndexBegin, int pairIndexEnd, vector<CollisionPair>& collisionPairs)
{
    collisionPairs.clear();

    for(int i=pairIndexBegin; i < pairIndexEnd; ++i){
        ColdetModelPairEx* modelPair;
        if(ENABLE_SHUFFLE){
            modelPair = modelPairs[shuffledPairIndices[i]];
        } else {
            modelPair = modelPairs[i];
        }

        collisionPairs.push_back(CollisionPair());
        CollisionPair& collisionPair = collisionPairs.back();
        do {
            if(modelPair->model(0)->isEnabled && modelPair->model(1)->isEnabled){
                if(!isDynamicGeometryPairChangeEnabled || checkIfModelPairEnabled(modelPair)){
                    if(!modelPair->detectCollisions().empty()){
                        copyCollisionPairCollisions(modelPair, collisionPair, true);
                    }
                }
            }
            modelPair = modelPair->sibling;
        } while(modelPair);

        if(collisionPair.empty()){
            collisionPairs.pop_back();
        }
    }
}

void AISTCollisionDetector::Impl::dispatchCollisionsInCollisionPairArrays
(std::function<void(const CollisionPair&)> callback)
{
    for(int i=0; i < numThreads; ++i){
        const vector<CollisionPair>& collisionPairs = collisionPairArrays[i];
        for(size_t j=0; j < collisionPairs.size(); ++j){
            callback(collisionPairs[j]);
        }
    }
}


double AISTCollisionDetector::detectDistance
(GeometryHandle geometry1, GeometryHandle geometry2, Vector3& out_point1, Vector3& out_point2)
{
    return ColdetModelPair::computeDistance(
        getColdetModel(geometry1), getColdetModel(geometry2), out_point1.data(), out_point2.data());
}


stdx::optional<double> AISTCollisionDetector::detectDistanceToRayIntersection
(GeometryHandle geometry, const Vector3& point, const Vector3& direction)
{
    return getColdetModel(geometry)->computeDistanceWithRay(point.data(), direction.data());
}
