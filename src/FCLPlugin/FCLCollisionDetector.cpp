#include "FCLCollisionDetector.h"
#include <cnoid/Plugin>
#include <cnoid/IdPair>
#include <cnoid/MeshExtractor>
#include <cnoid/SceneDrawables>
#include <cnoid/stdx/optional>
#include <memory>

#include <fcl/config.h>
#if FCL_MAJOR_VERSION==0 && FCL_MINOR_VERSION < 6
#define CNOID_FCL_05
#include <fcl/collision.h>
#include <fcl/BVH/BVH_model.h>
#include <fcl/BV/BV.h>
#include <fcl/narrowphase/gjk.h>
#else
#include <fcl/fcl.h>
#endif

using namespace std;
using namespace cnoid;

namespace {

const bool USE_PRIMITIVE = true;

#ifdef CNOID_FCL_05
typedef fcl::BVHModel<fcl::OBBRSS> MeshModelf;
typedef fcl::CollisionObject CollisionObjectf;
typedef fcl::Box Boxf;
typedef fcl::Sphere Spheref;
typedef fcl::Cylinder Cylinderf;
typedef fcl::Cone Conef;
typedef fcl::CollisionRequest CollisionRequestf;
typedef fcl::CollisionResult CollisionResultf;
typedef fcl::Contact Contactf;
#else
typedef fcl::BVHModel<fcl::OBBRSSf> MeshModelf;
typedef fcl::CollisionObject<float> CollisionObjectf;
typedef fcl::Box<float> Boxf;
typedef fcl::Sphere<float> Spheref;
typedef fcl::Cylinder<float> Cylinderf;
typedef fcl::Cone<float> Conef;
typedef fcl::CollisionRequest<float> CollisionRequestf;
typedef fcl::CollisionResult<float> CollisionResultf;
typedef fcl::Contact<float> Contactf;
#endif

class ColdetModel : public Referenced
{
public:
    shared_ptr<MeshModelf> meshModel;
    shared_ptr<CollisionObjectf> meshObject;
    vector<shared_ptr<CollisionObjectf>> primitiveObjects;
    vector<Isometry3, Eigen::aligned_allocator<Isometry3>> primitiveLocalPositions;
    ReferencedPtr object;
#ifdef CNOID_FCL_05
    vector<fcl::Vec3f> points;
#else
    vector<Vector3f> points;
#endif
    vector<fcl::Triangle> tri_indices;
    bool isStatic;

    ColdetModel(){
        isStatic = false;
    }
};

typedef ref_ptr<ColdetModel> ColdetModelPtr;

typedef CollisionDetector::GeometryHandle GeometryHandle;

ColdetModel* getColdetModel(GeometryHandle handle)
{
    return reinterpret_cast<ColdetModel*>(handle);
}

GeometryHandle getHandle(ColdetModel* model)
{
    return reinterpret_cast<GeometryHandle>(model);
}

#ifdef CNOID_FCL_05
fcl::Transform3f convertToFclTransform(const Isometry3& T)
{
    fcl::Transform3f T_fcl;
    
    auto M = T.linear();
    T_fcl.setRotation(
        fcl::Matrix3f(
            M(0,0), M(0, 1), M(0, 2),
            M(1,0), M(1, 1), M(1, 2),
            M(2,0), M(2, 1), M(2, 2)));

    auto p = T.translation();
    T_fcl.setTranslation(fcl::Vec3f(p.x(), p.y(), p.z()));

    return T_fcl;
}
#endif

}

namespace cnoid {

class FCLCollisionDetector::Impl
{
public:
    vector<ColdetModelPtr> models;
    vector<pair<ColdetModelPtr, ColdetModelPtr>> modelPairs;
    set<IdPair<GeometryHandle>> ignoredPairs;
    MeshExtractor meshExtractor;
    bool isReady;

    Impl();
    stdx::optional<GeometryHandle> addGeometry(SgNode* geometry);
    void addMesh(ColdetModel* geometry, SgMesh* mesh);
    bool addPrimitive(ColdetModel* model, SgMesh* mesh);
    void makeReady();
    void updatePosition(ColdetModel* model, const Isometry3& position);
    void detectCollisions(std::function<void(const CollisionPair&)> callback);
    void detectObjectCollisions(
        CollisionObjectf* object1, CollisionObjectf* object2, CollisionPair& collisionPair);
};

}


FCLCollisionDetector::FCLCollisionDetector()
{
    impl = new Impl;
}


FCLCollisionDetector::Impl::Impl()
{
    isReady = false;
}    


FCLCollisionDetector::~FCLCollisionDetector()
{
    delete impl;
}


const char* FCLCollisionDetector::name() const
{
    return "FCLCollisionDetector";
}


CollisionDetector* FCLCollisionDetector::clone() const
{
    return new FCLCollisionDetector;
}

        
void FCLCollisionDetector::clearGeometries()
{
    impl->models.clear();
    impl->modelPairs.clear();
    impl->ignoredPairs.clear();
    impl->isReady = false;
}


int FCLCollisionDetector::numGeometries() const
{
    return impl->models.size();
}


stdx::optional<GeometryHandle> FCLCollisionDetector::addGeometry(SgNode* geometry)
{
    return impl->addGeometry(geometry);
}


stdx::optional<GeometryHandle> FCLCollisionDetector::Impl::addGeometry(SgNode* geometry)
{
    const int index = models.size();
    ColdetModel* model = nullptr;

    if(geometry){
        model = new ColdetModel;
        bool extracted =
            meshExtractor.extract(
                geometry,
                [this, model](SgMesh* mesh){ addMesh(model, mesh); });

        if(extracted){
            if(!model->points.empty()){
                model->meshModel = make_shared<MeshModelf>();
                model->meshModel->beginModel();
                model->meshModel->addSubModel(model->points, model->tri_indices);
                /*
                for(size_t i = 0; i < model->tri_indices.size(); ++i){
                    model->meshModel->addTriangle(
                        model->points[model->tri_indices[i][0]],
                        model->points[model->tri_indices[i][1]],
                        model->points[model->tri_indices[i][2]]);
                }
                */
                model->meshModel->endModel();
                model->meshObject = make_shared<CollisionObjectf>(model->meshModel);
            }
        }
    }

    models.push_back(model);
    isReady = false;
    
    return getHandle(model);
}


void FCLCollisionDetector::Impl::addMesh(ColdetModel* model, SgMesh* mesh)
{
    const Affine3& T = meshExtractor.currentTransform();

    bool meshAdded = false;

    if(USE_PRIMITIVE){
        if(mesh->primitiveType() != SgMesh::MESH){
            meshAdded = addPrimitive(model, mesh);
        }
    }

    if(!meshAdded){
        const int vertexIndexTop = model->points.size();

        const SgVertexArray& vertices = *mesh->vertices();
        const int numVertices = vertices.size();
        auto& points = model->points;
        points.resize(numVertices);
        for(int i=0; i < numVertices; ++i){
            const Vector3 v = T * vertices[i].cast<Isometry3::Scalar>();;
#ifdef CNOID_FCL_05
            points[i].setValue(v.x(), v.y(), v.z());
#else
            points[i] << v.x(), v.y(), v.z();
#endif
        }

        const int numTriangles = mesh->numTriangles();
        for(int i=0; i < numTriangles; ++i){
            SgMesh::TriangleRef src = mesh->triangle(i);
            int i0 = vertexIndexTop + src[0];
            int i1 = vertexIndexTop + src[1];
            int i2 = vertexIndexTop + src[2];
            model->tri_indices.emplace_back(i0, i1, i2);
        }
    }
}


bool FCLCollisionDetector::Impl::addPrimitive(ColdetModel* model, SgMesh* mesh)
{
    bool meshAdded = false;
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
        
        switch(mesh->primitiveType()){
        case SgMesh::BOX : {
            const Vector3& s = mesh->primitive<SgMesh::Box>().size;
            auto box = make_shared<Boxf>(s.x() * scale.x(), s.y() * scale.y(), s.z() * scale.z());
            model->primitiveObjects.push_back(make_shared<CollisionObjectf>(box));
            created = true;
            break;
        }
        case SgMesh::SPHERE : {
            double radius = mesh->primitive<SgMesh::Sphere>().radius;
            auto sphere = make_shared<Spheref>(radius * scale.x());
            model->primitiveObjects.push_back(make_shared<CollisionObjectf>(sphere));
            created = true;
            break;
        }
        case SgMesh::CYLINDER : {
            SgMesh::Cylinder cylinder = mesh->primitive<SgMesh::Cylinder>();
            auto cylinder_ = make_shared<Cylinderf>(cylinder.radius * scale.x(), cylinder.height * scale.y());
            model->primitiveObjects.push_back(make_shared<CollisionObjectf>(cylinder_));
            created = true;
            break;
        }
        case SgMesh::CONE : {
            SgMesh::Cone cone = mesh->primitive<SgMesh::Cone>();
            auto cone_ = make_shared<Conef>(cone.radius * scale.x(), cone.height * scale.y());
            model->primitiveObjects.push_back(make_shared<CollisionObjectf>(cone_));
            created = true;
            break;
        }
        default :
            break;
        }
        if(created){
            Isometry3 T;
            if(translation){
                T = meshExtractor.currentTransformWithoutScaling() * Translation3(*translation);
            } else {
                T = meshExtractor.currentTransformWithoutScaling();
            }
            model->primitiveLocalPositions.push_back(T);
            meshAdded = true;
        }
    }

    return meshAdded;
}


void FCLCollisionDetector::setCustomObject(GeometryHandle geometry, Referenced* object)
{
    getColdetModel(geometry)->object = object;
}


void FCLCollisionDetector::setGeometryStatic(GeometryHandle geometry, bool isStatic)
{
    getColdetModel(geometry)->isStatic = isStatic;
    impl->isReady = false;
}


void FCLCollisionDetector::ignoreGeometryPair(GeometryHandle geometry1, GeometryHandle geometry2, bool ignore)
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


bool FCLCollisionDetector::makeReady()
{
    impl->makeReady();
    return true;
}


void FCLCollisionDetector::Impl::makeReady()
{
    modelPairs.clear();
    const int n = models.size();
    for(int i=0; i < n; ++i){
        if(auto& model1 = models[i]){
            for(int j = i+1; j < n; ++j){
                if(auto& model2 = models[j]){
                    if(!model1->isStatic || !model2->isStatic){
                        IdPair<GeometryHandle> handlePair(getHandle(model1), getHandle(model2));
                        if(ignoredPairs.find(handlePair) == ignoredPairs.end()){
                            modelPairs.push_back(make_pair(model1, model2));
                        }
                    }
                }
            }
        }
    }

    isReady = true;
}


void FCLCollisionDetector::updatePosition(GeometryHandle geometry, const Isometry3& position)
{
    if(auto model = getColdetModel(geometry)){
        impl->updatePosition(model, position);
    }
}


void FCLCollisionDetector::Impl::updatePosition(ColdetModel* model, const Isometry3& T)
{
    if(model->meshObject){
#ifdef CNOID_FCL_05
        model->meshObject->setTransform(convertToFclTransform(T));
#else
        model->meshObject->setTransform(T.cast<float>());
#endif
    }
    const int n = model->primitiveObjects.size();
    for(int i=0; i < n; ++i){
        auto& primitive = model->primitiveObjects[i];
        if(primitive){
            auto& T_local = model->primitiveLocalPositions[i];
            Isometry3 Tp = T * T_local;
#ifdef CNOID_FCL_05
            primitive->setTransform(convertToFclTransform(Tp));
#else
            primitive->setTransform(Tp.cast<float>());
#endif
        }
    }
}


void FCLCollisionDetector::updatePositions
(std::function<void(Referenced* object, Isometry3*& out_position)> positionQuery)
{
    for(auto& model : impl->models){
        if(model && model->object){
            Isometry3* pPosition;
            positionQuery(model->object, pPosition);
            impl->updatePosition(model, *pPosition);
        }
    }
}


void FCLCollisionDetector::detectCollisions(std::function<void(const CollisionPair&)> callback)
{
    if(!impl->isReady){
        impl->makeReady();
    }
    impl->detectCollisions(callback);
}


void FCLCollisionDetector::Impl::detectCollisions(std::function<void(const CollisionPair&)> callback)
{
    CollisionPair collisionPair;
    auto& collisions = collisionPair.collisions();

    for(auto& modelPair : modelPairs){
        auto& model1 = modelPair.first;
        auto& model2 = modelPair.second;
        collisions.clear();

        if(model1->meshObject){
            if(model2->meshObject){
                detectObjectCollisions(model1->meshObject.get(), model2->meshObject.get(), collisionPair);
            }
            for(auto& primitive : model2->primitiveObjects){
                detectObjectCollisions(model1->meshObject.get(), primitive.get(), collisionPair);
            }
        }
        for(auto& primitive1 : model1->primitiveObjects){
            if(model2->meshObject){
                detectObjectCollisions(primitive1.get(), model2->meshObject.get(), collisionPair);
            }
            for(auto& primitive2 : model2->primitiveObjects){
                detectObjectCollisions(primitive1.get(), primitive2.get(), collisionPair);
            }
        }

        if(!collisions.empty()){
            collisionPair.geometry(0) = getHandle(model1);
            collisionPair.geometry(1) = getHandle(model2);
            collisionPair.object(0) = model1->object;
            collisionPair.object(1) = model2->object;
            callback(collisionPair);
        }
    }
}


void FCLCollisionDetector::Impl::detectObjectCollisions
(CollisionObjectf* object1, CollisionObjectf* object2, CollisionPair& collisionPair)
{
    vector<Collision>& collisions = collisionPair.collisions();

    CollisionRequestf request(std::numeric_limits<int>::max(), true);
    CollisionResultf result;
    int numContacts = collide(object1, object2, request, result);
    std::vector<Contactf> contacts;
    result.getContacts(contacts);

    for(auto& contact : contacts){
        if(contact.penetration_depth < 0.0){
            contact.penetration_depth *= -1.0;
        }
        if(contact.penetration_depth < 1.0e-6){
            continue;
        }

        collisions.push_back(Collision());
        auto& collision = collisions.back();
        auto& p = contact.pos;
        collision.point << p[0], p[1], p[2];
        auto& n = contact.normal;
        collision.normal << n[0], n[1], n[2];
        collision.depth = contact.penetration_depth;
    }
}


class FCLPlugin : public Plugin
{
public:
    FCLPlugin() : Plugin("FCL")
    { 
        require("Body");
    }
        
    virtual bool initialize()
    {
        return CollisionDetector::registerFactory(
            "FCLCollisionDetector",
            []() -> CollisionDetector* { return new FCLCollisionDetector; });
    }
        
    virtual bool finalize()
    {
        return true;
    }
};

CNOID_IMPLEMENT_PLUGIN_ENTRY(FCLPlugin);
