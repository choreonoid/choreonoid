#include "FCLCollisionDetector.h"
#include <cnoid/Plugin>
#include <cnoid/IdPair>
#include <cnoid/MeshExtractor>
#include <cnoid/SceneDrawables>
#include <cnoid/stdx/optional>
#include <fcl/collision.h>
#include <fcl/BVH/BVH_model.h>
#include <fcl/BV/BV.h>
#include <fcl/narrowphase/gjk.h>
#include <memory>

using namespace std;
using namespace cnoid;

namespace {

const bool USE_PRIMITIVE = true;

typedef shared_ptr<fcl::CollisionObject> CollisionObjectPtr;
typedef fcl::BVHModel<fcl::OBBRSS> MeshModel;
typedef shared_ptr<MeshModel> MeshModelPtr;

class CollisionModel : public Referenced
{
public :
    CollisionObjectPtr meshObject;
    MeshModelPtr meshModel;
    vector<CollisionObjectPtr> primitiveObjects;
    vector<fcl::Transform3f> primitiveLocalPositions;
    ReferencedPtr object;
    vector<fcl::Vec3f> points;
    vector<fcl::Triangle> tri_indices;
    bool isStatic;

    CollisionModel(){
        primitiveObjects.clear();
        primitiveLocalPositions.clear();
        isStatic = false;
    }
};

typedef ref_ptr<CollisionModel> CollisionModelPtr;

typedef CollisionDetector::GeometryHandle GeometryHandle;

CollisionModel* getCollisionModel(GeometryHandle handle)
{
    return reinterpret_cast<CollisionModel*>(handle);
}

GeometryHandle getHandle(CollisionModel* model)
{
    return reinterpret_cast<GeometryHandle>(model);
}

inline fcl::Transform3f convertToFclTransform(const Isometry3& T)
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

}

namespace cnoid {

class FCLCollisionDetector::Impl
{
public:
    vector<CollisionModelPtr> models;
    vector<pair<CollisionModelPtr, CollisionModelPtr>> modelPairs;
    set<IdPair<GeometryHandle>> ignoredPairs;
    MeshExtractor meshExtractor;
    bool isReady;

    Impl();
    stdx::optional<GeometryHandle> addGeometry(SgNode* geometry);
    void addMesh(CollisionModel* geometry, SgMesh* mesh);
    bool addPrimitive(CollisionModel* model, SgMesh* mesh);
    void makeReady();
    void updatePosition(CollisionModel* model, const Isometry3& position);
    void detectCollisions(std::function<void(const CollisionPair&)> callback);
    void detectObjectCollisions(
        fcl::CollisionObject* object1, fcl::CollisionObject* object2, CollisionPair& collisionPair);
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
    CollisionModel* model = nullptr;

    if(geometry){
        model = new CollisionModel;
        bool extracted =
            meshExtractor.extract(
                geometry,
                [this, model](SgMesh* mesh){ addMesh(model, mesh); });

        if(extracted){
            if(!model->points.empty()){
                model->meshModel = make_shared<MeshModel>();
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
                model->meshObject = make_shared<fcl::CollisionObject>(model->meshModel);
            }
        }
    }

    models.push_back(model);
    isReady = false;
    
    return getHandle(model);
}


void FCLCollisionDetector::Impl::addMesh(CollisionModel* model, SgMesh* mesh)
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
            points[i].setValue(v.x(), v.y(), v.z());
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


bool FCLCollisionDetector::Impl::addPrimitive(CollisionModel* model, SgMesh* mesh)
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
            auto box = make_shared<fcl::Box>(s.x() * scale.x(), s.y() * scale.y(), s.z() * scale.z());
            model->primitiveObjects.push_back(make_shared<fcl::CollisionObject>(box));
            created = true;
            break;
        }
        case SgMesh::SPHERE : {
            double radius = mesh->primitive<SgMesh::Sphere>().radius;
            auto sphere = make_shared<fcl::Sphere>(radius * scale.x());
            model->primitiveObjects.push_back(make_shared<fcl::CollisionObject>(sphere));
            created = true;
            break;
        }
        case SgMesh::CYLINDER : {
            SgMesh::Cylinder cylinder = mesh->primitive<SgMesh::Cylinder>();
            auto cylinder_ = make_shared<fcl::Cylinder>(cylinder.radius * scale.x(), cylinder.height * scale.y());
            model->primitiveObjects.push_back(make_shared<fcl::CollisionObject>(cylinder_));
            created = true;
            break;
        }
        case SgMesh::CONE : {
            SgMesh::Cone cone = mesh->primitive<SgMesh::Cone>();
            auto cone_ = make_shared<fcl::Cone>(cone.radius * scale.x(), cone.height * scale.y());
            model->primitiveObjects.push_back(make_shared<fcl::CollisionObject>(cone_));
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
            model->primitiveLocalPositions.push_back(convertToFclTransform(T));
            meshAdded = true;
        }
    }

    return meshAdded;
}


void FCLCollisionDetector::setCustomObject(GeometryHandle geometry, Referenced* object)
{
    getCollisionModel(geometry)->object = object;
}


void FCLCollisionDetector::setGeometryStatic(GeometryHandle geometry, bool isStatic)
{
    getCollisionModel(geometry)->isStatic = isStatic;
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
    if(auto model = getCollisionModel(geometry)){
        impl->updatePosition(model, position);
    }
}


void FCLCollisionDetector::Impl::updatePosition(CollisionModel* model, const Isometry3& position)
{
    auto T = convertToFclTransform(position);
    if(model->meshObject){
        model->meshObject->setTransform(T);
    }
    auto pLocalPosition = model->primitiveLocalPositions.begin();
    for(auto& primitive : model->primitiveObjects){
        if(primitive){
            const auto& T_local = *pLocalPosition;
            primitive->setTransform(T * T_local);
        }
        ++pLocalPosition;
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
(fcl::CollisionObject* object1, fcl::CollisionObject* object2, CollisionPair& collisionPair)
{
    vector<Collision>& collisions = collisionPair.collisions();

    fcl::CollisionRequest request(std::numeric_limits<int>::max(), true);
    fcl::CollisionResult result;
    int numContacts = collide(object1, object2, request, result);
    std::vector<fcl::Contact> contacts;
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
