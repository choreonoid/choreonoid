/**
   \file
   \author Shizuko Hattori
*/

#include "FCLCollisionDetector.h"
#include <cnoid/Plugin>
#include <cnoid/IdPair>
#include <cnoid/MeshExtractor>
#include <cnoid/SceneDrawables>
#include <cnoid/stdx/optional>
#include <fcl/narrowphase/collision.h>
#include <fcl/geometry/bvh/BVH_model.h>
#include <memory>

using namespace std;
using namespace fcl;
using namespace cnoid;

namespace {
const bool USE_PRIMITIVE = true;

class FCLPlugin : public Plugin
{
public:
    FCLPlugin() : Plugin("FCL")
        { 
            require("Body");
        }
        
    virtual ~FCLPlugin()
        {

        }

    virtual bool initialize()
        {
            return true;
        }
        
    virtual bool finalize()
        {
            return true;
        }
};
CNOID_IMPLEMENT_PLUGIN_ENTRY(FCLPlugin);


CollisionDetectorPtr factory()
{
    return std::make_shared<FCLCollisionDetector>();
}

struct FactoryRegistration
{
    FactoryRegistration(){
        CollisionDetector::registerFactory("FCLCollisionDetector", factory);
    }
} factoryRegistration;

typedef std::shared_ptr<CollisionObjectd> CollisionObjectPtr;
typedef BVHModel<OBBRSSd> MeshModel;
typedef std::shared_ptr<MeshModel> MeshModelPtr;

class CollisionObjectEx
{
public :
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    CollisionObjectEx();
    ~CollisionObjectEx();
    CollisionObjectPtr meshObject;
    MeshModelPtr meshModel;
    vector<CollisionObjectPtr> primitiveObjects;
    vector<Transform3d, Eigen::aligned_allocator<Transform3d> > primitiveLocalT;
    vector<Vector3d> points;
    vector<Triangle> tri_indices;
    bool isStatic;
};
typedef std::shared_ptr<CollisionObjectEx> CollisionObjectExPtr;

CollisionObjectEx::CollisionObjectEx()
{
    primitiveObjects.clear();
    primitiveLocalT.clear();
    isStatic = false;
}

CollisionObjectEx::~CollisionObjectEx()
{
        
}
}


namespace cnoid {

class FCLCollisionDetectorImpl
{
public:
    FCLCollisionDetectorImpl();
    ~FCLCollisionDetectorImpl();

    vector<CollisionObjectExPtr> models;
    typedef set< IdPair<> > IdPairSet;
    IdPairSet modelPairs;
    IdPairSet nonInterfarencePairs;

    MeshExtractor* meshExtractor;

    int addGeometry(SgNode* geometry);
    void addMesh(CollisionObjectEx* model);
    bool makeReady();
    void updatePosition(int geometryId, const Position& position);
    void detectCollisions(std::function<void(const CollisionPair&)> callback);
    void detectObjectCollisions(CollisionObjectd* object1, CollisionObjectd* object2, CollisionPair& collisionPair);

private :


};
}


FCLCollisionDetector::FCLCollisionDetector()
{
    impl = new FCLCollisionDetectorImpl();
}


FCLCollisionDetectorImpl::FCLCollisionDetectorImpl()
{
    meshExtractor = new MeshExtractor();
}


FCLCollisionDetector::~FCLCollisionDetector()
{
    delete impl;
}


FCLCollisionDetectorImpl::~FCLCollisionDetectorImpl()
{
    delete meshExtractor;
}


const char* FCLCollisionDetector::name() const
{
    return "FCLCollisionDetector";
}


CollisionDetectorPtr FCLCollisionDetector::clone() const
{
    return std::make_shared<FCLCollisionDetector>();
}

        
void FCLCollisionDetector::clearGeometries()
{
    impl->models.clear();
    impl->nonInterfarencePairs.clear();
    impl->modelPairs.clear();
}


int FCLCollisionDetector::numGeometries() const
{
    return impl->models.size();
}


int FCLCollisionDetector::addGeometry(SgNodePtr geometry)
{
    return impl->addGeometry(geometry.get());
}


int FCLCollisionDetectorImpl::addGeometry(SgNode* geometry)
{
    const int index = models.size();
    bool isValid = false;

    if(geometry){
        CollisionObjectExPtr model =  std::make_shared<CollisionObjectEx>();
        if(meshExtractor->extract(geometry, std::bind(&FCLCollisionDetectorImpl::addMesh, this, model.get()))){
            if(!model->points.empty()){
                model->meshModel = std::make_shared<MeshModel>();
                model->meshModel->beginModel();
                model->meshModel->addSubModel(model->points, model->tri_indices);
                //for (size_t i = 0; i < model->tri_indices.size(); ++i){
                //    model->meshModel->addTriangle(model->points[model->tri_indices[i][0]], model->points[model->tri_indices[i][1]], model->points[model->tri_indices[i][2]]);
                //}
                model->meshModel->endModel();
                model->meshObject = std::make_shared<CollisionObjectd>(model->meshModel);
            }
            models.push_back(model);
            isValid = true;
        }
    }

    if(!isValid){
        models.push_back(CollisionObjectExPtr());
    }
    
    return index;

}


void FCLCollisionDetectorImpl::addMesh(CollisionObjectEx* model)
{
    SgMesh* mesh = meshExtractor->currentMesh();
    const Affine3& T = meshExtractor->currentTransform();

    bool meshAdded = false;
    
    if(USE_PRIMITIVE){
        if(mesh->primitiveType() != SgMesh::MESH){
            bool doAddPrimitive = false;
            Vector3 scale;
            stdx::optional<Vector3> translation;
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

                switch(mesh->primitiveType()){
                case SgMesh::BOX : {
                    const Vector3& s = mesh->primitive<SgMesh::Box>().size;
                    std::shared_ptr< CollisionGeometry<double> > box = std::make_shared<fcl::Boxd>(s.x() * scale.x(), s.y() * scale.y(), s.z() * scale.z());
                    CollisionObjectPtr obj = std::make_shared<CollisionObjectd>(box);
                    model->primitiveObjects.push_back(obj);
                    created = true;
                    break; }
                case SgMesh::SPHERE : {
                    double radius = mesh->primitive<SgMesh::Sphere>().radius;
                    std::shared_ptr< CollisionGeometry<double> > sphere = std::make_shared<fcl::Sphered>(radius * scale.x());
                    CollisionObjectPtr obj = std::make_shared<CollisionObjectd>(sphere);
                    model->primitiveObjects.push_back(obj);
                    created = true;
                    break; }
                case SgMesh::CYLINDER : {
                    SgMesh::Cylinder cylinder = mesh->primitive<SgMesh::Cylinder>();
                    std::shared_ptr< CollisionGeometry<double> > cylinder_ = std::make_shared<fcl::Cylinderd>(cylinder.radius * scale.x(), cylinder.height * scale.y());
                    CollisionObjectPtr obj = std::make_shared<CollisionObjectd>(cylinder_);
                    model->primitiveObjects.push_back(obj);
                    created = true;
                    break; }
                case SgMesh::CONE : {
                    SgMesh::Cone cone = mesh->primitive<SgMesh::Cone>();
                    std::shared_ptr< CollisionGeometry<double> > cone_ = std::make_shared<fcl::Coned>(cone.radius * scale.x(), cone.height * scale.y());
                    CollisionObjectPtr obj = std::make_shared<CollisionObjectd>(cone_);
                    model->primitiveObjects.push_back(obj);
                    created = true;
                    break; }
                default :
                    break;
                }
                if(created){
                    Position t;
                    if(translation){
                        t = meshExtractor->currentTransformWithoutScaling() * Translation3(*translation);
                    } else {
                        t = meshExtractor->currentTransformWithoutScaling();
                    }
                    model->primitiveLocalT.push_back(t);
                    meshAdded = true;
                }
            }
        }
    }

    if(!meshAdded){
        const int vertexIndexTop = model->points.size();

        const SgVertexArray& vertices_ = *mesh->vertices();
        const int numVertices = vertices_.size();
        for(int i=0; i < numVertices; ++i){
            const Vector3 v = T * vertices_[i].cast<Position::Scalar>();
            model->points.push_back(v);
        }

        const int numTriangles = mesh->numTriangles();
        for(int i=0; i < numTriangles; ++i){
            SgMesh::TriangleRef src = mesh->triangle(i);
            int i0 = vertexIndexTop + src[0];
            int i1 = vertexIndexTop + src[1];
            int i2 = vertexIndexTop + src[2];
            model->tri_indices.push_back(Triangle(i0, i1, i2));
        }
    }
}



void FCLCollisionDetector::setGeometryStatic(int geometryId, bool isStatic)
{
    CollisionObjectExPtr& model = impl->models[geometryId];
    if(model){
        model->isStatic = isStatic;
    }
}


bool FCLCollisionDetector::enableGeometryCache(bool on)
{
    return false;
}


void FCLCollisionDetector::clearGeometryCache(SgNodePtr geometry)
{
    
}


void FCLCollisionDetector::clearAllGeometryCaches()
{

}


void FCLCollisionDetector::setNonInterfarenceGeometyrPair(int geometryId1, int geometryId2)
{
    impl->nonInterfarencePairs.insert(IdPair<>(geometryId1, geometryId2));
}


bool FCLCollisionDetector::makeReady()
{
    return impl->makeReady();
}


bool FCLCollisionDetectorImpl::makeReady()
{
    modelPairs.clear();

    const int n = models.size();
    for(int i=0; i < n; ++i){
        CollisionObjectExPtr& model1 = models[i];
        if(model1){
            for(int j = i+1; j < n; ++j){
                CollisionObjectExPtr& model2 = models[j];
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


void FCLCollisionDetector::updatePosition(int geometryId, const Position& position)
{
    impl->updatePosition(geometryId, position);
}


void FCLCollisionDetectorImpl::updatePosition(int geometryId, const Position& position)
{
    CollisionObjectExPtr& model = models[geometryId];
    if(model){
        if(model->meshObject){
            model->meshObject->setTransform(position);
        }
        vector<Transform3d, Eigen::aligned_allocator<Transform3d> >::iterator itt = model->primitiveLocalT.begin();
        for(vector<CollisionObjectPtr>::iterator it = model->primitiveObjects.begin();
            it!=model->primitiveObjects.end(); it++, itt++)
            if(*it){
                fcl::Transform3d trans;
                trans = position * (*itt);
                (*it)->setTransform(trans);
            }
    }
}


void FCLCollisionDetector::detectCollisions(std::function<void(const CollisionPair&)> callback)
{
    impl->detectCollisions(callback);
}


void FCLCollisionDetectorImpl::detectCollisions(std::function<void(const CollisionPair&)> callback)
{
    CollisionPair collisionPair;
    vector<Collision>& collisions = collisionPair.collisions;

    for(IdPairSet::iterator it = modelPairs.begin(); it!=modelPairs.end(); it++){
        CollisionObjectExPtr& model1 = models[(*it)(0)];
        CollisionObjectExPtr& model2 = models[(*it)(1)];
        collisions.clear();

        if(model1->meshObject){
            if(model2->meshObject)
                detectObjectCollisions(model1->meshObject.get(), model2->meshObject.get(), collisionPair);
            for(vector<CollisionObjectPtr>::iterator itt = model2->primitiveObjects.begin();
                itt!=model2->primitiveObjects.end(); itt++)
                detectObjectCollisions(model1->meshObject.get(), (*itt).get(), collisionPair);
        }
        for(vector<CollisionObjectPtr>::iterator iter = model1->primitiveObjects.begin();
            iter!=model1->primitiveObjects.end(); iter++){
            if(model2->meshObject)
                detectObjectCollisions((*iter).get(), model2->meshObject.get(), collisionPair);
            for(vector<CollisionObjectPtr>::iterator itt = model2->primitiveObjects.begin();
                itt!=model2->primitiveObjects.end(); itt++)
                detectObjectCollisions((*iter).get(), (*itt).get(), collisionPair);
        }

        if(!collisions.empty()){
            collisionPair.geometryId[0] = (*it)(0);
            collisionPair.geometryId[1] = (*it)(1);
            callback(collisionPair);
        }
    }

}


void FCLCollisionDetectorImpl::detectObjectCollisions(CollisionObjectd* object1, CollisionObjectd* object2, CollisionPair& collisionPair)
{
    vector<Collision>& collisions = collisionPair.collisions;
    CollisionRequestd request(std::numeric_limits<int>::max(), true);
    CollisionResultd result;
    std::vector<Contactd> contacts;
    int numContacts = collide(object1, object2, request, result);

    result.getContacts(contacts);

    for (int j=0;j<result.numContacts();j++)
    {
        if(contacts[j].penetration_depth<0)
            contacts[j].penetration_depth *= -1;
        if(contacts[j].penetration_depth < 1e-6)
            continue;

        fcl::Vector3d& pos = contacts[j].pos;
        fcl::Vector3d& normal = contacts[j].normal;
        collisions.push_back(Collision());
        Collision& collision = collisions.back();
        collision.point = Vector3(pos[0], pos[1], pos[2]);
        collision.normal = Vector3(normal[0], normal[1], normal[2]);
        collision.depth = contacts[j].penetration_depth;
    }


}

