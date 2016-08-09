/**
   \file
   \author Shizuko Hattori
*/

#include "ODECollisionDetector.h"
#include <cnoid/MeshExtractor>
#include <cnoid/SceneDrawables>
#include <cnoid/EigenUtil>
#include <boost/make_shared.hpp>
#include <boost/bind.hpp>
#include <boost/optional.hpp>
#ifdef GAZEBO_ODE
#include <gazebo/ode/ode.h>
#else
#include <ode/ode.h>
#endif


using namespace std;
using namespace boost;
using namespace cnoid;

namespace {

CollisionDetectorPtr factory()
{
    return boost::make_shared<ODECollisionDetector>();
}

struct FactoryRegistration
{
    FactoryRegistration(){
        CollisionDetector::registerFactory("ODECollisionDetector", factory);
    }
} factoryRegistration;

typedef Eigen::Matrix<float, 3, 1> Vertex;

struct Triangle {
    int indices[3];
};

class GeometryEx
{
public :
    GeometryEx();
    ~GeometryEx();
    dSpaceID spaceID;
    vector<dGeomID> primitiveGeomID;
    dGeomID meshGeomID;
    dTriMeshDataID triMeshDataID;
    bool isStatic;
    vector<Vertex> vertices;
    vector<Triangle> triangles;
};
typedef boost::shared_ptr<GeometryEx> GeometryExPtr;

GeometryEx::GeometryEx()
{
    spaceID = 0;
    meshGeomID = 0;
    primitiveGeomID.clear();
    triMeshDataID = 0;
    isStatic = false;
    vertices.clear();
    triangles.clear();
}

GeometryEx::~GeometryEx()
{
    for(vector<dGeomID>::iterator it=primitiveGeomID.begin(); it!=primitiveGeomID.end(); it++)
        dGeomDestroy(*it);
    if(meshGeomID)
        dGeomDestroy(meshGeomID);
    if(triMeshDataID)
        dGeomTriMeshDataDestroy(triMeshDataID);
    if(spaceID)
        dSpaceDestroy(spaceID);
}

}


namespace cnoid {

class ODECollisionDetectorImpl
{
public:
    ODECollisionDetectorImpl();
    ~ODECollisionDetectorImpl();

    dSpaceID spaceID;
    vector<GeometryExPtr> models;
       
    typedef set < pair<dSpaceID, dSpaceID> > SpaceIDPairSet;
    SpaceIDPairSet nonInterfarencePairs;
    typedef map< dGeomID, int > GeomIDMap;
    GeomIDMap geomIDMap;
    typedef map< dGeomID, Position, std::less<dGeomID>, 
                 Eigen::aligned_allocator< pair<const dGeomID, Position> > > OffsetMap;
    OffsetMap offsetMap;

    boost::function<void(const CollisionPair&)> callback_;

    MeshExtractor* meshExtractor;

    int addGeometry(SgNode* geometry);
    void addMesh(GeometryEx* model);
    void setNonInterfarenceGeometyrPair(int geometryId1, int geometryId2);
    bool makeReady();
    void updatePosition(int geometryId, const Position& position);
    void detectCollisions(boost::function<void(const CollisionPair&)> callback);

private :


};
}


ODECollisionDetector::ODECollisionDetector()
{
    impl = new ODECollisionDetectorImpl();
}


ODECollisionDetectorImpl::ODECollisionDetectorImpl()
{
    spaceID = dHashSpaceCreate(0);
    dSpaceSetCleanup(spaceID, 0);

    meshExtractor = new MeshExtractor;
}


ODECollisionDetector::~ODECollisionDetector()
{
    delete impl;
}


ODECollisionDetectorImpl::~ODECollisionDetectorImpl()
{
    if(spaceID)
        dSpaceDestroy(spaceID);
    delete meshExtractor;
}


const char* ODECollisionDetector::name() const
{
    return "ODECollisionDetector";
}


CollisionDetectorPtr ODECollisionDetector::clone() const
{
    return boost::make_shared<ODECollisionDetector>();
}

        
void ODECollisionDetector::clearGeometries()
{
    impl->models.clear();
    impl->nonInterfarencePairs.clear();
    impl->geomIDMap.clear();
    impl->offsetMap.clear();
}


int ODECollisionDetector::numGeometries() const
{
    return impl->models.size();
}


int ODECollisionDetector::addGeometry(SgNodePtr geometry)
{
    return impl->addGeometry(geometry.get());
}


int ODECollisionDetectorImpl::addGeometry(SgNode* geometry)
{
    const int index = models.size();
    bool isValid = false;

    if(geometry){
        GeometryExPtr model =  boost::make_shared<GeometryEx>();
        model->spaceID = dHashSpaceCreate(spaceID);
        dSpaceSetCleanup(model->spaceID, 0);
        if(meshExtractor->extract(geometry, boost::bind(&ODECollisionDetectorImpl::addMesh, this, model.get()))){
            if(!model->vertices.empty()){
                model->triMeshDataID = dGeomTriMeshDataCreate();
                dGeomTriMeshDataBuildSingle(model->triMeshDataID,
                                            &model->vertices[0], sizeof(Vertex), model->vertices.size(),
                                            &model->triangles[0], model->triangles.size() * 3, sizeof(Triangle));
                model->meshGeomID = dCreateTriMesh(model->spaceID, model->triMeshDataID, 0, 0, 0);
                geomIDMap.insert(make_pair( model->meshGeomID, index ));
            }
            for(vector<dGeomID>::iterator it1=model->primitiveGeomID.begin();
                it1!=model->primitiveGeomID.end(); it1++){
                geomIDMap.insert(make_pair( *it1, index ));
            }
            models.push_back(model);
            isValid = true;
        }
    }

    if(!isValid){
        models.push_back(GeometryExPtr());
    }
    
    return index;

}


void ODECollisionDetectorImpl::addMesh(GeometryEx* model)
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
                }
            }
        }
        if(doAddPrimitive){
            bool created = false;
            dGeomID geomId;
            switch(mesh->primitiveType()){
            case SgMesh::BOX : {
                const Vector3& s = mesh->primitive<SgMesh::Box>().size;
                geomId = dCreateBox(model->spaceID, s.x() * scale.x(), s.y() * scale.y(), s.z() * scale.z());
                created = true;
                break; }
            case SgMesh::SPHERE : {
                SgMesh::Sphere sphere = mesh->primitive<SgMesh::Sphere>();
                geomId = dCreateSphere(model->spaceID, sphere.radius * scale.x());
                created = true;
                break; }
            case SgMesh::CYLINDER : {
                SgMesh::Cylinder cylinder = mesh->primitive<SgMesh::Cylinder>();
                geomId = dCreateCylinder(model->spaceID, cylinder.radius * scale.x(), cylinder.height * scale.y());
                created = true;
                break; }
            default :
                break;
            }
            if(created){
                model->primitiveGeomID.push_back(geomId);
                Affine3 T_ = meshExtractor->currentTransformWithoutScaling();
                if(translation){
                    T_ *= Translation3(*translation);
                }
                if(mesh->primitiveType()==SgMesh::CYLINDER)
                    T_ *= AngleAxis(radian(90), Vector3::UnitX());
                offsetMap.insert(OffsetMap::value_type(geomId, T_));
                meshAdded = true;
            }
        }
    }

    if(!meshAdded){
        const int vertexIndexTop = model->vertices.size();

        const SgVertexArray& vertices_ = *mesh->vertices();
        const int numVertices = vertices_.size();
        for(int i=0; i < numVertices; ++i){
            const Vector3 v = T * vertices_[i].cast<Position::Scalar>();
            model->vertices.push_back(Vertex(v.x(), v.y(), v.z()));
        }

        const int numTriangles = mesh->numTriangles();
        for(int i=0; i < numTriangles; ++i){
            SgMesh::TriangleRef src = mesh->triangle(i);
            Triangle tri;
            tri.indices[0] = vertexIndexTop + src[0];
            tri.indices[1] = vertexIndexTop + src[1];
            tri.indices[2] = vertexIndexTop + src[2];
            model->triangles.push_back(tri);
        }
    }
}


void ODECollisionDetector::setGeometryStatic(int geometryId, bool isStatic)
{
    GeometryExPtr& model = impl->models[geometryId];
    if(model){
        model->isStatic = isStatic;
    }
}


bool ODECollisionDetector::enableGeometryCache(bool on)
{
    return false;
}


void ODECollisionDetector::clearGeometryCache(SgNodePtr geometry)
{
    
}


void ODECollisionDetector::clearAllGeometryCaches()
{

}


void ODECollisionDetector::setNonInterfarenceGeometyrPair(int geometryId1, int geometryId2)
{
    impl->setNonInterfarenceGeometyrPair(geometryId1, geometryId2);
}


void ODECollisionDetectorImpl::setNonInterfarenceGeometyrPair(int geometryId1, int geometryId2)
{
    GeometryExPtr& model1 = models[geometryId1];
    GeometryExPtr& model2 = models[geometryId2];
    if(model1 && model2){
        dSpaceID space1 = model1->spaceID;
        dSpaceID space2 = model2->spaceID;
        //if(nonInterfarencePairs.find(make_pair(space1, space2)) == nonInterfarencePairs.end())
        nonInterfarencePairs.insert(make_pair(space1, space2));
    }
}


bool ODECollisionDetector::makeReady()
{
    return impl->makeReady();
}


bool ODECollisionDetectorImpl::makeReady()
{
    const int n = models.size();
    for(int i=0; i < n; ++i){
        GeometryExPtr& model1 = models[i];
        if(!model1)
            continue;
        for(int j = i+1; j < n; ++j){
            GeometryExPtr& model2 = models[j];
            if(!model2)
                continue;
            if(model1->isStatic && model2->isStatic){
                dSpaceID space1 = model1->spaceID;
                dSpaceID space2 = model2->spaceID;
                if(nonInterfarencePairs.find(make_pair(space1, space2)) == nonInterfarencePairs.end())
                    nonInterfarencePairs.insert(make_pair(space1, space2));
            }
        }
    }
    return true;
}


void ODECollisionDetector::updatePosition(int geometryId, const Position& position)
{
    impl->updatePosition(geometryId, position);
}


void ODECollisionDetectorImpl::updatePosition(int geometryId, const Position& _position)
{
    GeometryExPtr& model = models[geometryId];
    if(model){
        if(model->meshGeomID){
            Vector3 p = _position.translation();
            dMatrix3 R = { _position(0,0), _position(0,1), _position(0,2), 0.0,
                           _position(1,0), _position(1,1), _position(1,2), 0.0,
                           _position(2,0), _position(2,1), _position(2,2), 0.0 };
            dGeomSetPosition(model->meshGeomID, p.x(), p.y(), p.z());
            dGeomSetRotation(model->meshGeomID, R);
        }
        for(vector<dGeomID>::iterator it = model->primitiveGeomID.begin();
            it!=model->primitiveGeomID.end(); it++)
            if(*it){
                Position position = _position * offsetMap[*it];
                Vector3 p = position.translation();
                dMatrix3 R = { position(0,0), position(0,1), position(0,2), 0.0,
                               position(1,0), position(1,1), position(1,2), 0.0,
                               position(2,0), position(2,1), position(2,2), 0.0 };
                dGeomSetPosition(*it, p.x(), p.y(), p.z());
                dGeomSetRotation(*it, R);
            }
    }
}


static void nearCallback(void* data, dGeomID g1, dGeomID g2)
{
    dSpaceID space1 = dGeomGetSpace (g1);
    dSpaceID space2 = dGeomGetSpace (g2);
    ODECollisionDetectorImpl* impl = (ODECollisionDetectorImpl*)data;
    if(impl->nonInterfarencePairs.find(make_pair(space1, space2)) != impl->nonInterfarencePairs.end())
        return;
    if(impl->nonInterfarencePairs.find(make_pair(space2, space1)) != impl->nonInterfarencePairs.end())
        return;

    if(dGeomIsSpace(g1) || dGeomIsSpace(g2)) { 
        dSpaceCollide2(g1, g2, data, &nearCallback);
    } else {
        static const int MaxNumContacts = 100;
        dContact contacts[MaxNumContacts];
        int numContacts= dCollide(g1, g2, MaxNumContacts, &contacts[0].geom, sizeof(dContact));

        if(numContacts > 0 ){
            CollisionPair collisionPair;
            vector<Collision>& collisions = collisionPair.collisions;
            int id1 = impl->geomIDMap.find(g1)->second;
            int id2 = impl->geomIDMap.find(g2)->second;
            collisionPair.geometryId[0] = id2;
            collisionPair.geometryId[1] = id1;
            for(size_t i=0; i < numContacts; i++){
                collisions.push_back(Collision());
                Collision& collision = collisions.back();
                collision.point[0] = contacts[i].geom.pos[0];
                collision.point[1] = contacts[i].geom.pos[1];
                collision.point[2] = contacts[i].geom.pos[2];
                collision.normal[0] = contacts[i].geom.normal[0];
                collision.normal[1] = contacts[i].geom.normal[1];
                collision.normal[2] = contacts[i].geom.normal[2];
                collision.depth = contacts[i].geom.depth;
            }
            impl->callback_(collisionPair);
        }
    }
}


void ODECollisionDetector::detectCollisions(boost::function<void(const CollisionPair&)> callback)
{
    impl->callback_ = callback;
    dSpaceCollide(impl->spaceID, (void*)impl, &nearCallback);
}
