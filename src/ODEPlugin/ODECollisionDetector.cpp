/**
   \file
   \author Shizuko Hattori
   \author Shin'ichiro Nakaoka
*/

#include "ODECollisionDetector.h"
#include <cnoid/MeshExtractor>
#include <cnoid/SceneDrawables>
#include <cnoid/EigenUtil>

#ifdef GAZEBO_ODE
#include <gazebo/ode/ode.h>
#else
#include <ode/ode.h>
#endif

using namespace std;
using namespace cnoid;

namespace {

typedef CollisionDetector::GeometryHandle GeometryHandle;

struct FactoryRegistration
{
    FactoryRegistration(){
        CollisionDetector::registerFactory(
            "ODECollisionDetector",
            [](){ return new ODECollisionDetector; });
    }
} factoryRegistration;

struct PrimitiveInfo
{
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    dGeomID geomId;
    Position localPosition;
};

class GeometryInfo : public Referenced
{
public :
    GeometryInfo();
    ~GeometryInfo();
    GeometryHandle geometryHandle;
    ReferencedPtr object;
    dSpaceID spaceID;
    vector<PrimitiveInfo, Eigen::aligned_allocator<PrimitiveInfo>> primitives;
    dGeomID meshGeomID;
    dTriMeshDataID triMeshDataID;
    bool isStatic;
};
typedef ref_ptr<GeometryInfo> GeometryInfoPtr;

GeometryInfo::GeometryInfo()
{
    geometryHandle = 0;
    spaceID = 0;
    meshGeomID = 0;
    triMeshDataID = 0;
    isStatic = false;
}

GeometryInfo::~GeometryInfo()
{
    for(auto& pinfo : primitives){
        dGeomDestroy(pinfo.geomId);
    }
    if(meshGeomID){
        dGeomDestroy(meshGeomID);
    }
    if(triMeshDataID){
        dGeomTriMeshDataDestroy(triMeshDataID);
    }
    if(spaceID){
        dSpaceDestroy(spaceID);
    }
}

}

namespace cnoid {

class ODECollisionDetectorImpl
{
public:
    dSpaceID spaceID;
    vector<GeometryInfoPtr> geometryInfos;
    typedef set <pair<dSpaceID, dSpaceID>> SpaceIDPairSet;
    SpaceIDPairSet nonInterfarencePairs;
    std::function<void(const CollisionPair&)> callbackOnCollisionDetected;

    MeshExtractor meshExtractor;
    typedef Eigen::Matrix<float, 3, 1> Vertex;
    vector<Vertex> vertices;
    struct Triangle {
        int indices[3];
    };
    vector<Triangle> triangles;

    ODECollisionDetectorImpl();
    ~ODECollisionDetectorImpl();
    stdx::optional<GeometryHandle> addGeometry(SgNode* geometry);
    void addMesh(GeometryInfo* model);
    void setNonInterfarenceGeometyrPair(GeometryHandle geometry1, GeometryHandle geometry2);
    bool makeReady();
    void setGeometryPosition(GeometryInfo* ginfo, const Position& position);
    void detectCollisions(std::function<void(const CollisionPair&)> callback);
};

}


ODECollisionDetector::ODECollisionDetector()
{
    impl = new ODECollisionDetectorImpl;
}


ODECollisionDetectorImpl::ODECollisionDetectorImpl()
{
    spaceID = dHashSpaceCreate(0);
    dSpaceSetCleanup(spaceID, 0);
}


ODECollisionDetector::~ODECollisionDetector()
{
    delete impl;
}


ODECollisionDetectorImpl::~ODECollisionDetectorImpl()
{
    if(spaceID){
        dSpaceDestroy(spaceID);
    }
}


const char* ODECollisionDetector::name() const
{
    return "ODECollisionDetector";
}


CollisionDetector* ODECollisionDetector::clone() const
{
    return new ODECollisionDetector;
}

        
void ODECollisionDetector::clearGeometries()
{
    impl->geometryInfos.clear();
    impl->nonInterfarencePairs.clear();
}


int ODECollisionDetector::numGeometries() const
{
    return impl->geometryInfos.size();
}


stdx::optional<GeometryHandle> ODECollisionDetector::addGeometry(SgNode* geometry)
{
    return impl->addGeometry(geometry);
}


stdx::optional<GeometryHandle> ODECollisionDetectorImpl::addGeometry(SgNode* geometry)
{
    if(geometry){
        GeometryHandle handle = geometryInfos.size();
        GeometryInfoPtr ginfo = new GeometryInfo;
        ginfo->geometryHandle = handle;
        ginfo->spaceID = dHashSpaceCreate(spaceID);
        dSpaceSetCleanup(ginfo->spaceID, 0);
        if(meshExtractor.extract(geometry, [this, ginfo](){ addMesh(ginfo.get()); })){
           if(!vertices.empty()){
               ginfo->triMeshDataID = dGeomTriMeshDataCreate();
               dGeomTriMeshDataBuildSingle(ginfo->triMeshDataID,
                                           &vertices[0], sizeof(Vertex), vertices.size(),
                                           &triangles[0], triangles.size() * 3, sizeof(Triangle));
               auto id = dCreateTriMesh(ginfo->spaceID, ginfo->triMeshDataID, 0, 0, 0);
               ginfo->meshGeomID = id;
               dGeomSetData(id, ginfo.get());
           }
           for(auto& pinfo : ginfo->primitives){
               dGeomSetData(pinfo.geomId, ginfo.get());
           }
           geometryInfos.push_back(ginfo);
           return handle;
        }
    }

    return stdx::nullopt;
}


void ODECollisionDetectorImpl::addMesh(GeometryInfo* ginfo)
{
    SgMesh* mesh = meshExtractor.currentMesh();
    const Affine3& T = meshExtractor.currentTransform();

    bool meshAdded = false;
    
    if(mesh->primitiveType() != SgMesh::MESH){
        bool doAddPrimitive = false;
        Vector3 scale;
        bool hasTranslationByScaling = false;
        Vector3 translationByScaling;
        if(!meshExtractor.isCurrentScaled()){
            scale.setOnes();
            doAddPrimitive = true;
        } else {
            Affine3 S =
                meshExtractor.currentTransformWithoutScaling().inverse() *
                meshExtractor.currentTransform();

            if(S.linear().isDiagonal()){
                if(!S.translation().isZero()){
                    hasTranslationByScaling = true;
                    translationByScaling = S.translation();
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
                }
            }
        }
        if(doAddPrimitive){
            bool created = false;
            dGeomID geomId;
            switch(mesh->primitiveType()){
            case SgMesh::BOX : {
                const Vector3& s = mesh->primitive<SgMesh::Box>().size;
                geomId = dCreateBox(ginfo->spaceID, s.x() * scale.x(), s.y() * scale.y(), s.z() * scale.z());
                created = true;
                break; }
            case SgMesh::SPHERE : {
                SgMesh::Sphere sphere = mesh->primitive<SgMesh::Sphere>();
                geomId = dCreateSphere(ginfo->spaceID, sphere.radius * scale.x());
                created = true;
                break; }
            case SgMesh::CYLINDER : {
                SgMesh::Cylinder cylinder = mesh->primitive<SgMesh::Cylinder>();
                geomId = dCreateCylinder(ginfo->spaceID, cylinder.radius * scale.x(), cylinder.height * scale.y());
                created = true;
                break; }
            case SgMesh::CAPSULE : {
                SgMesh::Capsule capsule = mesh->primitive<SgMesh::Capsule>();
                geomId = dCreateCapsule(ginfo->spaceID, capsule.radius * scale.x(), capsule.height * scale.y());
                created = true;
                break; }
            default :
                break;
            }
            if(created){
                PrimitiveInfo pinfo;
                pinfo.geomId = geomId;
                Position& T = pinfo.localPosition;
                T = meshExtractor.currentTransformWithoutScaling();
                if(hasTranslationByScaling){
                    T.translation() += translationByScaling;
                }
                if(mesh->primitiveType()==SgMesh::CYLINDER || mesh->primitiveType()==SgMesh::CAPSULE){
                    T *= AngleAxis(radian(90.0), Vector3::UnitX());
                }
                ginfo->primitives.push_back(pinfo);
                meshAdded = true;
            }
        }
    }

    if(!meshAdded){
        const int vertexIndexTop = vertices.size();

        const SgVertexArray& srcVertices = *mesh->vertices();
        const int numVertices = srcVertices.size();
        for(int i=0; i < numVertices; ++i){
            const Vector3 v = T * srcVertices[i].cast<Position::Scalar>();
            vertices.push_back(Vertex(v.x(), v.y(), v.z()));
        }

        const int numTriangles = mesh->numTriangles();
        for(int i=0; i < numTriangles; ++i){
            SgMesh::TriangleRef src = mesh->triangle(i);
            Triangle tri;
            tri.indices[0] = vertexIndexTop + src[0];
            tri.indices[1] = vertexIndexTop + src[1];
            tri.indices[2] = vertexIndexTop + src[2];
            triangles.push_back(tri);
        }
    }
}


void ODECollisionDetector::setCustomObject(GeometryHandle geometry, Referenced* object)
{
    GeometryInfo* ginfo = impl->geometryInfos[geometry];
    if(ginfo){
        ginfo->object = object;
    }
}


void ODECollisionDetector::setGeometryStatic(GeometryHandle geometry, bool isStatic)
{
    GeometryInfo* ginfo = impl->geometryInfos[geometry];
    if(ginfo){
        ginfo->isStatic = isStatic;
    }
}


void ODECollisionDetector::setNonInterfarenceGeometyrPair(GeometryHandle geometry1, GeometryHandle geometry2)
{
    impl->setNonInterfarenceGeometyrPair(geometry1, geometry2);
}


void ODECollisionDetectorImpl::setNonInterfarenceGeometyrPair(GeometryHandle geometry1, GeometryHandle geometry2)
{
    GeometryInfo* ginfo1 = geometryInfos[geometry1];
    GeometryInfo* ginfo2 = geometryInfos[geometry2];
    if(ginfo1 && ginfo2){
        dSpaceID space1 = ginfo1->spaceID;
        dSpaceID space2 = ginfo2->spaceID;
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
    const int n = geometryInfos.size();
    for(int i=0; i < n; ++i){
        GeometryInfo* info1 = geometryInfos[i];
        if(info1){
            for(int j=i + 1; j < n; ++j){
                GeometryInfo* info2 = geometryInfos[j];
                if(info2){
                    if(info1->isStatic && info2->isStatic){
                        dSpaceID space1 = info1->spaceID;
                        dSpaceID space2 = info2->spaceID;
                        if(nonInterfarencePairs.find(make_pair(space1, space2)) == nonInterfarencePairs.end()){
                            nonInterfarencePairs.insert(make_pair(space1, space2));
                        }
                    }
                }
            }
        }
    }
    return true;
}


void ODECollisionDetectorImpl::setGeometryPosition(GeometryInfo* ginfo, const Position& position)
{
    if(ginfo->meshGeomID){
        Vector3 p = position.translation();
        const Position& T = position;
        dMatrix3 R = { T(0,0), T(0,1), T(0,2), 0.0,
                       T(1,0), T(1,1), T(1,2), 0.0,
                       T(2,0), T(2,1), T(2,2), 0.0 };
        dGeomSetPosition(ginfo->meshGeomID, p.x(), p.y(), p.z());
        dGeomSetRotation(ginfo->meshGeomID, R);
    }
    for(auto& pinfo : ginfo->primitives){
        Position T = position * pinfo.localPosition;
        auto p = T.translation();
        dMatrix3 R = { T(0,0), T(0,1), T(0,2), 0.0,
                       T(1,0), T(1,1), T(1,2), 0.0,
                       T(2,0), T(2,1), T(2,2), 0.0 };
        dGeomSetPosition(pinfo.geomId, p.x(), p.y(), p.z());
        dGeomSetRotation(pinfo.geomId, R);
    }
}


void ODECollisionDetector::updatePosition(GeometryHandle geometry, const Position& position)
{
    GeometryInfo* ginfo = impl->geometryInfos[geometry];
    if(ginfo){
        impl->setGeometryPosition(ginfo, position);
    }
}


void ODECollisionDetector::updatePositions
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

                                      
static void nearCallback(void* data, dGeomID g1, dGeomID g2)
{
    dSpaceID space1 = dGeomGetSpace(g1);
    dSpaceID space2 = dGeomGetSpace(g2);
    ODECollisionDetectorImpl* impl = static_cast<ODECollisionDetectorImpl*>(data);
    if(impl->nonInterfarencePairs.find(make_pair(space1, space2)) != impl->nonInterfarencePairs.end()){
        return;
    }
    if(impl->nonInterfarencePairs.find(make_pair(space2, space1)) != impl->nonInterfarencePairs.end()){
        return;
    }

    if(dGeomIsSpace(g1) || dGeomIsSpace(g2)) { 
        dSpaceCollide2(g1, g2, data, &nearCallback);
    } else {
        static const int MaxNumContacts = 100;
        dContact contacts[MaxNumContacts];
        int numContacts= dCollide(g1, g2, MaxNumContacts, &contacts[0].geom, sizeof(dContact));

        if(numContacts > 0 ){
            GeometryInfo* ginfo1 = static_cast<GeometryInfo*>(dGeomGetData(g1));
            GeometryInfo* ginfo2 = static_cast<GeometryInfo*>(dGeomGetData(g2));
            CollisionPair collisionPair(
                ginfo2->geometryHandle, ginfo2->object, ginfo1->geometryHandle, ginfo1->object);

            for(size_t i=0; i < numContacts; i++){
                Collision& collision = collisionPair.newCollision();
                collision.point[0] = contacts[i].geom.pos[0];
                collision.point[1] = contacts[i].geom.pos[1];
                collision.point[2] = contacts[i].geom.pos[2];
                collision.normal[0] = contacts[i].geom.normal[0];
                collision.normal[1] = contacts[i].geom.normal[1];
                collision.normal[2] = contacts[i].geom.normal[2];
                collision.depth = contacts[i].geom.depth;
            }
            impl->callbackOnCollisionDetected(collisionPair);
        }
    }
}


void ODECollisionDetector::detectCollisions(std::function<void(const CollisionPair&)> callback)
{
    impl->callbackOnCollisionDetected = callback;
    dSpaceCollide(impl->spaceID, (void*)impl, &nearCallback);
}
