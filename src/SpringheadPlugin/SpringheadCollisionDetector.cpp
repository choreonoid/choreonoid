/**
   \file
   \author Yuichi Tazaki
*/

#include "SpringheadCollisionDetector.h"
#include <cnoid/MeshExtractor>
#include <cnoid/SceneDrawables>
#include <cnoid/EigenUtil>
#include <cnoid/stdx/optional>
#include <Springhead.h>
#include <Physics/PHContactPoint.h>
#include <Physics/PHConstraintEngine.h>

using namespace std;
using namespace cnoid;

#include "SpringheadConvert.h"

namespace {

CollisionDetector* factory()
{
    return new SpringheadCollisionDetector;
}

struct FactoryRegistration
{
    FactoryRegistration(){
        CollisionDetector::registerFactory("SpringheadCollisionDetector", factory);
    }
} factoryRegistration;

class GeometryEx
{
public :
     GeometryEx();
    ~GeometryEx();
    
	bool isStatic;

	Spr::PHSolidIf*          phSolid;
    vector<Spr::CDShapeIf*>  cdShapes;

};
typedef std::shared_ptr<GeometryEx> GeometryExPtr;

GeometryEx::GeometryEx()
{
    isStatic = false;
	cdShapes.clear();
}

GeometryEx::~GeometryEx()
{
    // do cleanup T.B.D.
}

}


namespace cnoid {

class SpringheadCollisionDetectorImpl
{
public:
     SpringheadCollisionDetectorImpl();
    ~SpringheadCollisionDetectorImpl();

    vector<GeometryExPtr> models;

	typedef set< std::pair<int, int> > GeometryIDPairSet;
	GeometryIDPairSet nonInterferencePairs;

	vector<CollisionPair> collisionPairs;

	typedef map< Spr::CDShapeIf*, int > CDShapeMap;
	CDShapeMap cdShapeMap;
	typedef map< Spr::PHSolidIf*, int > PHSolidMap;
	PHSolidMap phSolidMap;

	Spr::PHSdkIf*   phSdk;
	Spr::PHSceneIf* phScene;
       
    std::function<void(const CollisionPair&)> callback_;

    MeshExtractor* meshExtractor;

    int  addGeometry(SgNode* geometry);
    void addMesh(GeometryEx* model);
    void setNonInterfarenceGeometyrPair(int geometryId1, int geometryId2);
    bool makeReady();
    void updatePosition(int geometryId, const Position& position);
    void detectCollisions(std::function<void(const CollisionPair&)> callback);

private :


};
}


SpringheadCollisionDetector::SpringheadCollisionDetector()
{
    impl = new SpringheadCollisionDetectorImpl();
}


SpringheadCollisionDetectorImpl::SpringheadCollisionDetectorImpl()
{
    meshExtractor = new MeshExtractor;
}


SpringheadCollisionDetector::~SpringheadCollisionDetector()
{
    delete impl;
}


SpringheadCollisionDetectorImpl::~SpringheadCollisionDetectorImpl()
{
    delete meshExtractor;
}


const char* SpringheadCollisionDetector::name() const
{
    return "SpringheadCollisionDetector";
}


CollisionDetector* SpringheadCollisionDetector::clone() const
{
    return new SpringheadCollisionDetector;
}

        
void SpringheadCollisionDetector::clearGeometries()
{
	impl->models.clear();
	impl->nonInterferencePairs.clear();
	impl->phSolidMap.clear();
	impl->cdShapeMap.clear();
}


int SpringheadCollisionDetector::numGeometries() const
{
    return impl->models.size();
}


int SpringheadCollisionDetector::addGeometry(SgNode* geometry)
{
    return impl->addGeometry(geometry);
}


int SpringheadCollisionDetectorImpl::addGeometry(SgNode* geometry)
{
    const int index = models.size();
    bool isValid = false;

    if(geometry){
        GeometryExPtr model = std::make_shared<GeometryEx>();
        model->phSolid = phScene->CreateSolid();

        if(meshExtractor->extract(geometry, [&](){ addMesh(model); })){
            phSolidMap.insert( make_pair(model->phSolid, index) );
            for(int i = 0; i < model->cdShapes.size(); i++){
                cdShapeMap.insert( make_pair(model->cdShapes[i], index) );
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

void SpringheadCollisionDetectorImpl::addMesh(GeometryEx* model)
{
    SgMesh* mesh = meshExtractor->currentMesh();
    const Affine3& T = meshExtractor->currentTransform();

    bool meshAdded = false;

	Spr::CDShapeIf* cdShape;
    
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
                }
            }
        }
        if(doAddPrimitive){
            bool created = false;
            
            switch(mesh->primitiveType()){
            case SgMesh::BOX : {
                const Vector3& s = mesh->primitive<SgMesh::Box>().size;
				Spr::CDBoxDesc bd;
				bd.boxsize.x = s.x()*scale.x();
				bd.boxsize.y = s.y()*scale.y();
				bd.boxsize.z = s.z()*scale.z();
				cdShape = phSdk->CreateShape(bd);
                created = true;
                break; }
            case SgMesh::SPHERE : {
                SgMesh::Sphere sphere = mesh->primitive<SgMesh::Sphere>();
				Spr::CDSphereDesc sd;
				sd.radius = sphere.radius*scale.x();
				cdShape = phSdk->CreateShape(sd);
                created = true;
                break; }
            case SgMesh::CYLINDER : {
                SgMesh::Cylinder cylinder = mesh->primitive<SgMesh::Cylinder>();
				// cylinder is not supported
                created = false;
                break; }
            default :
                break;
            }
            if(created){
				model->cdShapes.push_back(cdShape);
				model->phSolid->AddShape(cdShape);

				Affine3 T_ = meshExtractor->currentTransformWithoutScaling();
                if(translation){
                    T_ *= Translation3(*translation);
                }
                if(mesh->primitiveType()==SgMesh::CYLINDER)
                    T_ *= AngleAxis(radian(90), Vector3::UnitX());

                Spr::Vec3d p    = ToSpr((Vector3)(T_.translation()));
                Spr::Matrix3d R = ToSpr(T_.rotation());
				
				int   idx = (int)model->cdShapes.size()-1;
				Spr::Posed pose;
				pose.Pos() = p;
				pose.Ori().FromMatrix(R);
				model->phSolid->SetShapePose(idx, pose);

                meshAdded = true;
            }
        }
    }

    if(!meshAdded){
        const SgVertexArray& vertices_ = *mesh->vertices();
        const int numVertices = vertices_.size();
		Spr::CDConvexMeshDesc meshDesc;

        for(int i=0; i < numVertices; ++i){
            Spr::Vec3f v = ToSpr((Vector3)(T * vertices_[i].cast<Position::Scalar>()));
			meshDesc.vertices.push_back(v);
        }

		cdShape = phSdk->CreateShape(meshDesc);
		model->cdShapes.push_back(cdShape);
		model->phSolid->AddShape(cdShape);
    }
}


void SpringheadCollisionDetector::setGeometryStatic(int geometryId, bool isStatic)
{
    GeometryExPtr& model = impl->models[geometryId];
    if(model){
        model->isStatic = isStatic;
    }
}


bool SpringheadCollisionDetector::enableGeometryCache(bool)
{
    return false;
}


void SpringheadCollisionDetector::clearGeometryCache(SgNode*)
{
    
}


void SpringheadCollisionDetector::clearAllGeometryCaches()
{

}


void SpringheadCollisionDetector::setNonInterfarenceGeometyrPair(int geometryId1, int geometryId2)
{
    impl->setNonInterfarenceGeometyrPair(geometryId1, geometryId2);
}


void SpringheadCollisionDetectorImpl::setNonInterfarenceGeometyrPair(int geometryId1, int geometryId2)
{
    GeometryExPtr& model1 = models[geometryId1];
    GeometryExPtr& model2 = models[geometryId2];
    if(model1 && model2){
        //dSpaceID space1 = model1->spaceID;
        //dSpaceID space2 = model2->spaceID;
        //if(nonInterfarencePairs.find(make_pair(space1, space2)) == nonInterfarencePairs.end())
        nonInterferencePairs.insert(make_pair(geometryId1, geometryId2));
    }
}


bool SpringheadCollisionDetector::makeReady()
{
    return impl->makeReady();
}


bool SpringheadCollisionDetectorImpl::makeReady()
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
				setNonInterfarenceGeometyrPair(i, j);
                //dSpaceID space1 = model1->spaceID;
                //dSpaceID space2 = model2->spaceID;
                //if(nonInterfarencePairs.find(make_pair(space1, space2)) == nonInterfarencePairs.end())
                //    nonInterfarencePairs.insert(make_pair(space1, space2));
            }
        }
    }
    return true;
}


void SpringheadCollisionDetector::updatePosition(int geometryId, const Position& position)
{
    impl->updatePosition(geometryId, position);
}


void SpringheadCollisionDetectorImpl::updatePosition(int geometryId, const Position& _position)
{
    GeometryExPtr& model = models[geometryId];
    if(model){
		Spr::Posed pose;
		pose.Pos() = ToSpr((Vector3)(_position.translation()));
		pose.Ori().FromMatrix(ToSpr((Matrix3)(_position.rotation())));
		model->phSolid->SetPose(pose);

        /*if(model->meshGeomID){
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
			*/
    }
}

/*
static void nearCallback(void* data, dGeomID g1, dGeomID g2)
{
    dSpaceID space1 = dGeomGetSpace (g1);
    dSpaceID space2 = dGeomGetSpace (g2);
    SpringheadCollisionDetectorImpl* impl = (SpringheadCollisionDetectorImpl*)data;
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
*/

void SpringheadCollisionDetector::detectCollisions(std::function<void(const CollisionPair&)> callback)
{
	impl->detectCollisions(callback);
}

void SpringheadCollisionDetectorImpl::detectCollisions(std::function<void(const CollisionPair&)> callback)
{
	// execute collision detection part of Springhead
	phScene->IntegratePart1();

	// enumerate all point contact constraints
	Spr::PHSolidIf* so[2];
	Spr::CDShapeIf* sh[2];

	int ncon = phScene->NContacts();
	for(int i = 0; i < ncon; i++){
		Spr::PHContactPoint* con = phScene->GetContact(i)->Cast();
		Spr::PHShapePair* sp = con->shapePair;
		so[0] = sp->solidPair->solid[0]->Cast();
		so[1] = sp->solidPair->solid[1]->Cast();
		sh[0] = sp->frame[0]->shape->Cast();
		sh[1] = sp->frame[1]->shape->Cast();

		int gid[2];
		gid[0] = phSolidMap[so[0]];
		gid[1] = phSolidMap[so[1]];

		int ngeo = (int)models.size();
		CollisionPair cp = collisionPairs[ngeo * gid[0] + gid[1]];
		
		Spr::Posed p, p0, ps;
		p0 = so[0]->GetPose();
		con->GetSocketPose(ps);
		p = p0 * ps;

		Collision c;
		c.point  = FromSpr(p.Pos());
		c.normal = FromSpr(p.Ori() * Spr::Vec3d(1.0, 0.0, 0.0));
		c.depth  = 0.0;

		cp.collisions.push_back(c);
	}

	// call callback for all collision pairs
	for(int i = 0; i < collisionPairs.size(); i++)
		callback(collisionPairs[i]);

    //impl->callback_ = callback;
    //dSpaceCollide(impl->spaceID, (void*)impl, &nearCallback);
}
