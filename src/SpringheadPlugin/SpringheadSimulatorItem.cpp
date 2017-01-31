/*!
  @file
  @author Yuichi Tazaki
*/

#include "SpringheadSimulatorItem.h"
#include <cnoid/ItemManager>
#include <cnoid/Archive>
#include <cnoid/EigenUtil>
#include <cnoid/EigenArchive>
#include <cnoid/MeshExtractor>
#include <cnoid/SceneDrawables>
#include <cnoid/FloatingNumberString>
#include <cnoid/Body>
#include <cnoid/Link>
#include <cnoid/Sensor>
#include <cnoid/BasicSensorSimulationHelper>
#include <cnoid/BodyItem>
#include <cnoid/BodyCollisionDetectorUtil>
#include <boost/bind.hpp>
#include <QElapsedTimer>
#include "gettext.h"

#undef INFINITY

#include <Springhead.h>

#define ITEM_NAME N_("SpringheadSimulatorItem")
#include <iostream>

using namespace std;
using namespace boost;
using namespace cnoid;

#include "SpringheadConvert.h"

namespace {

const bool MEASURE_PHYSICS_CALCULATION_TIME = true;

const double DEFAULT_GRAVITY_ACCELERATION = 9.80665;

typedef Eigen::Matrix<float, 3, 1> Vertex;

struct Triangle {
    int indices[3];
};

Spr::Matrix3d identity(
	1.0, 0.0, 0.0, 
    0.0, 1.0, 0.0, 
    0.0, 0.0, 1.0
);

Spr::Matrix3d flippedIdentity(
    1.0,  0.0, 0.0,
    0.0,  0.0, 1.0,
    0.0, -1.0, 0.0
);
    
inline void makeInternal(Vector3& v) {
    double a =  v.z();
    v.z()    = -v.y();
    v.y()    =  a;
}
inline void toInternal(const Vector3& v, Vector3& out_v) {
    out_v.x() =  v.x();
	out_v.y() =  v.z();
	out_v.z() = -v.y();
}

class SpringheadBody;

class SpringheadLink : public Referenced
{
public:
    Link*              link;
	SpringheadBody*    body;
	
	Spr::PHSolidIf*         phSolid;
	Spr::PHJointIf*         phJoint;
	Spr::PH1DJointIf*       phJoint1D;
	Spr::PH1DJointLimitIf*  phJointLimit1D;

    vector<Spr::CDShapeIf*> cdShapes;

     SpringheadLink(SpringheadSimulatorItemImpl* simImpl, SpringheadBody* sprBody, SpringheadLink* parent, const Vector3& parentOrigin, Link* link);
    ~SpringheadLink();
    void createLinkBody(SpringheadSimulatorItemImpl* simImpl, SpringheadLink* parent, const Vector3& origin);
    void createGeometry(SpringheadBody* sprBody);
    void setKinematicStateToSpringhead();
    void setKinematicStateToSpringheadflip();
    void setTorqueToSpringhead();
    void setVelocityToSpringhead();
    void getKinematicStateFromSpringhead();
    void getKinematicStateFromSpringheadflip();
    void addMesh(MeshExtractor* extractor, SpringheadBody* sprBody);
};
typedef ref_ptr<SpringheadLink> SpringheadLinkPtr;

class SpringheadBody : public SimulationBody
{
public:
    vector<SpringheadLinkPtr>   sprLinks;
    BasicSensorSimulationHelper sensorHelper;
    int                         geometryId;

	 SpringheadBody(const Body& orgBody);
    ~SpringheadBody();
    void createBody(SpringheadSimulatorItemImpl* simImpl);
    void setExtraJoints(bool flipYZ);
    void setKinematicStateToSpringhead(bool flipYZ);
    void setTorqueToSpringhead();
    void setVelocityToSpringhead();
    void getKinematicStateFromSpringhead(bool flipYZ);
    void updateForceSensors(bool flipYZ);
    void alignToZAxisIn2Dmode();
};
}


namespace cnoid {
  
class SpringheadSimulatorItemImpl
{
public:
    SpringheadSimulatorItem* self;

   	Spr::PHSdkIf*           phSdk;
	Spr::PHSceneIf*         phScene;
        
    bool                    flipYZ;
	double                  timeStep;
    Vector3                 gravity;
    double                  friction;
    bool                    isJointLimitMode;
    bool                    is2Dmode;
    int                     numIterations;
    bool                    useWorldCollision;
    CollisionDetectorPtr    collisionDetector;
    bool                    velocityMode;

    double                  physicsTime;
    QElapsedTimer           physicsTimer;
    double                  collisionTime;
    QElapsedTimer           collisionTimer;

     SpringheadSimulatorItemImpl(SpringheadSimulatorItem* self);
     SpringheadSimulatorItemImpl(SpringheadSimulatorItem* self, const SpringheadSimulatorItemImpl& org);
    ~SpringheadSimulatorItemImpl();
    void clear();
    bool initializeSimulation(const std::vector<SimulationBody*>& simBodies);
    void addBody             (SpringheadBody* sprBody);
    bool stepSimulation      (const std::vector<SimulationBody*>& activeSimBodies);
    void doPutProperties     (PutPropertyFunction& putProperty);
    void store               (Archive& archive);
    void restore             (const Archive& archive);
    void collisionCallback   (const CollisionPair& collisionPair);
};
}


SpringheadLink::SpringheadLink(SpringheadSimulatorItemImpl* simImpl, SpringheadBody* sprBody, SpringheadLink* parent, const Vector3& parentOrigin, Link* link)
{
	phSolid        = 0;
	phJoint        = 0;
	phJoint1D      = 0;
	phJointLimit1D = 0;

    sprBody->sprLinks.push_back(this);

    this->link = link;
	this->body = sprBody;
    
    Vector3 o = parentOrigin + link->b();
    
    createLinkBody(simImpl, parent, o);
    
	if(!simImpl->useWorldCollision){
        createGeometry(sprBody);
    }

    for(Link* child = link->child(); child; child = child->sibling()){
        new SpringheadLink(simImpl, sprBody, this, o, child);
    }
}

void SpringheadLink::createLinkBody(SpringheadSimulatorItemImpl* simImpl, SpringheadLink* parent, const Vector3& origin)
{
	phSolid = simImpl->phScene->CreateSolid();

	phSolid->SetMass   (link->m());
	phSolid->SetInertia(ToSpr(link->I()));

    Vector3 c; ///< center of mass of the link
    Vector3 o; ///< origin of the joint (in world coord.)
    Vector3 a; ///< direction of the joint axis (in world coord.)
    Vector3 d; ///< same as 'a'

    if(!simImpl->flipYZ){
        c = link->c();
        o = origin;
        a = link->a();
        d = link->d();

		Spr::Quaterniond q;
		q.FromMatrix(identity);
		phSolid->SetOrientation(q);
    }
	else {
        toInternal(link->c(), c);
        toInternal(origin   , o);
        toInternal(link->a(), a);
        toInternal(link->d(), d);

		Spr::Quaterniond q;
		q.FromMatrix(flippedIdentity);
		phSolid->SetOrientation(q);
    }

	// set center-of-mass position
	phSolid->SetCenterOfMass(ToSpr(c));
    
	// set the default global position to set a joint
    phSolid->SetFramePosition(ToSpr(o));

	switch(link->jointType()){
    case Link::ROTATIONAL_JOINT:
		phJoint   = simImpl->phScene->CreateJoint(parent->phSolid, phSolid, Spr::PHHingeJointDesc());
		phJoint1D = phJoint->Cast();
	    break;
        
    case Link::SLIDE_JOINT:
        phJoint   = simImpl->phScene->CreateJoint(parent->phSolid, phSolid, Spr::PHSliderJointDesc());
		phJoint1D = phJoint->Cast();
	    break;

    case Link::FREE_JOINT:
        break;

    case Link::FIXED_JOINT:
    default:
        if(parent){
            phJoint = simImpl->phScene->CreateJoint(parent->phSolid, phSolid, Spr::PHFixJointDesc());
        }
		else{
			phSolid->SetDynamical(false);
        }
    }

	// set joint configuration
	if(phJoint){
		Spr::Posed p0 = (parent ? parent->phSolid->GetPose() : Spr::Posed());
		Spr::Posed p1 = phSolid->GetPose();
		Spr::Posed pj;
		pj.Pos() = ToSpr(o);
		pj.Ori().RotationArc(Spr::Vec3d(0,0,1), ToSpr(a));   //< quaternion that maps [0 0 1] to 'a'

		Spr::Posed poseSock, posePlug;
		poseSock = p0.Inv() * pj;
		posePlug = p1.Inv() * pj;

		phJoint->SetSocketPose(poseSock);
		phJoint->SetPlugPose  (posePlug);
	}

	// joint limit
	if(phJoint1D && simImpl->isJointLimitMode){
		phJointLimit1D = phJoint1D->CreateLimit();
		phJointLimit1D->SetRange(Spr::Vec2d(link->q_lower(), link->q_upper()));
	}

}


void SpringheadLink::createGeometry(SpringheadBody* sprBody)
{
    if(link->shape()){
        MeshExtractor* extractor = new MeshExtractor;
        extractor->extract(link->shape(), boost::bind(&SpringheadLink::addMesh, this, extractor, sprBody));
        delete extractor;
    }
}


void SpringheadLink::addMesh(MeshExtractor* extractor, SpringheadBody* sprBody)
{
    SgMesh* mesh = extractor->currentMesh();
    const Affine3& T = extractor->currentTransform();
    
    bool meshAdded = false;

	Spr::PHSceneIf* phScene = phSolid->GetScene()->Cast();
	Spr::PHSdkIf  * phSdk   = phScene->GetSdk();
	Spr::CDShapeIf* cdShape;

    if(mesh->primitiveType() != SgMesh::MESH){
        bool doAddPrimitive = false;
        Vector3 scale;
        optional<Vector3> translation;
        if(!extractor->isCurrentScaled()){
            scale.setOnes();
            doAddPrimitive = true;
        } else {
            Affine3 S = extractor->currentTransformWithoutScaling().inverse() *
                extractor->currentTransform();

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
				cdShapes.push_back(cdShape);
				phSolid->AddShape(cdShape);

				Affine3 T_ = extractor->currentTransformWithoutScaling();
                if(translation){
                    T_ *= Translation3(*translation);
                }
                if(mesh->primitiveType()==SgMesh::CYLINDER)
                    T_ *= AngleAxis(radian(90), Vector3::UnitX());
                Spr::Vec3d p    = ToSpr((Vector3)(T_.translation() - link->c()));
                Spr::Matrix3d R = ToSpr(T_.rotation());
				Spr::Quaterniond q;
				q.FromMatrix(R);

				int   idx = (int)cdShapes.size()-1;
				Spr::Posed pose;
				pose.Pos() = p;
				pose.Ori() = q;
				phSolid->SetShapePose(idx, pose);

                meshAdded = true;
            }
        }
    }

    if(!meshAdded){
        const SgVertexArray& vertices_ = *mesh->vertices();
        const int numVertices = vertices_.size();
		Spr::CDConvexMeshDesc meshDesc;

        for(int i=0; i < numVertices; ++i){
            Spr::Vec3f v = ToSpr((Vector3)(T * vertices_[i].cast<Position::Scalar>() - link->c()));
			meshDesc.vertices.push_back(v);
        }

		cdShape = phSdk->CreateShape(meshDesc);
		cdShapes.push_back(cdShape);
		phSolid->AddShape(cdShape);
    }
}


SpringheadLink::~SpringheadLink()
{

}


void SpringheadLink::setKinematicStateToSpringhead()
{
    const Position& T = link->T();
    if(phSolid){
        Spr::Matrix3d    R = ToSpr((Matrix3)link->R());
		Spr::Quaterniond q;
		q.FromMatrix(R);

		Spr::Vec3d p = ToSpr((Vector3)link->p());
		Spr::Vec3d v = ToSpr((Vector3)link->v());
		Spr::Vec3d w = ToSpr((Vector3)link->w());
		
		phSolid->SetFramePosition  (p);
		phSolid->SetOrientation    (q);
		phSolid->SetVelocity       (v);
		phSolid->SetAngularVelocity(w);
    }else{
    }
}


void SpringheadLink::setKinematicStateToSpringheadflip()
{
    const Position& T = link->T();
	Spr::Matrix3d R2;
	R2[0][0] =  T(0,0); R2[0][1] =  T(0,1); R2[0][2] =  T(0,2);
	R2[1][0] =  T(2,0); R2[1][1] =  T(2,1); R2[1][2] =  T(2,2);
	R2[2][0] = -T(1,0); R2[2][1] = -T(1,1); R2[2][2] = -T(1,2);

	Spr::Quaterniond q;
	q.FromMatrix(R2);

    if(phSolid){
		phSolid->SetOrientation(q);

		Spr::Vec3d p = ToSpr((Vector3)link->p());
		Spr::Vec3d w = ToSpr((Vector3)link->w());
        Spr::Vec3d v = ToSpr((Vector3)link->v());
        
		phSolid->SetFramePosition  ( Spr::Vec3d(p.x, p.z, -p.y) );
		phSolid->SetVelocity       ( Spr::Vec3d(v.x, v.z, -v.y) );
		phSolid->SetAngularVelocity( Spr::Vec3d(w.x, w.z, -w.y) );
    }else{
    }
}


/**
   \note This method must not be called for a static body.
*/
void SpringheadLink::getKinematicStateFromSpringhead()
{
	if(phJoint1D){
		link->q () = phJoint1D->GetPosition();
		link->dq() = phJoint1D->GetVelocity();
	}

	Spr::Posed pose = phSolid->GetPose();
	Spr::Matrix3d R;
	pose.Ori().ToMatrix(R);

	link->R() = FromSpr(R);
	link->p() = FromSpr(pose.Pos());
	link->v() = FromSpr(phSolid->GetVelocity());
	link->w() = FromSpr(phSolid->GetAngularVelocity());
}


/**
   \note This method must not be called for a static body.
*/
void SpringheadLink::getKinematicStateFromSpringheadflip()
{
	if(phJoint1D){
		link->q () = phJoint1D->GetPosition();
		link->dq() = phJoint1D->GetVelocity();
	}

	Spr::Posed pose = phSolid->GetPose();
	Spr::Matrix3d R, Rflip;
	pose.Ori().ToMatrix(R);

	Rflip.row(0) =  R.row(0);
	Rflip.row(1) = -R.row(2);
	Rflip.row(2) =  R.row(1);

	Spr::Vec3d p = pose.Pos();
	Spr::Vec3d v = phSolid->GetVelocity();
	Spr::Vec3d w = phSolid->GetAngularVelocity();

	link->R() = FromSpr(Rflip);
	link->p() = FromSpr( Spr::Vec3d(p.x, -p.z, p.y) );
	link->v() = FromSpr( Spr::Vec3d(v.x, -v.z, v.y) );
	link->w() = FromSpr( Spr::Vec3d(w.x, -w.z, w.y) );
}

/**
   \note This method must not be called for the root link and a static body.
*/
void SpringheadLink::setTorqueToSpringhead()
{
	if(phJoint1D){
		phJoint1D->SetOffsetForce(link->u());
	}
}


void SpringheadLink::setVelocityToSpringhead()
{
	if(phJoint1D){
		phJoint1D->SetTargetVelocity(link->dq());
	}
}


SpringheadBody::SpringheadBody(const Body& orgBody)
    : SimulationBody(new Body(orgBody))
{
    geometryId = 0;
}


SpringheadBody::~SpringheadBody()
{
}


void SpringheadBody::createBody(SpringheadSimulatorItemImpl* simImpl)
{
    Body* body = this->body();
    
    //worldID = body->isStaticModel() ? 0 : simImpl->worldID;
    if(simImpl->useWorldCollision){
        geometryId = addBodyToCollisionDetector(*body, *simImpl->collisionDetector, 
                                                bodyItem()->isSelfCollisionDetectionEnabled());
    }else{
    }

    SpringheadLink* rootLink = new SpringheadLink(simImpl, this, 0, Vector3::Zero(), body->rootLink());

    setKinematicStateToSpringhead(simImpl->flipYZ);

    if(simImpl->useWorldCollision){
		// world collision not supported
    }

    setExtraJoints(simImpl->flipYZ);

    if(simImpl->is2Dmode){
		// 2D mode not supported
    }
    
    setTorqueToSpringhead();

    sensorHelper.initialize(body, simImpl->timeStep, simImpl->gravity);
}


void SpringheadBody::setExtraJoints(bool flipYZ)
{
    Body* body = this->body();
    const int n = body->numExtraJoints();

    for(int j=0; j < n; ++j){

        Body::ExtraJoint& extraJoint = body->extraJoint(j);

        SpringheadLinkPtr sprLinkPair[2];
        for(int i=0; i < 2; ++i){
            SpringheadLinkPtr sprLink;
            Link* link = extraJoint.link[i];
            if(link->index() < sprLinks.size()){
                sprLink = sprLinks[link->index()];
                if(sprLink->link == link){
                    sprLinkPair[i] = sprLink;
                }
            }
            if(!sprLink){
                break;
            }
        }

        if(sprLinkPair[1]){
			Link* link = sprLinkPair[0]->link;
            Vector3 p = link->attitude() * extraJoint.point[0] + link->p();
            Vector3 a = link->attitude() * extraJoint.axis;
            if(flipYZ){
                makeInternal(p);
                makeInternal(a);
            }

            // \todo do the destroy management for these joints
            if(extraJoint.type == Body::EJ_PISTON){

            } else if(extraJoint.type == Body::EJ_BALL){

			}
        }
    }
}


void SpringheadBody::setKinematicStateToSpringhead(bool flipYZ)
{
    if(!flipYZ){
        for(size_t i=0; i < sprLinks.size(); ++i){
            sprLinks[i]->setKinematicStateToSpringhead();
        }
    } else {
        for(size_t i=0; i < sprLinks.size(); ++i){
            sprLinks[i]->setKinematicStateToSpringheadflip();
        }
    }
}


void SpringheadBody::setTorqueToSpringhead()
{
    // Skip the root link
    for(size_t i=1; i < sprLinks.size(); ++i){
        sprLinks[i]->setTorqueToSpringhead();
    }
}


void SpringheadBody::setVelocityToSpringhead()
{
    // Skip the root link
    for(size_t i=1; i < sprLinks.size(); ++i){
        sprLinks[i]->setVelocityToSpringhead();
    }
}


void SpringheadBody::getKinematicStateFromSpringhead(bool flipYZ)
{
    if(!flipYZ){
        for(size_t i=0; i < sprLinks.size(); ++i){
            sprLinks[i]->getKinematicStateFromSpringhead();
        }
    } else {
        for(size_t i=0; i < sprLinks.size(); ++i){
            sprLinks[i]->getKinematicStateFromSpringheadflip();
        }
    }
}


void SpringheadBody::updateForceSensors(bool flipYZ)
{
    const DeviceList<ForceSensor>& forceSensors = sensorHelper.forceSensors();
    for(int i=0; i < forceSensors.size(); ++i){
        ForceSensor* sensor = forceSensors[i];
        const Link* link = sensor->link();
		const SpringheadLink* sprLink = sprLinks[link->index()];

		// get constraint force
		Spr::Vec3d f, t;
		if(sprLink->phJoint){
			sprLink->phJoint->GetConstraintForce(f, t);

			// transform constraint force from socket local coord. to global coord.
			Spr::Posed poseSolid, poseSocket;
			poseSolid = sprLink->phJoint->GetSocketSolid()->GetPose();
			sprLink->phJoint->GetSocketPose(poseSocket);

			Spr::Quaterniond q = poseSolid.Ori() * poseSocket.Ori();
			f = q * f;
			t = q * t;
		}

		Vector3 f2, t2;
		if(!flipYZ){
            f2 << f.x, f.y, f.z;
            t2 << t.x, t.y, t.z;
        } else {
            f2 << f.x, -f.z, f.y;
            t2 << t.x, -t.z, t.y;
        }
        const Matrix3 R = link->R() * sensor->R_local();
        const Vector3 p = link->R() * sensor->p_local();
		
        sensor->f()   = R.transpose() * f2;
        sensor->tau() = R.transpose() * (t2 - p.cross(f2));
        sensor->notifyStateChange();
    }
}


void SpringheadBody::alignToZAxisIn2Dmode()
{
    static const Quat r(AngleAxis(PI / 2.0, Vector3(1.0, 0.0, 0.0)));

    Spr::PHSolidIf* phSolid = sprLinks.front()->phSolid;

    Spr::Quaterniond q0 = phSolid->GetOrientation();
    Quat q(q0[0], q0[1], q0[2], q0[3]);
    Quat q2 = r * q;
    q2.x() = 0.0;
    q2.z() = 0.0;
    q2.normalize();
    Quat q3 = r.inverse() * q2;
    Spr::Quaterniond q4;
    q4[0] = q3.w();
    q4[1] = q3.x();    
    q4[2] = q3.y();    
    q4[3] = q3.z();    
    phSolid->SetOrientation(q4);

    Spr::Vec3d w = phSolid->GetAngularVelocity();
	phSolid->SetAngularVelocity( Spr::Vec3d(0.0, 0.0, w.z) );
}


void SpringheadSimulatorItem::initializeClass(ExtensionManager* ext)
{
    ext->itemManager().registerClass<SpringheadSimulatorItem>(ITEM_NAME);
    ext->itemManager().addCreationPanel<SpringheadSimulatorItem>();
}


SpringheadSimulatorItem::SpringheadSimulatorItem()
{
    impl = new SpringheadSimulatorItemImpl(this);
}


SpringheadSimulatorItemImpl::SpringheadSimulatorItemImpl(SpringheadSimulatorItem* self)
    : self(self)
{
    phSdk   = 0;
	phScene = 0;

    gravity << 0.0, 0.0, -DEFAULT_GRAVITY_ACCELERATION;
    numIterations          = 50;
    friction               = 1.0;
    isJointLimitMode       = false;
    is2Dmode               = false;
    flipYZ                 = false;
    useWorldCollision      = false;
    velocityMode           = false;
}


SpringheadSimulatorItem::SpringheadSimulatorItem(const SpringheadSimulatorItem& org)
    : SimulatorItem(org)
{
    impl = new SpringheadSimulatorItemImpl(this, *org.impl);
}


SpringheadSimulatorItemImpl::SpringheadSimulatorItemImpl(SpringheadSimulatorItem* self, const SpringheadSimulatorItemImpl& org)
    : self(self)
{
    phSdk = 0;

    gravity           = org.gravity;
    numIterations     = org.numIterations;
    friction          = org.friction;
    isJointLimitMode  = org.isJointLimitMode;
    is2Dmode          = org.is2Dmode;
    flipYZ            = org.flipYZ;
    useWorldCollision = org.useWorldCollision;
    velocityMode      = org.velocityMode;
}


SpringheadSimulatorItem::~SpringheadSimulatorItem()
{
    delete impl;
}


SpringheadSimulatorItemImpl::~SpringheadSimulatorItemImpl()
{
    clear();
}

void SpringheadSimulatorItem::setGravity(const Vector3& gravity)
{
    impl->gravity = gravity;
}

void SpringheadSimulatorItem::setFriction(double friction)
{
    impl->friction = friction;
}

void SpringheadSimulatorItem::setJointLimitMode(bool on)
{
    impl->isJointLimitMode = on;
}

void SpringheadSimulatorItem::set2Dmode(bool on)
{
    impl->is2Dmode = on;
}

void SpringheadSimulatorItem::setNumIterations(int n)
{
    impl->numIterations = n;
}

void SpringheadSimulatorItem::useWorldCollisionDetector(bool on)
{
    impl->useWorldCollision = on;
}

void SpringheadSimulatorItemImpl::clear()
{
	if(phSdk)
		phSdk->Clear();
}    

Item* SpringheadSimulatorItem::doDuplicate() const
{
    return new SpringheadSimulatorItem(*this);
}

SimulationBody* SpringheadSimulatorItem::createSimulationBody(Body* orgBody)
{
    return new SpringheadBody(*orgBody);
}


bool SpringheadSimulatorItem::initializeSimulation(const std::vector<SimulationBody*>& simBodies)
{
    return impl->initializeSimulation(simBodies);
}


bool SpringheadSimulatorItemImpl::initializeSimulation(const std::vector<SimulationBody*>& simBodies)
{
    clear();

    flipYZ   = is2Dmode;
	timeStep = self->worldTimeStep();

    phSdk   = Spr::PHSdkIf::CreateSdk();
	phScene = phSdk->CreateScene();
	
	Vector3 g = gravity;
    if(flipYZ){
        toInternal(gravity, g);
    }
	phScene->SetGravity( Spr::Vec3d(g.x(), g.y(), g.z()) );
	phScene->SetNumIteration(numIterations);
	phScene->SetTimeStep(timeStep);

    if(useWorldCollision){
        collisionDetector = self->collisionDetector();
        collisionDetector->clearGeometries();
    }
	else{
        
    }


    for(size_t i=0; i < simBodies.size(); ++i){
        addBody(static_cast<SpringheadBody*>(simBodies[i]));
    }
    if(useWorldCollision)
        collisionDetector->makeReady();

    if(MEASURE_PHYSICS_CALCULATION_TIME){
        physicsTime   = 0;
        collisionTime = 0;
    }

    return true;
}


void SpringheadSimulatorItemImpl::addBody(SpringheadBody* sprBody)
{
	Body& body = *sprBody->body();

    Link* rootLink = body.rootLink();
    rootLink->v ().setZero();
    rootLink->dv().setZero();
    rootLink->w ().setZero();
    rootLink->dw().setZero();

    for(int i=0; i < body.numJoints(); ++i){
        Link* joint = body.joint(i);
        joint->u  () = 0.0;
        joint->dq () = 0.0;
        joint->ddq() = 0.0;
    }
    
    body.clearExternalForces();
    body.calcForwardKinematics(true, true);

    sprBody->createBody(this);
}


void SpringheadSimulatorItem::initializeSimulationThread()
{
}

bool SpringheadSimulatorItem::stepSimulation(const std::vector<SimulationBody*>& activeSimBodies)
{
    return impl->stepSimulation(activeSimBodies);
}


bool SpringheadSimulatorItemImpl::stepSimulation(const std::vector<SimulationBody*>& activeSimBodies)
{
	for(size_t i=0; i < activeSimBodies.size(); ++i){
        SpringheadBody* sprBody = static_cast<SpringheadBody*>(activeSimBodies[i]);
        sprBody->body()->setVirtualJointForces();
        if(velocityMode)
        	sprBody->setVelocityToSpringhead();
        else
        	sprBody->setTorqueToSpringhead();
    }

	if(MEASURE_PHYSICS_CALCULATION_TIME){
	    physicsTimer.start();
	}

	if(useWorldCollision){
	
	}
	else{
		if(MEASURE_PHYSICS_CALCULATION_TIME)
			collisionTimer.start();

		// ìØÇ∂bodyÇ…ëÆÇ∑ÇÈPHSolidÇÃè’ìÀîªíËÇñ≥å¯âª
		std::vector<Spr::PHSolidIf*> solids;
		for(int i = 0; i < (int)activeSimBodies.size(); i++){
			SpringheadBody* sprBody = static_cast<SpringheadBody*>(activeSimBodies[i]);

			solids.clear();
			for(int j = 0; j < sprBody->sprLinks.size(); j++)
				solids.push_back(sprBody->sprLinks[j]->phSolid);

			phScene->SetContactMode(&solids[0], solids.size(), Spr::PHSceneDesc::MODE_NONE);
		}

		phScene->Step();
		
		if(MEASURE_PHYSICS_CALCULATION_TIME)
			collisionTime += collisionTimer.nsecsElapsed();
	}
	
    if(MEASURE_PHYSICS_CALCULATION_TIME){
        physicsTime += physicsTimer.nsecsElapsed();
    }

    //! \todo Bodies with sensors should be managed by the specialized container to increase the efficiency
    for(size_t i=0; i < activeSimBodies.size(); ++i){
        SpringheadBody* sprBody = static_cast<SpringheadBody*>(activeSimBodies[i]);

        // Move the following code to the SpringheadBody class
        if(is2Dmode){
            sprBody->alignToZAxisIn2Dmode();
        }
        if(!sprBody->sensorHelper.forceSensors().empty()){
            sprBody->updateForceSensors(flipYZ);
        }
        sprBody->getKinematicStateFromSpringhead(flipYZ);
        if(sprBody->sensorHelper.hasGyroOrAccelerationSensors()){
            sprBody->sensorHelper.updateGyroAndAccelerationSensors();
        }
    }

    return true;
}

void SpringheadSimulatorItem::finalizeSimulation()
{
    if(MEASURE_PHYSICS_CALCULATION_TIME){
        cout << "Springhead physicsTime= "   << impl->physicsTime *1.0e-9   << "[s]"<< endl;
        cout << "Springhead collisionTime= " << impl->collisionTime *1.0e-9 << "[s]"<< endl;
    }
}


void SpringheadSimulatorItem::doPutProperties(PutPropertyFunction& putProperty)
{
    SimulatorItem::doPutProperties(putProperty);
    impl->doPutProperties(putProperty);
}


void SpringheadSimulatorItemImpl::doPutProperties(PutPropertyFunction& putProperty)
{
    putProperty(_("Gravity"), str(gravity), boost::bind(toVector3, _1, boost::ref(gravity)));

    putProperty.decimals(2).min(0.0)
        (_("Friction"), friction, changeProperty(friction));

    putProperty(_("Limit joint range"), isJointLimitMode, changeProperty(isJointLimitMode));

    putProperty.min(1)
        (_("Iterations"), numIterations, changeProperty(numIterations));

    putProperty(_("2D mode"), is2Dmode, changeProperty(is2Dmode));

    putProperty(_("Use WorldItem's Collision Detector"), useWorldCollision, changeProperty(useWorldCollision));

    putProperty(_("Velocity Control Mode"), velocityMode, changeProperty(velocityMode));

}


bool SpringheadSimulatorItem::store(Archive& archive)
{
    SimulatorItem::store(archive);
    impl->store(archive);
    return true;
}


void SpringheadSimulatorItemImpl::store(Archive& archive)
{
    write(archive, "gravity", gravity);
    archive.write("friction", friction);
    archive.write("jointLimitMode", isJointLimitMode);
    archive.write("numIterations", numIterations);
    archive.write("2Dmode", is2Dmode);
    archive.write("UseWorldItem'sCollisionDetector", useWorldCollision);
    archive.write("velocityMode", velocityMode);
}


bool SpringheadSimulatorItem::restore(const Archive& archive)
{
    SimulatorItem::restore(archive);
    impl->restore(archive);
    return true;
}


void SpringheadSimulatorItemImpl::restore(const Archive& archive)
{
    string symbol;
    read(archive, "gravity", gravity);
    archive.read("friction", friction);
    archive.read("jointLimitMode", isJointLimitMode);
    archive.read("numIterations", numIterations);
    archive.read("2Dmode", is2Dmode);
    archive.read("UseWorldItem'sCollisionDetector", useWorldCollision);
    archive.read("velocityMode", velocityMode);
}
