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
#include <QElapsedTimer>
#include "gettext.h"

#undef INFINITY

#include <Springhead.h>

#define ITEM_NAME N_("SpringheadSimulatorItem")
#include <iostream>

using namespace std;
using namespace std::placeholders;
using namespace cnoid;

#include "SpringheadConvert.h"

namespace cnoid {
  
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

class SpringheadLink;
class SpringheadBody;
class SpringheadSimulatorItemImpl;

class SpringheadLink : public Referenced
{
public:
    Link*                        link;
    SpringheadBody*              body;
    SpringheadSimulatorItemImpl* impl;
	
    Spr::PHSolidIf*         phSolid;
    Spr::PHJointIf*         phJoint;
    Spr::PH1DJointIf*       phJoint1D;
    Spr::PH1DJointLimitIf*  phJointLimit1D;
    Spr::PHTreeNodeIf*      phTreeNode;

    vector<Spr::CDShapeIf*> cdShapes;

     SpringheadLink(SpringheadSimulatorItemImpl* simImpl, SpringheadBody* sprBody, SpringheadLink* parent, const Vector3& parentOrigin, Link* link);
    ~SpringheadLink();
    void createLinkBody(SpringheadSimulatorItemImpl* simImpl, SpringheadLink* parent, const Vector3& origin);
    void createGeometry(SpringheadBody* sprBody);
    void setKinematicStateToSpringhead();
    void setTorqueToSpringhead();
    void setVelocityToSpringhead();
    void getKinematicStateFromSpringhead();
    void addMesh(MeshExtractor* extractor, SpringheadBody* sprBody);
};
typedef ref_ptr<SpringheadLink> SpringheadLinkPtr;

class SpringheadBody : public SimulationBody
{
public:
    vector<SpringheadLinkPtr>   sprLinks;
    BasicSensorSimulationHelper sensorHelper;
    int                         geometryId;

    Spr::PHRootNodeIf*          rootNode;

     SpringheadBody(const Body& orgBody);
    ~SpringheadBody();
    void createBody(SpringheadSimulatorItemImpl* simImpl);
    void setExtraJoints();
    void setKinematicStateToSpringhead();
    void setTorqueToSpringhead();
    void setVelocityToSpringhead();
    void getKinematicStateFromSpringhead();
    void updateForceSensors();
    void alignToZAxisIn2Dmode();
};

class SpringheadSimulatorItemImpl
{
public:
    SpringheadSimulatorItem* self;

    Spr::PHSdkIf*           phSdk;
    Spr::PHSceneIf*         phScene;

    struct Param{
        double  timeStep;
        Vector3 gravity;
        double  staticFriction;
        double  dynamicFriction;
        double  elasticity   ;
		double  contactSpring;
		double  contactDamper;
		int     numIterations;
		bool    velocityMode;
		bool    useABA;
	    bool    useWorldCollision;

		Param();
    };

	Param                   param;
	CollisionDetectorPtr    collisionDetector;
    
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

///////////////////////////////////////////////////////////////////////////////////////////////////

SpringheadLink::SpringheadLink(SpringheadSimulatorItemImpl* simImpl, SpringheadBody* sprBody, SpringheadLink* parent, const Vector3& parentOrigin, Link* link)
{
	phSolid        = 0;
	phJoint        = 0;
	phJoint1D      = 0;
	phJointLimit1D = 0;

    sprBody->sprLinks.push_back(this);

	this->impl = simImpl;
    this->link = link;
	this->body = sprBody;
    
    Vector3 o = parentOrigin + link->b();
    
    createLinkBody(simImpl, parent, o);
    
	if(!simImpl->param.useWorldCollision){
        createGeometry(sprBody);
    }

    for(Link* child = link->child(); child; child = child->sibling()){
        new SpringheadLink(simImpl, sprBody, this, o, child);
    }
}

void SpringheadLink::createLinkBody(SpringheadSimulatorItemImpl* simImpl, SpringheadLink* parent, const Vector3& origin)
{
	phSolid = simImpl->phScene->CreateSolid();

	phSolid->SetMass(link->m());
	phSolid->SetInertia(ToSpr(link->I()));

    Vector3 c; ///< center of mass of the link
    Vector3 o; ///< origin of the joint (in world coord.)
    Vector3 a; ///< direction of the joint axis (in world coord.)
    Vector3 d; ///< same as 'a'

    c = link->c();
    o = origin;
    a = link->a();
    d = link->d();

	Spr::Quaterniond q;
	q.FromMatrix(identity);
	phSolid->SetOrientation(q);
    
	// set center-of-mass position
	phSolid->SetCenterOfMass(ToSpr(c));
    
	// set the default global position to set a joint
    phSolid->SetFramePosition(ToSpr(Vector3(o)));

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
	if(phJoint1D){
		phJointLimit1D = phJoint1D->CreateLimit();
		phJointLimit1D->SetRange(Spr::Vec2d(link->q_lower(), link->q_upper()));
	}

	// tree node
	if(simImpl->param.useABA){
		if(!parent){
			phTreeNode = simImpl->phScene->CreateRootNode(phSolid);
		}
		else{
			phTreeNode = simImpl->phScene->CreateTreeNode(parent->phTreeNode, phSolid);
		}
	}

}


void SpringheadLink::createGeometry(SpringheadBody* sprBody)
{
    if(link->shape()){
        MeshExtractor* extractor = new MeshExtractor;
        extractor->extract(link->shape(), std::bind(&SpringheadLink::addMesh, this, extractor, sprBody));
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
        boost::optional<Vector3> translation;
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
				// set physical materials
				cdShape->SetStaticFriction (impl->param.staticFriction );
				cdShape->SetDynamicFriction(impl->param.dynamicFriction);
				cdShape->SetElasticity     (impl->param.elasticity     );
				cdShape->SetContactSpring  (impl->param.contactSpring  );
				cdShape->SetContactDamper  (impl->param.contactDamper  );

				cdShapes.push_back(cdShape);
				phSolid->AddShape(cdShape);

				Affine3 T_ = extractor->currentTransformWithoutScaling();
                if(translation){
                    T_ *= Translation3(*translation);
                }
                if(mesh->primitiveType()==SgMesh::CYLINDER)
                    T_ *= AngleAxis(radian(90), Vector3::UnitX());
                Spr::Vec3d p    = ToSpr((Vector3)T_.translation());
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
		//Spr::Vec3d c = R * ToSpr((Vector3)link->c());
		//Spr::Vec3d vc = v + w % c;
		
		phSolid->SetFramePosition  (p);
		phSolid->SetOrientation    (q);
		phSolid->SetVelocity       (v);
		phSolid->SetAngularVelocity(w);
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

	//Spr::Vec3d vc = phSolid->GetVelocity();
	//Spr::Vec3d w  = phSolid->GetAngularVelocity();
	//Spr::Vec3d c  = R * phSolid->GetCenterOfMass();
	//Spr::Vec3d v  = vc - w % c;

	link->v() = FromSpr(phSolid->GetVelocity());
	link->w() = FromSpr(phSolid->GetAngularVelocity());
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

///////////////////////////////////////////////////////////////////////////////////////////////////

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
    if(simImpl->param.useWorldCollision){
        geometryId = addBodyToCollisionDetector(*body, *simImpl->collisionDetector, 
                                                bodyItem()->isSelfCollisionDetectionEnabled());
    }else{
    }

    SpringheadLink* rootLink = new SpringheadLink(simImpl, this, 0, Vector3::Zero(), body->rootLink());
	
    setKinematicStateToSpringhead();

    if(simImpl->param.useWorldCollision){
		// world collision not supported
    }

    setExtraJoints();

    setTorqueToSpringhead();

    sensorHelper.initialize(body, simImpl->param.timeStep, simImpl->param.gravity);
}


void SpringheadBody::setExtraJoints()
{
    Body* body = this->body();
    const int n = body->numExtraJoints();

    for(int j=0; j < n; ++j){

        ExtraJoint& extraJoint = body->extraJoint(j);

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
            
            // \todo do the destroy management for these joints
            if(extraJoint.type == ExtraJoint::EJ_PISTON){

            } else if(extraJoint.type == ExtraJoint::EJ_BALL){

			}
        }
    }
}


void SpringheadBody::setKinematicStateToSpringhead()
{
    for(size_t i=0; i < sprLinks.size(); ++i){
        sprLinks[i]->setKinematicStateToSpringhead();
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


void SpringheadBody::getKinematicStateFromSpringhead()
{
    for(size_t i=0; i < sprLinks.size(); ++i){
        sprLinks[i]->getKinematicStateFromSpringhead();
    }
}


void SpringheadBody::updateForceSensors()
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
		f2 << f.x, f.y, f.z;
        t2 << t.x, t.y, t.z;
        
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

///////////////////////////////////////////////////////////////////////////////////////////////////

SpringheadSimulatorItemImpl::Param::Param()
{
    gravity << 0.0, 0.0, -DEFAULT_GRAVITY_ACCELERATION;
    numIterations          = 50;

	// default values are taken from Spr::PHMaterial
    staticFriction         = 0.4;
	dynamicFriction        = 0.4;
	elasticity             = 0.4;
	contactSpring          = 0.0;
	contactDamper          = 0.0;
	useABA                 = false;
    useWorldCollision      = false;
    velocityMode           = false;
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

    param = org.param;
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
    impl->param.gravity = gravity;
}

void SpringheadSimulatorItem::setStaticFriction(double mu0)
{
    impl->param.staticFriction = mu0;
}

void SpringheadSimulatorItem::setDynamicFriction(double mu)
{
	impl->param.dynamicFriction = mu;
}

void SpringheadSimulatorItem::setBouncingFactor(double e)
{
	impl->param.elasticity = e;
}

void SpringheadSimulatorItem::setContactSpring(double K)
{
	impl->param.contactSpring = K;
}

void SpringheadSimulatorItem::setContactDamper(double D)
{
	impl->param.contactDamper = D;
}
    
void SpringheadSimulatorItem::setNumIterations(int n)
{
    impl->param.numIterations = n;
}

void SpringheadSimulatorItem::useWorldCollisionDetector(bool on)
{
    impl->param.useWorldCollision = on;
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

///////////////////////////////////////////////////////////////////////////////////////////////////

bool SpringheadSimulatorItemImpl::initializeSimulation(const std::vector<SimulationBody*>& simBodies)
{
    clear();

	// apply parameters
	param.timeStep = self->worldTimeStep();

    phSdk   = Spr::PHSdkIf::CreateSdk();
	phScene = phSdk->CreateScene();
	
	phScene->SetGravity        (ToSpr(param.gravity));
	phScene->SetNumIteration   (param.numIterations);
	phScene->SetTimeStep       (param.timeStep);
	phScene->SetImpactThreshold(1.0);
	phScene->SetContactTolerance(0.0001);
	phScene->GetConstraintEngine()->SetVelCorrectionRate(0.5);
	phScene->GetConstraintEngine()->SetContactCorrectionRate(0.5);
	phScene->GetConstraintEngine()->SetShrinkRate(0.0);
	//phScene->GetConstraintEngine()->SetUseContactSurface(true);

	if(param.useWorldCollision){
        collisionDetector = self->collisionDetector();
        collisionDetector->clearGeometries();
    }
	else{
        
    }

    for(size_t i=0; i < simBodies.size(); ++i){
        addBody(static_cast<SpringheadBody*>(simBodies[i]));
    }
    if(param.useWorldCollision)
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
        if(param.velocityMode)
        	sprBody->setVelocityToSpringhead();
        else
        	sprBody->setTorqueToSpringhead();
    }

	if(MEASURE_PHYSICS_CALCULATION_TIME){
	    physicsTimer.start();
	}

	if(param.useWorldCollision){
	
	}
	else{
		if(MEASURE_PHYSICS_CALCULATION_TIME)
			collisionTimer.start();

		// disable collisions between solids that belong to the same choreonoid body
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
        if(!sprBody->sensorHelper.forceSensors().empty()){
            sprBody->updateForceSensors();
        }
        sprBody->getKinematicStateFromSpringhead();
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
    putProperty                     (_("Gravity"                           ), str(param.gravity)     , [&](const string& v){ return toVector3(v, param.gravity); });
	putProperty.decimals(2).min(0.0)(_("Static friction"                   ), param.staticFriction   , changeProperty(param.staticFriction   ));
	putProperty.decimals(2).min(0.0)(_("Dynamic friction"                  ), param.dynamicFriction  , changeProperty(param.dynamicFriction  ));
	putProperty.decimals(2).min(0.0)(_("Elasticity"                        ), param.elasticity       , changeProperty(param.elasticity       ));
	putProperty.decimals(2).min(0.0)(_("Contact spring"                    ), param.contactSpring    , changeProperty(param.contactSpring    ));
	putProperty.decimals(2).min(0.0)(_("Contact damper"                    ), param.contactDamper    , changeProperty(param.contactDamper    ));
	putProperty.min(1)              (_("Iterations"                        ), param.numIterations    , changeProperty(param.numIterations    ));
	putProperty                     (_("Use Joint Coordinate Simulation"   ), param.useABA           , changeProperty(param.useABA           ));
	putProperty                     (_("Use WorldItem's Collision Detector"), param.useWorldCollision, changeProperty(param.useWorldCollision));
}


bool SpringheadSimulatorItem::store(Archive& archive)
{
    SimulatorItem::store(archive);
    impl->store(archive);
    return true;
}


void SpringheadSimulatorItemImpl::store(Archive& archive)
{
    write(archive, "gravity"         , param.gravity          );
    archive.write("staticFriction"   , param.staticFriction   );
    archive.write("dynamicFriction"  , param.dynamicFriction  );
    archive.write("elasticity"       , param.elasticity       );
    archive.write("contactSpring"    , param.contactSpring    );
    archive.write("contactDamper"    , param.contactDamper    );
    archive.write("numIterations"    , param.numIterations    );
	archive.write("useABA"           , param.useABA           );
    archive.write("useWorldCollision", param.useWorldCollision);
}


bool SpringheadSimulatorItem::restore(const Archive& archive)
{
    SimulatorItem::restore(archive);
    impl->restore(archive);
    return true;
}


void SpringheadSimulatorItemImpl::restore(const Archive& archive)
{
    read(archive, "gravity"         , param.gravity          );
    archive.read("staticFriction"   , param.staticFriction   );
    archive.read("dynamicFriction"  , param.dynamicFriction  );
    archive.read("elasticity"       , param.elasticity       );
    archive.read("contactSpring"    , param.contactSpring    );
    archive.read("contactDamper"    , param.contactDamper    );
    archive.read("numIterations"    , param.numIterations    );
	archive.read("useABA"           , param.useABA           );
    archive.read("useWorldCollision", param.useWorldCollision);
}
