#include "ODESimulatorItem.h"
#include <cnoid/ItemManager>
#include <cnoid/PutPropertyFunction>
#include <cnoid/Archive>
#include <cnoid/EigenUtil>
#include <cnoid/EigenArchive>
#include <cnoid/MeshExtractor>
#include <cnoid/SceneDrawables>
#include <cnoid/FloatingNumberString>
#include <cnoid/Format>
#include <cnoid/MessageOut>
#include <cnoid/Body>
#include <cnoid/Link>
#include <cnoid/Sensor>
#include <cnoid/BasicSensorSimulationHelper>
#include <cnoid/BodyItem>
#include <cnoid/BodyCollisionDetector>
#include <cnoid/BodyCollisionLinkFilter>
#include <cnoid/WorldItem>
#include <cnoid/Material>
#include <cnoid/MaterialTable>
#include <cnoid/IdPair>
#include <cnoid/ValueTree>
#include <cnoid/CloneMap>
#include <QElapsedTimer>
#include <memory>
#include <unordered_map>
#include <unordered_set>
#include "gettext.h"

#include <ode/ode.h>
#define ITEM_NAME N_("ODESimulatorItem")

using namespace std;
using namespace cnoid;

namespace {

const bool TRACE_FUNCTIONS = false;
const bool USE_AMOTOR = false;

const bool MEASURE_PHYSICS_CALCULATION_TIME = true;

const double DEFAULT_GRAVITY_ACCELERATION = 9.80665;

// Restitution is applied only when the impact velocity exceeds this multiple
// of the velocity that the normal component of gravity regenerates at a
// resting contact in a single step (with a small absolute minimum for
// gravity-orthogonal contacts such as walls).
const double IMPACT_VELOCITY_THRESH_RATIO = 2.0;
const double MIN_IMPACT_VELOCITY_THRESH = 1.0e-4;

typedef Eigen::Matrix<float, 3, 1> Vertex;

struct Triangle {
    int indices[3];
};

void flipYZ(Vector3& v)
{
    double z = v.z();
    v.z() = -v.y();
    v.y() = z;
}

Vector3 getFlipYZ(const Vector3& v)
{
    return Vector3(v.x(), v.z(), -v.y());
}

void flipYZ(dMatrix3& R)
{
    Vector3 y(R[4], R[5], R[6]);
    // Y <- Z
    R[4] = R[8];
    R[5] = R[9];
    R[6] = R[10];
    // Z = -Y
    R[8] = y.x();
    R[9] = y.y();
    R[10] = y.z();
}

bool hasLinkDisablingCollisionRules(Body* body)
{
    ListingPtr rules = body->info()->findListing("collision_detection_rules");
    if(rules->isValid()){
        for(auto& node : *rules){
            auto mapping = node->toMapping();
            if(mapping->isValid() && mapping->find("disabled_links")->isValid()){
                return true;
            }
        }
    }

    MappingPtr info = body->info()->findMapping("collisionDetection");
    if(info->isValid()){
        return info->findListing("excludeLinks")->isValid();
    }

    return false;
}

bool isSurfaceVelocityLink(Link* link)
{
    return link->jointType() == Link::PseudoContinuousTrackJoint &&
        (link->actuationMode() == Link::JointVelocity ||
         link->actuationMode() == Link::DeprecatedJointSurfaceVelocity);
}

class ODEBody;

class ODELink : public Referenced
{
public:
    Link* link;
    ODEBody* odeBody;
    dBodyID bodyID;
    dJointID jointID;
    vector<dGeomID> geomID;
    dTriMeshDataID triMeshDataID;
    vector<Vertex> vertices;
    vector<Triangle> triangles;
    typedef map<dGeomID, Isometry3, std::less<dGeomID>, 
                Eigen::aligned_allocator< pair<const dGeomID, Isometry3>>> OffsetMap;
    OffsetMap offsetMap;
    dJointID motorID;

    ODELink(
        ODESimulatorItemImpl* simImpl, ODEBody* odeBody, ODELink* parent,
        const Isometry3& T_origin, Link* link);
    ~ODELink();
    void createLinkBody(
        ODESimulatorItemImpl* simImpl, dWorldID worldID, ODELink* parent, const Isometry3& T_origin);
    void createGeometry(ODEBody* odeBody, bool doFlipYZ);
    void setKinematicStateToODE();
    void setKinematicStateToODEflip();
    void setTorqueToODE();
    void setVelocityToODE();
    void getKinematicStateFromODE();
    void getKinematicStateFromODEflip();
    void addMesh(MeshExtractor* extractor, ODEBody* odeBody, bool doFlipYZ);
};
typedef ref_ptr<ODELink> ODELinkPtr;


class ODEBody : public SimulationBody
{
public:
    vector<ODELinkPtr> odeLinks;
    dWorldID worldID;
    dSpaceID spaceID;
    unique_ptr<BodyCollisionLinkFilter> bodyCollisionLinkFilter;
    bool isJointAxisInertiaWarningIssued;
    bool selfCollision;
    vector<dJointFeedback> forceSensorFeedbacks;
    BasicSensorSimulationHelper sensorHelper;
        
    ODEBody(Body* body);
    ~ODEBody();
    void createBody(ODESimulatorItemImpl* simImpl);
    void setKinematicStateToODE(bool doFlipYZ);
    void setControlValToODE();
    void getKinematicStateFromODE(bool doFlipYZ);
    void updateForceSensors(bool doFlipYZ);
    void alignToZAxisIn2Dmode();
};

}


namespace cnoid {
  
typedef std::map<dBodyID, Link*> CrawlerLinkMap;
class ODESimulatorItemImpl
{
public:
    ODESimulatorItem* self;

    bool doFlipYZ;
        
    dWorldID worldID;
    dSpaceID spaceID;
    dJointGroupID contactJointGroupID;
    double timeStep;
    CrawlerLinkMap crawlerLinks;

    Selection stepMode;
    Vector3 gravity;
    double friction;
    bool isJointLimitMode;
    bool isAddingJointAxisInertiaToLinkInertia;
    bool is2Dmode;
    double globalERP;
    FloatingNumberString globalCFM;
    int numIterations;
    double overRelaxation;
    bool enableMaxCorrectingVel;
    FloatingNumberString maxCorrectingVel;
    double surfaceLayerDepth;
    double surfaceVelocityCullingDepth;
    bool useWorldCollisionDetector;
    BodyCollisionDetector bodyCollisionDetector;

    // Contact material support. The friction and restitution coefficients are
    // resolved per material-id pair from the MaterialTable of the world item.
    // When the materials cannot be resolved, the friction falls back to the
    // global friction property and the restitution to zero.
    MaterialTable* materialTable;
    struct ContactParam {
        double friction;
        double restitution;
    };
    std::unordered_map<IdPair<int>, ContactParam> contactParamCache;
    Vector3 odeGravity; // gravity in the ODE world frame (flipped in the 2D mode)

    double physicsTime;
    QElapsedTimer physicsTimer;
    double collisionTime;
    QElapsedTimer collisionTimer;

    ODESimulatorItemImpl(ODESimulatorItem* self);
    ODESimulatorItemImpl(ODESimulatorItem* self, const ODESimulatorItemImpl& org);
    void initialize();
    ~ODESimulatorItemImpl();
    void clear();
    bool initializeSimulation(const std::vector<SimulationBody*>& simBodies);
    void addBody(ODEBody* odeBody);
    void setExtraJoints(const std::vector<SimulationBody*>& simBodies);
    bool stepSimulation(const std::vector<SimulationBody*>& activeSimBodies);
    ContactParam resolveContactParam(Link* link1, Link* link2);
    void setBounceParameters(dContact& contact, double restitution);
    void doPutProperties(PutPropertyFunction& putProperty);
    void store(Archive& archive);
    void restore(const Archive& archive);
    void onCollisionPairDetected(const CollisionPair& collisionPair);
};
}


ODELink::ODELink
(ODESimulatorItemImpl* simImpl, ODEBody* odeBody, ODELink* parent, const Isometry3& T_origin, Link* link)
{
    odeBody->odeLinks.push_back(this);

    this->link = link;
    this->odeBody = odeBody;
    bodyID = 0;
    jointID = 0;
    triMeshDataID = 0;
    geomID.clear();
    motorID = 0;
    if(odeBody->worldID){
        createLinkBody(simImpl, odeBody->worldID, parent, T_origin);
    }
    if(!simImpl->useWorldCollisionDetector &&
       (!odeBody->bodyCollisionLinkFilter ||
        odeBody->bodyCollisionLinkFilter->checkIfEnabledLinkIndex(link->index()))){
        createGeometry(odeBody, simImpl->doFlipYZ);
        // Make the ODE link accessible from the geoms in the collision callback
        // (dBodyGetData cannot be used as static links have no ODE body)
        for(auto& id : geomID){
            dGeomSetData(id, this);
        }
    }

    for(Link* child = link->child(); child; child = child->sibling()){
        new ODELink(simImpl, odeBody, this, T_origin * child->Tb(), child);
    }

}


void ODELink::createLinkBody(ODESimulatorItemImpl* simImpl, dWorldID worldID, ODELink* parent, const Isometry3& T_origin)
{
    bodyID = dBodyCreate(worldID);
    dBodySetData(bodyID, link);

    if(!odeBody->isJointAxisInertiaWarningIssued && link->Jm2() != 0.0){
        if(simImpl->isAddingJointAxisInertiaToLinkInertia){
            MessageOut::master()->putWarningln(
                formatR(_("{0} has joint-axis inertia at joint \"{1}\" ({2}). "
                          "ODESimulatorItem cannot reproduce it directly, "
                          "so it is approximately added to the link inertia. "
                          "This behavior can be changed with the "
                          "\"Add joint-axis inertia to link inertia\" property."),
                        odeBody->body()->name(), link->name(), link->Jm2()));
        } else {
            MessageOut::master()->putWarningln(
                formatR(_("{0} has joint-axis inertia at joint \"{1}\" ({2}). "
                          "ODESimulatorItem does not support it directly. "
                          "Enable \"Add joint-axis inertia to link inertia\" "
                          "to approximately add it to the link inertia."),
                        odeBody->body()->name(), link->name(), link->Jm2()));
        }
        odeBody->isJointAxisInertiaWarningIssued = true;
    }

    dMass mass;
    dMassSetZero(&mass);
    const Matrix3& I = link->I();
    Matrix3 I0;
    const Matrix3* I_ode = &I;
    if(simImpl->isAddingJointAxisInertiaToLinkInertia && link->Jm2() != 0.0){
        const Vector3 axis = link->a();
        I0 = I + axis * axis.transpose() * link->Jm2();
        I_ode = &I0;
    }
    dMassSetParameters(&mass, link->m(),
            0.0, 0.0, 0.0,
            (*I_ode)(0,0), (*I_ode)(1,1), (*I_ode)(2,2),
            (*I_ode)(0,1), (*I_ode)(0,2), (*I_ode)(1,2));

    dBodySetMass(bodyID, &mass);

    Vector3 p = T_origin * link->c();
    Vector3 o = T_origin.translation();
    Vector3 a = T_origin.linear() * link->a();
    Vector3 d = T_origin.linear() * link->d();
    auto& T = T_origin;
    dMatrix3 R = {
        T(0,0), T(0,1), T(0,2), 0.0,
        T(1,0), T(1,1), T(1,2), 0.0,
        T(2,0), T(2,1), T(2,2), 0.0 };

    if(simImpl->doFlipYZ){
        flipYZ(p);
        flipYZ(o);
        flipYZ(a);
        flipYZ(d);
        flipYZ(R);
    }
        
    // set the default global position to set a joint
    dBodySetPosition(bodyID, p.x(), p.y(), p.z());
    dBodySetRotation(bodyID, R);

    dBodyID parentBodyID = parent ? parent->bodyID : 0;

    if(link->actuationMode() == Link::DeprecatedJointSurfaceVelocity){
        link->setJointType(Link::PseudoContinuousTrackJoint);
        link->setActuationMode(Link::JointVelocity);
    }

    switch(link->jointType()){
        
    case Link::RevoluteJoint:
        jointID = dJointCreateHinge(worldID, 0);
        dJointAttach(jointID, bodyID, parentBodyID);
        dJointSetHingeAnchor(jointID, o.x(), o.y(), o.z());
        dJointSetHingeAxis(jointID, a.x(), a.y(), a.z());
        if(simImpl->isJointLimitMode){
            if(link->q_upper() < numeric_limits<double>::max()){
                dJointSetHingeParam(jointID, dParamHiStop, link->q_upper());
            }
            if(link->q_lower() > -numeric_limits<double>::max()){
                dJointSetHingeParam(jointID, dParamLoStop, link->q_lower());
            }
        }
        if(link->actuationMode() == Link::JointVelocity){
            if(!USE_AMOTOR){
                dJointSetHingeParam(jointID, dParamFMax, numeric_limits<dReal>::max());
                dJointSetHingeParam(jointID, dParamFudgeFactor, 1);
            }else{
                motorID = dJointCreateAMotor(worldID, 0);
                dJointAttach(motorID, bodyID, parentBodyID);
                dJointSetAMotorMode(motorID, dAMotorUser);
                dJointSetAMotorNumAxes(motorID, 1);
                dJointSetAMotorAxis(motorID, 0, 2, a.x(), a.y(), a.z());
                dJointSetAMotorParam(motorID, dParamFMax, numeric_limits<dReal>::max() );
                dJointSetAMotorParam(motorID, dParamFudgeFactor, 1);
            }
        }
        break;
        
    case Link::PrismaticJoint:
        jointID = dJointCreateSlider(worldID, 0);
        dJointAttach(jointID, bodyID, parentBodyID);
        dJointSetSliderAxis(jointID, d.x(), d.y(), d.z());
        if(simImpl->isJointLimitMode){
            if(link->q_upper() < numeric_limits<double>::max()){
                dJointSetSliderParam(jointID, dParamHiStop, link->q_upper());
            }
            if(link->q_lower() > -numeric_limits<double>::max()){
                dJointSetSliderParam(jointID, dParamLoStop, link->q_lower());
            }
        }
        if(link->actuationMode() == Link::JointVelocity){
        	dJointSetSliderParam(jointID, dParamFMax, numeric_limits<dReal>::max() );
  			dJointSetSliderParam(jointID, dParamFudgeFactor, 1 );
        }
        break;

    case Link::FreeJoint:
        break;

    case Link::PseudoContinuousTrackJoint:
        if(parentBodyID){
            if(link->actuationMode() == Link::JointVelocity ||
               link->actuationMode() == Link::DeprecatedJointSurfaceVelocity){
                simImpl->crawlerLinks.insert(make_pair(bodyID, link));
            }
    	}

    case Link::FixedJoint:
    default:
        if(parentBodyID){
            jointID = dJointCreateFixed(worldID, 0);
            dJointAttach(jointID, bodyID, parentBodyID);
            dJointSetFixed(jointID);
        } else {
            dBodySetKinematic(bodyID);
        }
        break;
    }
}


void ODELink::createGeometry(ODEBody* odeBody, bool doFlipYZ)
{
    if(link->collisionShape()){
        MeshExtractor* extractor = new MeshExtractor;
        if(extractor->extract(
               link->collisionShape(),
               [this, extractor, odeBody, doFlipYZ](){ addMesh(extractor, odeBody, doFlipYZ); })){
            if(!vertices.empty()){
                triMeshDataID = dGeomTriMeshDataCreate();
                dGeomTriMeshDataBuildSingle(
                    triMeshDataID,
                    &vertices[0], sizeof(Vertex), vertices.size(),
                    &triangles[0],triangles.size() * 3, sizeof(Triangle));
            
                dGeomID gId = dCreateTriMesh(odeBody->spaceID, triMeshDataID, 0, 0, 0);
                geomID.push_back(gId);
                dGeomSetBody(gId, bodyID);
            }
        }
        delete extractor;
    }
}


void ODELink::addMesh(MeshExtractor* extractor, ODEBody* odeBody, bool doFlipYZ)
{
    SgMesh* mesh = extractor->currentMesh();
    const Affine3& T = extractor->currentTransform();
    
    bool meshAdded = false;
    
    if(mesh->primitiveType() != SgMesh::MESH){
        bool doAddPrimitive = false;
        Vector3 scale;
        std::optional<Vector3> translation;
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
                geomId = dCreateBox(odeBody->spaceID, s.x() * scale.x(), s.y() * scale.y(), s.z() * scale.z());
                created = true;
                break; }
            case SgMesh::SPHERE : {
                SgMesh::Sphere sphere = mesh->primitive<SgMesh::Sphere>();
                geomId = dCreateSphere(odeBody->spaceID, sphere.radius * scale.x());
                created = true;
                break; }
            case SgMesh::CYLINDER : {
                SgMesh::Cylinder cylinder = mesh->primitive<SgMesh::Cylinder>();
                geomId = dCreateCylinder(odeBody->spaceID, cylinder.radius * scale.x(), cylinder.height * scale.y());
                created = true;
                break; }
            case SgMesh::CAPSULE : {
                SgMesh::Capsule capsule = mesh->primitive<SgMesh::Capsule>();
                geomId = dCreateCapsule(odeBody->spaceID, capsule.radius * scale.x(), capsule.height * scale.y());
                created = true;
                break; }
            default :
                break;
            }
            if(created){
                geomID.push_back(geomId);
                dGeomSetBody(geomId, bodyID);
                Isometry3 T_ = extractor->currentTransformWithoutScaling();
                if(translation){
                    T_ *= Translation3(*translation);
                }
                if(mesh->primitiveType()==SgMesh::CYLINDER ||
                        mesh->primitiveType()==SgMesh::CAPSULE )
                    T_ *= AngleAxis(radian(90), Vector3::UnitX());
                Vector3 p = T_.translation() - link->c();
                dMatrix3 R = { T_(0,0), T_(0,1), T_(0,2), 0.0,
                               T_(1,0), T_(1,1), T_(1,2), 0.0,
                               T_(2,0), T_(2,1), T_(2,2), 0.0 };
                if(bodyID){
                    if(doFlipYZ){
                        flipYZ(p);
                        flipYZ(R);
                    }
                    dGeomSetOffsetPosition(geomId, p.x(), p.y(), p.z());
                    dGeomSetOffsetRotation(geomId, R);
                } else {
                    offsetMap.insert(OffsetMap::value_type(geomId,T_));
                }
                meshAdded = true;
            }
        }
    }

    if(!meshAdded){
        const int vertexIndexTop = vertices.size();

        const SgVertexArray& vertices_ = *mesh->vertices();
        const int numVertices = vertices_.size();
        if(!doFlipYZ){
            for(int i=0; i < numVertices; ++i){
                const Vector3 v = T * vertices_[i].cast<Vector3::Scalar>() - link->c();
                vertices.push_back(Vertex(v.x(), v.y(), v.z()));
            }
        } else {
            for(int i=0; i < numVertices; ++i){
                const Vector3 v = T * vertices_[i].cast<Vector3::Scalar>() - link->c();
                vertices.push_back(Vertex(v.x(), v.z(), -v.y()));
            }
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


ODELink::~ODELink()
{
    for(vector<dGeomID>::iterator it=geomID.begin(); it!=geomID.end(); it++){
        dGeomDestroy(*it);
    }
    if(triMeshDataID){
        dGeomTriMeshDataDestroy(triMeshDataID);
    }
}


void ODELink::setKinematicStateToODE()
{
    const Isometry3& T = link->T();
    if(bodyID){
        dMatrix3 R2 = {
            T(0,0), T(0,1), T(0,2), 0.0,
            T(1,0), T(1,1), T(1,2), 0.0,
            T(2,0), T(2,1), T(2,2), 0.0 };
    
        dBodySetRotation(bodyID, R2);
        const Vector3 lc = link->R() * link->c();
        const Vector3 c = link->p() + lc;
        dBodySetPosition(bodyID, c.x(), c.y(), c.z());
        const Vector3& w = link->w();
        const Vector3 v = link->v() + w.cross(lc);
        dBodySetLinearVel(bodyID, v.x(), v.y(), v.z());
        dBodySetAngularVel(bodyID, w.x(), w.y(), w.z());

    } else {
        for(vector<dGeomID>::iterator it = geomID.begin(); it!=geomID.end(); it++){
            OffsetMap::iterator it0 = offsetMap.find(*it);
            Isometry3 offset(Isometry3::Identity());
            if(it0!=offsetMap.end()){
                offset = it0->second;
            }
            Isometry3 T_ = T * offset;
            Vector3 p = T_.translation() + link->c();
            dMatrix3 R2 = {
                T_(0,0), T_(0,1), T_(0,2), 0.0,
                T_(1,0), T_(1,1), T_(1,2), 0.0,
                T_(2,0), T_(2,1), T_(2,2), 0.0 };

            dGeomSetPosition(*it, p.x(), p.y(), p.z());
            dGeomSetRotation(*it, R2);
        }
    }
}


void ODELink::setKinematicStateToODEflip()
{
    const Isometry3& T = link->T();
    dMatrix3 R2 = {
         T(0,0),  T(0,1),  T(0,2), 0.0,
         T(2,0),  T(2,1),  T(2,2), 0.0,
        -T(1,0), -T(1,1), -T(1,2), 0.0 };
    
    if(bodyID){
        dBodySetRotation(bodyID, R2);
        const Vector3 lc = link->R() * link->c();
        const Vector3 c = link->p() + lc;
        dBodySetPosition(bodyID, c.x(), c.z(), -c.y());
        const Vector3& w = link->w();
        const Vector3 v = link->v() + w.cross(lc);
        dBodySetLinearVel(bodyID, v.x(), v.z(), -v.y());
        dBodySetAngularVel(bodyID, w.x(), w.z(), -w.y());

    } else {
        const Vector3 c = link->T() * link->c();
        for(vector<dGeomID>::iterator it = geomID.begin(); it!=geomID.end(); it++){
            dGeomSetPosition(*it, c.x(), c.y(), -c.z());
            dGeomSetRotation(*it, R2);
        }
    }
}


/**
   \note This method must not be called for a static body.
*/
void ODELink::getKinematicStateFromODE()
{
    if(jointID){
        if(link->isRevoluteJoint()){
            link->q() = dJointGetHingeAngle(jointID);
            link->dq() = dJointGetHingeAngleRate(jointID);
        } else if(link->isPrismaticJoint()){
            link->q() = dJointGetSliderPosition(jointID);
            link->dq() = dJointGetSliderPositionRate(jointID);
        }
    }

    const dReal* R = dBodyGetRotation(bodyID);
    link->R() <<
        R[0], R[1], R[2],
        R[4], R[5], R[6],
        R[8], R[9], R[10];

    typedef Eigen::Map<const Eigen::Matrix<dReal, 3, 1> > toVector3;
    const Vector3 c = link->R() * link->c();
    link->p() = toVector3(dBodyGetPosition(bodyID)) - c;
    link->w() = toVector3(dBodyGetAngularVel(bodyID));
    link->v() = toVector3(dBodyGetLinearVel(bodyID)) - link->w().cross(c);
}


/**
   \note This method must not be called for a static body.
*/
void ODELink::getKinematicStateFromODEflip()
{
    if(jointID){
        if(link->isRevoluteJoint()){
            link->q() = dJointGetHingeAngle(jointID);
            link->dq() = dJointGetHingeAngleRate(jointID);
        } else if(link->isPrismaticJoint()){
            link->q() = dJointGetSliderPosition(jointID);
            link->dq() = dJointGetSliderPositionRate(jointID);
        }
    }

    const dReal* R = dBodyGetRotation(bodyID);
    link->R() <<
         R[0],  R[1],  R[2],
        -R[8], -R[9], -R[10],
         R[4],  R[5],  R[6];

    Vector3 c = getFlipYZ(link->R() * link->c());
    
    typedef Eigen::Map<const Eigen::Matrix<dReal, 3, 1> > toVector3;
    const Vector3 p = toVector3(dBodyGetPosition(bodyID)) - c;
    toVector3 w(dBodyGetAngularVel(bodyID));
    const Vector3 v = toVector3(dBodyGetLinearVel(bodyID)) - w.cross(c);
    
    link->p() << p.x(), -p.z(), p.y();
    link->w() << w.x(), -w.z(), w.y();
    link->v() << v.x(), -v.z(), v.y();
}


/**
   \note This method must not be called for the root link and a static body.
*/
void ODELink::setTorqueToODE()
{
    if(link->isRevoluteJoint()){
        dJointAddHingeTorque(jointID, link->u());
    } else if(link->isPrismaticJoint()){
        dJointAddSliderForce(jointID, link->u());
    }
}


void ODELink::setVelocityToODE()
{
    if(link->isRevoluteJoint()){
    	dReal v = link->dq_target();
    	if(!USE_AMOTOR){
    		dJointSetHingeParam(jointID, dParamVel, v);
        } else {
    		dJointSetAMotorParam(motorID, dParamVel, v);
        }
    } else if(link->isPrismaticJoint()){
    	dReal v = link->dq_target();
    	dJointSetSliderParam(jointID, dParamVel, v);
    }
}


ODEBody::ODEBody(Body* body)
    : SimulationBody(body)
{
    worldID = 0;
    spaceID = 0;
    isJointAxisInertiaWarningIssued = false;
    selfCollision = false;
}


ODEBody::~ODEBody()
{
    if(spaceID){
        dSpaceDestroy(spaceID);
    }
}


void ODEBody::createBody(ODESimulatorItemImpl* simImpl)
{
    Body* body = this->body();
    isJointAxisInertiaWarningIssued = false;
    selfCollision = false;
    bodyCollisionLinkFilter.reset();
    bool selfCollisionRequested = bodyItem() && bodyItem()->isSelfCollisionDetectionEnabled();
    int numCollisionLinks = 0;
    for(int i = 0; i < body->numLinks(); ++i){
        if(body->link(i)->collisionShape()){
            ++numCollisionLinks;
        }
    }
    bool needsSelfCollisionFilter =
        !simImpl->useWorldCollisionDetector && selfCollisionRequested && numCollisionLinks >= 2;
    bool needsLinkExclusionFilter =
        !simImpl->useWorldCollisionDetector && hasLinkDisablingCollisionRules(body);
    if(needsSelfCollisionFilter || needsLinkExclusionFilter){
        bodyCollisionLinkFilter = std::make_unique<BodyCollisionLinkFilter>();
        bodyCollisionLinkFilter->setTargetBody(body, selfCollisionRequested);

        int numEnabledCollisionLinks = 0;
        for(int i = 0; i < body->numLinks(); ++i){
            Link* link = body->link(i);
            if(link->collisionShape()){
                if(bodyCollisionLinkFilter->checkIfEnabledLinkIndex(i)){
                    ++numEnabledCollisionLinks;
                }
            }
        }
        selfCollision = needsSelfCollisionFilter && numEnabledCollisionLinks >= 2;
        bool needsFilterForGeometry = numEnabledCollisionLinks < numCollisionLinks;
        if(!selfCollision && !needsFilterForGeometry){
            bodyCollisionLinkFilter.reset();
        }
    }
    
    worldID = body->isStaticModel() ? 0 : simImpl->worldID;

    if(!simImpl->useWorldCollisionDetector){
        spaceID = dHashSpaceCreate(simImpl->spaceID);
        dSpaceSetCleanup(spaceID, 0);
    }

    ODELink* rootLink = new ODELink(simImpl, this, nullptr, body->rootLink()->T(), body->rootLink());

    setKinematicStateToODE(simImpl->doFlipYZ);

    if(simImpl->useWorldCollisionDetector){
        simImpl->bodyCollisionDetector.setLinkAssociatedObjectFunction(
            [this](Link* link, CollisionDetector::GeometryHandle geometry){
                return odeLinks[link->index()];
            });
        simImpl->bodyCollisionDetector.addBody(body, bodyItem()->isSelfCollisionDetectionEnabled());
    }

    if(simImpl->is2Dmode && worldID){
        dJointID planeJointID = dJointCreatePlane2D(worldID, 0);
        dJointAttach(planeJointID, rootLink->bodyID, 0);
    }
    
    setControlValToODE();

    sensorHelper.initialize(body, simImpl->timeStep, simImpl->gravity);

    // set joint feedbacks for force sensors
    const DeviceList<ForceSensor>& forceSensors = sensorHelper.forceSensors();
    forceSensorFeedbacks.resize(forceSensors.size());
    for(size_t i=0; i < forceSensors.size(); ++i){
        dJointSetFeedback(odeLinks[forceSensors[i]->link()->index()]->jointID, &forceSensorFeedbacks[i]);
    }
}


void ODEBody::setKinematicStateToODE(bool doFlipYZ)
{
    if(!doFlipYZ){
        for(size_t i=0; i < odeLinks.size(); ++i){
            odeLinks[i]->setKinematicStateToODE();
        }
    } else {
        for(size_t i=0; i < odeLinks.size(); ++i){
            odeLinks[i]->setKinematicStateToODEflip();
        }
    }
}


void ODEBody::setControlValToODE()
{
    // Skip the root link
    for(size_t i=1; i < odeLinks.size(); ++i){
        switch(odeLinks[i]->link->actuationMode()){
        case Link::StateNone:
            break;
        case Link::JointTorque:
            odeLinks[i]->setTorqueToODE();
            break;
        case Link::JointVelocity :
            odeLinks[i]->setVelocityToODE();
            break;
        default :
            break;
        }
     }
}


void ODEBody::getKinematicStateFromODE(bool doFlipYZ)
{
    if(!doFlipYZ){
        for(size_t i=0; i < odeLinks.size(); ++i){
            odeLinks[i]->getKinematicStateFromODE();
        }
    } else {
        for(size_t i=0; i < odeLinks.size(); ++i){
            odeLinks[i]->getKinematicStateFromODEflip();
        }
    }
}


void ODEBody::updateForceSensors(bool doFlipYZ)
{
    const DeviceList<ForceSensor>& forceSensors = sensorHelper.forceSensors();
    for(int i=0; i < forceSensors.size(); ++i){
        ForceSensor* sensor = forceSensors[i];
        const Link* link = sensor->link();
        const dJointFeedback& fb = forceSensorFeedbacks[i];
        Vector3 f, tau;
        if(!doFlipYZ){
            f   << fb.f2[0], fb.f2[1], fb.f2[2];
            tau << fb.t2[0], fb.t2[1], fb.t2[2];
        } else {
            f   << fb.f2[0], -fb.f2[2], fb.f2[1];
            tau << fb.t2[0], -fb.t2[2], fb.t2[1];
        }
        const Matrix3 R = link->R() * sensor->R_local();
        const Vector3 p = link->R() * sensor->p_local();

        sensor->f()   = R.transpose() * f;
        sensor->tau() = R.transpose() * (tau - p.cross(f));
        sensor->notifyStateChange();
    }
}


void ODEBody::alignToZAxisIn2Dmode()
{
    static const Quaternion r(AngleAxis(PI / 2.0, Vector3(1.0, 0.0, 0.0)));

    dBodyID& bodyID = odeLinks.front()->bodyID;

    const dReal* q0 = dBodyGetQuaternion(bodyID);
    Quaternion q(q0[0], q0[1], q0[2], q0[3]);
    Quaternion q2 = r * q;
    q2.x() = 0.0;
    q2.z() = 0.0;
    q2.normalize();
    Quaternion q3 = r.inverse() * q2;
    dReal q4[4];
    q4[0] = q3.w();
    q4[1] = q3.x();    
    q4[2] = q3.y();    
    q4[3] = q3.z();    
    dBodySetQuaternion(bodyID, q4);

    const dReal* w = dBodyGetAngularVel(bodyID);
    dBodySetAngularVel(bodyID, 0, 0, w[2]);
}


void ODESimulatorItem::initializeClass(ExtensionManager* ext)
{
    ext->itemManager().registerClass<ODESimulatorItem, SimulatorItem>(ITEM_NAME);
    ext->itemManager().addCreationPanel<ODESimulatorItem>();
}


ODESimulatorItem::ODESimulatorItem()
{
    impl = new ODESimulatorItemImpl(this);
}


ODESimulatorItemImpl::ODESimulatorItemImpl(ODESimulatorItem* self)
    : self(self),
      stepMode(ODESimulatorItem::NUM_STEP_MODES, CNOID_GETTEXT_DOMAIN_NAME)

{
    initialize();

    stepMode.setSymbol(ODESimulatorItem::STEP_ITERATIVE,  N_("Iterative (quick step)"));
    stepMode.setSymbol(ODESimulatorItem::STEP_BIG_MATRIX, N_("Big matrix"));
    stepMode.select(ODESimulatorItem::STEP_ITERATIVE);
    
    gravity << 0.0, 0.0, -DEFAULT_GRAVITY_ACCELERATION;
    globalERP = 0.4;
    globalCFM = "1.0e-10";
    numIterations = 50;
    overRelaxation = 1.3;
    enableMaxCorrectingVel = true;
    maxCorrectingVel = "1.0e-2";
    surfaceLayerDepth = 0.0001;
    surfaceVelocityCullingDepth = 0.005;
    friction = 1.0;
    isJointLimitMode = false;
    isAddingJointAxisInertiaToLinkInertia = false;
    is2Dmode = false;
    doFlipYZ = false;
    useWorldCollisionDetector = false;
}


ODESimulatorItem::ODESimulatorItem(const ODESimulatorItem& org)
    : SimulatorItem(org)
{
    impl = new ODESimulatorItemImpl(this, *org.impl);
}


ODESimulatorItemImpl::ODESimulatorItemImpl(ODESimulatorItem* self, const ODESimulatorItemImpl& org)
    : self(self)
{
    initialize();

    stepMode = org.stepMode;
    gravity = org.gravity;
    globalERP = org.globalERP;
    globalCFM = org.globalCFM;
    numIterations = org.numIterations;
    overRelaxation = org.overRelaxation;
    enableMaxCorrectingVel = org.enableMaxCorrectingVel;
    maxCorrectingVel = org.maxCorrectingVel;
    surfaceLayerDepth = org.surfaceLayerDepth;
    surfaceVelocityCullingDepth = org.surfaceVelocityCullingDepth;
    friction = org.friction;
    isJointLimitMode = org.isJointLimitMode;
    isAddingJointAxisInertiaToLinkInertia = org.isAddingJointAxisInertiaToLinkInertia;
    is2Dmode = org.is2Dmode;
    doFlipYZ = org.doFlipYZ;
    useWorldCollisionDetector = org.useWorldCollisionDetector;
}


void ODESimulatorItemImpl::initialize()
{
    worldID = 0;
    spaceID = 0;
    contactJointGroupID = dJointGroupCreate(0);
    materialTable = nullptr;
    self->SimulatorItem::setAllLinkPositionOutputMode(true);
}


ODESimulatorItem::~ODESimulatorItem()
{
    delete impl;
}


ODESimulatorItemImpl::~ODESimulatorItemImpl()
{
    clear();

    if(contactJointGroupID){
        dJointGroupDestroy(contactJointGroupID);
    }
}


void ODESimulatorItem::setStepMode(int value)
{
    impl->stepMode.select(value);
}


void ODESimulatorItem::setGravity(const Vector3& gravity)
{
    impl->gravity = gravity;
}


void ODESimulatorItem::setFriction(double friction)
{
    impl->friction = friction;
}


void ODESimulatorItem::setJointLimitMode(bool on)
{
    impl->isJointLimitMode = on;
}


void ODESimulatorItem::setJointAxisInertiaAdditionMode(bool on)
{
    impl->isAddingJointAxisInertiaToLinkInertia = on;
}


void ODESimulatorItem::set2Dmode(bool on)
{
    impl->is2Dmode = on;
}


void ODESimulatorItem::setGlobalERP(double erp)
{
    impl->globalERP = erp;
}


void ODESimulatorItem::setGlobalCFM(double value)
{
    impl->globalCFM = value;
}


void ODESimulatorItem::setNumIterations(int n)
{
    impl->numIterations = n;
}


void ODESimulatorItem::setOverRelaxation(double value)
{
    impl->overRelaxation = value;
}


void ODESimulatorItem::setCorrectingVelocityLimitMode(bool on)
{
    impl->enableMaxCorrectingVel = on;
}


void ODESimulatorItem::setMaxCorrectingVelocity(double vel)
{
    impl->maxCorrectingVel = vel;
}


void ODESimulatorItem::setSurfaceLayerDepth(double value)
{
    impl->surfaceLayerDepth = value;
}


void ODESimulatorItem::useWorldCollisionDetector(bool on)
{
    impl->useWorldCollisionDetector = on;
}


void ODESimulatorItem::setAllLinkPositionOutputMode(bool)
{
    // The mode is not changed.
    // This simulator only supports the all link position output
    // because joint positions may be slightly changed
}


Vector3 ODESimulatorItem::getGravity() const
{
    return impl->gravity;
}


void ODESimulatorItemImpl::clear()
{
    dJointGroupEmpty(contactJointGroupID);

    if(worldID){
        dWorldDestroy(worldID);
        worldID = 0;
    }
    if(spaceID){
        dSpaceDestroy(spaceID);
        spaceID = 0;
    }

    crawlerLinks.clear();
}    


Item* ODESimulatorItem::doCloneItem(CloneMap* /* cloneMap */) const
{
    return new ODESimulatorItem(*this);
}


SimulationBody* ODESimulatorItem::createSimulationBody(Body* orgBody, CloneMap& cloneMap)
{
    return new ODEBody(cloneMap.getClone(orgBody));
}


bool ODESimulatorItem::initializeSimulation(const std::vector<SimulationBody*>& simBodies)
{
    return impl->initializeSimulation(simBodies);
}


bool ODESimulatorItemImpl::initializeSimulation(const std::vector<SimulationBody*>& simBodies)
{
    clear();

    doFlipYZ = is2Dmode;

    Vector3 g = gravity;
    
    if(doFlipYZ){
        flipYZ(g);
    }

    worldID = dWorldCreate();
    if(useWorldCollisionDetector){
        bodyCollisionDetector.setCollisionDetector(self->getOrCreateCollisionDetector());
    } else {
        spaceID = dHashSpaceCreate(0);
        dSpaceSetCleanup(spaceID, 0);
    }

    dRandSetSeed(0);
    dWorldSetGravity(worldID, g.x(), g.y(), g.z());
    odeGravity = g;
    dWorldSetERP(worldID, globalERP);
    dWorldSetCFM(worldID, globalCFM.value());
    dWorldSetContactSurfaceLayer(worldID, 0.0);
    dWorldSetQuickStepNumIterations(worldID, numIterations);
    dWorldSetQuickStepW(worldID, overRelaxation);
    dWorldSetContactMaxCorrectingVel(worldID, enableMaxCorrectingVel ? maxCorrectingVel.value() : dInfinity);
    dWorldSetContactSurfaceLayer(worldID, surfaceLayerDepth);

    timeStep = self->worldTimeStep();

    materialTable = nullptr;
    contactParamCache.clear();
    if(WorldItem* worldItem = self->worldItem()){
        materialTable = worldItem->materialTable();
    }

    for(size_t i=0; i < simBodies.size(); ++i){
        addBody(static_cast<ODEBody*>(simBodies[i]));
    }
    setExtraJoints(simBodies);
    if(useWorldCollisionDetector){
        bodyCollisionDetector.makeReady();
    }

    if(MEASURE_PHYSICS_CALCULATION_TIME){
        physicsTime = 0;
        collisionTime = 0;
    }

    return true;
}


void ODESimulatorItemImpl::addBody(ODEBody* odeBody)
{
    Body& body = *odeBody->body();

    Link* rootLink = body.rootLink();
    rootLink->v().setZero();
    rootLink->dv().setZero();
    rootLink->w().setZero();
    rootLink->dw().setZero();

    for(int i=0; i < body.numJoints(); ++i){
        Link* joint = body.joint(i);
        joint->u() = 0.0;
        joint->dq() = 0.0;
        joint->ddq() = 0.0;
        joint->q_target() = joint->q();
        joint->dq_target() = joint->dq();
    }
    
    body.clearExternalForces();
    body.calcForwardKinematics(true, true);

    odeBody->createBody(this);
}


void ODESimulatorItemImpl::setExtraJoints(const std::vector<SimulationBody*>& simBodies)
{
    unordered_map<Link*, ODELink*> odeLinkMap;
    unordered_set<ExtraJoint*> initializedExtraJoints;

    for(auto simBody : simBodies){
        auto odeBody = static_cast<ODEBody*>(simBody);
        for(auto& odeLink : odeBody->odeLinks){
            odeLinkMap[odeLink->link] = odeLink;
        }
    }

    for(auto simBody : simBodies){
        Body* body = simBody->body();
        const int n = body->numExtraJoints();

        for(int j = 0; j < n; ++j){
            ExtraJoint* extraJoint = body->extraJoint(j);
            if(!initializedExtraJoints.insert(extraJoint).second){
                continue;
            }

            ODELink* odeLinkPair[2] = { nullptr, nullptr };
            for(int i = 0; i < 2; ++i){
                Link* link = extraJoint->link(i);
                if(!link){
                    break;
                }
                auto p = odeLinkMap.find(link);
                if(p == odeLinkMap.end()){
                    break;
                }
                odeLinkPair[i] = p->second;
            }

            if(!odeLinkPair[0] || !odeLinkPair[1] ||
               (!odeLinkPair[0]->bodyID && !odeLinkPair[1]->bodyID)){
                continue;
            }

            dJointID jointID = 0;
            Link* link = odeLinkPair[0]->link;
            Vector3 p = link->T() * extraJoint->point(0);
            Vector3 a = link->R() * extraJoint->localRotation(0) * extraJoint->axis();
            if(doFlipYZ){
                flipYZ(p);
                flipYZ(a);
            }

            if(extraJoint->type() == ExtraJoint::Piston){
                jointID = dJointCreatePiston(worldID, 0);
                dJointAttach(jointID, odeLinkPair[0]->bodyID, odeLinkPair[1]->bodyID);
                dJointSetPistonAnchor(jointID, p.x(), p.y(), p.z());
                dJointSetPistonAxis(jointID, a.x(), a.y(), a.z());

            } else if(extraJoint->type() == ExtraJoint::Fixed){
                jointID = dJointCreateFixed(worldID, 0);
                dJointAttach(jointID, odeLinkPair[0]->bodyID, odeLinkPair[1]->bodyID);
                dJointSetFixed(jointID);

            } else if(extraJoint->type() == ExtraJoint::Hinge){
                jointID = dJointCreateHinge(worldID, 0);
                dJointAttach(jointID, odeLinkPair[0]->bodyID, odeLinkPair[1]->bodyID);
                dJointSetHingeAnchor(jointID, p.x(), p.y(), p.z());
                dJointSetHingeAxis(jointID, a.x(), a.y(), a.z());

            } else if(extraJoint->type() == ExtraJoint::Ball){
                jointID = dJointCreateBall(worldID, 0);
                dJointAttach(jointID, odeLinkPair[0]->bodyID, odeLinkPair[1]->bodyID);
                dJointSetBallAnchor(jointID, p.x(), p.y(), p.z());
            }
        }
    }
}


void ODESimulatorItem::initializeSimulationThread()
{
    dAllocateODEDataForThread(dAllocateMaskAll);
}


ODESimulatorItemImpl::ContactParam ODESimulatorItemImpl::resolveContactParam(Link* link1, Link* link2)
{
    if(!materialTable || !link1 || !link2){
        return { friction, 0.0 };
    }
    const int id1 = link1->materialId();
    const int id2 = link2->materialId();
    const IdPair<int> key(id1, id2);
    auto it = contactParamCache.find(key);
    if(it != contactParamCache.end()){
        return it->second;
    }

    ContactParam param{ friction, 0.0 };
    // Prefer an explicitly defined contact material pair; otherwise derive the
    // values from the two single materials in the same way as the other
    // simulator items (friction = sqrt(r1*r2), restitution = sqrt((1-v1)*(1-v2))).
    if(ContactMaterial* cm = materialTable->contactMaterial(id1, id2)){
        param.friction = cm->friction();
        param.restitution = cm->restitution();
    } else {
        Material* m1 = materialTable->material(id1);
        Material* m2 = materialTable->material(id2);
        if(m1 && m2){
            param.friction = std::sqrt(m1->roughness() * m2->roughness());
            param.restitution = std::sqrt(std::max(0.0, 1.0 - m1->viscosity())
                                          * std::max(0.0, 1.0 - m2->viscosity()));
        }
    }
    param.friction = std::max(param.friction, 0.0);
    param.restitution = std::min(std::max(param.restitution, 0.0), 1.0);
    contactParamCache[key] = param;
    return param;
}


void ODESimulatorItemImpl::setBounceParameters(dContact& contact, double restitution)
{
    if(restitution > 0.0){
        dSurfaceParameters& surface = contact.surface;
        surface.mode |= dContactBounce;
        surface.bounce = restitution;
        // Apply restitution only when the impact velocity exceeds the velocity
        // that the normal component of gravity regenerates at a resting contact
        // in a single step. The threshold adapts to the contact direction
        // (zero gravity component for walls, g*cos(theta) for slopes) and to
        // the gravity setting of the simulation.
        const Vector3 normal(
            contact.geom.normal[0], contact.geom.normal[1], contact.geom.normal[2]);
        surface.bounce_vel = std::max(
            IMPACT_VELOCITY_THRESH_RATIO * std::fabs(odeGravity.dot(normal)) * timeStep,
            MIN_IMPACT_VELOCITY_THRESH);
    }
}


static void nearCallback(void* data, dGeomID g1, dGeomID g2)
{
    if(dGeomIsSpace(g1) || dGeomIsSpace(g2)) { 
        dSpaceCollide2(g1, g2, data, &nearCallback);
        if(false) { // Same-body pairs are handled per ODEBody when self-collision is enabled.
            if(dGeomIsSpace(g1)){
                dSpaceCollide((dSpaceID)g1, data, &nearCallback);
            }
            if(dGeomIsSpace(g2)){
                dSpaceCollide((dSpaceID)g2, data, &nearCallback);
            }
        }
    } else {
        ODESimulatorItemImpl* impl = (ODESimulatorItemImpl*)data;
        auto odeLink1 = static_cast<ODELink*>(dGeomGetData(g1));
        auto odeLink2 = static_cast<ODELink*>(dGeomGetData(g2));
        if(!odeLink1 || !odeLink2 || odeLink1 == odeLink2){
            return;
        }
        if(odeLink1->odeBody == odeLink2->odeBody){
            auto& filter = odeLink1->odeBody->bodyCollisionLinkFilter;
            if(!filter || !filter->checkIfEnabledLinkPair(
                   odeLink1->link->index(), odeLink2->link->index())){
                return;
            }
        }
        static const int MaxNumContacts = 100;
        dContact contacts[MaxNumContacts];
        int numContacts = dCollide(g1, g2, MaxNumContacts, &contacts[0].geom, sizeof(dContact));
        
        if(numContacts > 0){
            dBodyID body1ID = dGeomGetBody(g1);
            dBodyID body2ID = dGeomGetBody(g2);
            Link* crawlerlink = 0;
            double sign = 1.0;
            double motionSign = 1.0;
            if(!impl->crawlerLinks.empty()){
                CrawlerLinkMap::iterator p = impl->crawlerLinks.find(body1ID);
                if(p != impl->crawlerLinks.end()){
                    crawlerlink = p->second;
                }
                p = impl->crawlerLinks.find(body2ID);
                if(p != impl->crawlerLinks.end()){
                    crawlerlink = p->second;
                    sign = -1.0;
                }
            }
            if(!crawlerlink){
                if(isSurfaceVelocityLink(odeLink1->link)){
                    crawlerlink = odeLink1->link;
                    motionSign = -1.0;
                } else if(isSurfaceVelocityLink(odeLink2->link)){
                    crawlerlink = odeLink2->link;
                    sign = -1.0;
                    motionSign = -1.0;
                }
            }
            auto param = impl->resolveContactParam(odeLink1->link, odeLink2->link);
            for(int i=0; i < numContacts; ++i){
                dSurfaceParameters& surface = contacts[i].surface;
                bool applySurfaceVelocity =
                    crawlerlink &&
                    (impl->surfaceVelocityCullingDepth <= 0.0 ||
                     contacts[i].geom.depth <= impl->surfaceVelocityCullingDepth);
                if(!applySurfaceVelocity){
                    surface.mode = dContactApprox1;
                    surface.mu = param.friction;
                    impl->setBounceParameters(contacts[i], param.restitution);

                } else {
                    surface.mode = dContactFDir1 | dContactMotion1 | dContactMu2 | dContactApprox1_2 | dContactApprox1_1;
                    const Vector3 axis = crawlerlink->R() * crawlerlink->a();
                    const Vector3 n(contacts[i].geom.normal);
                    Vector3 dir = axis.cross(n);
                    if(dir.norm() < 1.0e-5){
                        surface.mode = dContactApprox1;
                        surface.mu = impl->friction;
                    } else {
                        dir *= sign;
                        dir.normalize();
                        contacts[i].fdir1[0] = dir[0];
                        contacts[i].fdir1[1] = dir[1];
                        contacts[i].fdir1[2] = dir[2];
                        //dVector3& dpos = contacts[i].geom.pos;
                        //Vector3 pos(dpos[0], dpos[1], dpos[2]);
                        //Vector3 v = crawlerlink->v + crawlerlink->w.cross(pos-crawlerlink->p);
                        //surface.motion1 = dir.dot(v) + crawlerlink->u;
                        surface.motion1 = motionSign * crawlerlink->dq_target();
                        surface.mu = impl->friction;
                        surface.mu2 = 0.5;
                    }
                }
                dJointID jointID = dJointCreateContact(impl->worldID, impl->contactJointGroupID, &contacts[i]);
                dJointAttach(jointID, body1ID, body2ID);
            }
        }
    }
}


bool ODESimulatorItem::stepSimulation(const std::vector<SimulationBody*>& activeSimBodies)
{
    return impl->stepSimulation(activeSimBodies);
}


bool ODESimulatorItemImpl::stepSimulation(const std::vector<SimulationBody*>& activeSimBodies)
{
    for(size_t i=0; i < activeSimBodies.size(); ++i){
        ODEBody* odeBody = static_cast<ODEBody*>(activeSimBodies[i]);
        odeBody->body()->setVirtualJointForces();
        if(odeBody->worldID){
            odeBody->setControlValToODE();
        }
    }

    if(MEASURE_PHYSICS_CALCULATION_TIME){
        physicsTimer.start();
    }

    dJointGroupEmpty(contactJointGroupID);

    if(useWorldCollisionDetector){
        bodyCollisionDetector.updatePositions(
            [](Referenced* object, Isometry3*& out_Position){
                out_Position = &(static_cast<ODELink*>(object)->link->position()); });
        
        bodyCollisionDetector.detectCollisions(
            [this](const CollisionPair& collisionPair){
                onCollisionPairDetected(collisionPair);
                return false; // Continue checking all collisions
            });
        
    } else {
        if(MEASURE_PHYSICS_CALCULATION_TIME){
            collisionTimer.start();
        }
        dSpaceCollide(spaceID, (void*)this, &nearCallback);
        for(auto& simBody : activeSimBodies){
            auto odeBody = static_cast<ODEBody*>(simBody);
            if(odeBody->spaceID && odeBody->selfCollision){
                dSpaceCollide(odeBody->spaceID, (void*)this, &nearCallback);
            }
        }
        if(MEASURE_PHYSICS_CALCULATION_TIME){
            collisionTime += collisionTimer.nsecsElapsed();
        }
    }

    if(stepMode.is(ODESimulatorItem::STEP_ITERATIVE)){
        dWorldQuickStep(worldID, timeStep);
    } else {
        dWorldStep(worldID, timeStep);
    }

    if(MEASURE_PHYSICS_CALCULATION_TIME){
        physicsTime += physicsTimer.nsecsElapsed();
    }

    //! \todo Bodies with sensors should be managed by the specialized container to increase the efficiency
    for(size_t i=0; i < activeSimBodies.size(); ++i){
        ODEBody* odeBody = static_cast<ODEBody*>(activeSimBodies[i]);

        if(odeBody->worldID){
            // Move the following code to the ODEBody class
            if(is2Dmode){
                odeBody->alignToZAxisIn2Dmode();
            }
            if(!odeBody->sensorHelper.forceSensors().empty()){
                odeBody->updateForceSensors(doFlipYZ);
            }
            odeBody->getKinematicStateFromODE(doFlipYZ);
            if(odeBody->sensorHelper.hasGyroOrAccelerationSensors()){
                odeBody->sensorHelper.updateGyroAndAccelerationSensors();
            }
        }
    }

    return true;
}


void ODESimulatorItemImpl::onCollisionPairDetected(const CollisionPair& collisionPair)
{
    ODELink* link1 = static_cast<ODELink*>(collisionPair.object(0));
    ODELink* link2 = static_cast<ODELink*>(collisionPair.object(1));
    const vector<Collision>& collisions = collisionPair.collisions();

    dBodyID body1ID = link1->bodyID;
    dBodyID body2ID = link2->bodyID;
    Link* crawlerlink = 0;
    double sign = 1.0;
    double motionSign = 1.0;
    if(!crawlerLinks.empty()){
        CrawlerLinkMap::iterator p = crawlerLinks.find(body1ID);
        if(p != crawlerLinks.end()){
            crawlerlink = p->second;
        }
        p = crawlerLinks.find(body2ID);
        if(p != crawlerLinks.end()){
            crawlerlink = p->second;
            sign = -1.0;
        }
    }
    if(!crawlerlink){
        if(isSurfaceVelocityLink(link1->link)){
            crawlerlink = link1->link;
            motionSign = -1.0;
        } else if(isSurfaceVelocityLink(link2->link)){
            crawlerlink = link2->link;
            sign = -1.0;
            motionSign = -1.0;
        }
    }

    auto param = resolveContactParam(link1->link, link2->link);

    int numContacts = collisions.size();
    for(int i=0; i < numContacts; ++i){
        dContact contact;
        contact.geom.pos[0] = collisions[i].point[0];
        contact.geom.pos[1] = collisions[i].point[1];
        contact.geom.pos[2] = collisions[i].point[2];
        contact.geom.normal[0] = -collisions[i].normal[0];
        contact.geom.normal[1] = -collisions[i].normal[1];
        contact.geom.normal[2] = -collisions[i].normal[2];
        contact.geom.depth = collisions[i].depth;

        dSurfaceParameters& surface = contact.surface;
        bool applySurfaceVelocity =
            crawlerlink &&
            (surfaceVelocityCullingDepth <= 0.0 || contact.geom.depth <= surfaceVelocityCullingDepth);
        if(!applySurfaceVelocity){
            surface.mode = dContactApprox1;
            surface.mu = param.friction;
            setBounceParameters(contact, param.restitution);
        } else {
            surface.mode = dContactFDir1 | dContactMotion1 | dContactMu2 | dContactApprox1_2;
            const Vector3 axis = crawlerlink->R() * crawlerlink->a();
            const Vector3 n(contact.geom.normal);
            Vector3 dir = axis.cross(n);
            if(dir.norm() < 1.0e-5){
                surface.mode = dContactApprox1;
                surface.mu = friction;
            } else {
                dir *= sign;
                dir.normalize();
                contact.fdir1[0] = dir[0];
                contact.fdir1[1] = dir[1];
                contact.fdir1[2] = dir[2];
                surface.motion1 = motionSign * crawlerlink->dq_target();
                surface.mu = friction;
                surface.mu2 = 0.5;
            }
        }
        dJointID jointID = dJointCreateContact(worldID, contactJointGroupID, &contact);
        dJointAttach(jointID, body1ID, body2ID);
    }
}


void ODESimulatorItem::finalizeSimulation()
{
    if(MEASURE_PHYSICS_CALCULATION_TIME){
        auto mout = MessageOut::master();
        mout->putln(formatC("ODE physicsTime= {}[s]", impl->physicsTime * 1.0e-9));
        mout->putln(formatC("ODE collisionTime= {}[s]", impl->collisionTime * 1.0e-9));
    }
}


void ODESimulatorItem::doPutProperties(PutPropertyFunction& putProperty)
{
    SimulatorItem::doPutProperties(putProperty);
    impl->doPutProperties(putProperty);
}


void ODESimulatorItemImpl::doPutProperties(PutPropertyFunction& putProperty)
{
    putProperty(_("Step mode"), stepMode, changeProperty(stepMode));

    putProperty(_("Gravity"), str(gravity), [&](const string& v){ return toVector3(v, gravity); });

    putProperty.decimals(2).min(0.0)(_("Friction"), friction, changeProperty(friction));

    putProperty(_("Limit joint range"), isJointLimitMode, changeProperty(isJointLimitMode));

    putProperty(_("Add joint-axis inertia to link inertia"),
                isAddingJointAxisInertiaToLinkInertia,
                changeProperty(isAddingJointAxisInertiaToLinkInertia));

    putProperty.decimals(1).range(0.0, 1.0);
    putProperty(_("Global ERP"), globalERP, changeProperty(globalERP));
    putProperty(_("Global CFM"), globalCFM,
                [&](const string& value){ return globalCFM.setNonNegativeValue(value); });

    putProperty.range(1, 999)(_("Iterations"), numIterations, changeProperty(numIterations));

    putProperty.range(0.1, 1.9)(_("Over relaxation"), overRelaxation, changeProperty(overRelaxation));

    putProperty(_("Limit correcting vel."), enableMaxCorrectingVel, changeProperty(enableMaxCorrectingVel));

    putProperty(_("Max correcting vel."), maxCorrectingVel,
                [&](const string& value){ return maxCorrectingVel.setNonNegativeValue(value); });

    putProperty.decimals(4).min(0.0)(
        _("Surface velocity culling depth"), surfaceVelocityCullingDepth, changeProperty(surfaceVelocityCullingDepth));

    putProperty(_("2D mode"), is2Dmode, changeProperty(is2Dmode));

    putProperty(_("Use WorldItem's Collision Detector"), useWorldCollisionDetector, changeProperty(useWorldCollisionDetector));
}


bool ODESimulatorItem::store(Archive& archive)
{
    SimulatorItem::store(archive);
    impl->store(archive);
    return true;
}


void ODESimulatorItemImpl::store(Archive& archive)
{
    archive.write("step_mode", stepMode.selectedSymbol());
    write(archive, "gravity", gravity);
    archive.write("friction", friction);
    if(isJointLimitMode){
        archive.write("enable_joint_limit", true);
    }
    if(isAddingJointAxisInertiaToLinkInertia){
        archive.write("add_joint_axis_inertia_to_link_inertia", true);
    }
    archive.write("global_erp", globalERP);
    archive.write("global_cfm", globalCFM);
    archive.write("num_iterations", numIterations);
    archive.write("over_relaxation", overRelaxation);
    archive.write("limit_correcting_velocity", enableMaxCorrectingVel);
    archive.write("max_correcting_velocity", maxCorrectingVel);
    archive.write("surface_velocity_culling_depth", surfaceVelocityCullingDepth);
    if(is2Dmode){
        archive.write("use_2d_mode", true);
    }
    if(useWorldCollisionDetector){
        archive.write("use_world_collision_detector", true);
    }
}


bool ODESimulatorItem::restore(const Archive& archive)
{
    SimulatorItem::restore(archive);
    impl->restore(archive);
    return true;
}


void ODESimulatorItemImpl::restore(const Archive& archive)
{
    string symbol;
    if(archive.read({ "step_mode", "stepMode" }, symbol)){
        stepMode.select(symbol);
    }
    read(archive, "gravity", gravity);
    archive.read("friction", friction);
    archive.read({ "enable_joint_limit", "jointLimitMode" }, isJointLimitMode);
    archive.read("add_joint_axis_inertia_to_link_inertia", isAddingJointAxisInertiaToLinkInertia);
    archive.read({ "global_erp", "globalERP" }, globalERP);
    globalCFM = archive.get({ "global_cfm", "globalCFM" }, globalCFM.string());
    archive.read({ "num_iterations", "numIterations" }, numIterations);
    archive.read({ "over_relaxation", "overRelaxation" }, overRelaxation);
    archive.read({ "limit_correcting_velocity", "limitCorrectingVel" }, enableMaxCorrectingVel);
    maxCorrectingVel = archive.get({ "max_correcting_velocity", "maxCorrectingVel" }, maxCorrectingVel.string());
    archive.read({ "surface_velocity_culling_depth", "surface_velocity_depth_limit" }, surfaceVelocityCullingDepth);
    archive.read({ "use_2d_mode", "2Dmode" }, is2Dmode);
    if(!archive.read({ "use_world_collision_detector", "useWorldCollisionDetector" }, useWorldCollisionDetector)){
        archive.read("UseWorldItem'sCollisionDetector", useWorldCollisionDetector);
    }
}
