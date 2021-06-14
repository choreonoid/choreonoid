/*!
  @file
  @author Shizuko Hattori
*/

#include "RokiSimulatorItem.h"
#include <roki/rk_fd.h>
#include <roki/rk_link.h>
#include <roki/rk_joint.h>
#include <roki/rk_motor.h>
#include <zeo/zeo.h>
#include <cnoid/ItemManager>
#include <cnoid/Archive>
#include <cnoid/EigenArchive>
#include <cnoid/Body>
#include <cnoid/Link>
#include <cnoid/Sensor>
#include <cnoid/BasicSensorSimulationHelper>
#include <cnoid/BodyItem>
#include <cnoid/FloatingNumberString>
#include <cnoid/EigenUtil>
#include <cnoid/MeshExtractor>
#include <cnoid/SceneDrawables>
#include "gettext.h"

#include <iostream>
#include <QElapsedTimer>

using namespace std;
using namespace std::placeholders;
using namespace cnoid;

extern "C" {
rkFDCell* _rkFDCellPush(rkFD* fd, rkFDCell* lc);

void _rkFDChainConnectJointState(rkFD *fd, zVec dis, zVec vel, zVec acc);
void _rkFDChainExtWrenchDestroy(rkFD *fd);
void _rkFDJointCalcFriction(rkFD *fd, bool doUpRef);
void _rkFDContactCalcForce(rkFD *fd);
void _rkFDContactModForce(rkFD *fd, bool doUpRef);
void _rkFDUpdateAcc(rkFD *fd);
void _rkFDUpdateRefDrivingTorque(rkFD *fd);

////// Torque motor was redefined for choreonoid
typedef struct {
  /* value */
  double t;  /* applied torque */

  /* properties */
  double Jm2;              /* rotorInertia */
} rkMotorPrpJm2;

static void _rkMotorSetInputJm2(void *prp, double *val);

static void _rkMotorInertiaJm2(void *prp, double *val);
static void _rkMotorInputTrqJm2(void *prp, double *val);
static void _rkMotorRegistanceJm2(void *prp, double *dis, double *vel, double *val);
static void _rkMotorDrivingTrqJm2(void *prp, double *dis, double *vel, double *acc, double *val);

static void _rkMotorStateCopyJm2(void *src, void *dst);

static bool _rkMotorQueryFReadJm2(FILE *fp, char *key, void *prp);
static void _rkMotorFWriteJm2(FILE *fp, void *prp);

#define _rkc(p) ((rkMotorPrpJm2 *)p)

void _rkMotorSetInputJm2(void *prp, double *val){
  //  _rkc(prp)->t = zLimit( *val, _rkc(prp)->min, _rkc(prp)->max );
  _rkc(prp)->t = *val;
}

void _rkMotorInertiaJm2(void *prp, double *val){
  *val =  _rkc(prp)->Jm2;
}

void _rkMotorInputTrqJm2(void *prp, double *val){
  *val = _rkc(prp)->t;
}

void _rkMotorRegistanceJm2(void *prp, double *dis, double *vel, double *val){
  *val = 0.0;
}
void _rkMotorDrivingTrqJm2(void *prp, double *dis, double *vel, double *acc, double *val){
    _rkMotorInputTrqJm2( prp, val);
}

void _rkMotorStateCopyJm2(void *src, void *dst){
  memcpy(dst, src, sizeof(rkMotorPrpJm2));
}

bool _rkMotorQueryFReadJm2(FILE *fp, char *key, void *prp)
{
  return true;
}

void _rkMotorFWriteJm2(FILE *fp, void *prp)
{
}

static rkMotorCom rk_motor_Jm2 = {
  1,
  _rkMotorSetInputJm2,
  _rkMotorInertiaJm2,
  _rkMotorInputTrqJm2,
  _rkMotorRegistanceJm2,
  _rkMotorDrivingTrqJm2,
  _rkMotorStateCopyJm2,
  _rkMotorQueryFReadJm2,
  _rkMotorFWriteJm2,
};

void _rkMotorInitPrpJm2(void *prp)
{
  _rkc(prp)->Jm2 = 0;
}

rkMotor *rkMotorCreateJm2(rkMotor *m)
{
  if( !( m->prp = zAlloc( rkMotorPrpJm2, 1 ) ) )
    return NULL;
  _rkMotorInitPrpJm2( m->prp );
  m->com = &rk_motor_Jm2;
  return m;
}

#undef _rkc
///////////////////////////////////////////
}

namespace {

struct Triangle {
    int indices[3];
};

class RokiBody;
class RokiLink : public Referenced
{
public:
    Link* link;
    rkLink* rklink;
    vector<zShape3D*> shapes;
    vector<Vector3> vertices;
    vector<Triangle> triangles;
    Matrix3 wAtt;  // world
    Matrix3 lAtt;  //parent
    Affine3 wFrame;
    Affine3 lFrame;
    RokiLink* parent;
    bool isCrawler;
    vector<rkCDCell*> cd_cells;
    bool breakJoint;

    RokiLink(RokiSimulatorItemImpl* simImpl, RokiBody* rokiBody, RokiLink* parent,
            const Vector3& parentOrigin, Link* link, bool stuffisLinkName);
    ~RokiLink();
    void calcFrame();
    void createLink(RokiSimulatorItemImpl* simImpl, RokiBody* body, const Vector3& origin, bool stuffisLinkName);
    void createGeometry();
    void addMesh(MeshExtractor* extractor);
    void getKinematicStateFromRoki();
    void setKinematicStateToRoki(zVec dis, int k);
    void setTorqueToRoki();
};
typedef ref_ptr<RokiLink> RokiLinkPtr;

typedef map<Link*, RokiLink*> RokiLinkMap;
class RokiBreakLinkTraverse;
class RokiBody : public SimulationBody
{
public:
    vector<RokiLinkPtr> rokiLinks;
    rkFDCell* lc;
    rkChain* chain;
    BasicSensorSimulationHelper sensorHelper;
    int geometryId;
    RokiLinkMap rokiLinkMap;
    vector<RokiBreakLinkTraverse> linkTraverseList;

    RokiBody(const Body& orgBody);
    ~RokiBody();
    void createBody(RokiSimulatorItemImpl* simImpl, bool stuffisLinkName);
    void getKinematicStateFromRoki();
    void setKinematicStateToRoki();
    void setTorqueToRoki();
    void updateForceSensors();

};

class RokiBreakLinkTraverse :public LinkTraverse
{
public:
    RokiBreakLinkTraverse(RokiLink* root, RokiBody* body)
        : rokiBody(body)
    {
        clear();
        breakLinkTraverse(root->link);
    }

    void breakLinkTraverse(Link* link){
        links.push_back(link);
        for(Link* child = link->child(); child; child = child->sibling()){
            if(!rokiBody->rokiLinkMap[child]->breakJoint){
                breakLinkTraverse(child);
            }
        }
    }
private:
    RokiBody* rokiBody;
};

}

namespace cnoid {

class RokiSimulatorItemImpl
{
public:
    RokiSimulatorItem* self;

    rkFD fd;
    double timeStep;
    bool createdFD;
    vector<RokiBody*> staticBodyList;
    vector<rkLink*> fixedLinkList;

    Selection fdSolverType;
    Selection contactType;
    double staticfriction;
    double kineticfriction;
    double compensation;
    double relaxation;
    double elasticity;
    double viscosity;
    bool useContactFile;
    string contactFileName;
    map<rkChain*, Body*> bodyMap;
    vector<RokiLink*> geometryIdToLink;

    double simulationTime;
    QElapsedTimer timer;

    RokiSimulatorItemImpl(RokiSimulatorItem* self);
    RokiSimulatorItemImpl(RokiSimulatorItem* self, const RokiSimulatorItemImpl& org);
    ~RokiSimulatorItemImpl();

    bool initializeSimulation(const std::vector<SimulationBody*>& simBodies);
    bool stepSimulation(const std::vector<SimulationBody*>& activeSimBodies);
    void addBody(RokiBody* simBody);
    CollisionLinkPairListPtr getCollisions();

    void doPutProperties(PutPropertyFunction& putProperty);
    bool store(Archive& archive);
    bool restore(const Archive& archive);
    //void collisionCallback(const CollisionPair& collisionPair, rkCD* cd);
};

enum{ FD_SOLVER_VERT, FD_SOLVER_VOLUME };
}


RokiLink::RokiLink
(RokiSimulatorItemImpl* simImpl, RokiBody* rokiBody, RokiLink* parent, const Vector3& parentOrigin, Link* link, bool stuffisLinkName)
{
    isCrawler = false;

    rokiBody->rokiLinks.push_back(this);
    rokiBody->rokiLinkMap[link] = this;

    this->link = link;
    this->parent = parent;

    Vector3 o = parentOrigin;
    if(!link->isRoot())
        o += link->b();
    else if(link->jointType()==Link::FIXED_JOINT)
        o = link->p();

    rklink = rkChainLink(rokiBody->chain, link->index());
    if(link->isRoot() && link->jointType()==Link::FIXED_JOINT)
        simImpl->fixedLinkList.push_back(rklink);

    createLink(simImpl, rokiBody, o, stuffisLinkName);
    rkChainMass(rokiBody->chain) += link->mass();

    createGeometry();

    for(Link* child = link->child(); child; child = child->sibling()){
        new RokiLink(simImpl, rokiBody, this, o, child, stuffisLinkName);
    }

}


void RokiLink::calcFrame()
{
    if(link->jointType()==Link::FIXED_JOINT || link->jointType()==Link::FREE_JOINT){
        wAtt = Matrix3::Identity();
        return;
    }

    Vector3 z(0,0,1);
    Vector3 nx = z.cross(link->a());

    if(nx.norm() < 1.0e-6){
        wAtt = Matrix3::Identity();
    }else{
        nx.normalize();
        wAtt.col(0) = nx;
        Vector3 ny = link->a().cross(nx);
        ny.normalize();
        wAtt.col(1) = ny;
        wAtt.col(2) = link->a();
    }
}


void RokiLink::createLink(RokiSimulatorItemImpl* simImpl, RokiBody* body, const Vector3& origin, bool stuffisLinkName)
{
    rkLinkInit(rklink);

    zNameSet( rklink, const_cast<char*>(link->name().c_str()) );
    string stuff = body->body()->name();
    if(stuffisLinkName)
        stuff +=  "_" + link->name();
    rkLinkSetStuff( rklink, const_cast<char*>(stuff.c_str()) );

    calcFrame();
    Matrix3 invwAtt = wAtt.transpose();

    rkLinkSetMass( rklink, link->mass() );

    zVec3D* v = rkLinkCOM(rklink);
    Vector3 c = invwAtt * link->c();
    zVec3DSetElem(v, 0, c.x());
    zVec3DSetElem(v, 1, c.y());
    zVec3DSetElem(v, 2, c.z());

    zMat3D* m = rkLinkInertia(rklink);
    const Matrix3 I = invwAtt * link->I() * wAtt;
    zMat3DSetElem(m, 0, 0, I(0,0));
    zMat3DSetElem(m, 0, 1, I(0,1));
    zMat3DSetElem(m, 0, 2, I(0,2));
    zMat3DSetElem(m, 1, 0, I(1,0));
    zMat3DSetElem(m, 1, 1, I(1,1));
    zMat3DSetElem(m, 1, 2, I(1,2));
    zMat3DSetElem(m, 2, 0, I(2,0));
    zMat3DSetElem(m, 2, 1, I(2,1));
    zMat3DSetElem(m, 2, 2, I(2,2));

    zFrame3D* f = rkLinkOrgFrame(rklink);
    if(link->jointType()==Link::FIXED_JOINT || (!link->isRoot() && link->jointType()==Link::FREE_JOINT)){
        wFrame = link->T();
    }else{
        wFrame.linear() = wAtt;
        wFrame.translation() = origin;
    }
    if(parent)
        lFrame = parent->wFrame.inverse() * wFrame;
    else
        lFrame = wFrame;

    zVec3D* p = &f->pos;
    zVec3DSetElem(p, 0, lFrame(0,3));
    zVec3DSetElem(p, 1, lFrame(1,3));
    zVec3DSetElem(p, 2, lFrame(2,3));

    zMat3D* att = &f->att;
    zMat3DSetElem(att, 0, 0, lFrame(0,0));
    zMat3DSetElem(att, 0, 1, lFrame(0,1));
    zMat3DSetElem(att, 0, 2, lFrame(0,2));
    zMat3DSetElem(att, 1, 0, lFrame(1,0));
    zMat3DSetElem(att, 1, 1, lFrame(1,1));
    zMat3DSetElem(att, 1, 2, lFrame(1,2));
    zMat3DSetElem(att, 2, 0, lFrame(2,0));
    zMat3DSetElem(att, 2, 1, lFrame(2,1));
    zMat3DSetElem(att, 2, 2, lFrame(2,2));

    bool dc_motor = false;
    breakJoint = false;
    double motorconstant, admitance, minvoltage, maxvoltage,
           inertia, gearinertia, ratio, compk,
           compl_, stiff, viscos, coulomb, staticfriction;
    motorconstant = admitance = minvoltage = maxvoltage
        = inertia = gearinertia = ratio = compk
        = compl_ = stiff = viscos = coulomb
        = staticfriction = std::numeric_limits<double>::max();
    const Mapping* jointParams = link->info();
    if(jointParams->isValid()){
        jointParams->read("rotorInertia", inertia);
        jointParams->read("gearRatio", ratio);
        dc_motor |= jointParams->read("gearInertia", gearinertia);
        dc_motor |= jointParams->read("motorAdmittance", admitance);
        dc_motor |= jointParams->read("motorConstant", motorconstant);
        dc_motor |= jointParams->read("motorMinVoltage", minvoltage);
        dc_motor |= jointParams->read("motorMaxVoltage", maxvoltage);
        dc_motor |= jointParams->read("jointStiffness", stiff);
        dc_motor |= jointParams->read("jointViscosity", viscos);
        dc_motor |= jointParams->read("jointFriction", coulomb);
        dc_motor |= jointParams->read("jointStaticFriction", staticfriction);
        dc_motor |= jointParams->read("compk", compk);
        dc_motor |= jointParams->read("compl", compl_);

        Vector2 breakParam;
        if(read(*jointParams, "break", breakParam)){
            rkLinkBreakPrp(rklink) = zAlloc( rkBreakPrp, 1 );
            rkLinkBreakPrp(rklink)->is_broken = false;
            rkLinkBreakPrp(rklink)->ep_f = breakParam[0];
            rkLinkBreakPrp(rklink)->ep_t = breakParam[1];
            breakJoint = true;
        }
    }

    rkMotor* rkMotor;
    rkJoint* joint = rkLinkJoint(rklink);
    switch(link->jointType()){
    case Link::ROTATIONAL_JOINT:
        rkJointCreate( joint, RK_JOINT_REVOL );
        rkJointGetMotor( joint, &rkMotor );
        if(!dc_motor && link->Jm2() != 0.0){
            rkMotorInit( rkMotor );
            rkMotorCreateJm2(rkMotor);
            ((rkMotorPrpJm2 *)rkMotor->prp)->Jm2 = link->Jm2();
        }else if(dc_motor){
            rkMotorCreate ( rkMotor, RK_MOTOR_DC );
            if(motorconstant!=std::numeric_limits<double>::max())
                ((rkMotorPrpDC *)rkMotor->prp)->k = motorconstant;
            if(admitance!=std::numeric_limits<double>::max())
                ((rkMotorPrpDC *)rkMotor->prp)->admit = admitance;
            if(minvoltage!=std::numeric_limits<double>::max())
                ((rkMotorPrpDC *)rkMotor->prp)->minvol = minvoltage;
            if(maxvoltage!=std::numeric_limits<double>::max())
                ((rkMotorPrpDC *)rkMotor->prp)->maxvol = maxvoltage;
            if(ratio!=std::numeric_limits<double>::max())
                ((rkMotorPrpDC *)rkMotor->prp)->decratio = ratio;
            if(inertia!=std::numeric_limits<double>::max())
                ((rkMotorPrpDC *)rkMotor->prp)->inertia = inertia;
            if(gearinertia!=std::numeric_limits<double>::max())
                ((rkMotorPrpDC *)rkMotor->prp)->inertia_gear = gearinertia;
            if(compk!=std::numeric_limits<double>::max())
                ((rkMotorPrpDC *)rkMotor->prp)->_comp_k = compk;
            if(compl_!=std::numeric_limits<double>::max())
                ((rkMotorPrpDC *)rkMotor->prp)->_comp_l = compl_;
            if(stiff!=std::numeric_limits<double>::max())
                ((rkJointPrpRevol *)joint->prp)->stiff = stiff;
            if(viscos!=std::numeric_limits<double>::max())
                ((rkJointPrpRevol *)joint->prp)->viscos = viscos;
            if(coulomb!=std::numeric_limits<double>::max())
                ((rkJointPrpRevol *)joint->prp)->coulomb = coulomb;
            if(staticfriction!=std::numeric_limits<double>::max())
                ((rkJointPrpRevol *)joint->prp)->sf = staticfriction;
        }else{
            rkMotorCreate ( rkMotor, RK_MOTOR_TRQ );
            ((rkMotorPrpTRQ *)rkMotor->prp)->max = std::numeric_limits<double>::max();
            ((rkMotorPrpTRQ *)rkMotor->prp)->min = -std::numeric_limits<double>::max();
        }
        break;
    case Link::SLIDE_JOINT:
        rkJointCreate( joint, RK_JOINT_PRISM );
        rkJointGetMotor( joint, &rkMotor );
        if(!dc_motor && link->Jm2() != 0.0){
            rkMotorInit( rkMotor );
            rkMotorCreateJm2(rkMotor);
            ((rkMotorPrpJm2 *)rkMotor->prp)->Jm2 = link->Jm2();
        }else if(dc_motor){
            rkMotorCreate ( rkMotor, RK_MOTOR_DC );
            if(motorconstant!=std::numeric_limits<double>::max())
                ((rkMotorPrpDC *)rkMotor->prp)->k = motorconstant;
            if(admitance!=std::numeric_limits<double>::max())
                ((rkMotorPrpDC *)rkMotor->prp)->admit = admitance;
            if(minvoltage!=std::numeric_limits<double>::max())
                ((rkMotorPrpDC *)rkMotor->prp)->minvol = minvoltage;
            if(maxvoltage!=std::numeric_limits<double>::max())
                ((rkMotorPrpDC *)rkMotor->prp)->maxvol = maxvoltage;
            if(ratio!=std::numeric_limits<double>::max())
                ((rkMotorPrpDC *)rkMotor->prp)->decratio = ratio;
            if(inertia!=std::numeric_limits<double>::max())
                ((rkMotorPrpDC *)rkMotor->prp)->inertia = inertia;
            if(gearinertia!=std::numeric_limits<double>::max())
                ((rkMotorPrpDC *)rkMotor->prp)->inertia_gear = gearinertia;
            if(compk!=std::numeric_limits<double>::max())
                ((rkMotorPrpDC *)rkMotor->prp)->_comp_k = compk;
            if(compl_!=std::numeric_limits<double>::max())
                ((rkMotorPrpDC *)rkMotor->prp)->_comp_l = compl_;
            if(stiff!=std::numeric_limits<double>::max())
                ((rkJointPrpRevol *)joint->prp)->stiff = stiff;
            if(viscos!=std::numeric_limits<double>::max())
                ((rkJointPrpRevol *)joint->prp)->viscos = viscos;
            if(coulomb!=std::numeric_limits<double>::max())
                ((rkJointPrpRevol *)joint->prp)->coulomb = coulomb;
            if(staticfriction!=std::numeric_limits<double>::max())
                ((rkJointPrpRevol *)joint->prp)->sf = staticfriction;
        }else{
            rkMotorCreate ( rkMotor, RK_MOTOR_TRQ );
            ((rkMotorPrpTRQ *)rkMotor->prp)->max = std::numeric_limits<double>::max();
            ((rkMotorPrpTRQ *)rkMotor->prp)->min = -std::numeric_limits<double>::max();
        }
        break;
    case Link::FIXED_JOINT:
    default :
        rkJointCreate( joint, RK_JOINT_FIXED );
        break;
    case Link::FREE_JOINT:
        rkJointCreate( joint, RK_JOINT_FLOAT );
        break;
    }

    if(link->actuationMode() == Link::JOINT_SURFACE_VELOCITY){
        isCrawler = true;
    }

    if(parent)
        rkLinkAddChild( parent->rklink, rklink);
}


void RokiLink::createGeometry()
{
    if(link->collisionShape()){
        MeshExtractor* extractor = new MeshExtractor;
        if(extractor->extract(link->collisionShape(), [&](){  addMesh(extractor); } )){
            if(!vertices.empty()){
                zShape3D* sp = zAlloc( zShape3D, 1 );
                zShape3DInit(sp);
                zShape3DType(sp) = zShapeTypeByStr((char*)"polyhedron");
                zNameSet( sp, const_cast<char*>(link->name().c_str()) );
                sp->com = &zprim_ph3d_com;

                zPH3D* ph = (zPH3D*)&sp->body;
                zPH3DInit( ph );
                int vc = vertices.size();
                int fc = triangles.size();
                zPH3DAlloc( ph, vc, fc );
                for(int i=0; i<vc; i++){
                    zVec3D* v = zPH3DVert(ph, i);
                    zVec3DSetElem( v, zX, vertices[i].x() );
                    zVec3DSetElem( v, zY, vertices[i].y() );
                    zVec3DSetElem( v, zZ, vertices[i].z() );
                }
                for(int i=0; i<fc; i++){
                    zTri3DCreate( zPH3DFace(ph, i), zPH3DVert(ph, triangles[i].indices[0]),
                            zPH3DVert(ph, triangles[i].indices[1]), zPH3DVert(ph, triangles[i].indices[2]) );
                }
                shapes.push_back(sp);
                zAABox3D aabb;
                zAABB( &aabb, zShape3DVertBuf(sp), zShape3DVertNum(sp), NULL );
                zAABox3DToBox3D( &aabb, zShape3DBB(sp) );
                rkLinkShapePush( rklink, sp );
            }
        }
        delete extractor;
    }

}


void RokiLink::addMesh(MeshExtractor* extractor)
{
    SgMesh* mesh = extractor->currentMesh();
    const Affine3& T = extractor->currentTransform();

    bool meshAdded = false;
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
                } else if(mesh->primitiveType() == SgMesh::CONE){
                    if(scale.x() == scale.z()){
                        doAddPrimitive = true;
                    }
                }
            }
        }
        if(doAddPrimitive){
            bool created = false;
            Affine3 T_ = extractor->currentTransformWithoutScaling();
            if(translation){
                T_ *= Translation3(*translation);
            }
            Affine3 invT = Affine3::Identity();
            invT.linear() = wAtt.transpose();
            const Affine3 T0 = invT * T;
            Vector3 p = T0.translation();

            zShape3D* sp;
            switch(mesh->primitiveType()){
            case SgMesh::BOX : {
                sp = zAlloc( zShape3D, 1 );
                const Vector3& s = mesh->primitive<SgMesh::Box>().size;
                zVec3D zp = { { p.x(), p.y(), p.z() } };
                zVec3D ax = { { T0(0,0), T0(1,0), T0(2,0) } };
                zVec3D ay = { { T0(0,1), T0(1,1), T0(2,1) } };
                zVec3D az = { { T0(0,2), T0(1,2), T0(2,2) } };
                zShape3DCreateBox( sp, &zp, &ax, &ay, &az,
                        s.x()* scale.x(), s.y()* scale.y(), s.z()* scale.z());
                created = true;
                break; }
            case SgMesh::SPHERE : {
                sp = zAlloc( zShape3D, 1 );
                SgMesh::Sphere sphere = mesh->primitive<SgMesh::Sphere>();
                zVec3D zp = { { p.x(), p.y(), p.z() } };
                zShape3DCreateSphere( sp, &zp, sphere.radius* scale.x(), 0 );
                created = true;
                break; }
            case SgMesh::CYLINDER : {
                sp = zAlloc( zShape3D, 1 );
                SgMesh::Cylinder cylinder = mesh->primitive<SgMesh::Cylinder>();
                Vector3 ay(T_(0,1), T_(1,1), T_(2,1));
                ay *= cylinder.height/2.0 * scale.y();
                Vector3 c1_ = T0 * ay.cast<Position::Scalar>();
                Vector3 c2_ = T0 * (-ay).cast<Position::Scalar>();
                zVec3D c1 = { { c1_.x(), c1_.y(), c1_.z() } };
                zVec3D c2 = { { c2_.x(), c2_.y(), c2_.z() } };
                zShape3DCreateCyl( sp, &c1, &c2, cylinder.radius * scale.x(), 0);
                created = true;
                break; }
            case SgMesh::CONE : {
                sp = zAlloc( zShape3D, 1 );
                SgMesh::Cone cone = mesh->primitive<SgMesh::Cone>();
                Vector3 ay(T_(0,1), T_(1,1), T_(2,1));
                ay *= cone.height/2.0 * scale.y();
                Vector3 c1_ = T0 * ay.cast<Position::Scalar>();
                Vector3 c2_ = T0 * (-ay).cast<Position::Scalar>();
                zVec3D c1 = { { c1_.x(), c1_.y(), c1_.z() } };
                zVec3D c2 = { { c2_.x(), c2_.y(), c2_.z() } };
                zShape3DCreateCone( sp, &c1, &c2, cone.radius * scale.x(), 0);
                created = true;
                break; }
            default :
                break;
            }
            if(created){
                shapes.push_back(sp);
                zBox3DInit( zShape3DBB(sp) );
                zNameSet( sp, const_cast<char*>(link->name().c_str()) );
                rkLinkShapePush( rklink, sp );
                meshAdded = true;
            }
        }
    }

    if(!meshAdded){
        const int vertexIndexTop = vertices.size();

        Affine3 invT = Affine3::Identity();
        invT.linear() = wAtt.transpose();
        const Affine3 T0 = invT * T;

        const SgVertexArray& vertices_ = *mesh->vertices();
        const int numVertices = vertices_.size();
        for(int i=0; i < numVertices; ++i){
            const Vector3 v = T0 * vertices_[i].cast<Position::Scalar>();
            vertices.push_back(v);
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


RokiLink::~RokiLink()
{
    for(size_t i=0; i<shapes.size(); i++)
        zShape3DDestroy(shapes[i]);
}


void RokiLink::getKinematicStateFromRoki()
{
    double dis[6];
    double v[6];
    rkLinkGetJointDis( rklink, dis );
    rkLinkGetJointVel( rklink, v);

    switch(link->jointType()){
        case Link::ROTATIONAL_JOINT:
        case Link::SLIDE_JOINT:
            link->q() = dis[0];
            link->dq() = v[0];
            break;
        case Link::FREE_JOINT:{
            Vector3 aa(dis[3], dis[4], dis[5]);
            double angle = aa.norm();
            Matrix3 R;
            if(angle==0){
                R.setIdentity();
            }else{
                R = AngleAxisd(angle, aa/angle);
            }

            if(breakJoint){
                Affine3 lT,wT;
                wT = parent->link->T() * lFrame;
                if(rkLinkBreakPrp(rklink)->is_broken){
                    lT.translation() = Vector3(dis[0], dis[1], dis[2]);
                    lT.linear() = R;
                    wT = wT * lT;
                }
                link->p() = wT.translation();
                link->R() = wT.linear();
            }else{
                link->p() = Vector3(dis[0], dis[1], dis[2]);
                link->R() = R;
                link->v() = Vector3(v[0], v[1], v[2]);
                link->w() = Vector3(v[3], v[4], v[5]);
            }
            break;
        }
        default:
            break;
    }
}


void RokiLink::setKinematicStateToRoki(zVec dis, int k)
{
    switch(link->jointType()){
        case Link::ROTATIONAL_JOINT:
            zVecElem(dis,k) = link->q();
            break;
        case Link::SLIDE_JOINT:
            zVecElem(dis,k) = link->q();
            break;
        case Link::FREE_JOINT:{
            if(parent)
                break;
            zVecElem(dis,k) = link->p().x();
            zVecElem(dis,k+1) = link->p().y();
            zVecElem(dis,k+2) = link->p().z();
            zMat3D m;
            const Matrix3& R = link->R();
            zMat3DCreate( &m, R(0,0), R(0,1), R(0,2),
                              R(1,0), R(1,1), R(1,2),
                              R(2,0), R(2,1), R(2,2) );
            zMat3DToAA( &m, (zVec3D *)&zVecElem(dis,3) );
            break;
        }
        default:
            break;
    }
}


void RokiLink::setTorqueToRoki()
{
    if(isCrawler){
        for(int i=0; i<cd_cells.size(); i++)
            rkFDCDCellSetSlideVel( cd_cells[i], link->dq_target() );
    } else {
        rkJointMotorSetInput( rkLinkJoint(rklink), &link->u() );
    }
}


RokiBody::RokiBody(const Body& orgBody)
    : SimulationBody(new Body(orgBody))
{
    Body* body = this->body();
    rokiLinkMap.clear();
    linkTraverseList.clear();
}


RokiBody::~RokiBody()
{
    rokiLinks.clear();
    rokiLinkMap.clear();
    linkTraverseList.clear();
    rkChainDestroy( chain );
}


void RokiBody::createBody(RokiSimulatorItemImpl* simImpl, bool stuffisLinkName)
{
    Body* body = this->body();

    lc = zAlloc( rkFDCell, 1 );
    chain = &lc->data.chain;

    rkChainInit( chain );
    zNameSet(chain, const_cast<char*>(body->name().c_str()));
    zArrayAlloc( &chain->link, rkLink, body->numLinks() );

    RokiLink* rootLink = new RokiLink(simImpl, this, 0, Vector3::Zero(), body->rootLink(), stuffisLinkName);

    for(int i=0; i<rokiLinks.size(); i++){
        if(rokiLinks[i]->link->isRoot() || rokiLinks[i]->breakJoint){
            RokiBreakLinkTraverse linkTraverse(rokiLinks[i].get(), this);
            linkTraverseList.push_back(linkTraverse);
        }
    }

    rkChainSetOffset( chain ); /* offset value arrangement */
    rkChainUpdateFK( chain );
    rkChainUpdateID( chain );


    int size = rkChainJointSize(chain);
    if(size){
        _rkFDCellPush( &simImpl->fd, lc );

        for(int i=0; i<rokiLinks.size(); i++){
            RokiLink* rokiLink = rokiLinks[i].get();
            if(rokiLink->isCrawler){
                for(int j=0; j<rokiLink->shapes.size(); j++){
                    rkCDCell* cd_cell =  rkFDShape3DGetCDCell( &simImpl->fd, rokiLink->shapes[j]);
                    rokiLink->cd_cells.push_back(cd_cell);
                    rkFDCDCellSetSlideMode( cd_cell, true );
                    zVec3D za = { { 0, 0, 1 } };
                    rkFDCDCellSetSlideAxis( cd_cell, &za );
                }
            }
        }
        
        setKinematicStateToRoki();
    }else{
        simImpl->staticBodyList.push_back(this);
    }

    // Do not check for self collisions
    if(!bodyItem()->isSelfCollisionDetectionEnabled())
        rkCDPairChainUnreg( &simImpl->fd.cd, &lc->data.chain );

    sensorHelper.initialize(body, simImpl->timeStep, Vector3(0,0,-9.80665));

}


void RokiBody::getKinematicStateFromRoki()
{
    for(size_t i=0; i < rokiLinks.size(); ++i){
        rokiLinks[i]->getKinematicStateFromRoki();
    }

    for(int i=0; i<linkTraverseList.size(); i++){
        linkTraverseList[i].calcForwardKinematics(true, false);
    }
}


void RokiBody::setKinematicStateToRoki()
{
    int size = rkChainJointSize(chain);
    zVec dis = zVecAlloc(size);
    for(size_t i=0; i < rokiLinks.size(); ++i){
        int j = rokiLinks[i]->link->index();
        int k=rkChainLinkOffset(chain, j);
        rokiLinks[i]->setKinematicStateToRoki(dis, k);
    }
    rkFDChainSetDis( lc, dis );
    zVecFree(dis);
}


void RokiBody::setTorqueToRoki()
{
    for(size_t i=1; i < rokiLinks.size(); ++i){
        rokiLinks[i]->setTorqueToRoki();
    }
}


void RokiBody::updateForceSensors()
{
    const DeviceList<ForceSensor>& forceSensors = sensorHelper.forceSensors();
    for(int i=0; i < forceSensors.size(); ++i){
        ForceSensor* sensor = forceSensors[i];
        const Link* link = sensor->link();
        const RokiLink* rokiLink = rokiLinks[link->index()];
        zVec6D* wrnch = rkLinkWrench(rokiLink->rklink);

        Vector3 f, tau;
        f[0] = zVec6DElem(wrnch, 0);
        f[1] = zVec6DElem(wrnch, 1);
        f[2] = zVec6DElem(wrnch, 2);
        tau[0] = zVec6DElem(wrnch, 3);
        tau[1] = zVec6DElem(wrnch, 4);
        tau[2] = zVec6DElem(wrnch, 5);

        Vector3 f0 = rokiLink->wAtt * -f;
        Vector3 tau0 = rokiLink->wAtt * -tau;
        const Matrix3 R = sensor->R_local();
        const Vector3 p = sensor->p_local();

        sensor->f()   = R.transpose() * f0;
        sensor->tau() = R.transpose() * (tau0 - p.cross(f0));
        sensor->notifyStateChange();
    }
}


void RokiSimulatorItem::initializeClass(ExtensionManager* ext)
{
    ext->itemManager().registerClass<RokiSimulatorItem, SimulatorItem>(N_("RokiSimulatorItem"));
    ext->itemManager().addCreationPanel<RokiSimulatorItem>();
}


RokiSimulatorItem::RokiSimulatorItem()
{
    impl = new RokiSimulatorItemImpl(this);
}
 

RokiSimulatorItemImpl::RokiSimulatorItemImpl(RokiSimulatorItem* self)
    : self(self)
{
    createdFD = false;

    contactType.setSymbol( RK_CONTACT_RIGID,  N_("contact rigid"));
    contactType.setSymbol( RK_CONTACT_ELASTIC,  N_("contact elastic"));
    contactType.select(RK_CONTACT_RIGID);
    fdSolverType.setSymbol( FD_SOLVER_VERT, N_("Vert"));
    fdSolverType.setSymbol( FD_SOLVER_VOLUME, N_("Volume"));
    fdSolverType.select(FD_SOLVER_VOLUME);
    staticfriction = 0.5;
    kineticfriction = 0.3;;
    compensation = 1000.0;
    relaxation = 1.0;
    elasticity = 0.0;
    viscosity = 0.0;
    useContactFile = false;
    contactFileName.clear();

}


RokiSimulatorItem::RokiSimulatorItem(const RokiSimulatorItem& org)
    : SimulatorItem(org),
      impl(new RokiSimulatorItemImpl(this, *org.impl))
{

}


RokiSimulatorItemImpl::RokiSimulatorItemImpl(RokiSimulatorItem* self, const RokiSimulatorItemImpl& org)
    : self(self)
{
    createdFD = false;

    contactType = org.contactType;
    fdSolverType = org.fdSolverType;
    staticfriction = org.staticfriction;
    kineticfriction = org.kineticfriction;
    compensation = org.compensation;
    relaxation = org.relaxation;
    elasticity = org.elasticity;
    viscosity = org.viscosity;
    useContactFile = org.useContactFile;
    contactFileName = org.contactFileName;

}


RokiSimulatorItem::~RokiSimulatorItem()
{
    delete impl;
}


RokiSimulatorItemImpl::~RokiSimulatorItemImpl()
{
    if(createdFD){
        rkFDUpdateDestroy( &fd );
        rkFDDestroy( &fd );
    }
}


Item* RokiSimulatorItem::doDuplicate() const
{
    return new RokiSimulatorItem(*this);
}


SimulationBody* RokiSimulatorItem::createSimulationBody(Body* orgBody)
{
    return new RokiBody(*orgBody);
}


bool RokiSimulatorItem::initializeSimulation(const std::vector<SimulationBody*>& simBodies)
{
    return impl->initializeSimulation(simBodies);
}


bool RokiSimulatorItemImpl::initializeSimulation(const std::vector<SimulationBody*>& simBodies)
{
    bodyMap.clear();
    staticBodyList.clear();
    fixedLinkList.clear();

    if(createdFD){
        rkFDUpdateDestroy( &fd );
        rkFDDestroy( &fd );
    }

    rkFDCreate( &fd );
    createdFD = true;

    if(useContactFile){
        const char* fileName = getNativePathString(contactFileName).c_str();
        rkFDContactInfoReadFile( &fd, const_cast<char*>(fileName) );
    }else{
        int num = simBodies.size();
        zArrayAlloc( &fd.ci, rkContactInfo, num*(num+1)/2);
        for(int i=0, k=0; i<num; i++){
            for(int j=i; j<num; j++){
                rkContactInfo* info = zArrayElem( &fd.ci, k++ );
                rkContactInfoInit(info);
                info->__stf[0] = zStrClone(const_cast<char*>(simBodies[i]->body()->name().c_str()));
                info->__stf[1] = zStrClone(const_cast<char*>(simBodies[j]->body()->name().c_str()));
                rkContactInfoSetSF( info, staticfriction );
                rkContactInfoSetKF( info, kineticfriction );
                if(contactType.selectedIndex() == RK_CONTACT_ELASTIC){
                    rkContactInfoSetE( info, elasticity );
                    rkContactInfoSetType( info, RK_CONTACT_ELASTIC );
                    rkContactInfoSetV( info, viscosity );
                }else{
                    rkContactInfoSetK( info, compensation );
                    rkContactInfoSetL( info, relaxation );
                    rkContactInfoSetType( info, RK_CONTACT_RIGID );
                }
            }
        }
    }

    for(size_t i=0; i < simBodies.size(); ++i){
        addBody(static_cast<RokiBody*>(simBodies[i]));
    }

    for(size_t i=0; i< staticBodyList.size(); i++){
        _rkFDCellPush( &fd, staticBodyList[i]->lc );
    }

    if(fixedLinkList.size()>1){
        for(int i=0; i<fixedLinkList.size()-1; i++)
            for(int j=i+1; j<fixedLinkList.size(); j++)
                rkCDPairUnreg(&fd.cd, fixedLinkList[i], fixedLinkList[j]);
    }

    rkFDODE2Assign( &fd, Regular );
    rkFDODE2AssignRegular( &fd, RKG );
    timeStep = self->worldTimeStep();
    rkFDSetDT( &fd, timeStep );
    if(fdSolverType.selectedIndex() == FD_SOLVER_VOLUME)
        rkFDSetSolver( &fd, Volume );
    else
        rkFDSetSolver( &fd, Vert );

    rkFDUpdateInit( &fd );

    simulationTime = 0;

    return true;
}


void RokiSimulatorItemImpl::addBody(RokiBody* rokiBody)
{
    Body& body = *rokiBody->body();

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

    rokiBody->createBody(this, useContactFile);

    bodyMap[rokiBody->chain] = rokiBody->body();
}


bool RokiSimulatorItem::stepSimulation(const std::vector<SimulationBody*>& activeSimBodies)
{
    return impl->stepSimulation(activeSimBodies);
}


bool RokiSimulatorItemImpl::stepSimulation(const std::vector<SimulationBody*>& activeSimBodies)
{
    for(size_t i=0; i < activeSimBodies.size(); ++i){
        RokiBody* rokiBody = static_cast<RokiBody*>(activeSimBodies[i]);
        rokiBody->setTorqueToRoki();
    }

    timer.start();
    rkFDUpdate( &fd );
    simulationTime += timer.nsecsElapsed();


    for(size_t i=0; i < activeSimBodies.size(); i++){
        RokiBody* rokiBody = static_cast<RokiBody*>(activeSimBodies[i]);

        rokiBody->getKinematicStateFromRoki();
        if(!rokiBody->sensorHelper.forceSensors().empty()){
            rokiBody->updateForceSensors();
        }
        if(rokiBody->sensorHelper.hasGyroOrAccelerationSensors()){
            rokiBody->sensorHelper.updateGyroAndAccelerationSensors();
        }
    }

    return true;
}


void RokiSimulatorItem::finalizeSimulation()
{
    cout << "Roki simulationTime= " << impl->simulationTime *1.0e-9 << "[s]"<< endl;
}


CollisionLinkPairListPtr RokiSimulatorItem::getCollisions()
{
    return impl->getCollisions();
}


CollisionLinkPairListPtr RokiSimulatorItemImpl::getCollisions()
{
    CollisionLinkPairListPtr collisionPairs = std::make_shared<CollisionLinkPairList>();
    rkCDPair *cdp;
    zListForEach( &fd.cd.plist, cdp ){
        if( !cdp->data.is_col ) continue;
        CollisionLinkPairPtr dest = std::make_shared<CollisionLinkPair>();
        for(int j=0; j<2; j++){
            dest->body[j] = bodyMap[cdp->data.cell[j]->data.chain];
            dest->link[j] = dest->body[j]->link(zName(cdp->data.cell[j]->data.shape));
        }
        rkCDVert *v;
        zListForEach( &cdp->data.vlist, v ){
            dest->collisions.push_back(Collision());
            Collision& col = dest->collisions.back();
            for(int i=0; i<3; i++)
                col.point[i] = zVec3DElem(v->data.vert, i);
            for(int i=0; i<3; i++)
                col.normal[i] = -zVec3DElem(&v->data.norm, i);
            Vector3 pro;
            for(int i=0; i<3; i++)
                pro[i] = (&v->data.pro)->e[i];
            col.depth = (col.point - pro).norm();
        }
        collisionPairs->push_back(dest);
    }
    return collisionPairs;
}

#if 0
void RokiSimulatorItemImpl::collisionCallback(const CollisionPair& collisionPair, rkCD* cd)
{
    RokiLink* link1 = geometryIdToLink[collisionPair.geometryId[0]];
    RokiLink* link2 = geometryIdToLink[collisionPair.geometryId[1]];
    const vector<Collision>& collisions = collisionPair.collisions;

    rkCDPair *cp;
    zListForEach( &cd->plist, cp )
        if( ( cp->data.cell[0]->data.link == link1->rklink &&
              cp->data.cell[1]->data.link == link2->rklink ) ||
            ( cp->data.cell[0]->data.link == link2->rklink &&
              cp->data.cell[1]->data.link == link1->rklink ) )
            break;

    cp->data.is_col = true;
    zListDestroy( rkCDVert, &cp->data.vlist );
    cd->colnum++;

    for(size_t i=0; i<collisions.size(); i++){
        const Collision& col = collisions[i];
        rkCDVert * v = zAlloc( rkCDVert, 1 );
        v->data.vert = zAlloc( zVec3D, 1);
        for(int j=0; j<3; j++)
            zVec3DSetElem(v->data.vert, j, col.point[j]);
        for(int j=0; j<3; j++)
            zVec3DSetElem(&v->data.norm, j, col.normal[j]);
        zListInsertHead( &cp->data.vlist, v );
    }

}
#endif

void RokiSimulatorItem::doPutProperties(PutPropertyFunction& putProperty)
{
    SimulatorItem::doPutProperties(putProperty);
    impl->doPutProperties(putProperty);
}


void RokiSimulatorItemImpl::doPutProperties(PutPropertyFunction& putProperty)
{
    putProperty.decimals(2).min(0.0)
            (_("Static Friction"), staticfriction, changeProperty(staticfriction));
    putProperty.decimals(2).min(0.0)
            (_("Kinetic Friction"), kineticfriction, changeProperty(kineticfriction));

    putProperty(_("Contact TYpe"), contactType, changeProperty(contactType));
    putProperty(_("Solver TYpe"), fdSolverType, changeProperty(fdSolverType));
    putProperty.min(0.0)
            (_("Compensation"), compensation, changeProperty(compensation));
    putProperty.decimals(4).min(0.0)
            (_("Relaxation"), relaxation, changeProperty(relaxation));
    putProperty.decimals(2).min(0.0)
            (_("Elasticity"), elasticity, changeProperty(elasticity));
    putProperty.decimals(2).min(0.0)
            (_("Viscosity"), viscosity, changeProperty(viscosity));
    putProperty(_("Use Contact Configuration file"), useContactFile, changeProperty(useContactFile));
    putProperty(_("Contact Configuration file name"), contactFileName, changeProperty(contactFileName));
}


bool RokiSimulatorItem::store(Archive& archive)
{
    SimulatorItem::store(archive);
    return impl->store(archive);
}


bool RokiSimulatorItemImpl::store(Archive& archive)
{
    archive.write("staticfriction", staticfriction);
    archive.write("kineticfriction", kineticfriction);
    archive.write("contactType", contactType.selectedSymbol());
    archive.write("solverType", fdSolverType.selectedSymbol());
    archive.write("compensation", compensation);
    archive.write("relaxation", relaxation);
    archive.write("elasticity", elasticity);
    archive.write("viscosity", viscosity);
    archive.write("useContactFile", useContactFile);
    archive.writeRelocatablePath("contactFileName", contactFileName);
    return true;
}


bool RokiSimulatorItem::restore(const Archive& archive)
{
    SimulatorItem::restore(archive);
    return impl->restore(archive);
}


bool RokiSimulatorItemImpl::restore(const Archive& archive)
{
    archive.read("staticfriction", staticfriction);
    archive.read("kineticfriction", kineticfriction);
    string symbol;
    if(archive.read("contactType", symbol)){
            contactType.select(symbol);
    }
    if(archive.read("solverType", symbol)){
               fdSolverType.select(symbol);
    }
    archive.read("compensation", compensation);
    archive.read("relaxation", relaxation);
    archive.read("elasticity", elasticity);
    archive.read("viscosity", viscosity);
    archive.read("useContactFile", useContactFile);
    archive.readRelocatablePath("contactFileName", contactFileName);
    return true;
}


