/*!
  @file
  @author Shizuko Hattori
*/
#include "AgXSimulatorItem.h"
#include <agxSDK/Simulation.h>
#include <agx/RigidBody.h>
#include <agx/Thread.h>
#include <agx/UniformGravityField.h>
#include <agxCollide/Box.h>
#include <agxCollide/Sphere.h>
#include <agxCollide/Cylinder.h>
#include <agxCollide/Trimesh.h>
#include <agx/Hinge.h>
#include <agx/Prismatic.h>
#include <agx/LockJoint.h>
#include <agx/BallJoint.h>
#include <agx/ForceField.h>
#include <agxSDK/GeometryFilter.h>
#include <agx/ConstraintImplementation.h>
#include <agxDriveTrain/Gear.h>
#include <agxDriveTrain/Shaft.h>
#include <agxPowerLine/TranslationalUnit.h>
#include <agxUtil/Convert/Convert.h>
#include <agx/PrismaticUniversalJoint.h>
#include <agxPowerLine/Actuator1DOF.h>
#include <agx/version.h>
#include <cnoid/ItemManager>
#include <cnoid/Archive>
#include <cnoid/EigenUtil>
#include <cnoid/EigenArchive>
#include <cnoid/MeshExtractor>
#include <cnoid/SceneDrawables>
#include <cnoid/Body>
#include <cnoid/Link>
#include <cnoid/Sensor>
#include <cnoid/BasicSensorSimulationHelper>
#include <cnoid/BodyItem>
#include <cnoid/ControllerItem>
#include <cnoid/BodyMotionItem>
#include <boost/bind.hpp>
#include "gettext.h"

using namespace std;
using namespace cnoid;


namespace {

const double DEFAULT_GRAVITY_ACCELERATION = 9.80665;

class AgXBody;

class AgXLink : public Referenced
{
public :
    AgXSimulatorItemImpl* simImpl;
    AgXBody* agxBody;
    Link* link;
    AgXLink* parent;
    agx::RigidBodyRef agxRigidBody;
    agx::Constraint* joint;
    agx::Vec3Vector vertices;
    agx::UInt32Vector indices;
    Vector3 origin;
    vector<AgXLink*> constraintLinks;
    int customConstraintIndex;
    agxPowerLine::Unit* unit;
    bool isHRP2;
    AgXSimulatorItem::ControlMode controlMode;
    enum InputMode { NON, VEL, TOR };
    InputMode inputMode;

    AgXLink(AgXSimulatorItemImpl* simImpl, AgXBody* agxXBody, AgXLink* parent,
                  const Vector3& parentOrigin, Link* link);
    ~AgXLink();
    void createLinkBody(bool isStatic);
    void createJoint();
    void createGeometry(AgXBody* agxBody);
    void addMesh(MeshExtractor* extractor, AgXBody* agxBody);
    void setKinematicStateToAgX();
    void getKinematicStateFromAgX();
    void setTorqueToAgX();
};
typedef ref_ptr<AgXLink> AgXLinkPtr;


class AgXLinkContainer : public agx::Referenced
{
public :
    AgXLinkContainer(AgXLink* agxLink) : agxLink(agxLink) { }
    AgXLink* agxLink;
};


class AgXBody :  public SimulationBody
{
public :
    AgXSimulatorItemImpl* simImpl;
    bool isStatic;
    bool collisionDetectionEnabled;
    bool selfCollisionDetectionEnabled;
    int geometryGroupId;
    vector<AgXLinkPtr> agxLinks;
    BasicSensorSimulationHelper sensorHelper;

    AgXBody(Body& orgBody);
    ~AgXBody();

    void createBody(AgXSimulatorItemImpl* simImpl);
    void setKinematicStateToAgX();
    void getKinematicStateFromAgX();
    void setExtraJoints();
    AgXLink* findLink(Link* link);
    void updateForceSensors();
    void setTorqueToAgX();
};


class AgXForceField : public agx::ForceField
{
public:
  AgXForceField(){ };
  void updateForce(agx::DynamicsSystem *);
};


class AgxContactEventListener : public agxSDK::ContactEventListener
{
public:
    AgxContactEventListener( AgXSimulatorItemImpl* impl )
    : impl( impl )
    {
        setMask( ALL );
    }
    virtual KeepContactPolicy impact(const agx::TimeStamp&, agxCollide::GeometryContact*);
    virtual KeepContactPolicy contact( const agx::TimeStamp&, agxCollide::GeometryContact*);
    virtual void separation(const agx::TimeStamp& , agxCollide::GeometryPair&  );
private:
    AgXSimulatorItemImpl* impl;
};

class CrawlerGeometry : public agxCollide::Geometry
{
public :
    CrawlerGeometry(Link* link, agx::RigidBody* agxBody) : link(link), agxBody(agxBody)
    {
        const Vector3& a = link->a();
        axis.set( a(0), a(1), a(2) );
     }

#if defined(AGX_MAJOR_VERSION) && AGX_MAJOR_VERSION < 14
    virtual agx::Vec3f calculateSurfaceVelocity( const agxCollide::LocalContactPoint& point ) const
#else
    virtual agx::Vec3f calculateSurfaceVelocity( const agxCollide::LocalContactPoint& point , size_t index ) const
#endif
    {
        agx::Vec3 axis0 = agxBody->getFrame()->transformVectorToWorld( axis );
        agx::Vec3 n(point.normal());
        agx::Vec3 dir = axis0.cross(n);
        if(dir.length() < 1e-5)
            return agx::Vec3f( 0.0, 0.0, 0.0 );
        dir.normalize();
        dir *= -link->u();
        agx::Vec3 ret = agxBody->getFrame()->transformVectorToLocal( dir );
        return agx::Vec3f(ret);
    }
private :
    Link* link;
    agx::RigidBody* agxBody;
    agx::Vec3 axis;
};


bool createFunc( agx::HighLevelConstraintImplementation* implementation );
class CustomConstraintImplementation : public agx::HighLevelConstraintImplementation
{
public:
    CustomConstraintImplementation(AgXLink* parent, vector<AgXLink*>& agxLinks_)
    {
        agxLinks = agxLinks_;
        agx::RigidBody* rb1 = agxLinks.back()->agxRigidBody;
        agx::RigidBody* rb2 = parent->agxRigidBody;
        agx::FrameRef frame1 = new agx::Frame();
        agx::FrameRef frame2 = new agx::Frame();

        const Vector3& a0 = agxLinks[0]->link->a();
        Vector3 a1;
        for(size_t i=1; i<agxLinks.size(); i++){
            const Vector3& a = agxLinks[i]->link->a();
            if(a0.dot(a)<1.e-8){
                a1 = a;
                break;
            }
        }
        if(a1.isZero()){
            axis[0].set( a0(0), a0(1), a0(2) );
            agx::Attachment::createAttachmentBase( axis[0], axis[1], axis[2] );
        }else{
            axis[0].set( a0(0), a0(1), a0(2) );
            axis[1].set( a1(0), a1(1), a1(2) );
            axis[2] = axis[0].cross(axis[1]);
        }
        agx::OrthoMatrix3x3 R;
        for(int i=0; i<3; i++)
            R.setRow(i, axis[i]);
        frame1->setLocalMatrix( agx::AffineMatrix4x4( R, agx::Vec3(0,0,0) ) );
        const Vector3& b = agxLinks.back()->origin - parent->origin;
        frame2->setLocalMatrix( agx::AffineMatrix4x4( R, agx::Vec3(b(0), b(1), b(2)) ) );

        //cout << frame1->getMatrix() << endl;
        //cout << frame2->getMatrix() << endl;
        signs.reserve(agxLinks.size());
        construct( rb1, frame1, rb2, frame2, createFunc );
    }

    enum AxisType {X, Y, Z, INVALID};
    AxisType detectAxis( const agx::Vec3& a, double& sign ){
        for(int i=0; i<3; i++){
            double d = a*axis[i];
            if(fabs(d-1)<1e-8){
                sign = 1;
                return static_cast<AxisType>(i);
            }
            if(fabs(d+1)<1e-8){
                sign = -1;
                return static_cast<AxisType>(i);
            }
        }
         return INVALID;
    }

    inline AxisType exclusionAxis( AxisType a, AxisType b){
        if( a==INVALID || b==INVALID)
            return INVALID;

        return static_cast<AxisType>(3-a-b);
    }

    inline agx::Attachment::Transformed attachmentType(AxisType axisType){
        switch(axisType){
        case X: return agx::Attachment::U;
        case Y: return agx::Attachment::V;
        case Z: return agx::Attachment::N;
        default: return agx::Attachment::NUM_ELEMENTS;
        }
    }

    inline void exclusionAttachmentType(AxisType axisType, agx::Attachment::Transformed& at0, agx::Attachment::Transformed& at1){
        switch(axisType){
        case X: at0 = agx::Attachment::V; at1 = agx::Attachment::N; return;
        case Y: at0 = agx::Attachment::N; at1 = agx::Attachment::U; return;
        case Z: at0 = agx::Attachment::U; at1 = agx::Attachment::V; return;
        default: return;
        }
    }

    inline agx::Angle::Axis angleType(AxisType axisType){
        switch(axisType){
        case X: return agx::Angle::U;
        case Y: return agx::Angle::V;
        case Z: return agx::Angle::N;
        default: return agx::Angle::U;
        }
    }

    bool create(){
        vector<int> rotateJoints;
        vector<int> slideJoints;
        for(size_t i=0; i<agxLinks.size(); i++){
            if(agxLinks[i]->link->jointType()==Link::ROTATIONAL_JOINT)
                rotateJoints.push_back(i);
            else if(agxLinks[i]->link->jointType()==Link::SLIDE_JOINT)
                slideJoints.push_back(i);
            else{
                return false;
            }
        }

        agx::AttachmentPair* ap = getAttachmentPair();
        double sign;
        switch(rotateJoints.size()){
        case 0:
            addElementaryConstraint( " ", new agx::QuatLock( agx::QuatLockData( ap ) ) );
            break;
        case 1:{
            const Vector3& a = agxLinks[rotateJoints[0]]->link->a();
            const string& name = agxLinks[rotateJoints[0]]->link->name();
            AxisType t = detectAxis( agx::Vec3(a(0), a(1), a(2)) ,sign );

            agx::Attachment::Transformed at0, at1, at2;
            at0 = attachmentType(t);
            exclusionAttachmentType(t, at1, at2);
            agx::Angle::Axis angle = angleType(t);
            signs[rotateJoints[0]] = sign;

            addElementaryConstraint( "", new agx::Dot1( agx::Dot1Data( ap, at0, at1 ) ) );
            addElementaryConstraint( "", new agx::Dot1( agx::Dot1Data( ap, at0, at2 ) ) );
            agx::RotationalAngleRef rotAngle = new agx::RotationalAngle( angle );
            addSecondaryConstraint( name, new agx::TargetSpeedController( agx::ConstraintAngleBasedData( ap, rotAngle ) ) );
            break;
        }
        case 2:{
            const Vector3& a0 = agxLinks[rotateJoints[0]]->link->a();
            const string& name0 = agxLinks[rotateJoints[0]]->link->name();
            AxisType t0 = detectAxis( agx::Vec3(a0(0), a0(1), a0(2)) ,sign );
            signs[rotateJoints[0]] = sign;
            const Vector3& a1 = agxLinks[rotateJoints[1]]->link->a();
            const string& name1 = agxLinks[rotateJoints[1]]->link->name();
            AxisType t1 = detectAxis( agx::Vec3(a1(0), a1(1), a1(2)) ,sign);
            signs[rotateJoints[1]] = sign;
            agx::Attachment::Transformed at0 = attachmentType(t0);
            agx::Attachment::Transformed at1 = attachmentType(t1);
            agx::Angle::Axis angle0 = angleType(t0);
            agx::Angle::Axis angle1 = angleType(t1);

            addElementaryConstraint( "", new agx::Dot1( agx::Dot1Data( ap, at0, at1 ) ) );
            agx::RotationalAngleRef rotAngle = new agx::RotationalAngle( angle0 );
            addSecondaryConstraint( name0, new agx::TargetSpeedController( agx::ConstraintAngleBasedData( ap, rotAngle ) ) );
            rotAngle = new agx::RotationalAngle( angle1 );
            addSecondaryConstraint( name1, new agx::TargetSpeedController( agx::ConstraintAngleBasedData( ap, rotAngle ) ) );
            break;
        }
        case 3: {//free
            for(int i=0; i<3; i++){
                const Vector3& a = agxLinks[rotateJoints[i]]->link->a();
                AxisType t = detectAxis( agx::Vec3( a(0), a(1), a(2) ) ,sign);
                signs[rotateJoints[i]] = sign;
                const string& name = agxLinks[rotateJoints[i]]->link->name();
                agx::Angle::Axis angle = angleType(t);
                agx::RotationalAngleRef rotAngle = new agx::RotationalAngle( angle );
                addSecondaryConstraint( name, new agx::TargetSpeedController( agx::ConstraintAngleBasedData( ap, rotAngle ) ) );
            }
            break;
        }
        default :
            return false;
        }
        switch(slideJoints.size()){
        case 0:
            addElementaryConstraint( "SR", new agx::SphericalRel( agx::ElementaryConstraintData( ap ) ) );
            break;
        case 1:{
            const Vector3& a = agxLinks[slideJoints[0]]->link->a();
            AxisType t = detectAxis( agx::Vec3( a(0), a(1), a(2) ) ,sign);
            signs[slideJoints[0]] = sign;
            const string& name = agxLinks[slideJoints[0]]->link->name();
            agx::Attachment::Transformed at0, at1;
            exclusionAttachmentType(t, at0, at1);
            agx::Angle::Axis angle = angleType(t);

            addElementaryConstraint( "", new agx::Dot2( agx::Dot2Data( ap, at0 ) ) );
            addElementaryConstraint( "", new agx::Dot2( agx::Dot2Data( ap, at1 ) ) );
            agx::SeparationAngleRef sepAngle = new agx::SeparationAngle( angle );
            addSecondaryConstraint( name, new agx::TargetSpeedController( agx::ConstraintAngleBasedData( ap, sepAngle ) ) );
            break;
        }
        case 2:{
            const Vector3& a0 = agxLinks[slideJoints[0]]->link->a();
            const string& name0 = agxLinks[slideJoints[0]]->link->name();
            AxisType t0 = detectAxis( agx::Vec3(a0(0), a0(1), a0(2)), sign );
            signs[slideJoints[0]] = sign;
            const Vector3& a1 = agxLinks[slideJoints[1]]->link->a();
            const string& name1 = agxLinks[slideJoints[1]]->link->name();
            AxisType t1 = detectAxis( agx::Vec3(a1(0), a1(1), a1(2)), sign );
            signs[slideJoints[1]] = sign;
            AxisType t2 = exclusionAxis(t0,t1);

            agx::Angle::Axis angle0 = angleType(t0);
            agx::Angle::Axis angle1 = angleType(t1);
            agx::Attachment::Transformed at0 = attachmentType(t2);

            addElementaryConstraint( "", new agx::Dot2( agx::Dot2Data( ap, at0 ) ) );
            agx::SeparationAngleRef sepAngle = new agx::SeparationAngle( angle0 );
            addSecondaryConstraint( name0, new agx::TargetSpeedController( agx::ConstraintAngleBasedData( ap, sepAngle ) ) );
            sepAngle = new agx::SeparationAngle( angle1 );
            addSecondaryConstraint( name1, new agx::TargetSpeedController( agx::ConstraintAngleBasedData( ap, sepAngle ) ) );
            break;
        }
        case 3: { //free;
            for(int i=0; i<3; i++){
                const Vector3& a = agxLinks[slideJoints[i]]->link->a();
                AxisType t = detectAxis( agx::Vec3( a(0), a(1), a(2) ), sign );
                signs[slideJoints[i]] = sign;
                const string& name = agxLinks[slideJoints[i]]->link->name();
                agx::Angle::Axis angle = angleType(t);
                agx::SeparationAngleRef sepAngle = new agx::SeparationAngle( angle );
                addSecondaryConstraint( name, new agx::TargetSpeedController( agx::ConstraintAngleBasedData( ap, sepAngle ) ) );
            }
            break;
        }
        default :
            return false;
        }

        controllers.clear();
        for(size_t i=0; i<agxLinks.size(); i++){
            const string& name = agxLinks[i]->link->name();
            agx::TargetSpeedController* controller =
                    dynamic_cast<agx::TargetSpeedController*>( getSecondaryConstraint( name ) );
            if(controller && agxLinks[i]->inputMode!=AgXLink::NON)
                controller->setEnable(true);
            controllers.push_back(controller);
        }

        angles.reserve(agxLinks.size());
        for(size_t i=0; i<rotateJoints.size(); i++)
            angles[rotateJoints[i]] = i;
        for(size_t i=0; i<slideJoints.size(); i++)
            angles[slideJoints[i]] = rotateJoints.size() + i;
    }

    double getAngle(int i){
        getAttachmentPair()->transform();
        return signs[i]*getAttachmentPair()->getAngle(angles[i])->getValue();
    }

    void setSpeed( int i, agx::Real speed ) {
        if(controllers[i]){
            controllers[i]->setSpeed(signs[i]*speed);
        }
    }

    void setForceRange( int i, agx::RangeReal forceRange ) {
        if(controllers[i]){
            controllers[i]->setForceRange(agx::RangeReal(signs[i]*forceRange.upper()));
        }
    }

    int getNumDOF(){
        return agxLinks.size();
    }

private :
    vector<AgXLink*> agxLinks;
    agx::Vec3 axis[3];
    vector<agx::TargetSpeedController*> controllers;
    vector<double> signs;
    vector<int> angles;

};

bool createFunc( agx::HighLevelConstraintImplementation* implementation )
{
  return dynamic_cast< CustomConstraintImplementation* >( implementation )->create();
}


class CustomConstraint : public agx::Constraint
{
public:
    CustomConstraint( AgXLink* parent, vector<AgXLink*>& agxLinks ){
        m_implementation = new CustomConstraintImplementation( parent, agxLinks );
    }

    virtual int getNumDOF() const {
        return dynamic_cast< CustomConstraintImplementation* >(m_implementation)->getNumDOF();
    }

    virtual void render( agxRender::RenderManager *mgr, float scale=1.0f ) const {
    }

    double getAngle(int i) {
        return dynamic_cast< CustomConstraintImplementation* >(m_implementation)->getAngle(i);
    }

    void setSpeed( int i, agx::Real speed ) {
        dynamic_cast< CustomConstraintImplementation* >(m_implementation)->setSpeed(i, speed);
    }

    void setForceRange( int i, agx::RangeReal forceRange ) {
        dynamic_cast< CustomConstraintImplementation* >(m_implementation)->setForceRange(i, forceRange);
    }

protected:
    virtual ~CustomConstraint(){
        if ( m_implementation != 0L )
        {
            delete m_implementation;
            m_implementation = 0L;
        }
    }

};

}

namespace cnoid {

class AgXSimulatorItemImpl
{
public:
    AgXSimulatorItem* self;
    Selection dynamicsMode;
    Vector3 gravity;
    double restitution;
    double friction;
    enum FrictionModelType { BOX, SCALE_BOX, ITERATIVE_PROJECTED };
    Selection frictionModelType;
    enum FrictionSolveType { DIRECT, ITERATIVE, SPLIT, DIRECT_AND_ITERATIVE };
    inline agx::FrictionModel::SolveType solveType( int type ){
        switch(type){
        case DIRECT: return agx::FrictionModel::DIRECT;
        case ITERATIVE: return agx::FrictionModel::ITERATIVE;
        case SPLIT: return agx::FrictionModel::SPLIT;
        case DIRECT_AND_ITERATIVE: return agx::FrictionModel::DIRECT_AND_ITERATIVE;
        default : return agx::FrictionModel::NOT_DEFINED;
        }
    };
    Selection frictionSolveType;
    int numThreads;
    Selection contactReductionMode;
    int contactReductionBinResolution;
    int contactReductionThreshold;

    agx::MaterialRef material;
    double timeStep;
    agxSDK::SimulationRef agxSimulation;
    agxPowerLine::PowerLineRef powerLine;
    typedef std::map<Link*, AgXSimulatorItem::ControlMode> ControlModeMap;
    ControlModeMap controlModeMap;

    double springConstant[3];
    double dampingCoefficient[3];

    AgXSimulatorItemImpl(AgXSimulatorItem* self);
    AgXSimulatorItemImpl(AgXSimulatorItem* self, const AgXSimulatorItemImpl& org);
    void initialize();
    ~AgXSimulatorItemImpl();

    bool initializeSimulation(const std::vector<SimulationBody*>& simBodies);
    void addBody(AgXBody* agxBody, int i);
    void clear();
    bool stepSimulation(const std::vector<SimulationBody*>& activeSimBodies);
    void doPutProperties(PutPropertyFunction& putProperty);
    void store(Archive& archive);
    void restore(const Archive& archive);
    void setJointControlMode(Link* joint, AgXSimulatorItem::ControlMode type);
    void setJointCompliance(Link* joint, double spring, double damping);
};

class HighGainControllerItem : public ControllerItem
{
    AgXSimulatorItemImpl* simulator;
    BodyPtr body;
    MultiValueSeqPtr qseqRef;
    int currentFrame;
    int lastFrame;
    int numJoints;
    vector<bool> controlEnable;

public:
    HighGainControllerItem(BodyItem* bodyItem, BodyMotionItem* bodyMotionItem, AgXSimulatorItemImpl* simulator)
    : simulator(simulator)
    {
        qseqRef = bodyMotionItem->motion()->jointPosSeq();
        setName(str(fmt(_("HighGain Controller with %1%")) % bodyMotionItem->name()));
    }

    virtual bool start(Target* target) {
        body = target->body();
        currentFrame = 0;
        lastFrame = std::max(0, qseqRef->numFrames() - 1);
        numJoints = std::min(body->numJoints(), qseqRef->numParts());
        if(qseqRef->numFrames() == 0){
            putMessage(_("Reference motion is empty()."));
            return false;
        }
        if(fabs(qseqRef->frameRate() - (1.0 / target->worldTimeStep())) > 1.0e-6){
            putMessage(_("The frame rate of the reference motion is different from the world frame rate."));
            return false;
        }

        controlEnable.reserve(numJoints);
        for(int i=0; i < numJoints; ++i){
            Link* joint = body->joint(i);
            AgXSimulatorItemImpl::ControlModeMap::iterator it = simulator->controlModeMap.find(joint);
            if(it!=simulator->controlModeMap.end() && it->second!=AgXSimulatorItem::HIGH_GAIN)
                controlEnable[i] = false;
            else
                controlEnable[i] = true;
        }

        control();
        return true;
    }

    virtual double timeStep() const {
        return qseqRef->getTimeStep();
    }

    virtual void input() { }

    virtual bool control() {

        if(++currentFrame > lastFrame){
            currentFrame = lastFrame;
            return false;
        }
        return true;
    }

    virtual void output() {

        int prevFrame = std::max(currentFrame - 1, 0);
        int nextFrame = std::min(currentFrame + 1, lastFrame);

        MultiValueSeq::Frame q0 = qseqRef->frame(prevFrame);
        MultiValueSeq::Frame q1 = qseqRef->frame(currentFrame);
        MultiValueSeq::Frame q2 = qseqRef->frame(nextFrame);

        double dt = qseqRef->getTimeStep();
        double dt2 = dt * dt;

        for(int i=0; i < numJoints; ++i){
            if(!controlEnable[i])
                continue;
            Link* joint = body->joint(i);
            //joint->q() = q1[i];
            joint->dq() = (q2[i] - q1[i]) / dt;
            joint->ddq() = (q2[i] - 2.0 * q1[i] + q0[i]) / dt2;
        }
    }

    virtual void stop() { }
};
}


AgXLink::AgXLink
(AgXSimulatorItemImpl* simImpl, AgXBody* agxBody, AgXLink* parent, const Vector3& parentOrigin, Link* link) :
    simImpl(simImpl),
    agxBody(agxBody),
    link(link),
    parent(parent),
    unit(0)
{
    AgXSimulatorItemImpl::ControlModeMap::iterator it = simImpl->controlModeMap.find(link);
    if(it!=simImpl->controlModeMap.end())
        controlMode = it->second;
    else
        controlMode = AgXSimulatorItem::DEFAULT;

    if(agxBody->body()->name().find("HRP2",0) == string::npos)
        isHRP2 = false;
    else
        isHRP2 = true;

    agxBody->agxLinks.push_back(this);

    origin = parentOrigin + link->b();
    agxRigidBody = 0;

    if(!link->m() && link->jointType()==Link::FIXED_JOINT){  //for Hose & Cabinet Box
        Link* palink = link->parent();
        for( ; palink; palink = palink->parent())
            if(palink->jointType()!=Link::FIXED_JOINT)
                break;
        if(!palink){
            link->setMass(1.0);
            link->setInertia(Matrix3::Identity());
            link->setCenterOfMass(Vector3(0,0,0));
        }
    }

    if(link->m()){
        createLinkBody(agxBody->isStatic);
       // createGeometry(agxBody);
        simImpl->agxSimulation->add(agxRigidBody);

        const agxCollide::GeometryRefVector& geometries = agxRigidBody->getGeometries();
        for(agxCollide::GeometryRefVector::const_iterator it = geometries.begin(); it != geometries.end(); it++){
            if(!agxBody->collisionDetectionEnabled)
                (*it)->setEnableCollisions(false);
            (*it)->addGroup(agxBody->geometryGroupId);
            (*it)->setMaterial( simImpl->material );
        }
    }

    for(Link* child = link->child(); child; child = child->sibling()){
        new AgXLink(simImpl, agxBody, this, origin, child);
    }

    // add Contact Event Listener
    /*
    const agxCollide::GeometryRefVector& geometries = agxRigidBody->getGeometries();
    for(agxCollide::GeometryRefVector::const_iterator itr = geometries.begin();
            itr != geometries.end(); itr++){
        agxSDK::ContactEventListenerRef listener = new AgxContactEventListener(this);
        listener->setFilter( new agxSDK::GeometryFilter( *itr ) );
        agxSimulation->addEventListener( listener );
    }
    */
}


AgXLink::~AgXLink()
{

}


void AgXLink::createLinkBody(bool isStatic)
{
    constraintLinks.clear();
    for(  ; parent && !parent->agxRigidBody; parent = parent->parent ){
        constraintLinks.push_back(parent);
    }
    if(constraintLinks.size())
        constraintLinks.push_back(this);

    agxRigidBody = new agx::RigidBody();
    agx::ref_ptr<AgXLinkContainer> agxLinkC = new AgXLinkContainer(this);
    agxRigidBody->setCustomData( agxLinkC );
    if(isStatic){
        agxRigidBody->setMotionControl(agx::RigidBody::STATIC);
    }else{
        agxRigidBody->setMotionControl(agx::RigidBody::DYNAMICS);
        agxRigidBody->getMassProperties()->setAutoGenerateMask(0);
        agxRigidBody->getMassProperties()->setMass(link->m(), false);
#if 0
        Matrix3 I = link->I();
        if(constraintLinks.size()){
            for(size_t i=0; i<constraintLinks.size(); i++){
                Link* link_ = constraintLinks[i]->link;
                if(link_->jointType()==Link::ROTATIONAL_JOINT){
                    Vector3 axis = link_->a();
                    I += axis * axis.transpose() * link_->Jm2();
                }
            }
        }
#else
        const Matrix3& I = link->I();
#endif
        agx::SPDMatrix3x3 inertia(I(0,0), I(1,0), I(2,0),
                                  I(0,1), I(1,1), I(2,1),
                                  I(0,2), I(1,2), I(2,2));
        agxRigidBody->getMassProperties()->setInertiaTensor(inertia, false);
        const Vector3& c = link->c();
        const agx::Vec3 com(c(0), c(1), c(2));
        agxRigidBody->setCmLocalTranslate(com);
    }

    agxRigidBody->setPosition(origin(0), origin(1), origin(2));
    agxRigidBody->setRotation(agx::Quat(0,0,0,1));

    createGeometry(agxBody);

    if(!constraintLinks.size())
        createJoint();
    else{
        if(isHRP2){
            agx::FrameRef attFrame0 = new agx::Frame();
            agx::FrameRef attFrame1 = new agx::Frame();
            const Vector3& a = constraintLinks[1]->link->b();
            attFrame1->setTranslate( a(0), a(1), a(2) );
            agx::PrismaticUniversalJointRef prismaticUniversal = new agx::PrismaticUniversalJoint(
                    agxRigidBody, attFrame0, parent->agxRigidBody, attFrame1 );
            agx::Real complianceZ = agxUtil::convert::convertSpringConstantToCompliance( simImpl->springConstant[2] );
            agx::Real complianceX = agxUtil::convert::convertSpringConstantToCompliance( simImpl->springConstant[0] );
            agx::Real complianceY = agxUtil::convert::convertSpringConstantToCompliance( simImpl->springConstant[1] );
            agx::Real spookDampingZ = agxUtil::convert::convertDampingCoefficientToSpookDamping( simImpl->dampingCoefficient[2], simImpl->springConstant[2] );
            agx::Real spookDampingX = agxUtil::convert::convertDampingCoefficientToSpookDamping( simImpl->dampingCoefficient[0], simImpl->springConstant[0] );
            agx::Real spookDampingY = agxUtil::convert::convertDampingCoefficientToSpookDamping( simImpl->dampingCoefficient[1], simImpl->springConstant[1] );
            prismaticUniversal->getLock1D(agx::PrismaticUniversalJoint::TRANSLATIONAL_CONTROLLER_1)->setPosition(0.0);
            prismaticUniversal->getLock1D(agx::PrismaticUniversalJoint::TRANSLATIONAL_CONTROLLER_1)->setCompliance(complianceZ);
            prismaticUniversal->getLock1D(agx::PrismaticUniversalJoint::TRANSLATIONAL_CONTROLLER_1)->setDamping(spookDampingZ);
            prismaticUniversal->getLock1D(agx::PrismaticUniversalJoint::TRANSLATIONAL_CONTROLLER_1)->setEnable(true);
            prismaticUniversal->getLock1D(agx::PrismaticUniversalJoint::ROTATIONAL_CONTROLLER_1)->setPosition(0.0);
            prismaticUniversal->getLock1D(agx::PrismaticUniversalJoint::ROTATIONAL_CONTROLLER_1)->setCompliance(complianceX);
            prismaticUniversal->getLock1D(agx::PrismaticUniversalJoint::ROTATIONAL_CONTROLLER_1)->setDamping(spookDampingX);
            prismaticUniversal->getLock1D(agx::PrismaticUniversalJoint::ROTATIONAL_CONTROLLER_1)->setEnable(true);
            prismaticUniversal->getLock1D(agx::PrismaticUniversalJoint::ROTATIONAL_CONTROLLER_2)->setPosition(0.0);
            prismaticUniversal->getLock1D(agx::PrismaticUniversalJoint::ROTATIONAL_CONTROLLER_2)->setCompliance(complianceY);
            prismaticUniversal->getLock1D(agx::PrismaticUniversalJoint::ROTATIONAL_CONTROLLER_2)->setDamping(spookDampingY);
            prismaticUniversal->getLock1D(agx::PrismaticUniversalJoint::ROTATIONAL_CONTROLLER_2)->setEnable(true);
            simImpl->agxSimulation->add( prismaticUniversal );
            for(size_t i=0; i<constraintLinks.size(); i++){
                constraintLinks[i]->joint = prismaticUniversal;
            }
            constraintLinks[0]->customConstraintIndex = agx::PrismaticUniversalJoint::ROTATIONAL_CONTROLLER_1;
            constraintLinks[1]->customConstraintIndex = agx::PrismaticUniversalJoint::TRANSLATIONAL_CONTROLLER_1;
            constraintLinks[2]->customConstraintIndex = agx::PrismaticUniversalJoint::ROTATIONAL_CONTROLLER_2;
        }else{
            for(size_t i=0; i<constraintLinks.size(); i++){
                Link::JointType type = constraintLinks[i]->link->jointType();
                Vector3 v(0,0,0);
                if(i!=0)
                    v = constraintLinks[i-1]->origin - constraintLinks[i]->origin;
                if(!v.isZero() || (type!=Link::ROTATIONAL_JOINT && type!=Link::SLIDE_JOINT)) {
                    cout << "Create Joint Error" << endl;
                    for(size_t j=0; j<constraintLinks.size(); j++){
                        Vector3 o = constraintLinks[j]->origin;
                        Link::JointType type = constraintLinks[j]->link->jointType();
                        cout << "Link " << constraintLinks[j]->link->name() << " : " << "mass=" << constraintLinks[j]->link->mass()
                        << " jointType=" << (type==Link::ROTATIONAL_JOINT? "Rotational" : type==Link::SLIDE_JOINT? "Slide" : type==Link::FREE_JOINT? "Free" : type==Link::FIXED_JOINT? "Fixed" : type==Link::CRAWLER_JOINT? "Crawler" : "Unknown")
                        << " origin=" << o(0) << " " << o(1) <<" " << o(2) << endl;
                    }
                    return;
                }
                switch(constraintLinks[i]->controlMode){
                case AgXSimulatorItem::HIGH_GAIN :
                    constraintLinks[i]->inputMode = VEL;
                    break;
                case AgXSimulatorItem::TORQUE :
                    constraintLinks[i]->inputMode = TOR;
                    break;
                case AgXSimulatorItem::FREE :
                    constraintLinks[i]->inputMode = NON;
                    break;
                case AgXSimulatorItem::DEFAULT :
                    if(simImpl->dynamicsMode.is(AgXSimulatorItem::HG_DYNAMICS))
                        constraintLinks[i]->inputMode = VEL;
                    else
                        constraintLinks[i]->inputMode = TOR;
                    break;
                default:
                    break;
                }
            }
            agx::ref_ptr< CustomConstraint > customConstraint = new CustomConstraint( parent, constraintLinks );
            simImpl->agxSimulation->add( customConstraint );
            for(size_t i=0; i<constraintLinks.size(); i++){
                constraintLinks[i]->customConstraintIndex = i;
                constraintLinks[i]->joint = customConstraint;
            }
        }
    }
}


void AgXLink::createJoint()
{
    enum { MOTOR, SHAFT, NON };

    switch(link->jointType()){
    case Link::ROTATIONAL_JOINT:
    case Link::SLIDE_JOINT:
    {
        int part = MOTOR;
        switch(controlMode){
        case AgXSimulatorItem::HIGH_GAIN :
            part = MOTOR;
            inputMode = VEL;
            break;
        case AgXSimulatorItem::TORQUE :
            if(link->Jm2()){
                part = SHAFT;
                inputMode = TOR;
            }else{
                part = MOTOR;
                inputMode = TOR;
            }
            break;
        case AgXSimulatorItem::FREE :
            part = NON;
            inputMode = InputMode::NON;
            break;
        case AgXSimulatorItem::DEFAULT :
            if(!link->Jm2() || simImpl->dynamicsMode.is(AgXSimulatorItem::HG_DYNAMICS)){
                part = MOTOR;
                if(simImpl->dynamicsMode.is(AgXSimulatorItem::HG_DYNAMICS))
                    inputMode = VEL;
                else
                    inputMode = TOR;
            }else{
                part = SHAFT;
                inputMode = TOR;
            }
            break;
        default:
            break;
        }
        if(link->jointType()==Link::ROTATIONAL_JOINT){
            const Vector3& a = link->a();
            agx::HingeFrame hingeFrame;
            hingeFrame.setAxis( agx::Vec3( a(0), a(1), a(2)) );
            hingeFrame.setCenter( agx::Vec3( origin(0), origin(1), origin(2)) );
            agx::HingeRef hinge = new agx::Hinge( hingeFrame, agxRigidBody, parent->agxRigidBody );
            simImpl->agxSimulation->add( hinge );
            joint = hinge;

            if(part==MOTOR){
                hinge->getMotor1D()->setEnable(true);
            }else if(part==SHAFT){
                agxPowerLine::RotationalActuatorRef rotationalActuator = new agxPowerLine::RotationalActuator(hinge);
                agxDriveTrain::GearRef gear = new agxDriveTrain::Gear();
                agxDriveTrain::ShaftRef shaft = new agxDriveTrain::Shaft();
                shaft->connect(rotationalActuator, gear);
                shaft->setInertia( link->Jm2() );
                simImpl->powerLine->add(shaft);
                unit = shaft;
            }else if(part==NON)
                ;
        }else{
            const Vector3& d = link->d();
            agx::PrismaticFrame prismaticFrame;
            prismaticFrame.setAxis( agx::Vec3( d(0), d(1), d(2) ));
            prismaticFrame.setPoint( agx::Vec3(  origin(0), origin(1), origin(2)) );
            agx::PrismaticRef prismatic = new agx::Prismatic( prismaticFrame, agxRigidBody, parent->agxRigidBody );
            simImpl->agxSimulation->add( prismatic );
            joint = prismatic;

            if(part==MOTOR){
                prismatic->getMotor1D()->setEnable(true);
            }else if(part==SHAFT){
                agxPowerLine::TranslationalActuatorRef translationalActuator = new agxPowerLine::TranslationalActuator(prismatic);
                agxPowerLine::TranslationalConnectorRef connector = new agxPowerLine::TranslationalConnector();
                agxPowerLine::TranslationalUnitRef translationalUnit = new agxPowerLine::TranslationalUnit();
                translationalUnit->connect(translationalActuator, connector);
                translationalUnit->setMass( link->Jm2() );
                simImpl->powerLine->add(translationalUnit);
                unit = translationalUnit;
            }else if(part==NON)
                ;
        }
        break;
    }
    case Link::FIXED_JOINT:
    case Link::CRAWLER_JOINT:{
        if(!parent){
            agxRigidBody->setMotionControl(agx::RigidBody::STATIC);
        }else{
            agx::LockJointRef lockJoint;
            lockJoint = new agx::LockJoint(agxRigidBody, parent->agxRigidBody);
            simImpl->agxSimulation->add( lockJoint );
            joint = lockJoint;
        }
        break;
    }
    case Link::FREE_JOINT:
    default :
        break;
    }
}


void AgXLink::createGeometry(AgXBody* agxBody)
{
    if(link->shape()){
        MeshExtractor* extractor = new MeshExtractor;
        if(extractor->extract(link->shape(), boost::bind(&AgXLink::addMesh, this, extractor, agxBody))){
            if(!vertices.empty()){
                agxCollide::TrimeshRef triangleMesh = new agxCollide::Trimesh( &vertices, &indices, "" );
                if(link->jointType()!=Link::CRAWLER_JOINT)
                    agxRigidBody->add( new agxCollide::Geometry( triangleMesh ) );
                else{
                    agx::ref_ptr<CrawlerGeometry> crawlerGeometry = new CrawlerGeometry( link, agxRigidBody.get() );
                    crawlerGeometry->add( triangleMesh );
                    crawlerGeometry->setSurfaceVelocity( agx::Vec3f(1,0,0) );   //適当に設定しておかないとcalculateSurfaceVelocityが呼び出されない。
                    agxRigidBody->add( crawlerGeometry );
                }
            }
        }
        delete extractor;
    }
}


void AgXLink::addMesh(MeshExtractor* extractor, AgXBody* agxBody)
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
                }
            }
        }
        if(doAddPrimitive){
            bool created = false;
            agxCollide::GeometryRef agxGeometry;
            if(link->jointType()!=Link::CRAWLER_JOINT)
                agxGeometry = new agxCollide::Geometry();
            else{
                agxGeometry = new CrawlerGeometry(link, agxRigidBody.get());
                agxGeometry->setSurfaceVelocity( agx::Vec3f(1,0,0) );
            }
            switch(mesh->primitiveType()){
            case SgMesh::BOX : {
                const Vector3& s = mesh->primitive<SgMesh::Box>().size / 2.0;
                agxCollide::BoxRef box = new agxCollide::Box( agx::Vec3( s.x()*scale.x(), s.y()*scale.y(), s.z()*scale.z() ) );
                agxGeometry->add(box);
                agxRigidBody->add( agxGeometry );
                created = true;
                break;
            }
            case SgMesh::SPHERE : {
                SgMesh::Sphere sphere = mesh->primitive<SgMesh::Sphere>();
                agxCollide::SphereRef sphere_ = new agxCollide::Sphere( sphere.radius * scale.x() );
                agxGeometry->add(sphere_);
                agxRigidBody->add( agxGeometry );
                created = true;
                break;
            }
            case SgMesh::CYLINDER : {
                SgMesh::Cylinder cylinder = mesh->primitive<SgMesh::Cylinder>();
                agxCollide::CylinderRef cylinder_ = new agxCollide::Cylinder(cylinder.radius * scale.x(), cylinder.height * scale.y());
                agxGeometry->add(cylinder_);
                agxRigidBody->add( agxGeometry );
                created = true;
                break;
            }
            default :
                break;
            }
            if(created){
                Affine3 T_ = extractor->currentTransformWithoutScaling();
                if(translation){
                    T_ *= Translation3(*translation);
                }
                agxGeometry->setLocalTransform( agx::AffineMatrix4x4( T_(0,0), T_(1,0), T_(2,0), 0.0,
                                                                      T_(0,1), T_(1,1), T_(2,1), 0.0,
                                                                      T_(0,2), T_(1,2), T_(2,2), 0.0,
                                                                      T_(0,3), T_(1,3), T_(2,3), 1.0) );
                meshAdded = true;
            }
        }
    }

    if(!meshAdded){
        const size_t vertexIndexTop = vertices.size();

        const SgVertexArray& vertices_ = *mesh->vertices();
        const int numVertices = vertices_.size();
        for(int i=0; i < numVertices; ++i){
            const Vector3 v = T * vertices_[i].cast<Position::Scalar>();
            vertices.push_back( agx::Vec3(v.x(), v.y(), v.z()) );
        }

        const int numTriangles = mesh->numTriangles();
        for(int i=0; i < numTriangles; ++i){
            SgMesh::TriangleRef src = mesh->triangle(i);
            indices.push_back(vertexIndexTop + src[0]);
            indices.push_back(vertexIndexTop + src[1]);
            indices.push_back(vertexIndexTop + src[2]);
        }
    }
}


void AgXLink::setKinematicStateToAgX()
{
    if(!agxRigidBody)
        return;

    const Vector3& p = link->p();
    const Matrix3& R = link->R();
    agx::Vec3 translation(p(0), p(1), p(2));
    agx::OrthoMatrix3x3 rotation(R(0,0), R(1,0), R(2,0),
                                 R(0,1), R(1,1), R(2,1),
                                 R(0,2), R(1,2), R(2,2));
    agxRigidBody->setTransform( agx::AffineMatrix4x4( rotation, translation) );

    const Vector3 lc = link->R() * link->c();
    const Vector3& w = link->w();
    const Vector3 v = link->v() + w.cross(lc);
    agxRigidBody->setVelocity( agx::Vec3(v(0),v(1),v(2)) );     // the linear velocity of the center of mass
    agxRigidBody->setAngularVelocity( agx::Vec3(w(0),w(1),w(2)) );

}


void AgXLink::getKinematicStateFromAgX()
{
    switch(link->jointType()){
        case Link::ROTATIONAL_JOINT:
        case Link::SLIDE_JOINT:{
            agx::Constraint1DOF* joint1DOF = agx::Constraint1DOF::safeCast(joint);
            if(joint1DOF){
                link->q() = joint1DOF->getAngle();
                link->dq() = joint1DOF->getCurrentSpeed();
                if(inputMode==VEL){
                	link->u() = joint1DOF->getMotor1D()->getCurrentForce();
                	//cout << link->name() << " " << link->u() << endl;
                }
                break;
            }
            CustomConstraint* jointCustom = dynamic_cast<CustomConstraint*>(joint);
            if(jointCustom){
                double oldq = link->q();
                link->q() = jointCustom->getAngle(customConstraintIndex);
                link->dq() = ( link->q() - oldq ) / simImpl->timeStep;
                break;
            }
            agx::PrismaticUniversalJoint* pujoint = dynamic_cast<agx::PrismaticUniversalJoint*>(joint);
            if(pujoint){
                link->q() = pujoint->getAngle(customConstraintIndex);
                link->dq() = pujoint->getCurrentSpeed(customConstraintIndex);
                break;
            }
        }
        default :
            break;
    }

    if(!agxRigidBody)
            return;

    agx::AffineMatrix4x4 t = agxRigidBody->getTransform();
    link->p() = Vector3(t(3,0), t(3,1), t(3,2));
    link->R() << t(0,0), t(1,0), t(2,0),
                 t(0,1), t(1,1), t(2,1),
                 t(0,2), t(1,2), t(2,2);

    agx::Vec3 w = agxRigidBody->getAngularVelocity();
    link->w() = Vector3(w.x(), w.y(), w.z());
    agx::Vec3 v = agxRigidBody->getVelocity();
    Vector3 v0(v.x(), v.y(), v.z());
    const Vector3 c = link->R() * link->c();
    link->v() = v0 - link->w().cross(c);
}


void AgXLink::setTorqueToAgX()
{
    if(unit){
        agxDriveTrain::Shaft* shaft = dynamic_cast<agxDriveTrain::Shaft*>(unit);
        if(shaft){
            shaft->getRotationalDimension()->addLoad( link->u() );
            return;
        }
        agxPowerLine::TranslationalUnit* transUnit = dynamic_cast<agxPowerLine::TranslationalUnit*>(unit);
        if(transUnit){
            transUnit->getTranslationalDimension()->addLoad( link->u() );
            return;
        }
    }
    agx::Constraint1DOF* joint1DOF = agx::Constraint1DOF::safeCast(joint);
    if(joint1DOF){
        if(inputMode==VEL){
            joint1DOF->getMotor1D()->setSpeed( link->dq() );
            joint1DOF->getMotor1D()->setForceRange( -std::numeric_limits<agx::Real>::max(), std::numeric_limits<agx::Real>::max());
            return;
        }else if(inputMode==TOR){
            joint1DOF->getMotor1D()->setSpeed( link->u()<0? -1.0e12 : 1.0e12);
            joint1DOF->getMotor1D()->setForceRange( agx::RangeReal( link->u() ) );
            return;
        }
    }
    CustomConstraint* jointCustom = dynamic_cast<CustomConstraint*>(joint);
    if(jointCustom){
        if(inputMode==VEL){
            jointCustom->setSpeed( customConstraintIndex, link->dq() );
            jointCustom->setForceRange( customConstraintIndex, agx::RangeReal(-std::numeric_limits<agx::Real>::max(),
                    std::numeric_limits<agx::Real>::max()));
            return;
        }else if(inputMode==TOR){
            jointCustom->setSpeed( customConstraintIndex, link->u()<0? -1.0e12 : 1.0e12 );
            jointCustom->setForceRange( customConstraintIndex, agx::RangeReal(link->u()) );
            return;
        }
    }
}


void AgXForceField::updateForce(agx::DynamicsSystem* system)
{
#if 0
    agx::RigidBodyRefVector& bodies = system->getRigidBodies();
    for(agx::RigidBodyRefVector::iterator it = bodies.begin(); it != bodies.end(); ++it)
    {
        agx::RigidBody *body = (*it).get();
        AgXLink* agxLink = dynamic_cast<AgXLinkContainer*>( body->getCustomData() )->agxLink;
        if(agxLink){
            if(agxLink->constraintLinks.empty())
                agxLink->constraintLinks.push_back(agxLink);
            for(int i=0; i<agxLink->constraintLinks.size(); i++){
                Link* link = agxLink->constraintLinks[i]->link;
                if(link->isRotationalJoint()){
                    const Vector3 u = link->u() * link->a();
                    agx::Vec3 u_ = body->getFrame()->transformVectorToWorld( u(0), u(1), u(2) );
                    body->addTorque( u_ );
                    agxLink->parent->agxRigidBody->addTorque( -u_ );
                }else if(link->isSlideJoint()){
                    const Vector3 u = link->u() * link->d();
                    agx::Vec3 u_ = body->getFrame()->transformVectorToWorld( u(0), u(1), u(2) );
                    body->addForceAtLocalPosition( u_, agx::Vec3(0, 0, 0) );
                    agxLink->parent->agxRigidBody->addForceAtLocalPosition( -u_, agx::Vec3(0, 0, 0) );
                }
            }
        }
    }
#endif
}


AgXBody::AgXBody(Body& orgBody)
    : SimulationBody(&orgBody)
{

}


AgXBody::~AgXBody()
{

}


void AgXBody::createBody(AgXSimulatorItemImpl* _simImpl)
{
    simImpl = _simImpl;
    isStatic = body()->isStaticModel();
    collisionDetectionEnabled = bodyItem()->isCollisionDetectionEnabled();
    selfCollisionDetectionEnabled = bodyItem()->isSelfCollisionDetectionEnabled();

    AgXLink* rootLink = new AgXLink(simImpl, this, 0, Vector3::Zero(), body()->rootLink());

    setKinematicStateToAgX();

    if(!selfCollisionDetectionEnabled)
        simImpl->agxSimulation->getSpace()->setEnablePair(geometryGroupId,geometryGroupId,false);

    setExtraJoints();

    sensorHelper.initialize(body(), simImpl->timeStep, simImpl->gravity);
    const DeviceList<ForceSensor>& forceSensors = sensorHelper.forceSensors();
    for(size_t i=0; i < forceSensors.size(); ++i){
        AgXLink* agxLink = findLink(forceSensors[i]->link());
        agxLink->joint->setEnableComputeForces(true);
    }
}


void AgXBody::setKinematicStateToAgX()
{
    for(size_t i=0; i < agxLinks.size(); ++i){
        agxLinks[i]->setKinematicStateToAgX();
    }
}


void AgXBody::getKinematicStateFromAgX()
{
    for(size_t i=0; i < agxLinks.size(); ++i){
        agxLinks[i]->getKinematicStateFromAgX();
    }
}


AgXLink* AgXBody::findLink(Link* link)
{
    for( vector<AgXLinkPtr>::iterator itr=agxLinks.begin(); itr!=agxLinks.end(); itr++){
        if((*itr)->link == link)
            return (*itr).get();
    }
    return 0;
}


void AgXBody::updateForceSensors()
{
    const DeviceList<ForceSensor>& forceSensors = sensorHelper.forceSensors();
    for(size_t i=0; i < forceSensors.size(); ++i){
        ForceSensor* sensor = forceSensors[i];
        AgXLink* agxLink = findLink(sensor->link());
        if(agxLink && agxLink->parent && agxLink->joint){
           agx::Vec3 force,torque;
           agxLink->joint->getLastForce( agxLink->parent->agxRigidBody, force, torque, false);
           Vector3 f(force[0], force[1], force[2]);
           Vector3 tau(torque[0], torque[1], torque[2]);
           const Matrix3 R = sensor->link()->R() * sensor->R_local();
           const Vector3 p = sensor->link()->R() * sensor->p_local();
           sensor->f()   = R.transpose() * f;
           sensor->tau() = R.transpose() * (tau - p.cross(f));
           sensor->notifyStateChange();
        }
    }
}


void AgXBody::setExtraJoints()
{
    Body* body = this->body();
    const int n = body->numExtraJoints();
    for(int j=0; j < n; ++j){
        Body::ExtraJoint& extraJoint = body->extraJoint(j);

        AgXLinkPtr agxLinkPair[2];
        for(int i=0; i < 2; ++i){
            Link* link = extraJoint.link[i];
            agxLinkPair[i] = findLink( link );
            if(!agxLinkPair[i])
                break;
        }

        if(agxLinkPair[1]){
            Link* link = agxLinkPair[0]->link;
            Vector3 p = link->attitude() * extraJoint.point[0] + link->p();
            Vector3 a = link->attitude() * extraJoint.axis;
            if(extraJoint.type == Body::EJ_PISTON){
                agx::HingeFrame hingeFrame;
                hingeFrame.setAxis( agx::Vec3( a(0), a(1), a(2)) );
                hingeFrame.setCenter( agx::Vec3( p(0), p(1), p(2)) );
                agx::HingeRef hinge = new agx::Hinge( hingeFrame,
                        agxLinkPair[0]->agxRigidBody, agxLinkPair[1]->agxRigidBody );
                simImpl->agxSimulation->add( hinge );
            }else if(extraJoint.type == Body::EJ_BALL){
                agx::BallJointFrame ballJointFrame;
                ballJointFrame.setCenter( agx::Vec3( p(0), p(1), p(2)) );
                agx::BallJointRef ballJoint = new agx::BallJoint( ballJointFrame,
                        agxLinkPair[0]->agxRigidBody, agxLinkPair[1]->agxRigidBody );
                simImpl->agxSimulation->add( ballJoint );
            }
        }
    }
}


void AgXBody::setTorqueToAgX()
{
    // Skip the root link
    for(size_t i=1; i < agxLinks.size(); ++i){
        agxLinks[i]->setTorqueToAgX();
    }
}


void AgXSimulatorItem::initializeClass(ExtensionManager* ext)
{
    ext->itemManager().registerClass<AgXSimulatorItem>("AgXSimulatorItem");
    ext->itemManager().addCreationPanel<AgXSimulatorItem>();
}


AgXSimulatorItem::AgXSimulatorItem()
{
    impl = new AgXSimulatorItemImpl(this);
}


AgXSimulatorItemImpl::AgXSimulatorItemImpl(AgXSimulatorItem* self)
    : self(self),
      frictionModelType(3, CNOID_GETTEXT_DOMAIN_NAME),
      frictionSolveType(4, CNOID_GETTEXT_DOMAIN_NAME),
      contactReductionMode(3, CNOID_GETTEXT_DOMAIN_NAME),
      dynamicsMode(AgXSimulatorItem::N_DYNAMICS_MODES, CNOID_GETTEXT_DOMAIN_NAME)
{
    initialize();

    dynamicsMode.setSymbol(AgXSimulatorItem::FORWARD_DYNAMICS,  N_("Forward dynamics"));
    dynamicsMode.setSymbol(AgXSimulatorItem::HG_DYNAMICS,       N_("High-gain dynamics"));

    gravity << 0.0, 0.0, -DEFAULT_GRAVITY_ACCELERATION;
    friction = 0.5;
    restitution = 0.1;
    numThreads = 1;
    contactReductionBinResolution = 2;
    contactReductionThreshold = 4;

    frictionModelType.setSymbol(BOX, "Box");
    frictionModelType.setSymbol(SCALE_BOX, "Scale Box");
    frictionModelType.setSymbol(ITERATIVE_PROJECTED, "Iterative Projected");
    frictionModelType.select(ITERATIVE_PROJECTED);

    frictionSolveType.setSymbol(DIRECT, "Direct");
    frictionSolveType.setSymbol(ITERATIVE, "Iterative");
    frictionSolveType.setSymbol(SPLIT, "Split");
    frictionSolveType.setSymbol(DIRECT_AND_ITERATIVE, "Direct and Iterative");
    frictionSolveType.select(SPLIT);

    contactReductionMode.setSymbol(agx::ContactMaterial::REDUCE_NONE, "None");
    contactReductionMode.setSymbol(agx::ContactMaterial::REDUCE_GEOMETRY, "Geometry");
    contactReductionMode.setSymbol(agx::ContactMaterial::REDUCE_ALL, "All");
    contactReductionMode.select(agx::ContactMaterial::REDUCE_GEOMETRY);

}


AgXSimulatorItem::AgXSimulatorItem(const AgXSimulatorItem& org)
    : SimulatorItem(org)
{
    impl = new AgXSimulatorItemImpl(this, *org.impl);
}


AgXSimulatorItemImpl::AgXSimulatorItemImpl(AgXSimulatorItem* self, const AgXSimulatorItemImpl& org)
    : self(self),
      dynamicsMode(org.dynamicsMode)
{
    initialize();

    gravity = org.gravity;
    friction = org.friction;
    restitution = org.restitution;
    frictionModelType = org.frictionModelType;
    frictionSolveType = org.frictionSolveType;
    numThreads = org.numThreads;
    contactReductionMode = org.contactReductionMode;
    contactReductionBinResolution = org.contactReductionBinResolution;
    contactReductionThreshold = org.contactReductionThreshold;
}


void AgXSimulatorItemImpl::initialize()
{

}


AgXSimulatorItem::~AgXSimulatorItem()
{
    delete impl;
}


AgXSimulatorItemImpl::~AgXSimulatorItemImpl()
{
    // ここでagxSimulationのメソッドを呼ぶと終了時にエラーが発生する。agx::shutdown()が実行された後のため。
}


Item* AgXSimulatorItem::doDuplicate() const
{
    return new AgXSimulatorItem(*this);
}


SimulationBody* AgXSimulatorItem::createSimulationBody(Body* orgBody)
{
    return new AgXBody(*orgBody);
}


bool AgXSimulatorItem::initializeSimulation(const std::vector<SimulationBody*>& simBodies)
{
    return impl->initializeSimulation(simBodies);
}


bool AgXSimulatorItemImpl::initializeSimulation(const std::vector<SimulationBody*>& simBodies)
{
    clear();
    agxSimulation = new agxSDK::Simulation();
    agx::setNumThreads( numThreads );
    agxSimulation->setContactReductionBinResolution( contactReductionBinResolution );
    agxSimulation->setContactReductionThreshold( contactReductionThreshold );

    powerLine = new agxPowerLine::PowerLine();
    agxSimulation->add(powerLine);

    agx::UniformGravityField* uniformGravityField = dynamic_cast<agx::UniformGravityField*>( agxSimulation->getGravityField() );
    if(uniformGravityField)
        uniformGravityField->setGravity( agx::Vec3( gravity(0), gravity(1), gravity(2) ) );

    timeStep = self->worldTimeStep();
    agxSimulation->getDynamicsSystem()->getTimeGovernor()->setTimeStep( timeStep );

    material = new agx::Material( "Material", restitution, friction );
    agxSimulation->add( material );

    agx::ContactMaterial* contactMaterial = agxSimulation->
            getMaterialManager()->getOrCreateContactMaterial( material, material );
    contactMaterial->setContactReductionMode( (agx::ContactMaterial::ContactReductionMode)contactReductionMode.selectedIndex() );

    switch(frictionModelType.selectedIndex()){
    case BOX:{
        agx::BoxFrictionModelRef boxFriction = new agx::BoxFrictionModel();
        boxFriction->setSolveType( solveType(frictionSolveType.selectedIndex()) );
        contactMaterial->setFrictionModel( boxFriction );
        break;
    }
    case SCALE_BOX:{
        agx::ScaleBoxFrictionModelRef scaleBoxFriction = new agx::ScaleBoxFrictionModel();
        scaleBoxFriction->setSolveType( solveType(frictionSolveType.selectedIndex()) );
        contactMaterial->setFrictionModel( scaleBoxFriction );
        break;
    }
    case ITERATIVE_PROJECTED:{
        agx::IterativeProjectedConeFrictionRef ipcFriction = new agx::IterativeProjectedConeFriction();
        ipcFriction->setSolveType( solveType(frictionSolveType.selectedIndex()) );
        contactMaterial->setFrictionModel( ipcFriction );
        break;
    }
    default :
        break;
    }

    for(size_t i=0; i < simBodies.size(); ++i){
        addBody(static_cast<AgXBody*>(simBodies[i]),i);
    }

    // add Force Field

    //agx::InteractionRef forceField = new AgXForceField;
    //agxSimulation->getDynamicsSystem()->add( forceField );

    //  debug file output
    //agxSimulation->getSerializer()->setEnable(true);
    //agxSimulation->getSerializer()->setInterval(0.1); // 10hz
    //agxSimulation->getSerializer()->setFilename("aist.agx");

    return true;
}


void AgXSimulatorItem::initializeSimulationThread()
{
    agx::Thread::registerAsAgxThread();
    impl->agxSimulation->setMainWorkThread(agx::Thread::getCurrentThread());
}


void AgXSimulatorItem::finalizeSimulationThread()
{
    agx::Thread::unregisterAsAgxThread();
}


bool AgXSimulatorItem::stepSimulation(const std::vector<SimulationBody*>& activeSimBodies)
{
    return impl->stepSimulation(activeSimBodies);
}


bool AgXSimulatorItemImpl::stepSimulation(const std::vector<SimulationBody*>& activeSimBodies)
{
    for(size_t i=0; i < activeSimBodies.size(); ++i){
        AgXBody* agxBody = static_cast<AgXBody*>(activeSimBodies[i]);
        agxBody->body()->setVirtualJointForces();
        agxBody->setTorqueToAgX();
    }

    agxSimulation->stepForward();

    for(size_t i=0; i < activeSimBodies.size(); ++i){
        AgXBody* agxBody = static_cast<AgXBody*>(activeSimBodies[i]);
        agxBody->getKinematicStateFromAgX();

        if(!agxBody->sensorHelper.forceSensors().empty()){
            agxBody->updateForceSensors();
        }
        if(agxBody->sensorHelper.hasGyroOrAccelerationSensors()){
            agxBody->sensorHelper.updateGyroAndAccelerationSensors();
        }
    }

    return true;
}


void AgXSimulatorItem::finalizeSimulation()
{

}


void AgXSimulatorItemImpl::clear()
{
    if(agxSimulation)
        agxSimulation->cleanup( agxSDK::Simulation::CLEANUP_ALL );
}


void AgXSimulatorItemImpl::addBody(AgXBody* agxBody, int i)
{
    Body& body = *agxBody->body();

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
    }

    body.clearExternalForces();
    body.calcForwardKinematics(true, true);

    agxBody->geometryGroupId = i;
    agxBody->createBody(this);
}


CollisionLinkPairListPtr AgXSimulatorItem::getCollisions()
{
    const agxCollide::GeometryContactPtrVector& contacts = impl->agxSimulation->getSpace()->getGeometryContacts();
    CollisionLinkPairListPtr collisionPairs = boost::make_shared<CollisionLinkPairList>();
    for(agxCollide::GeometryContactPtrVector::const_iterator it=contacts.begin();
            it!=contacts.end(); it++){
        CollisionLinkPairPtr dest = boost::make_shared<CollisionLinkPair>();
        agxCollide::ContactPointVector& points = (*it)->points();
        for(int i=0; i<points.size(); i++){
            dest->collisions.push_back(Collision());
            Collision& col = dest->collisions.back();
            agx::Vec3 p = points[i].point();
            agx::Vec3f n = points[i].normal();
            col.depth = points[i].depth();
            for(int j=0; j<3; j++){
                col.point[j] = p[j];
                col.normal[j] = n[j];
            }
        }
        for(int j=0; j<2; j++){
            agx::RigidBody* body = (*it)->rigidBody(j);
            AgXLink* agxLink = dynamic_cast<AgXLinkContainer*>( body->getCustomData() )->agxLink;
            dest->link[j] = agxLink->link;
            dest->body[j] = agxLink->agxBody->body();
        }
        collisionPairs->push_back(dest);
    }
    return collisionPairs;
}


void AgXSimulatorItem::setJointControlMode(Link* joint, ControlMode type){
    impl->setJointControlMode(joint, type);
}


void AgXSimulatorItemImpl::setJointControlMode(Link* joint, AgXSimulatorItem::ControlMode type){

    controlModeMap[joint] = type;
}


void AgXSimulatorItem::setJointCompliance(Link* joint, double spring, double damping){
    impl->setJointCompliance(joint, spring, damping);
}


void AgXSimulatorItemImpl::setJointCompliance(Link* joint, double spring, double damping){

	if( joint->name()=="RLEG_BUSH_ROLL" || joint->name()=="LLEG_BUSH_ROLL"){
		springConstant[0] = spring;
		dampingCoefficient[0] = damping;
	}else if( joint->name()=="RLEG_BUSH_PITCH" || joint->name()=="LLEG_BUSH_PITCH"){
		springConstant[1] = spring;
		dampingCoefficient[1] = damping;
	}else if( joint->name()=="RLEG_BUSH_Z" || joint->name()=="LLEG_BUSH_Z"){
		springConstant[2] = spring;
		dampingCoefficient[2] = damping;
	}
}


void AgXSimulatorItem::doPutProperties(PutPropertyFunction& putProperty)
{
    SimulatorItem::doPutProperties(putProperty);
    impl->doPutProperties(putProperty);
}


void AgXSimulatorItemImpl::doPutProperties(PutPropertyFunction& putProperty)
{
    putProperty(_("Dynamics mode"), dynamicsMode,
            boost::bind(&Selection::selectIndex, &dynamicsMode, _1));
    putProperty(_("Gravity"), str(gravity), boost::bind(toVector3, _1, boost::ref(gravity)));
    putProperty.decimals(2).min(0.0)
            (_("Friction"), friction, changeProperty(friction));
    putProperty.decimals(2).min(0.0)
            (_("Restitution"), restitution, changeProperty(restitution));

    putProperty("Friction Model Type", frictionModelType, changeProperty(frictionModelType));
    putProperty("Friction Solve Type", frictionSolveType, changeProperty(frictionSolveType));

    putProperty.min(-1)
            ("Number of Threads", numThreads, changeProperty(numThreads));
    putProperty("Contact Reduction Mode", contactReductionMode, changeProperty(contactReductionMode));
    putProperty.min(0)
            ("Contact Reduction Bin Resolution", contactReductionBinResolution,
                    changeProperty(contactReductionBinResolution) );
    putProperty.min(0)
            ("Contact Reduction Threshold", contactReductionThreshold,
                    changeProperty(contactReductionThreshold) );

}


bool AgXSimulatorItem::store(Archive& archive)
{
    SimulatorItem::store(archive);
    impl->store(archive);
    return true;
}


void AgXSimulatorItemImpl::store(Archive& archive)
{
    archive.write("dynamicsMode", dynamicsMode.selectedSymbol());
    write(archive, "gravity", gravity);
    archive.write("friction", friction);
    archive.write("restitution", restitution);
    archive.write("frictionModelType", frictionModelType.selectedSymbol());
    archive.write("frictionSolveType", frictionSolveType.selectedSymbol());
    archive.write("numThreads", numThreads);
    archive.write("contactReductionMode", contactReductionMode.selectedSymbol());
    archive.write("contactReductionBinResolution", contactReductionBinResolution);
    archive.write("contactReductionThreshold", contactReductionThreshold);
}


bool AgXSimulatorItem::restore(const Archive& archive)
{
    SimulatorItem::restore(archive);
    impl->restore(archive);
    return true;
}


void AgXSimulatorItemImpl::restore(const Archive& archive)
{
    string symbol;
    if(archive.read("dynamicsMode", symbol)){
        dynamicsMode.select(symbol);
    }
    read(archive, "gravity", gravity);
    archive.read("friction", friction);
    archive.read("restitution", restitution);
    if(archive.read("frictionModelType", symbol)){
        frictionModelType.select(symbol);
    }
    if(archive.read("frictionSolveType", symbol)){
        frictionSolveType.select(symbol);
    }
    archive.read("numThreads", numThreads);
    if(archive.read("contactReductionMode", symbol)){
        contactReductionMode.select(symbol);
    }
    archive.read("contactReductionBinResolution", contactReductionBinResolution);
    archive.read("contactReductionThreshold", contactReductionThreshold);
}


ControllerItem* AgXSimulatorItem::createBodyMotionController(BodyItem* bodyItem, BodyMotionItem* bodyMotionItem)
{
    return new HighGainControllerItem(bodyItem, bodyMotionItem, impl);
}
