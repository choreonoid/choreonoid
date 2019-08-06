/**
   \file
   \brief Implementation of ConstraintForceSolver class
   \author Shin'ichiro Nakaoka
*/

#ifdef __WIN32__
#define NOMINMAX
#endif

#include "DyWorld.h"
#include "DyBody.h"
#include "LinkTraverse.h"
#include "ForwardDynamicsCBM.h"
#include "ConstraintForceSolver.h"
#include "BodyCollisionDetector.h"
#include "MaterialTable.h"
#include <cnoid/IdPair>
#include <cnoid/EigenUtil>
#include <cnoid/AISTCollisionDetector>
#include <cnoid/TimeMeasure>
#include <fmt/format.h>
#include <random>
#include <unordered_map>
#include <limits>
#include <fstream>
#include <iomanip>
#include <iostream>

using namespace std;
using namespace cnoid;

namespace {

typedef CollisionDetector::GeometryHandle GeometryHandle;

// Is LCP solved by Iterative or Pivoting method ?
// #define USE_PIVOTING_LCP
#ifdef USE_PIVOTING_LCP
#include "SimpleLCP_Path.h"
static const bool usePivotingLCP = true;
#else
static const bool usePivotingLCP = false;
#endif

// settings

static const double VEL_THRESH_OF_DYNAMIC_FRICTION = 1.0e-4;

static const bool ENABLE_STATIC_FRICTION = true;
static const bool ONLY_STATIC_FRICTION_FORMULATION = (true && ENABLE_STATIC_FRICTION);
static const bool STATIC_FRICTION_BY_TWO_CONSTRAINTS = true;
static const bool IGNORE_CURRENT_VELOCITY_IN_STATIC_FRICTION = false;

static const bool ENABLE_TRUE_FRICTION_CONE =
    (true && ONLY_STATIC_FRICTION_FORMULATION && STATIC_FRICTION_BY_TWO_CONSTRAINTS);

static const bool SKIP_REDUNDANT_ACCEL_CALC = true;
static const bool ASSUME_SYMMETRIC_MATRIX = false;

static const int DEFAULT_MAX_NUM_GAUSS_SEIDEL_ITERATION = 1000;

//static const int DEFAULT_NUM_GAUSS_SEIDEL_ITERATION_BLOCK = 10;
static const int DEFAULT_NUM_GAUSS_SEIDEL_ITERATION_BLOCK = 1;

static const int DEFAULT_NUM_GAUSS_SEIDEL_INITIAL_ITERATION = 0;
static const double DEFAULT_GAUSS_SEIDEL_ERROR_CRITERION = 1.0e-3;

static const double THRESH_TO_SWITCH_REL_ERROR = 1.0e-8;
//static const double THRESH_TO_SWITCH_REL_ERROR = numeric_limits<double>::epsilon();

static const bool USE_PREVIOUS_LCP_SOLUTION = true;

static const bool ENABLE_CONTACT_DEPTH_CORRECTION = true;

// normal setting
static const double DEFAULT_CONTACT_CORRECTION_DEPTH = 0.0001;
//static const double PENETRATION_A = 500.0;
//static const double PENETRATION_B = 80.0;
static const double DEFAULT_CONTACT_CORRECTION_VELOCITY_RATIO = 1.0;

static const double DEFAULT_CONTACT_CULLING_DISTANCE = 0.005;
static const double DEFAULT_CONTACT_CULLING_DEPTH = 0.05;


// test for mobile robots with wheels
//static const double DEFAULT_CONTACT_CORRECTION_DEPTH = 0.005;
//static const double PENETRATION_A = 500.0;
//static const double PENETRATION_B = 80.0;
//static const double NEGATIVE_VELOCITY_RATIO_FOR_PENETRATION = 10.0;
//static const bool ENABLE_CONTACT_POINT_THINNING = false;

// experimental options
static const bool PROPORTIONAL_DYNAMIC_FRICTION = false;
static const bool ENABLE_RANDOM_STATIC_FRICTION_BASE = false;

// debug options
static const bool CFS_DEBUG = false;
static const bool CFS_DEBUG_VERBOSE = false;
static const bool CFS_DEBUG_VERBOSE_2 = false;
static const bool CFS_DEBUG_VERBOSE_3 = false;
static const bool CFS_DEBUG_LCPCHECK = false;
static const bool CFS_MCP_DEBUG = false;
static const bool CFS_MCP_DEBUG_SHOW_ITERATION_STOP = false;

static const bool CFS_PUT_NUM_CONTACT_POINTS = false;

static const Vector3 local2dConstraintPoints[3] = {
    Vector3( 1.0, 0.0, (-sqrt(3.0) / 2.0)),
    Vector3(-1.0, 0.0, (-sqrt(3.0) / 2.0)),
    Vector3( 0.0, 0.0, ( sqrt(3.0) / 2.0))
};

}

namespace cnoid
{

class ConstraintForceSolverImpl
{
public:
    WorldBase& world;

    bool isConstraintForceOutputMode;
    vector<bool> isSelfCollisionDetectionEnabled;
        
    struct ConstraintPoint {
        int globalIndex;
        Vector3 point;
        Vector3 normalTowardInside[2];
        Vector3 defaultAccel[2];
        double normalProjectionOfRelVelocityOn0;
        double depth; // position error in the case of a connection point

        double mu;
        Vector3 relVelocityOn0;
        int globalFrictionIndex;
        int numFrictionVectors;
        Vector3 frictionVector[4][2];
    };
    typedef std::vector<ConstraintPoint> ConstraintPointArray;

    struct LinkData
    {
        Vector3 dvo;
        Vector3 dw;
        Vector3 pf0;
        Vector3 ptau0;
        double uu;
        double uu0;
        double ddq;
        int numberToCheckAccelCalcSkip;
        int parentIndex;
        DyLink* link;
    };
    typedef std::vector<LinkData> LinkDataArray;

    struct BodyData
    {
        DyBodyPtr body;
        bool isStatic;
        bool hasConstrainedLinks;
        bool isTestForceBeingApplied;
        LinkDataArray linksData;

        Vector3 dpf;
        Vector3 dptau;

        /**
           If the body includes high-gain mode joints,
           the ForwardDynamisCBM object of the body is set to this pointer.
           The pointer is null when all the joints are torque mode and
           the forward dynamics is calculated by ABM.
        */
        shared_ptr<ForwardDynamicsCBM> forwardDynamicsCBM;
    };

    std::vector<BodyData> bodiesData;

    MaterialTablePtr orgMaterialTable;
    MaterialTablePtr materialTable;

    typedef ConstraintForceSolver::CollisionHandler CollisionHandler;

    class ContactMaterialEx : public ContactMaterial
    {
    public:
        double cullingDistance;
        double cullingDepth;
        CollisionHandler collisionHandler;
        Connection collisionHandlerConnection;

        ContactMaterialEx() { }
        ContactMaterialEx(const ContactMaterial& org) : ContactMaterial(org) { }
        ~ContactMaterialEx(){ collisionHandlerConnection.disconnect(); }

        void onCollisionHandlerUnregistered(){
            collisionHandler = CollisionHandler();
            collisionHandlerConnection.disconnect();
        }
    };
    typedef ref_ptr<ContactMaterialEx> ContactMaterialExPtr;
    
    struct CollisionHandlerInfo : public Referenced {
        CollisionHandler handler;
        Signal<void()> sigHandlerUnregisterd;
        ~CollisionHandlerInfo(){
            sigHandlerUnregisterd();
        }
    };
    typedef ref_ptr<CollisionHandlerInfo> CollisionHandlerInfoPtr;

    unordered_map<string, CollisionHandlerInfoPtr> collisionHandlerMap;

    class LinkPair
    {
    public:
        virtual ~LinkPair() { }
        bool isSameBodyPair;
        int bodyIndex[2];
        BodyData* bodyData[2];
        DyLink* link[2];
        LinkData* linkData[2];
        ConstraintPointArray constraintPoints;
        bool isNonContactConstraint;
        ContactMaterialExPtr contactMaterial;
    };

    BodyCollisionDetector bodyCollisionDetector;
    
    unordered_map<Body*, int> bodyIndexMap;
    
    typedef unordered_map<IdPair<GeometryHandle>, LinkPair> GeometryPairToLinkPairMap;
    GeometryPairToLinkPairMap geometryPairToLinkPairMap;

    double defaultStaticFriction;
    double defaultSlipFriction;
    double defaultContactCullingDistance;
    double defaultContactCullingDepth;
    double defaultCoefficientOfRestitution;

    class ExtraJointLinkPair : public LinkPair
    {
    public:
        Vector3 jointPoint[2];
        Vector3 jointConstraintAxes[3];
    };
    typedef std::shared_ptr<ExtraJointLinkPair> ExtraJointLinkPairPtr;
    vector<ExtraJointLinkPairPtr> extraJointLinkPairs;

    bool is2Dmode;
    DyBodyPtr bodyFor2dConstraint;
    BodyData bodyDataFor2dConstraint;

    class Constrain2dLinkPair : public LinkPair
    {
    public:
        double globalYpositions[3];
    };
    typedef std::shared_ptr<Constrain2dLinkPair> Constrain2dLinkPairPtr;
    vector<Constrain2dLinkPairPtr> constrain2dLinkPairs;
        
    std::vector<LinkPair*> constrainedLinkPairs;

    int globalNumConstraintVectors;

    int globalNumContactNormalVectors;
    int globalNumFrictionVectors;

    int prevGlobalNumConstraintVectors;
    int prevGlobalNumFrictionVectors;

    bool areThereImpacts;
    int numUnconverged;

    typedef Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> MatrixX;
    typedef VectorXd VectorX;
        
    // Mlcp * solution + b   _|_  solution

    MatrixX Mlcp;

    // constant acceleration term when no external force is applied
    VectorX an0;
    VectorX at0;

    // constant vector of LCP
    VectorX b;

    // contact force solution: normal forces at contact points
    VectorX solution;

    // random number generator
    std::uniform_real_distribution<double> randomAngle;
    std::mt19937 randomEngine;
    
    // for special version of gauss sidel iterative solver
    std::vector<int> frictionIndexToContactIndex;
    VectorX contactIndexToMu;
    VectorX mcpHi;

    int  maxNumGaussSeidelIteration;
    int  numGaussSeidelInitialIteration;
    double gaussSeidelErrorCriterion;
    double contactCorrectionDepth;
    double contactCorrectionVelocityRatio;

    int numGaussSeidelTotalLoops;
    int numGaussSeidelTotalCalls;
    int numGaussSeidelTotalLoopsMax;


    ConstraintForceSolverImpl(WorldBase& world);
    ~ConstraintForceSolverImpl();
    void initBody(const DyBodyPtr& body, BodyData& bodyData);
    void initWorldExtraJoints();
    void initBodyExtraJoints(int bodyIndex);
    void initExtraJointSub(ExtraJoint& extrajoint, int bodyIndex0, int bodyIndex1);
    void init2Dconstraint(int bodyIndex);
    void initialize(void);
    void initializeContactMaterials();
    ContactMaterialEx* createContactMaterialFromMaterialPair(int material1, int material2);
    void clearExternalForces();
    void solve();
    void setConstraintPoints();
    void extractConstraintPoints(const CollisionPair& collisionPair);
    bool setContactConstraintPoint(LinkPair& linkPair, const Collision& collision);
    void setFrictionVectors(ConstraintPoint& constraintPoint);
    void setExtraJointConstraintPoints(const ExtraJointLinkPairPtr& linkPair);
    void set2dConstraintPoints(const Constrain2dLinkPairPtr& linkPair);
    void putContactPoints();
    void solveImpactConstraints();
    void initMatrices();
    void setAccelCalcSkipInformation();
    void setDefaultAccelerationVector();
    void setAccelerationMatrix();
    void initABMForceElementsWithNoExtForce(BodyData& bodyData);
    void calcABMForceElementsWithTestForce(
        BodyData& bodyData, DyLink* linkToApplyForce, const Vector3& f, const Vector3& tau);
    void calcAccelsABM(BodyData& bodyData, int constraintIndex);
    void calcAccelsMM(BodyData& bodyData, int constraintIndex);
    void extractRelAccelsOfConstraintPoints(
        Eigen::Block<MatrixX>& Kxn, Eigen::Block<MatrixX>& Kxt, int testForceIndex, int constraintIndex);
    void extractRelAccelsFromLinkPairCase1(
        Eigen::Block<MatrixX>& Kxn, Eigen::Block<MatrixX>& Kxt,
        LinkPair& linkPair, int testForceIndex, int constraintIndex);
    void extractRelAccelsFromLinkPairCase2(
        Eigen::Block<MatrixX>& Kxn, Eigen::Block<MatrixX>& Kxt,
        LinkPair& linkPair, int iTestForce, int iDefault, int testForceIndex, int constraintIndex);
    void extractRelAccelsFromLinkPairCase3(
        Eigen::Block<MatrixX>& Kxn, Eigen::Block<MatrixX>& Kxt,
        LinkPair& linkPair, int testForceIndex, int constraintIndex);
    void copySymmetricElementsOfAccelerationMatrix(
        Eigen::Block<MatrixX>& Knn, Eigen::Block<MatrixX>& Ktn, Eigen::Block<MatrixX>& Knt, Eigen::Block<MatrixX>& Ktt);
    void clearSingularPointConstraintsOfClosedLoopConnections();
    void setConstantVectorAndMuBlock();
    void addConstraintForceToLinks();
    void addConstraintForceToLink(LinkPair* linkPair, int ipair);
    void solveMCPByProjectedGaussSeidel(const MatrixX& M, const VectorX& b, VectorX& x);
    void solveMCPByProjectedGaussSeidelMainStep(const MatrixX& M, const VectorX& b, VectorX& x);
    void solveMCPByProjectedGaussSeidelInitial(
        const MatrixX& M, const VectorX& b, VectorX& x, const int numIteration);
    void checkLCPResult(MatrixX& M, VectorX& b, VectorX& x);
    void checkMCPResult(MatrixX& M, VectorX& b, VectorX& x);

#ifdef USE_PIVOTING_LCP
    bool callPathLCPSolver(MatrixX& Mlcp, VectorX& b, VectorX& solution);

    // for PATH solver
    std::vector<double> lb;
    std::vector<double> ub;
    std::vector<int> m_i;
    std::vector<int> m_j;
    std::vector<double> m_ij;
#endif

    ofstream os;

    template<class TMatrix>
    void putMatrix(TMatrix& M, const char *name) {
        if(M.cols() == 1){
            os << "Vector " << name << M << std::endl;
        } else {
            os << "Matrix " << name << ": \n";
            for(int i=0; i < M.rows(); i++){
                for(int j=0; j < M.cols(); j++){
                    os << fmt::format(" {:.50g} ", M(i, j));
                }
                os << std::endl;
            }
        }
    }

    template<class TVector>
    void putVector(const TVector& M, const char *name) {
        os << "Vector " << name << M << std::endl;
    }

    template<class TMatrix>
    void debugPutMatrix(const TMatrix& M, const char *name) {
        if(CFS_DEBUG_VERBOSE) putMatrix(M, name);
    }

    template<class TVector>
    void debugPutVector(const TVector& M, const char *name) {
        if(CFS_DEBUG_VERBOSE) putVector(M, name);
    }

    shared_ptr<CollisionLinkPairList> getCollisions();

#ifdef ENABLE_SIMULATION_PROFILING
    double collisionTime;
    TimeMeasure timer;
#endif

};

}


namespace {
typedef ConstraintForceSolverImpl CFSImpl;
}


CFSImpl::ConstraintForceSolverImpl(WorldBase& world) :
    world(world),
    randomAngle(0.0, 2.0 * PI)
{
    defaultStaticFriction = 1.0;
    defaultSlipFriction = 1.0;
    defaultContactCullingDistance = DEFAULT_CONTACT_CULLING_DISTANCE;
    defaultContactCullingDepth = DEFAULT_CONTACT_CULLING_DEPTH;
    defaultCoefficientOfRestitution = 0.0;
    
    maxNumGaussSeidelIteration = DEFAULT_MAX_NUM_GAUSS_SEIDEL_ITERATION;
    numGaussSeidelInitialIteration = DEFAULT_NUM_GAUSS_SEIDEL_INITIAL_ITERATION;
    gaussSeidelErrorCriterion = DEFAULT_GAUSS_SEIDEL_ERROR_CRITERION;
    contactCorrectionDepth = DEFAULT_CONTACT_CORRECTION_DEPTH;
    contactCorrectionVelocityRatio = DEFAULT_CONTACT_CORRECTION_VELOCITY_RATIO;

    isConstraintForceOutputMode = false;
    isSelfCollisionDetectionEnabled.clear();
    is2Dmode = false;
}


CFSImpl::~ConstraintForceSolverImpl()
{
    if(CFS_DEBUG){
        os.close();
    }
}


void CFSImpl::initBody(const DyBodyPtr& body, BodyData& bodyData)
{
    body->clearExternalForces();
    bodyData.body = body;
    bodyData.linksData.resize(body->numLinks());
    bodyData.hasConstrainedLinks = false;
    bodyData.isTestForceBeingApplied = false;
    bodyData.isStatic = body->isStaticModel();

    LinkDataArray& linksData = bodyData.linksData;
    const int n = body->numLinks();
    for(int i=0; i < n; ++i){
        DyLink* link = body->link(i);
        linksData[link->index()].link = link;
        linksData[link->index()].parentIndex = link->parent() ? link->parent()->index() : -1;
    }
}


void CFSImpl::initWorldExtraJoints()
{
    for(size_t i=0; i < world.extrajoints.size(); i++){
        ExtraJoint& extrajoint = world.extrajoints[i];
        int bodyIndex[2];
        for(int k=0; k < 2; k++){
            if(!extrajoint.body[k]){
                bodyIndex[k] = world.bodyIndex(extrajoint.bodyName[k]);
                Body* body = world.body(bodyIndex[k]);
                extrajoint.body[k] = body;
                extrajoint.link[k] = body->link(extrajoint.linkName[k]);
            }else{
                bodyIndex[k] = world.bodyIndex(extrajoint.body[k]->name());
            }
        }
        initExtraJointSub(extrajoint, bodyIndex[0], bodyIndex[1]);
    }
}


void CFSImpl::initBodyExtraJoints(int bodyIndex)
{
    const DyBodyPtr& body = world.body(bodyIndex);
    for(int j=0; j < body->numExtraJoints(); ++j){
        ExtraJoint& bodyExtraJoint = body->extraJoint(j);
        initExtraJointSub(bodyExtraJoint, bodyIndex, bodyIndex);
    }
}


// initialize extra joints for making closed links
void CFSImpl::initExtraJointSub(ExtraJoint& extrajoint, int bodyIndex0, int bodyIndex1)
{
    ExtraJointLinkPairPtr linkPair;
    linkPair = std::make_shared<ExtraJointLinkPair>();
    linkPair->isSameBodyPair = (bodyIndex0 == bodyIndex1);
    linkPair->isNonContactConstraint = true;

    if(extrajoint.type == ExtraJoint::EJ_PISTON){
        linkPair->constraintPoints.resize(2);
        // generate two vectors orthogonal to the joint axis
        Vector3 u = Vector3::Zero();
        int minElem = 0;
        Vector3 axis = extrajoint.link[0]->Rs() * extrajoint.axis;
        for(int k=1; k < 3; k++){
            if(fabs(axis(k)) < fabs(axis(minElem))){
                minElem = k;
            }
        }
        u(minElem) = 1.0;
        const Vector3 t1 = axis.cross(u).normalized();
        linkPair->jointConstraintAxes[0] = t1;
        linkPair->jointConstraintAxes[1] = axis.cross(t1).normalized();

     } else if(extrajoint.type == ExtraJoint::EJ_BALL){
        linkPair->constraintPoints.resize(3);
        linkPair->jointConstraintAxes[0] = Vector3(1.0, 0.0, 0.0);
        linkPair->jointConstraintAxes[1] = Vector3(0.0, 1.0, 0.0);
        linkPair->jointConstraintAxes[2] = Vector3(0.0, 0.0, 1.0);
     }

    int numConstraints = linkPair->constraintPoints.size();
    for(int k=0; k < numConstraints; ++k){
        ConstraintPoint& constraint = linkPair->constraintPoints[k];
        constraint.numFrictionVectors = 0;
        constraint.globalFrictionIndex = numeric_limits<int>::max();
    }

    int index[2];
    index[0] = bodyIndex0; index[1] = bodyIndex1;
    for(int k=0; k < 2; ++k){
        linkPair->bodyIndex[k] = index[k];
        linkPair->bodyData[k] = &bodiesData[index[k]];
        DyLink* link = static_cast<DyLink*>(extrajoint.link[k]);
        linkPair->link[k] = link;
        BodyData& bodyData = bodiesData[index[k]];
        linkPair->linkData[k] = &(bodyData.linksData[link->index()]);
        linkPair->jointPoint[k] = link->Rs() * extrajoint.point[k];
    }

    extraJointLinkPairs.push_back(linkPair);

}


void CFSImpl::init2Dconstraint(int bodyIndex)
{
    if(!bodyFor2dConstraint){
        bodyFor2dConstraint = new DyBody();
        Link* link = bodyFor2dConstraint->createLink();
        bodyFor2dConstraint->setRootLink(link);
        link->p().setZero();
        link->R().setIdentity();
        initBody(bodyFor2dConstraint, bodyDataFor2dConstraint);
        LinkData& linkData0 = bodyDataFor2dConstraint.linksData[0];
        linkData0.dw.setZero();
        linkData0.dvo.setZero();
    }

    DyLink* rootLink = world.body(bodyIndex)->rootLink();

    Constrain2dLinkPairPtr linkPair = std::make_shared<Constrain2dLinkPair>();
    linkPair->isSameBodyPair = false;
    linkPair->isNonContactConstraint = true;
    
    linkPair->constraintPoints.resize(3);
    for(int i=0; i < 3; ++i){
        ConstraintPoint& constraint = linkPair->constraintPoints[i];
        constraint.numFrictionVectors = 0;
        constraint.globalFrictionIndex = numeric_limits<int>::max();
        linkPair->globalYpositions[i] = (rootLink->R() * local2dConstraintPoints[i] + rootLink->p()).y();
    }
        
    linkPair->bodyIndex[0] = -1;
    linkPair->bodyData[0] = &bodyDataFor2dConstraint;
    linkPair->link[0] = bodyFor2dConstraint->rootLink();
    linkPair->linkData[0] = &bodyDataFor2dConstraint.linksData[0];
    
    linkPair->bodyIndex[1] = bodyIndex;
    linkPair->bodyData[1] = &bodiesData[bodyIndex];
    linkPair->link[1] = rootLink;
    linkPair->linkData[1] = &bodiesData[bodyIndex].linksData[0];
            
    constrain2dLinkPairs.push_back(linkPair);
}
        
    
void CFSImpl::initialize(void)
{
    if(CFS_DEBUG || CFS_MCP_DEBUG){
        static int ntest = 0;
        os.close();
        os.open((string("cfs-log-") + std::to_string(ntest++) + ".log").c_str());
        //os << setprecision(50);
    }

    if(CFS_MCP_DEBUG){
        numGaussSeidelTotalCalls = 0;
        numGaussSeidelTotalLoops = 0;
        numGaussSeidelTotalLoopsMax = 0;
    }

    int numBodies = world.numBodies();

    bodiesData.resize(numBodies);

    if(!bodyCollisionDetector.collisionDetector()){
        bodyCollisionDetector.setCollisionDetector(new AISTCollisionDetector);
    } else {
        bodyCollisionDetector.clearBodies();
    }
    bodyIndexMap.clear();
    geometryPairToLinkPairMap.clear();
    constrainedLinkPairs.clear();

    initializeContactMaterials();
    
    extraJointLinkPairs.clear();
    constrain2dLinkPairs.clear();

    for(int bodyIndex=0; bodyIndex < numBodies; ++bodyIndex){

        DyBody* body = world.body(bodyIndex);
        BodyData& bodyData = bodiesData[bodyIndex];
        initBody(body, bodyData);
        bodyIndexMap[body] = bodyIndex;

        bodyData.forwardDynamicsCBM =
            dynamic_pointer_cast<ForwardDynamicsCBM>(world.forwardDynamics(bodyIndex));

        if(bodyData.isStatic && !(bodyData.forwardDynamicsCBM)){
            LinkDataArray& linksData = bodyData.linksData;
            for(size_t j=0; j < linksData.size(); ++j){
                LinkData& linkData = linksData[j];
                linkData.dw.setZero();
                linkData.dvo.setZero();
            }
        }

        bodyCollisionDetector.addBody(body, isSelfCollisionDetectionEnabled[bodyIndex]);
        
        initBodyExtraJoints(bodyIndex);

        if(is2Dmode && !body->isStaticModel()){
            init2Dconstraint(bodyIndex);
        }
    }

    initWorldExtraJoints();

    bodyCollisionDetector.makeReady();

    prevGlobalNumConstraintVectors = 0;
    prevGlobalNumFrictionVectors = 0;
    numUnconverged = 0;

    if(ENABLE_RANDOM_STATIC_FRICTION_BASE){
        randomEngine.seed();
    }
}


void CFSImpl::initializeContactMaterials()
{
    if(!orgMaterialTable){
        materialTable = new MaterialTable;
    } else {
        string collisionHandlerName;
        materialTable =
            new MaterialTable(
                *orgMaterialTable,
                [&](const ContactMaterial* org){
                    auto cm = new ContactMaterialEx(*org);
                    cm->cullingDistance = cm->info("cullingDistance", defaultContactCullingDistance);
                    cm->cullingDepth = cm->info("cullingDepth", defaultContactCullingDepth);

                    if(cm->info()->read("collisionHandler", collisionHandlerName)){
                        auto iter = collisionHandlerMap.find(collisionHandlerName);
                        if(iter != collisionHandlerMap.end()){
                            auto& info = iter->second;
                            cm->collisionHandler = info->handler;
                            cm->collisionHandlerConnection = 
                                info->sigHandlerUnregisterd.connect(
                                    [cm](){ cm->onCollisionHandlerUnregistered(); });
                        }
                    }

                    return cm;
                });
    }
}


CFSImpl::ContactMaterialEx* CFSImpl::createContactMaterialFromMaterialPair(int material1, int material2)
{
    Material* m1 = materialTable->material(material1);
    Material* m2 = materialTable->material(material2);

    auto cm = new ContactMaterialEx;
    cm->setFriction(std::min(m1->roughness(), m2->roughness()));
    cm->setRestitution(std::max(m1->viscosity(), m2->viscosity()));
    cm->cullingDistance = defaultContactCullingDistance;
    cm->cullingDepth = defaultContactCullingDepth;
    materialTable->setContactMaterial(material1, material2, cm);

    return cm;
}


inline void CFSImpl::clearExternalForces()
{
    for(size_t i=0; i < bodiesData.size(); ++i){
        BodyData& bodyData = bodiesData[i];
        bodyData.body->clearExternalForces();
    }
}


void CFSImpl::solve()
{
    if(CFS_DEBUG){
        os << "Time: " << world.currentTime() << std::endl;
    }

    for(size_t i=0; i < bodiesData.size(); ++i){
        BodyData& data = bodiesData[i];
        data.hasConstrainedLinks = false;
        DyBody* body = data.body;
        const int n = body->numLinks();
        for(int j=0; j < n; ++j){
            body->link(j)->constraintForces().clear();
        }
    }

    bodyCollisionDetector.updatePositions();

    globalNumConstraintVectors = 0;
    globalNumFrictionVectors = 0;
    areThereImpacts = false;

    constrainedLinkPairs.clear();

    setConstraintPoints();

    if(CFS_PUT_NUM_CONTACT_POINTS){
        cout << globalNumContactNormalVectors;
    }

    if(globalNumConstraintVectors > 0){

        if(CFS_DEBUG){
            os << "Num Collisions: " << globalNumContactNormalVectors << std::endl;
        }
        if(CFS_DEBUG_VERBOSE) putContactPoints();

        const bool constraintsSizeChanged = ((globalNumFrictionVectors   != prevGlobalNumFrictionVectors) ||
                                             (globalNumConstraintVectors != prevGlobalNumConstraintVectors));

        if(constraintsSizeChanged){
            initMatrices();
        }

        if(areThereImpacts){
            solveImpactConstraints();
        }

        if(SKIP_REDUNDANT_ACCEL_CALC){
            setAccelCalcSkipInformation();
        }

        setDefaultAccelerationVector();
        setAccelerationMatrix();

        clearSingularPointConstraintsOfClosedLoopConnections();
		
        setConstantVectorAndMuBlock();

        if(CFS_DEBUG_VERBOSE){
            debugPutVector(an0, "an0");
            debugPutVector(at0, "at0");
            debugPutMatrix(Mlcp, "Mlcp");
            debugPutVector(b.head(globalNumConstraintVectors), "b1");
            debugPutVector(b.segment(globalNumConstraintVectors, globalNumFrictionVectors), "b2");
        }

        bool isConverged;
#ifdef USE_PIVOTING_LCP
        isConverged = callPathLCPSolver(Mlcp, b, solution);
#else
        if(!USE_PREVIOUS_LCP_SOLUTION || constraintsSizeChanged){
            solution.setZero();
        }
        solveMCPByProjectedGaussSeidel(Mlcp, b, solution);
        isConverged = true;
#endif

        if(!isConverged){
            ++numUnconverged;
            if(CFS_DEBUG)
                os << "LCP didn't converge" << numUnconverged << std::endl;
        } else {
            if(CFS_DEBUG)
                os << "LCP converged" << std::endl;
            if(CFS_DEBUG_LCPCHECK){
                // checkLCPResult(Mlcp, b, solution);
                checkMCPResult(Mlcp, b, solution);
            }

            addConstraintForceToLinks();
        }
    }

    prevGlobalNumConstraintVectors = globalNumConstraintVectors;
    prevGlobalNumFrictionVectors = globalNumFrictionVectors;
}


void CFSImpl::setConstraintPoints()
{
#ifdef ENABLE_SIMULATION_PROFILING
    timer.begin();
#endif

    bodyCollisionDetector.detectCollisions(
        [&](const CollisionPair& collisionPair){
            extractConstraintPoints(collisionPair); });

#ifdef ENABLE_SIMULATION_PROFILING
        collisionTime = timer.measure();
#endif

    globalNumContactNormalVectors = globalNumConstraintVectors;

    for(size_t i=0; i < extraJointLinkPairs.size(); ++i){
        setExtraJointConstraintPoints(extraJointLinkPairs[i]);
    }

    for(size_t i=0; i < constrain2dLinkPairs.size(); ++i){
        set2dConstraintPoints(constrain2dLinkPairs[i]);
    }
}


void CFSImpl::extractConstraintPoints(const CollisionPair& collisionPair)
{
    LinkPair* pLinkPair;
    
    const IdPair<GeometryHandle> idPair(collisionPair.geometries());
    auto p = geometryPairToLinkPairMap.find(idPair);

    if(p != geometryPairToLinkPairMap.end()){
        pLinkPair = &p->second;
        pLinkPair->constraintPoints.clear();
    } else {
        LinkPair& linkPair = geometryPairToLinkPairMap.insert(make_pair(idPair, LinkPair())).first->second;
        int material[2];
        
        for(int i=0; i < 2; ++i){
            DyLink* link = static_cast<DyLink*>(collisionPair.object(i));
            const int bodyIndex = bodyIndexMap[link->body()];
            linkPair.bodyIndex[i] = bodyIndex;
            BodyData& bodyData = bodiesData[bodyIndex];
            linkPair.bodyData[i] = &bodyData;
            linkPair.link[i] = link;
            linkPair.linkData[i] = &bodyData.linksData[link->index()];
            material[i] = link->materialId();
        }
        linkPair.isSameBodyPair = (linkPair.bodyIndex[0] == linkPair.bodyIndex[1]);
        linkPair.isNonContactConstraint = false;

        linkPair.contactMaterial = static_cast<ContactMaterialEx*>(materialTable->contactMaterial(material[0], material[1]));
        if(!linkPair.contactMaterial){
            linkPair.contactMaterial = createContactMaterialFromMaterialPair(material[0], material[1]);
        }
        
        pLinkPair = &linkPair;
    }

    const vector<Collision>& collisions = collisionPair.collisions();

    auto& collisionHandler = pLinkPair->contactMaterial->collisionHandler;
    if(collisionHandler){
        if(collisionHandler(
               pLinkPair->link[0], pLinkPair->link[1], collisions, pLinkPair->contactMaterial)){
            return; // skip the contact force calculation
        }
    }
    
    pLinkPair->bodyData[0]->hasConstrainedLinks = true;
    pLinkPair->bodyData[1]->hasConstrainedLinks = true;
    
    for(size_t i=0; i < collisions.size(); ++i){
        setContactConstraintPoint(*pLinkPair, collisions[i]);
    }

    if(!pLinkPair->constraintPoints.empty()){
        constrainedLinkPairs.push_back(pLinkPair);
    }
}


/**
   @retuen true if the point is actually added to the constraints
*/
bool CFSImpl::setContactConstraintPoint(LinkPair& linkPair, const Collision& collision)
{
    // skip the contact which has too much depth
    if(collision.depth > linkPair.contactMaterial->cullingDepth){
        return false;
    }
    
    ConstraintPointArray& constraintPoints = linkPair.constraintPoints;
    constraintPoints.push_back(ConstraintPoint());
    ConstraintPoint& contact = constraintPoints.back();

    contact.point = collision.point;

    // dense contact points are eliminated
    int nPrevPoints = constraintPoints.size() - 1;
    for(int i=0; i < nPrevPoints; ++i){
        if((constraintPoints[i].point - contact.point).norm() < linkPair.contactMaterial->cullingDistance){
            constraintPoints.pop_back();
            return false;
        }
    }

    contact.normalTowardInside[1] = collision.normal;
    contact.normalTowardInside[0] = -contact.normalTowardInside[1];
    contact.depth = collision.depth;
    contact.globalIndex = globalNumConstraintVectors++;

    // check velocities
    Vector3 v[2];
    for(int k=0; k < 2; ++k){
        DyLink* link = linkPair.link[k];
        if(link->isRoot() && link->isFixedJoint()){
            v[k].setZero();
        } else {
            v[k] = link->vo() + link->w().cross(contact.point);

            if(link->actuationMode() == Link::JOINT_SURFACE_VELOCITY){
                // tentative
                // invalid depths should be fixed
                if (contact.depth > contactCorrectionDepth * 2.0){
                    contact.depth = contactCorrectionDepth * 2.0;
                }
                const Vector3 axis = link->R() * link->a();
                Vector3 direction;
                if(k==0){
                    direction = axis.cross(collision.normal);
                } else {
                    direction = collision.normal.cross(axis);
                }
                direction.normalize();
                v[k] += link->dq_target() * direction;
            }
        }
    }
    contact.relVelocityOn0 = v[1] - v[0];

    contact.normalProjectionOfRelVelocityOn0 = contact.normalTowardInside[1].dot(contact.relVelocityOn0);

    if(!areThereImpacts){
        if(contact.normalProjectionOfRelVelocityOn0 < -1.0e-6){
            areThereImpacts = true;
        }
    }
    
    Vector3 v_tangent =
        contact.relVelocityOn0 - contact.normalProjectionOfRelVelocityOn0 * contact.normalTowardInside[1];
    
    contact.globalFrictionIndex = globalNumFrictionVectors;
    
    double vt_square = v_tangent.squaredNorm();
    static const double vsqrthresh = VEL_THRESH_OF_DYNAMIC_FRICTION * VEL_THRESH_OF_DYNAMIC_FRICTION;
    bool isSlipping = (vt_square > vsqrthresh);
    contact.mu = isSlipping ? linkPair.contactMaterial->dynamicFriction() : linkPair.contactMaterial->staticFriction();
    
    if( !ONLY_STATIC_FRICTION_FORMULATION && isSlipping){
        contact.numFrictionVectors = 1;
        double vt_mag = sqrt(vt_square);
        Vector3 t1 = v_tangent / vt_mag;
        Vector3 t2 = contact.normalTowardInside[1].cross(t1);
        Vector3 t3 = t2.cross(contact.normalTowardInside[1]);
        contact.frictionVector[0][0] = t3.normalized();
        contact.frictionVector[0][1] = -contact.frictionVector[0][0];
        
        // proportional dynamic friction near zero velocity
        if(PROPORTIONAL_DYNAMIC_FRICTION){
            vt_mag *= 10000.0;
            if(vt_mag < contact.mu){
                contact.mu = vt_mag;
            }
        }
    } else {
        if(ENABLE_STATIC_FRICTION){
            contact.numFrictionVectors = (STATIC_FRICTION_BY_TWO_CONSTRAINTS ? 2 : 4);
            setFrictionVectors(contact);
        } else {
            contact.numFrictionVectors = 0;
        }
    }
    globalNumFrictionVectors += contact.numFrictionVectors;

    return true;
}


void CFSImpl::setFrictionVectors(ConstraintPoint& contact)
{
    Vector3 u = Vector3::Zero();
    int minAxis = 0;
    Vector3& normal = contact.normalTowardInside[0];

    for(int i=1; i < 3; i++){
        if(fabs(normal(i)) < fabs(normal(minAxis))){
            minAxis = i;
        }
    }
    u(minAxis) = 1.0;

    Vector3 t1 = normal.cross(u).normalized();
    Vector3 t2 = normal.cross(t1).normalized();

    if(ENABLE_RANDOM_STATIC_FRICTION_BASE){
        double theta = randomAngle(randomEngine);
        contact.frictionVector[0][0] = cos(theta) * t1 + sin(theta) * t2;
        theta += PI_2;
        contact.frictionVector[1][0] = cos(theta) * t1 + sin(theta) * t2;
    } else {
        contact.frictionVector[0][0] = t1;
        contact.frictionVector[1][0] = t2;
    }

    if(STATIC_FRICTION_BY_TWO_CONSTRAINTS){
        contact.frictionVector[0][1] = -contact.frictionVector[0][0];
        contact.frictionVector[1][1] = -contact.frictionVector[1][0];
    } else {
        contact.frictionVector[2][0] = -contact.frictionVector[0][0];
        contact.frictionVector[3][0] = -contact.frictionVector[1][0];

        contact.frictionVector[0][1] = contact.frictionVector[2][0];
        contact.frictionVector[1][1] = contact.frictionVector[3][0];
        contact.frictionVector[2][1] = contact.frictionVector[0][0];
        contact.frictionVector[3][1] = contact.frictionVector[1][0];
    }
}


void CFSImpl::setExtraJointConstraintPoints(const ExtraJointLinkPairPtr& linkPair)
{
    ConstraintPointArray& constraintPoints = linkPair->constraintPoints;

    DyLink* link0 = linkPair->link[0];
    DyLink* link1 = linkPair->link[1];

    Vector3 point[2];
    point[0].noalias() = link0->p() + link0->R() * linkPair->jointPoint[0];
    point[1].noalias() = link1->p() + link1->R() * linkPair->jointPoint[1];
    Vector3 midPoint = (point[0] + point[1]) / 2.0;
    Vector3 error = midPoint - point[0];

    /*
      if(error.squaredNorm() > (0.04 * 0.04)){
      return false;
      }
    */

    // check velocities
    Vector3 v[2];
    for(int k=0; k < 2; ++k){
        DyLink* link = linkPair->link[k];
        if(link->isRoot() && link->isFixedJoint()){
            v[k].setZero();
        } else {
            v[k] = link->vo() + link->w().cross(point[k]);
        }
    }
    Vector3 relVelocityOn0 = v[1] - v[0];

    int n = linkPair->constraintPoints.size();
    for(int i=0; i < n; ++i){
        ConstraintPoint& constraint = constraintPoints[i];
        const Vector3 axis = link0->R() * linkPair->jointConstraintAxes[i];
        constraint.point = midPoint;
        constraint.normalTowardInside[0] =  axis;
        constraint.normalTowardInside[1] = -axis;
        constraint.depth = axis.dot(error);
        constraint.globalIndex = globalNumConstraintVectors++;
        constraint.normalProjectionOfRelVelocityOn0 = constraint.normalTowardInside[1].dot(relVelocityOn0);
    }
    linkPair->bodyData[0]->hasConstrainedLinks = true;
    linkPair->bodyData[1]->hasConstrainedLinks = true;

    constrainedLinkPairs.push_back(linkPair.get());
}


void CFSImpl::set2dConstraintPoints(const Constrain2dLinkPairPtr& linkPair)
{
    static const Vector3 yAxis(0.0, 1.0, 0.0);
    
    ConstraintPointArray& constraintPoints = linkPair->constraintPoints;
    
    int n = linkPair->constraintPoints.size();
    for(int i=0; i < n; ++i){
        DyLink* link1 = linkPair->link[1];
        Vector3 point1 = link1->p() + link1->R() * local2dConstraintPoints[i];
        ConstraintPoint& constraint = constraintPoints[i];
        constraint.point = point1;
        constraint.normalTowardInside[0] =  yAxis;
        constraint.normalTowardInside[1] = -yAxis;
        constraint.depth = point1.y() - linkPair->globalYpositions[i];
        constraint.globalIndex = globalNumConstraintVectors++;
        constraint.normalProjectionOfRelVelocityOn0 = -(link1->vo() + link1->w().cross(point1)).y();
    }
    linkPair->bodyData[0]->hasConstrainedLinks = true;
    linkPair->bodyData[1]->hasConstrainedLinks = true;

    constrainedLinkPairs.push_back(linkPair.get());
}



void CFSImpl::putContactPoints()
{
    os << "Contact Points\n";
    for(size_t i=0; i < constrainedLinkPairs.size(); ++i){
        LinkPair* linkPair = constrainedLinkPairs[i];
        ExtraJointLinkPair* ejLinkPair = dynamic_cast<ExtraJointLinkPair*>(linkPair);

        if(!ejLinkPair){

            os << " " << linkPair->link[0]->name() << " of " << linkPair->bodyData[0]->body->modelName();
            os << "<-->";
            os << " " << linkPair->link[1]->name() << " of " << linkPair->bodyData[1]->body->modelName();
            os << "\n";
            os << " culling thresh: " << linkPair->contactMaterial->cullingDistance << "\n";

            ConstraintPointArray& constraintPoints = linkPair->constraintPoints;
            for(size_t j=0; j < constraintPoints.size(); ++j){
                ConstraintPoint& contact = constraintPoints[j];
                os << " index " << contact.globalIndex;
                os << " point: " << contact.point;
                os << " normal: " << contact.normalTowardInside[1];
                os << " defaultAccel[0]: " << contact.defaultAccel[0];
                os << " defaultAccel[1]: " << contact.defaultAccel[1];
                os << " normal projectionOfRelVelocityOn0" << contact.normalProjectionOfRelVelocityOn0;
                os << " depth" << contact.depth;
                os << " mu" << contact.mu;
                os << " rel velocity: " << contact.relVelocityOn0;
                os << " friction[0][0]: " << contact.frictionVector[0][0];
                os << " friction[0][1]: " << contact.frictionVector[0][1];
                os << " friction[1][0]: " << contact.frictionVector[1][0];
                os << " friction[1][1]: " << contact.frictionVector[1][1];
                os << "\n";
            }
        }
    }
    os << std::endl;
}


void CFSImpl::solveImpactConstraints()
{
    if(CFS_DEBUG){
        os << "Impacts !" << std::endl;
    }
}


void CFSImpl::initMatrices()
{
    const int n = globalNumConstraintVectors;
    const int m = globalNumFrictionVectors;

    const int dimLCP = usePivotingLCP ? (n + m + m) : (n + m);

    Mlcp.resize(dimLCP, dimLCP);
    b.resize(dimLCP);
    solution.resize(dimLCP);

    if(usePivotingLCP){
        Mlcp.block(0, n + m, n, m).setZero();
        Mlcp.block(n + m, 0, m, n).setZero();
        Mlcp.block(n + m, n, m, m) = -MatrixX::Identity(m, m);
        Mlcp.block(n + m, n + m, m, m).setZero();
        Mlcp.block(n, n + m, m, m).setIdentity();
        b.tail(m).setZero();

    } else {
        frictionIndexToContactIndex.resize(m);
        contactIndexToMu.resize(globalNumContactNormalVectors);
        mcpHi.resize(globalNumContactNormalVectors);
    }

    an0.resize(n);
    at0.resize(m);
}


void CFSImpl::setAccelCalcSkipInformation()
{
    // clear skip check numbers
    for(size_t i=0; i < bodiesData.size(); ++i){
        BodyData& bodyData = bodiesData[i];
        if(bodyData.hasConstrainedLinks){
            LinkDataArray& linksData = bodyData.linksData;
            for(size_t j=0; j < linksData.size(); ++j){
                linksData[j].numberToCheckAccelCalcSkip = numeric_limits<int>::max();
            }
        }
    }

    // add the number of contact points to skip check numbers of the links from a contact target to the root
    int numLinkPairs = constrainedLinkPairs.size();
    for(int i=0; i < numLinkPairs; ++i){
        LinkPair* linkPair = constrainedLinkPairs[i];
        int constraintIndex = linkPair->constraintPoints.front().globalIndex;
        for(int j=0; j < 2; ++j){
            LinkDataArray& linksData = linkPair->bodyData[j]->linksData;
            int linkIndex = linkPair->link[j]->index();
            while(linkIndex >= 0){
                LinkData& linkData = linksData[linkIndex];
                if(linkData.numberToCheckAccelCalcSkip < constraintIndex){
                    break;
                }
                linkData.numberToCheckAccelCalcSkip = constraintIndex;
                linkIndex = linkData.parentIndex;
            }
        }
    }
}


void CFSImpl::setDefaultAccelerationVector()
{
    // calculate accelerations with no constraint force
    for(size_t i=0; i < bodiesData.size(); ++i){
        BodyData& bodyData = bodiesData[i];
        if(bodyData.hasConstrainedLinks && ! bodyData.isStatic){

            if(bodyData.forwardDynamicsCBM){
                bodyData.forwardDynamicsCBM->sumExternalForces();
                bodyData.forwardDynamicsCBM->solveUnknownAccels();
                calcAccelsMM(bodyData, numeric_limits<int>::max());

            } else {
                initABMForceElementsWithNoExtForce(bodyData);
                calcAccelsABM(bodyData, numeric_limits<int>::max());
            }
        }
    }

    // extract accelerations
    for(size_t i=0; i < constrainedLinkPairs.size(); ++i){

        LinkPair& linkPair = *constrainedLinkPairs[i];
        ConstraintPointArray& constraintPoints = linkPair.constraintPoints;

        for(size_t j=0; j < constraintPoints.size(); ++j){
            ConstraintPoint& constraint = constraintPoints[j];

            for(int k=0; k < 2; ++k){
                if(linkPair.bodyData[k]->isStatic){
                    constraint.defaultAccel[k].setZero();
                } else {
                    DyLink* link = linkPair.link[k];
                    LinkData* linkData = linkPair.linkData[k];
                    constraint.defaultAccel[k] =
                        linkData->dvo - constraint.point.cross(linkData->dw) +
                        link->w().cross(link->vo() + link->w().cross(constraint.point));
                }
            }

            Vector3 relDefaultAccel(constraint.defaultAccel[1] - constraint.defaultAccel[0]);
            an0[constraint.globalIndex] = constraint.normalTowardInside[1].dot(relDefaultAccel);

            for(int k=0; k < constraint.numFrictionVectors; ++k){
                at0[constraint.globalFrictionIndex + k] = constraint.frictionVector[k][1].dot(relDefaultAccel);
            }
        }
    }
}


void CFSImpl::setAccelerationMatrix()
{
    const int n = globalNumConstraintVectors;
    const int m = globalNumFrictionVectors;

    Eigen::Block<MatrixX> Knn = Mlcp.block(0, 0, n, n);
    Eigen::Block<MatrixX> Ktn = Mlcp.block(0, n, n, m);
    Eigen::Block<MatrixX> Knt = Mlcp.block(n, 0, m, n);
    Eigen::Block<MatrixX> Ktt = Mlcp.block(n, n, m, m);

    for(size_t i=0; i < constrainedLinkPairs.size(); ++i){

        LinkPair& linkPair = *constrainedLinkPairs[i];
        int numConstraintsInPair = linkPair.constraintPoints.size();

        for(int j=0; j < numConstraintsInPair; ++j){

            ConstraintPoint& constraint = linkPair.constraintPoints[j];
            int constraintIndex = constraint.globalIndex;

            // apply test normal force
            for(int k=0; k < 2; ++k){
                BodyData& bodyData = *linkPair.bodyData[k];
                if(!bodyData.isStatic){

                    bodyData.isTestForceBeingApplied = true;
                    const Vector3& f = constraint.normalTowardInside[k];

                    if(bodyData.forwardDynamicsCBM){
                        //! \todo This code does not work correctly when the links are in the same body. Fix it.
                        Vector3 arm = constraint.point - bodyData.body->rootLink()->p();
                        Vector3 tau = arm.cross(f);
                        Vector3 tauext = constraint.point.cross(f);
                        if(bodyData.forwardDynamicsCBM->solveUnknownAccels(linkPair.link[k], f, tauext, f, tau)){
                            calcAccelsMM(bodyData, constraintIndex);
                        }
                    } else {
                        Vector3 tau = constraint.point.cross(f);
                        calcABMForceElementsWithTestForce(bodyData, linkPair.link[k], f, tau);
                        if(!linkPair.isSameBodyPair || (k > 0)){
                            calcAccelsABM(bodyData, constraintIndex);
                        }
                    }
                }
            }
            extractRelAccelsOfConstraintPoints(Knn, Knt, constraintIndex, constraintIndex);

            // apply test friction force
            for(int l=0; l < constraint.numFrictionVectors; ++l){
                for(int k=0; k < 2; ++k){
                    BodyData& bodyData = *linkPair.bodyData[k];
                    if(!bodyData.isStatic){
                        const Vector3& f = constraint.frictionVector[l][k];

                        if(bodyData.forwardDynamicsCBM){
                            //! \todo This code does not work correctly when the links are in the same body. Fix it.
                            Vector3 arm = constraint.point - bodyData.body->rootLink()->p();
                            Vector3 tau = arm.cross(f);
                            Vector3 tauext = constraint.point.cross(f);
                            if(bodyData.forwardDynamicsCBM->solveUnknownAccels(linkPair.link[k], f, tauext, f, tau)){
                                calcAccelsMM(bodyData, constraintIndex);
                            }
                        } else {
                            Vector3 tau = constraint.point.cross(f);
                            calcABMForceElementsWithTestForce(bodyData, linkPair.link[k], f, tau);
                            if(!linkPair.isSameBodyPair || (k > 0)){
                                calcAccelsABM(bodyData, constraintIndex);
                            }
                        }
                    }
                }
                extractRelAccelsOfConstraintPoints(Ktn, Ktt, constraint.globalFrictionIndex + l, constraintIndex);
            }

            linkPair.bodyData[0]->isTestForceBeingApplied = false;
            linkPair.bodyData[1]->isTestForceBeingApplied = false;
        }
    }

    if(ASSUME_SYMMETRIC_MATRIX){
        copySymmetricElementsOfAccelerationMatrix(Knn, Ktn, Knt, Ktt);
    }
}


void CFSImpl::initABMForceElementsWithNoExtForce(BodyData& bodyData)
{
    bodyData.dpf.setZero();
    bodyData.dptau.setZero();

    std::vector<LinkData>& linksData = bodyData.linksData;
    const LinkTraverse& traverse = bodyData.body->linkTraverse();
    const int n = traverse.numLinks();

    for(int i = n-1; i >= 0; --i){
        DyLink* link = static_cast<DyLink*>(traverse[i]);
        LinkData& data = linksData[i];

        /*
          data.pf0   = link->pf;
          data.ptau0 = link->ptau;
        */
        data.pf0   = link->pf() - link->f_ext();
        data.ptau0 = link->ptau() - link->tau_ext();

        for(DyLink* child = link->child(); child; child = child->sibling()){

            LinkData& childData = linksData[child->index()];

            data.pf0   += childData.pf0;
            data.ptau0 += childData.ptau0;

            if(!child->isFixedJoint()){
                double uu_dd = childData.uu0 / child->dd();
                data.pf0   += uu_dd * child->hhv();
                data.ptau0 += uu_dd * child->hhw();
            }
        }

        if(i > 0){
            if(!link->isFixedJoint()){
                data.uu0  = link->uu() + link->u() - (link->sv().dot(data.pf0) + link->sw().dot(data.ptau0));
                data.uu = data.uu0;
            }
        }
    }
}


void CFSImpl::calcABMForceElementsWithTestForce
(BodyData& bodyData, DyLink* linkToApplyForce, const Vector3& f, const Vector3& tau)
{
    std::vector<LinkData>& linksData = bodyData.linksData;

    Vector3 dpf   = -f;
    Vector3 dptau = -tau;

    DyLink* link = linkToApplyForce;
    while(link->parent()){
        if(!link->isFixedJoint()){
            LinkData& data = linksData[link->index()];
            double duu = -(link->sv().dot(dpf) + link->sw().dot(dptau));
            data.uu += duu;
            double duudd = duu / link->dd();
            dpf   += duudd * link->hhv();
            dptau += duudd * link->hhw();
        }
        link = link->parent();
    }

    bodyData.dpf   += dpf;
    bodyData.dptau += dptau;
}


void CFSImpl::calcAccelsABM(BodyData& bodyData, int constraintIndex)
{
    std::vector<LinkData>& linksData = bodyData.linksData;
    LinkData& rootData = linksData[0];
    DyLink* rootLink = rootData.link;

    if(rootLink->isFreeJoint()){

        Eigen::Matrix<double, 6, 6> M;
        M << rootLink->Ivv(), rootLink->Iwv().transpose(),
            rootLink->Iwv(), rootLink->Iww();

        Eigen::Matrix<double, 6, 1> f;
        f << (rootData.pf0   + bodyData.dpf),
            (rootData.ptau0 + bodyData.dptau);
        f *= -1.0;

        Eigen::Matrix<double, 6, 1> a(M.colPivHouseholderQr().solve(f));

        rootData.dvo = a.head<3>();
        rootData.dw  = a.tail<3>();

    } else {
        rootData.dw .setZero();
        rootData.dvo.setZero();
    }

    // reset
    bodyData.dpf  .setZero();
    bodyData.dptau.setZero();

    int skipCheckNumber = ASSUME_SYMMETRIC_MATRIX ? constraintIndex : (numeric_limits<int>::max() - 1);
    int n = linksData.size();
    for(int linkIndex = 1; linkIndex < n; ++linkIndex){

        LinkData& linkData = linksData[linkIndex];

        if(!SKIP_REDUNDANT_ACCEL_CALC || linkData.numberToCheckAccelCalcSkip <= skipCheckNumber){

            DyLink* link = linkData.link;
            LinkData& parentData = linksData[linkData.parentIndex];

            if(!link->isFixedJoint()){
                linkData.ddq = (linkData.uu - (link->hhv().dot(parentData.dvo) + link->hhw().dot(parentData.dw))) / link->dd();
                linkData.dvo = parentData.dvo + link->cv() + link->sv() * linkData.ddq;
                linkData.dw  = parentData.dw  + link->cw() + link->sw() * linkData.ddq;
            }else{
                linkData.ddq = 0.0;
                linkData.dvo = parentData.dvo;
                linkData.dw  = parentData.dw;
            }

            // reset
            linkData.uu = linkData.uu0;
        }
    }
}


void CFSImpl::calcAccelsMM(BodyData& bodyData, int constraintIndex)
{
    std::vector<LinkData>& linksData = bodyData.linksData;

    LinkData& rootData = linksData[0];
    DyLink* rootLink = rootData.link;
    rootData.dvo = rootLink->dvo();
    rootData.dw  = rootLink->dw();

    const int skipCheckNumber = ASSUME_SYMMETRIC_MATRIX ? constraintIndex : (numeric_limits<int>::max() - 1);
    const int n = linksData.size();

    for(int linkIndex = 1; linkIndex < n; ++linkIndex){

        LinkData& linkData = linksData[linkIndex];

        if(!SKIP_REDUNDANT_ACCEL_CALC || linkData.numberToCheckAccelCalcSkip <= skipCheckNumber){

            DyLink* link = linkData.link;
            LinkData& parentData = linksData[linkData.parentIndex];
            if(!link->isFixedJoint()){
                linkData.dvo = parentData.dvo + link->cv() + link->ddq() * link->sv();
                linkData.dw  = parentData.dw  + link->cw() + link->ddq() * link->sw();
            }else{
                linkData.dvo = parentData.dvo;
                linkData.dw  = parentData.dw;
            }
        }
    }
}


void CFSImpl::extractRelAccelsOfConstraintPoints
(Eigen::Block<MatrixX>& Kxn, Eigen::Block<MatrixX>& Kxt, int testForceIndex, int constraintIndex)
{
    int maxConstraintIndexToExtract = ASSUME_SYMMETRIC_MATRIX ? constraintIndex : globalNumConstraintVectors;


    for(size_t i=0; i < constrainedLinkPairs.size(); ++i){

        LinkPair& linkPair = *constrainedLinkPairs[i];

        BodyData& bodyData0 = *linkPair.bodyData[0];
        BodyData& bodyData1 = *linkPair.bodyData[1];

        if(bodyData0.isTestForceBeingApplied){
            if(bodyData1.isTestForceBeingApplied){
                extractRelAccelsFromLinkPairCase1(Kxn, Kxt, linkPair, testForceIndex, maxConstraintIndexToExtract);
            } else {
                extractRelAccelsFromLinkPairCase2(Kxn, Kxt, linkPair, 0, 1, testForceIndex, maxConstraintIndexToExtract);
            }
        } else {
            if(bodyData1.isTestForceBeingApplied){
                extractRelAccelsFromLinkPairCase2(Kxn, Kxt, linkPair, 1, 0, testForceIndex, maxConstraintIndexToExtract);
            } else {
                extractRelAccelsFromLinkPairCase3(Kxn, Kxt, linkPair, testForceIndex, maxConstraintIndexToExtract);
            }
        }
    }
}


void CFSImpl::extractRelAccelsFromLinkPairCase1
(Eigen::Block<MatrixX>& Kxn, Eigen::Block<MatrixX>& Kxt,
 LinkPair& linkPair, int testForceIndex, int maxConstraintIndexToExtract)
{
    ConstraintPointArray& constraintPoints = linkPair.constraintPoints;

    for(size_t i=0; i < constraintPoints.size(); ++i){

        ConstraintPoint& constraint = constraintPoints[i];
        int constraintIndex = constraint.globalIndex;

        if(ASSUME_SYMMETRIC_MATRIX && constraintIndex > maxConstraintIndexToExtract){
            break;
        }

        DyLink* link0 = linkPair.link[0];
        DyLink* link1 = linkPair.link[1];
        LinkData* linkData0 = linkPair.linkData[0];
        LinkData* linkData1 = linkPair.linkData[1];

        //! \todo Can the follwoing equations be simplified ?
        Vector3 dv0 =
            linkData0->dvo - constraint.point.cross(linkData0->dw) +
            link0->w().cross(link0->vo() + link0->w().cross(constraint.point));

        Vector3 dv1 =
            linkData1->dvo - constraint.point.cross(linkData1->dw) +
            link1->w().cross(link1->vo() + link1->w().cross(constraint.point));

        Vector3 relAccel = dv1 - dv0;

        Kxn(constraintIndex, testForceIndex) = constraint.normalTowardInside[1].dot(relAccel) - an0(constraintIndex);

        for(int j=0; j < constraint.numFrictionVectors; ++j){
            const int index = constraint.globalFrictionIndex + j;
            Kxt(index, testForceIndex) = constraint.frictionVector[j][1].dot(relAccel) - at0(index);
        }
    }
}


void CFSImpl::extractRelAccelsFromLinkPairCase2
(Eigen::Block<MatrixX>& Kxn, Eigen::Block<MatrixX>& Kxt,
 LinkPair& linkPair, int iTestForce, int iDefault, int testForceIndex, int maxConstraintIndexToExtract)
{
    ConstraintPointArray& constraintPoints = linkPair.constraintPoints;

    for(size_t i=0; i < constraintPoints.size(); ++i){

        ConstraintPoint& constraint = constraintPoints[i];
        int constraintIndex = constraint.globalIndex;

        if(ASSUME_SYMMETRIC_MATRIX && constraintIndex > maxConstraintIndexToExtract){
            break;
        }

        DyLink* link = linkPair.link[iTestForce];
        LinkData* linkData = linkPair.linkData[iTestForce];

        Vector3 dv(linkData->dvo - constraint.point.cross(linkData->dw) + link->w().cross(link->vo() + link->w().cross(constraint.point)));

        if(CFS_DEBUG_VERBOSE_2){
            os << "dv " << constraintIndex << " = " << dv << "\n";
        }

        Vector3 relAccel = constraint.defaultAccel[iDefault] - dv;

        Kxn(constraintIndex, testForceIndex) = constraint.normalTowardInside[iDefault].dot(relAccel) - an0(constraintIndex);

        for(int j=0; j < constraint.numFrictionVectors; ++j){
            const int index = constraint.globalFrictionIndex + j;
            Kxt(index, testForceIndex) = constraint.frictionVector[j][iDefault].dot(relAccel) - at0(index);
        }

    }
}


void CFSImpl::extractRelAccelsFromLinkPairCase3
(Eigen::Block<MatrixX>& Kxn, Eigen::Block<MatrixX>& Kxt, LinkPair& linkPair, int testForceIndex, int maxConstraintIndexToExtract)
{
    ConstraintPointArray& constraintPoints = linkPair.constraintPoints;

    for(size_t i=0; i < constraintPoints.size(); ++i){

        ConstraintPoint& constraint = constraintPoints[i];
        int constraintIndex = constraint.globalIndex;

        if(ASSUME_SYMMETRIC_MATRIX && constraintIndex > maxConstraintIndexToExtract){
            break;
        }

        Kxn(constraintIndex, testForceIndex) = 0.0;

        for(int j=0; j < constraint.numFrictionVectors; ++j){
            Kxt(constraint.globalFrictionIndex + j, testForceIndex) = 0.0;
        }
    }
}


void CFSImpl::copySymmetricElementsOfAccelerationMatrix
(Eigen::Block<MatrixX>& Knn, Eigen::Block<MatrixX>& Ktn, Eigen::Block<MatrixX>& Knt, Eigen::Block<MatrixX>& Ktt)
{
    for(size_t linkPairIndex=0; linkPairIndex < constrainedLinkPairs.size(); ++linkPairIndex){

        ConstraintPointArray& constraintPoints = constrainedLinkPairs[linkPairIndex]->constraintPoints;

        for(size_t localConstraintIndex = 0; localConstraintIndex < constraintPoints.size(); ++localConstraintIndex){

            ConstraintPoint& constraint = constraintPoints[localConstraintIndex];

            int constraintIndex = constraint.globalIndex;
            int nextConstraintIndex = constraintIndex + 1;
            for(int i = nextConstraintIndex; i < globalNumConstraintVectors; ++i){
                Knn(i, constraintIndex) = Knn(constraintIndex, i);
            }
            int frictionTopOfNextConstraint = constraint.globalFrictionIndex + constraint.numFrictionVectors;
            for(int i = frictionTopOfNextConstraint; i < globalNumFrictionVectors; ++i){
                Knt(i, constraintIndex) = Ktn(constraintIndex, i);
            }

            for(int localFrictionIndex=0; localFrictionIndex < constraint.numFrictionVectors; ++localFrictionIndex){

                int frictionIndex = constraint.globalFrictionIndex + localFrictionIndex;

                for(int i = nextConstraintIndex; i < globalNumConstraintVectors; ++i){
                    Ktn(i, frictionIndex) = Knt(frictionIndex, i);
                }
                for(int i = frictionTopOfNextConstraint; i < globalNumFrictionVectors; ++i){
                    Ktt(i, frictionIndex) = Ktt(frictionIndex, i);
                }
            }
        }
    }
}


void CFSImpl::clearSingularPointConstraintsOfClosedLoopConnections()
{
    for(int i = 0; i < Mlcp.rows(); ++i){
        if(Mlcp(i, i) < 1.0e-4){
            for(int j=0; j < Mlcp.rows(); ++j){
                Mlcp(j, i) = 0.0;
            }
            Mlcp(i, i) = numeric_limits<double>::max();
        }
    }
}


void CFSImpl::setConstantVectorAndMuBlock()
{
    double dtinv = 1.0 / world.timeStep();
    const int block2 = globalNumConstraintVectors;
    const int block3 = globalNumConstraintVectors + globalNumFrictionVectors;

    for(size_t i=0; i < constrainedLinkPairs.size(); ++i){

        LinkPair& linkPair = *constrainedLinkPairs[i];
        int numConstraintsInPair = linkPair.constraintPoints.size();

        for(int j=0; j < numConstraintsInPair; ++j){
            ConstraintPoint& constraint = linkPair.constraintPoints[j];
            int globalIndex = constraint.globalIndex;

            // set constant vector of LCP

            // constraints for normal acceleration

            if(linkPair.isNonContactConstraint){
                // connection constraint

                const double& error = constraint.depth;
                double v;
                if(error >= 0){
                    v = 0.1 * (-1.0 + exp(-error * 20.0));
                } else {
                    v = 0.1 * ( 1.0 - exp( error * 20.0));
                }
					
                b(globalIndex) = an0(globalIndex) + (constraint.normalProjectionOfRelVelocityOn0 + v) * dtinv;

            } else {
                // contact constraint
                if(ENABLE_CONTACT_DEPTH_CORRECTION){
                    double velOffset;
                    const double depth = constraint.depth - contactCorrectionDepth;
                    if(depth <= 0.0){
                        velOffset = contactCorrectionVelocityRatio * depth;
                    } else {
                        velOffset = contactCorrectionVelocityRatio * (-1.0 / (depth + 1.0) + 1.0);
                    }
                    b(globalIndex) = an0(globalIndex) + (constraint.normalProjectionOfRelVelocityOn0 - velOffset) * dtinv;
                } else {
                    b(globalIndex) = an0(globalIndex) + constraint.normalProjectionOfRelVelocityOn0 * dtinv;
                }

                contactIndexToMu[globalIndex] = constraint.mu;

                int globalFrictionIndex = constraint.globalFrictionIndex;
                for(int k=0; k < constraint.numFrictionVectors; ++k){

                    // constraints for tangent acceleration
                    double tangentProjectionOfRelVelocity = constraint.frictionVector[k][1].dot(constraint.relVelocityOn0);

                    b(block2 + globalFrictionIndex) = at0(globalFrictionIndex);
                    if( !IGNORE_CURRENT_VELOCITY_IN_STATIC_FRICTION || constraint.numFrictionVectors == 1){
                        b(block2 + globalFrictionIndex) += tangentProjectionOfRelVelocity * dtinv;
                    }

                    if(usePivotingLCP){
                        // set mu (coefficients of friction)
                        Mlcp(block3 + globalFrictionIndex, globalIndex) = constraint.mu;
                    } else {
                        // for iterative solver
                        frictionIndexToContactIndex[globalFrictionIndex] = globalIndex;
                    }

                    ++globalFrictionIndex;
                }
            }
        }
    }
}


void CFSImpl::addConstraintForceToLinks()
{
    int n = constrainedLinkPairs.size();
    for(int i=0; i < n; ++i){
        LinkPair* linkPair = constrainedLinkPairs[i];
        for(int j=0; j < 2; ++j){
            // if(!linkPair->link[j]->isRoot() || linkPair->link[j]->jointType != Link::FIXED_JOINT){
            addConstraintForceToLink(linkPair, j);
            // }
        }
    }
}


void CFSImpl::addConstraintForceToLink(LinkPair* linkPair, int ipair)
{
    Vector3 f_total   = Vector3::Zero();
    Vector3 tau_total = Vector3::Zero();

    ConstraintPointArray& constraintPoints = linkPair->constraintPoints;
    int numConstraintPoints = constraintPoints.size();
    DyLink* link = linkPair->link[ipair];

    for(int i=0; i < numConstraintPoints; ++i){

        ConstraintPoint& constraint = constraintPoints[i];
        int globalIndex = constraint.globalIndex;

        Vector3 f = solution(globalIndex) * constraint.normalTowardInside[ipair];

        for(int j=0; j < constraint.numFrictionVectors; ++j){
            f += solution(globalNumConstraintVectors + constraint.globalFrictionIndex + j) * constraint.frictionVector[j][ipair];
        }

        f_total   += f;
        tau_total += constraint.point.cross(f);

        if(isConstraintForceOutputMode){
            link->constraintForces().push_back(DyLink::ConstraintForce(constraint.point, f));
        }
    }
    
    link->f_ext()   += f_total;
    link->tau_ext() += tau_total;


    if(CFS_DEBUG){
        os << "Constraint force to " << link->name() << ": f = " << f_total << ", tau = " << tau_total << std::endl;
    }
}



void CFSImpl::solveMCPByProjectedGaussSeidel(const MatrixX& M, const VectorX& b, VectorX& x)
{
    static const int loopBlockSize = DEFAULT_NUM_GAUSS_SEIDEL_ITERATION_BLOCK;

    if(numGaussSeidelInitialIteration > 0){
        solveMCPByProjectedGaussSeidelInitial(M, b, x, numGaussSeidelInitialIteration);
    }

    int numBlockLoops = maxNumGaussSeidelIteration / loopBlockSize;
    if(numBlockLoops==0){
        numBlockLoops = 1;
    }

    if(CFS_MCP_DEBUG){
        os << "Iteration ";
    }

    double error = 0.0;
    VectorXd x0;
    int i = 0;
    while(i < numBlockLoops){
        i++;

        for(int j=0; j < loopBlockSize - 1; ++j){
            solveMCPByProjectedGaussSeidelMainStep(M, b, x);
        }

        x0 = x;
        solveMCPByProjectedGaussSeidelMainStep(M, b, x);

        if(true){
            double n = x.norm();
            if(n > THRESH_TO_SWITCH_REL_ERROR){
                error = (x - x0).norm() / x.norm();
            } else {
                error = (x - x0).norm();
            }
        } else {
            error = 0.0;
            for(int j=0; j < x.size(); ++j){
                double d = fabs(x(j) - x0(j));
                if(d > THRESH_TO_SWITCH_REL_ERROR){
                    d /= x(j);
                }
                if(d > error){
                    error = d;
                }
            }
        }

        if(error < gaussSeidelErrorCriterion){
            if(CFS_MCP_DEBUG_SHOW_ITERATION_STOP){
                os << "stopped at " << (i * loopBlockSize) << ", error = " << error << endl;
            }
            break;
        }
    }

    if(CFS_MCP_DEBUG){

        if(i == numBlockLoops){
            os << "not stopped" << ", error = " << error << endl;
        }
        
        int n = loopBlockSize * i;
        numGaussSeidelTotalLoops += n;
        numGaussSeidelTotalCalls++;
        numGaussSeidelTotalLoopsMax = std::max(numGaussSeidelTotalLoopsMax, n);
        os << ", avarage = " << (numGaussSeidelTotalLoops / numGaussSeidelTotalCalls);
        os << ", max = " << numGaussSeidelTotalLoopsMax;
        os << endl;
    }
}


void CFSImpl::solveMCPByProjectedGaussSeidelMainStep(const MatrixX& M, const VectorX& b, VectorX& x)
{
    const int size = globalNumConstraintVectors + globalNumFrictionVectors;

    for(int j=0; j < globalNumContactNormalVectors; ++j){

        double xx;
        if(M(j,j) == numeric_limits<double>::max()){
            xx=0.0;
        } else {
            double sum = -M(j, j) * x(j);
            for(int k=0; k < size; ++k){
                sum += M(j, k) * x(k);
            }
            xx = (-b(j) - sum) / M(j, j);
        }
        if(xx < 0.0){
            x(j) = 0.0;
        } else {
            x(j) = xx;
        }
        mcpHi[j] = contactIndexToMu[j] * x(j);
    }
    
    for(int j=globalNumContactNormalVectors; j < globalNumConstraintVectors; ++j){
        
        if(M(j,j) == numeric_limits<double>::max()){
            x(j)=0.0;
        } else {
            double sum = -M(j, j) * x(j);
            for(int k=0; k < size; ++k){
                sum += M(j, k) * x(k);
            }
            x(j) = (-b(j) - sum) / M(j, j);
        }
    }
    
    
    if(ENABLE_TRUE_FRICTION_CONE){

        int contactIndex = 0;
        for(int j=globalNumConstraintVectors; j < size; ++j, ++contactIndex){
            
            double fx0;
            if(M(j,j) == numeric_limits<double>::max()) {
                fx0 = 0.0;
            } else {
                double sum = -M(j, j) * x(j);
                for(int k=0; k < size; ++k){
                    sum += M(j, k) * x(k);
                }
                fx0 = (-b(j) - sum) / M(j, j);
            }
            double& fx = x(j);
            
            ++j;
            
            double fy0;
            if(M(j,j) == numeric_limits<double>::max()) {
                fy0=0.0;
            } else {
                double sum = -M(j, j) * x(j);
                for(int k=0; k < size; ++k){
                    sum += M(j, k) * x(k);
                }
                fy0 = (-b(j) - sum) / M(j, j);
            }
            double& fy = x(j);
            
            const double fmax = mcpHi[contactIndex];
            const double fmax2 = fmax * fmax;
            const double fmag2 = fx0 * fx0 + fy0 * fy0;

            if(fmag2 > fmax2){
                const double s = fmax / sqrt(fmag2);
                fx = s * fx0;
                fy = s * fy0;
            } else {
                fx = fx0;
                fy = fy0;
            }
        }
        
    } else {

        int frictionIndex = 0;
        for(int j=globalNumConstraintVectors; j < size; ++j, ++frictionIndex){

            double xx;
            if(M(j,j) == numeric_limits<double>::max()) {
                xx=0.0;
            } else {
                double sum = -M(j, j) * x(j);
                for(int k=0; k < size; ++k){
                    sum += M(j, k) * x(k);
                }
                xx = (-b(j) - sum) / M(j, j);
            }
            
            const int contactIndex = frictionIndexToContactIndex[frictionIndex];
            const double fmax = mcpHi[contactIndex];
            const double fmin = (STATIC_FRICTION_BY_TWO_CONSTRAINTS ? -fmax : 0.0);
            
            if(xx < fmin){
                x(j) = fmin;
            } else if(xx > fmax){
                x(j) = fmax;
            } else {
                x(j) = xx;
            }
        }
    }
}


void CFSImpl::solveMCPByProjectedGaussSeidelInitial
(const MatrixX& M, const VectorX& b, VectorX& x, const int numIteration)
{
    const int size = globalNumConstraintVectors + globalNumFrictionVectors;

    const double rstep = 1.0 / (numIteration * size);
    double r = 0.0;

    for(int i=0; i < numIteration; ++i){

        for(int j=0; j < globalNumContactNormalVectors; ++j){

            double xx;
            if(M(j,j)==numeric_limits<double>::max()){
                xx=0.0;
            } else {
                double sum = -M(j, j) * x(j);
                for(int k=0; k < size; ++k){
                    sum += M(j, k) * x(k);
                }
                xx = (-b(j) - sum) / M(j, j);
            }
            if(xx < 0.0){
                x(j) = 0.0;
            } else {
                x(j) = r * xx;
            }
            r += rstep;
            mcpHi[j] = contactIndexToMu[j] * x(j);
        }

        for(int j=globalNumContactNormalVectors; j < globalNumConstraintVectors; ++j){

            if(M(j,j)==numeric_limits<double>::max()){
                x(j) = 0.0;
            } else {
                double sum = -M(j, j) * x(j);
                for(int k=0; k < size; ++k){
                    sum += M(j, k) * x(k);
                }
                x(j) = r * (-b(j) - sum) / M(j, j);
            }
            r += rstep;
        }

        if(ENABLE_TRUE_FRICTION_CONE){

            int contactIndex = 0;
            for(int j=globalNumConstraintVectors; j < size; ++j, ++contactIndex){

                double fx0;
                if(M(j,j)==numeric_limits<double>::max())
                    fx0 = 0.0;
                else{
                    double sum = -M(j, j) * x(j);
                    for(int k=0; k < size; ++k){
                        sum += M(j, k) * x(k);
                    }
                    fx0 = (-b(j) - sum) / M(j, j);
                }
                double& fx = x(j);

                ++j;

                double fy0;
                if(M(j,j)==numeric_limits<double>::max())
                    fy0 = 0.0;
                else{
                    double sum = -M(j, j) * x(j);
                    for(int k=0; k < size; ++k){
                        sum += M(j, k) * x(k);
                    }
                    fy0 = (-b(j) - sum) / M(j, j);
                }
                double& fy = x(j);

                const double fmax = mcpHi[contactIndex];
                const double fmax2 = fmax * fmax;
                const double fmag2 = fx0 * fx0 + fy0 * fy0;

                if(fmag2 > fmax2){
                    const double s = r * fmax / sqrt(fmag2);
                    fx = s * fx0;
                    fy = s * fy0;
                } else {
                    fx = r * fx0;
                    fy = r * fy0;
                }
                r += (rstep + rstep);
            }

        } else {

            int frictionIndex = 0;
            for(int j=globalNumConstraintVectors; j < size; ++j, ++frictionIndex){

                double xx;
                if(M(j,j)==numeric_limits<double>::max())
                    xx = 0.0;
                else{
                    double sum = -M(j, j) * x(j);
                    for(int k=0; k < size; ++k){
                        sum += M(j, k) * x(k);
                    }
                    xx = (-b(j) - sum) / M(j, j);
                }

                const int contactIndex = frictionIndexToContactIndex[frictionIndex];
                const double fmax = mcpHi[contactIndex];
                const double fmin = (STATIC_FRICTION_BY_TWO_CONSTRAINTS ? -fmax : 0.0);

                if(xx < fmin){
                    x(j) = fmin;
                } else if(xx > fmax){
                    x(j) = fmax;
                } else {
                    x(j) = xx;
                }
                x(j) *= r;
                r += rstep;
            }
        }
    }
}


void CFSImpl::checkLCPResult(MatrixX& M, VectorX& b, VectorX& x)
{
    os << "check LCP result\n";
    os << "-------------------------------\n";

    VectorX z = M * x + b;

    int n = x.size();
    for(int i=0; i < n; ++i){
        os << "(" << x(i) << ", " << z(i) << ")";

        if(x(i) < 0.0 || z(i) < 0.0 || x(i) * z(i) != 0.0){
            os << " - X";
        }
        os << "\n";

        if(i == globalNumConstraintVectors){
            os << "-------------------------------\n";
        } else if(i == globalNumConstraintVectors + globalNumFrictionVectors){
            os << "-------------------------------\n";
        }
    }

    os << "-------------------------------\n";


    os << std::endl;
}


void CFSImpl::checkMCPResult(MatrixX& M, VectorX& b, VectorX& x)
{
    os << "check MCP result\n";
    os << "-------------------------------\n";

    VectorX z = M * x + b;

    for(int i=0; i < globalNumConstraintVectors; ++i){
        os << "(" << x(i) << ", " << z(i) << ")";

        if(x(i) < 0.0 || z(i) < -1.0e-6){
            os << " - X";
        } else if(x(i) > 0.0 && fabs(z(i)) > 1.0e-6){
            os << " - X";
        } else if(z(i) > 1.0e-6 && fabs(x(i)) > 1.0e-6){
            os << " - X";
        }


        os << "\n";
    }

    os << "-------------------------------\n";

    int j = 0;
    for(int i=globalNumConstraintVectors; i < globalNumConstraintVectors + globalNumFrictionVectors; ++i, ++j){
        os << "(" << x(i) << ", " << z(i) << ")";

        int contactIndex = frictionIndexToContactIndex[j];
        double hi = contactIndexToMu[contactIndex] * x(contactIndex);

        os << " hi = " << hi;

        if(x(i) < 0.0 || x(i) > hi){
            os << " - X";
        } else if(x(i) == hi && z(i) > -1.0e-6){
            os << " - X";
        } else if(x(i) < hi && x(i) > 0.0 && fabs(z(i)) > 1.0e-6){
            os << " - X";
        }
        os << "\n";
    }

    os << "-------------------------------\n";

    os << std::endl;
}


#ifdef USE_PIVOTING_LCP
bool CFSImpl::callPathLCPSolver(MatrixX& Mlcp, VectorX& b, VectorX& solution)
{
    int size = solution.size();
    int square = size * size;
    std::vector<double> lb(size + 1, 0.0);
    std::vector<double> ub(size + 1, 1.0e20);

    int m_nnz = 0;
    std::vector<int> m_i(square + 1);
    std::vector<int> m_j(square + 1);
    std::vector<double> m_ij(square + 1);

    for(int i=0; i < size; ++i){
        solution(i) = 0.0;
    }

    for(int j=0; j < size; ++j){
        for(int i=0; i < size; ++i){
            double v = Mlcp(i, j);
            if(v != 0.0){
                m_i[m_nnz] = i+1;
                m_j[m_nnz] = j+1;
                m_ij[m_nnz] = v;
                ++m_nnz;
            }
        }
    }

    MCP_Termination status;

    SimpleLCP(size, m_nnz, &m_i[0], &m_j[0], &m_ij[0], &b(0), &lb[0], &ub[0], &status, &solution(0));

    return (status == MCP_Solved);
}
#endif


ConstraintForceSolver::ConstraintForceSolver(WorldBase& world)
{
    impl = new CFSImpl(world);
}


ConstraintForceSolver::~ConstraintForceSolver()
{
    delete impl;
}


void ConstraintForceSolver::setCollisionDetector(CollisionDetector* detector)
{
    impl->bodyCollisionDetector.setCollisionDetector(detector);
}


CollisionDetector* ConstraintForceSolver::collisionDetector()
{
    return impl->bodyCollisionDetector.collisionDetector();
}


void ConstraintForceSolver::setMaterialTable(MaterialTable* table)
{
    impl->orgMaterialTable = table;
}


void ConstraintForceSolver::setFriction(double staticFriction, double slipFliction)
{
    impl->defaultStaticFriction = staticFriction;
    impl->defaultSlipFriction = slipFliction;
}


double ConstraintForceSolver::staticFriction() const
{
    return impl->defaultStaticFriction;
}


double ConstraintForceSolver::slipFriction() const
{
    return impl->defaultSlipFriction;
}


void ConstraintForceSolver::registerCollisionHandler(const std::string& name, CollisionHandler handler)
{
    CFSImpl::CollisionHandlerInfoPtr info = new CFSImpl::CollisionHandlerInfo;;
    info->handler = handler;
    impl->collisionHandlerMap[name] = info;
}


bool ConstraintForceSolver::unregisterCollisionHandler(const std::string& name)
{
    return (impl->collisionHandlerMap.erase(name) > 0);
}


void ConstraintForceSolver::setSelfCollisionDetectionEnabled(int bodyIndex, bool on)
{
    vector<bool>& isSCE = impl->isSelfCollisionDetectionEnabled;
    if(bodyIndex >= isSCE.size()){
        isSCE.resize(bodyIndex+1);
    }
    isSCE[bodyIndex] = on;
}


bool ConstraintForceSolver::isSelfCollisionDetectionEnabled(int bodyIndex) const
{
    vector<bool>& isSCE = impl->isSelfCollisionDetectionEnabled;
    if(bodyIndex < isSCE.size()){
        return impl->isSelfCollisionDetectionEnabled[bodyIndex];
    }else{
        return false;
    }
}
    

void ConstraintForceSolver::setContactCullingDistance(double distance)
{
    impl->defaultContactCullingDistance = distance;
}


double ConstraintForceSolver::contactCullingDistance() const
{
    return impl->defaultContactCullingDistance;
}


void ConstraintForceSolver::setContactCullingDepth(double depth)
{
    impl->defaultContactCullingDepth = depth;
}


double ConstraintForceSolver::contactCullingDepth()
{
    return impl->defaultContactCullingDepth;
}


void ConstraintForceSolver::setCoefficientOfRestitution(double epsilon)
{
    impl->defaultCoefficientOfRestitution = epsilon;
}


double ConstraintForceSolver::coefficientOfRestitution() const
{
    return impl->defaultCoefficientOfRestitution;
}


void ConstraintForceSolver::setGaussSeidelErrorCriterion(double e)
{
    impl->gaussSeidelErrorCriterion = e;
}


double ConstraintForceSolver::gaussSeidelErrorCriterion()
{
    return impl->gaussSeidelErrorCriterion;
}


void ConstraintForceSolver::setGaussSeidelMaxNumIterations(int n)
{
    impl->maxNumGaussSeidelIteration = n;
}


int ConstraintForceSolver::gaussSeidelMaxNumIterations()
{
    return impl->maxNumGaussSeidelIteration;
}


void ConstraintForceSolver::setContactDepthCorrection(double depth, double velocityRatio)
{
    impl->contactCorrectionDepth = depth;
    impl->contactCorrectionVelocityRatio = velocityRatio;
}


double ConstraintForceSolver::contactCorrectionDepth()
{
    return impl->contactCorrectionDepth;
}


double ConstraintForceSolver::contactCorrectionVelocityRatio()
{
    return impl->contactCorrectionVelocityRatio;
}


void ConstraintForceSolver::enableConstraintForceOutput(bool on)
{
    impl->isConstraintForceOutputMode = on;
}


void ConstraintForceSolver::set2Dmode(bool on)
{
    impl->is2Dmode = on;
}


void ConstraintForceSolver::initialize(void)
{
    impl->initialize();
}


void ConstraintForceSolver::solve()
{
    impl->solve();
}


void ConstraintForceSolver::clearExternalForces()
{
    impl->clearExternalForces();
}


shared_ptr<CollisionLinkPairList> ConstraintForceSolver::getCollisions()
{
    return impl->getCollisions();
}


shared_ptr<CollisionLinkPairList> CFSImpl::getCollisions()
{
    auto collisionPairs = std::make_shared<CollisionLinkPairList>();
    for(size_t i=0; i < constrainedLinkPairs.size(); ++i){
        LinkPair& source = *constrainedLinkPairs[i];
        auto dest = std::make_shared<CollisionLinkPair>();
        int numConstraintsInPair = source.constraintPoints.size();

        for(int j=0; j < numConstraintsInPair; ++j){
            ConstraintPoint& constraint = source.constraintPoints[j];
            dest->collisions.push_back(Collision());
            Collision& col = dest->collisions.back();
            col.point = constraint.point;
            col.normal = constraint.normalTowardInside[1];
            col.depth = constraint.depth;
        }
        for(int j=0; j<2; j++){
            dest->body[j] = source.bodyData[j]->body;
            dest->link[j] = source.link[j];
        }
        collisionPairs->push_back(dest);
    }

    return collisionPairs;
}


#ifdef ENABLE_SIMULATION_PROFILING
double ConstraintForceSolver::getCollisionTime()
{
    return impl->collisionTime;
}
#endif
