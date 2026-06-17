#ifdef __WIN32__
#define NOMINMAX
#endif

#include "DyWorld.h"
#include "DyBody.h"
#include "ForwardDynamicsCBM.h"
#include "ConstraintForceSolver.h"
#include "BodyCollisionDetector.h"
#include "MaterialTable.h"
#include <cnoid/AISTCollisionDetector>
#include <cnoid/IdPair>
#include <cnoid/EigenUtil>
#include <cnoid/CloneMap>
#include <cnoid/TimeMeasure>
#include <cnoid/Format>
#include <algorithm>
#include <random>
#include <unordered_map>
#include <unordered_set>
#include <limits>
#include <fstream>
#include <iomanip>
#include <iostream>

using namespace std;
using namespace cnoid;

namespace {

typedef CollisionDetector::GeometryHandle GeometryHandle;

// settings

static const double VEL_THRESH_OF_DYNAMIC_FRICTION = 1.0e-4;

static const double DEFAULT_MIN_FRICTION_COEFFICIENT = 0.0;
static const double DEFAULT_MAX_FRICTION_COEFFICIENT = 100.0;

static const bool ENABLE_STATIC_FRICTION = true;
static const bool ONLY_STATIC_FRICTION_FORMULATION = (true && ENABLE_STATIC_FRICTION);
static const bool STATIC_FRICTION_BY_TWO_CONSTRAINTS = true;
static const bool IGNORE_CURRENT_VELOCITY_IN_STATIC_FRICTION = false;

static const bool ENABLE_TRUE_FRICTION_CONE =
    (true && ONLY_STATIC_FRICTION_FORMULATION && STATIC_FRICTION_BY_TWO_CONSTRAINTS);

static const bool SKIP_REDUNDANT_ACCEL_CALC = true;
static const bool ASSUME_SYMMETRIC_MATRIX = false;

// Maximum number of PGS (projected Gauss-Seidel) sweeps per step. This is a
// velocity-stepping LCP (the constant vector carries a velocity/dt term), so a
// sweep count that does not fully converge the static LCP is fine: the residual
// velocity reappears in the next step and is corrected there, i.e. convergence
// is spread over the time-step sequence. Combined with warm-starting (see
// USE_PREVIOUS_LCP_SOLUTION) and Gauss-Seidel's intra-sweep propagation, few
// sweeps suffice in practice. Measured on this code base (2026-06): the SR1
// walking sample stays upright even with 1 sweep; the Blocks stacking sample
// (deeper contact chains) needs about 2-3 sweeps to settle without jitter.
// The required count scales with the depth of the contact chain, so 10 is a
// safe margin for unmeasured worst cases (deep stacks, large mass ratios,
// impacts, grasping) while still being far below the old default of 25, which
// was overkill (it sat at the cap on ~88% of the Blocks steps). For comparison,
// Bullet defaults to 10 and ODE's QuickStep to 20. Reduce it for more speed if
// the application's contacts are shallow and slowly varying.
static const int DEFAULT_MAX_NUM_GAUSS_SEIDEL_ITERATION = 10;

static const int DEFAULT_NUM_GAUSS_SEIDEL_INITIAL_ITERATION = 0;

// Convergence threshold on the relative change of the solution between sweeps
// (||dx|| / ||x||). The solver stops early once this is met. Setting it to zero
// (or negative) disables convergence checking and forces a fixed number of
// sweeps (see solveMCPByProjectedGaussSeidel), as in many game-oriented engines
// that run a fixed iteration count for predictable per-step cost. Note this
// criterion is a relative measure scaled by ||x|| (the total contact force), so
// its effective strictness varies with the scene: measured on this code base,
// the SR1 sample converges below 1e-5 (the threshold is loose there) while the
// dense Blocks contacts only reach ~1e-4 within the iteration cap. It is enough
// for the velocity-stepping formulation in typical use, but it can be too loose
// for scenes that need accurate contact forces (deep stacks, large mass ratios,
// grasping with tight friction limits); raise the iteration count or tighten
// this per simulator instance for those.
static const double DEFAULT_GAUSS_SEIDEL_ERROR_CRITERION = 1.0e-3;

static const double THRESH_TO_SWITCH_REL_ERROR = 1.0e-8;
//static const double THRESH_TO_SWITCH_REL_ERROR = numeric_limits<double>::epsilon();

// Warm-starting: reuse the previous step's LCP solution as the initial guess
// for the next step (the solution vector is kept across steps and is only reset
// to zero when the total number of constraint vectors changes; see solve()).
// This is a coarse scheme: it does NOT track the identity of individual contact
// points, it simply reuses the solution per index position and falls back to a
// cold start whenever the constraint count changes. Note that an equal count is
// not a sufficient condition for identity: when the points are reordered, or
// when one body leaves contact while another begins contact and the totals
// happen to match, the per-index reuse feeds an unrelated constraint's force as
// the initial guess (and the solution vector also has a normal-force block
// followed by a friction block, so a shifted boundary can even map a normal
// force onto a friction slot). That is safe because the initial guess only
// affects how fast PGS converges, never the final solution; the worst case is a
// single step that converges a little less well, which the velocity-stepping
// self-correction absorbs in the next step. Tracking contact-point identity
// would close this, but the cold-start rate is already low (see below), so it
// is not worth the cost.
// Measured on this code base (2026-06): the contact-point reduction keeps the
// constraint count stable, so the cold-start rate is very low (~2% on Blocks,
// ~0.5% on SR1), i.e. warm-starting is in effect on almost every step. Turning
// it off did not clearly worsen these two samples, which indicates that the low
// sweep counts here are carried mainly by the velocity-stepping self-correction
// rather than by warm-starting; warm-starting is a near-zero-cost supplement
// that helps most in transients (impacts, fast-changing contacts) not covered
// by those samples. It is kept enabled as a cheap safety margin; do not remove
// it on the grounds that the steady-state samples look fine without it.
static const bool USE_PREVIOUS_LCP_SOLUTION = true;

static const bool ENABLE_CONTACT_DEPTH_CORRECTION = true;

// normal setting
static const double DEFAULT_CONTACT_CORRECTION_DEPTH = 0.00025;
//static const double PENETRATION_A = 500.0;
//static const double PENETRATION_B = 80.0;
static const double DEFAULT_CONTACT_CORRECTION_VELOCITY_RATIO = 5.0;

// Characteristic depth of the depth-correction saturation, expressed as a
// multiple of contactCorrectionDepth (d0). For penetrations whose excess over
// d0 is small compared to this, the correction velocity is linear (slope =
// contactCorrectionVelocityRatio); for larger excess it saturates toward an
// upper bound of contactCorrectionVelocityRatio * (this multiple) * d0. The
// point is that the depth correction exists to settle the penetration to the
// small target depth d0, not to remove a deep penetration in one step: a deep
// interference is itself an anomaly (e.g. a spurious horizontal contact normal
// returned by a mesh collision checker), and trying to correct it abruptly
// injects a large, non-physical disturbance. Saturating the correction keeps
// the disturbance bounded so the simulation stays stable while the user
// investigates the real cause. The function remains monotonically increasing in
// depth (deeper still corrects at least as fast), it only caps the unbounded
// growth.
static const double CONTACT_CORRECTION_SATURATION_DEPTH_RATIO = 4.0;

static const double DEFAULT_CONTACT_CULLING_DISTANCE = 0.005;
static const double DEFAULT_CONTACT_CULLING_DEPTH = 0.05;

// Default maximum number of contact points kept per contact pair (per normal
// cluster). The manifold reduction is enabled by default; set this (or a
// material-specific "max_num_contact_points") to 0 to reproduce the legacy
// behavior of keeping all the contact points filtered only by proximity culling.
static const int DEFAULT_MAX_NUM_CONTACT_POINTS = 4;

// Restitution is applied only when the approaching normal velocity exceeds
// this multiple of the velocity that the persistent (constraint-free) relative
// normal acceleration of the contact point regenerates in a single step, so
// that a contact held together by gravity, actuator forces, etc. never
// triggers a bounce while it is resting. A small absolute threshold is also
// applied to be robust against numerical noise when the persistent
// acceleration is zero (e.g. a side contact with a wall).
static const double IMPACT_VELOCITY_THRESH_RATIO = 2.0;
static const double MIN_IMPACT_VELOCITY_THRESH = 1.0e-4;


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

enum CollisionDetectionModeBit {
    BodyToBodyCollision = 1,
    SelfCollision = 2
};

}

namespace cnoid
{

class ConstraintForceSolver::Impl
{
public:
    DyWorldBase& world;

    vector<unsigned char> bodyIndexToCollisionDetectionModeMap;

    CloneMap cloneMap;
    MaterialTablePtr orgMaterialTable;
    MaterialTablePtr materialTable;

    typedef ConstraintForceSolver::CollisionHandler CollisionHandler;

    struct CollisionHandlerInfo : public Referenced {
        CollisionHandler handler;
        Signal<void()> sigHandlerUnregisterd;
        ~CollisionHandlerInfo(){
            sigHandlerUnregisterd();
        }
    };
    typedef ref_ptr<CollisionHandlerInfo> CollisionHandlerInfoPtr;

    unordered_map<string, CollisionHandlerInfoPtr> collisionHandlerMap;

    struct ConstraintPoint
    {
        int globalIndex;
        Vector3 point;
        Vector3 normalTowardInside[2];
        Vector3 defaultAccel[2];
        Vector3 relVelocityOn0;
        double normalProjectionOfRelVelocityOn0;
        double depth; // position error in the case of a connection point
        double mu;
        int globalFrictionIndex;
        int numFrictionVectors;
        Vector3 frictionVector[4][2];
    };

    class ContactMaterialEx : public ContactMaterial
    {
    public:
        double cullingDistance;
        double cullingDepth;
        int maxNumContactPoints; // 0 means no limit (reduction disabled)
        CollisionHandler collisionHandler;
        Connection collisionHandlerConnection;

        ContactMaterialEx() : maxNumContactPoints(0) { }
        ContactMaterialEx(const ContactMaterial& org) : ContactMaterial(org), maxNumContactPoints(0) { }
        ~ContactMaterialEx(){ collisionHandlerConnection.disconnect(); }
        
        void onCollisionHandlerUnregistered(){
            collisionHandler = CollisionHandler();
            collisionHandlerConnection.disconnect();
        }
    };
    typedef ref_ptr<ContactMaterialEx> ContactMaterialExPtr;

    BodyCollisionDetector bodyCollisionDetector;

    class LinkPair
    {
    public:
        virtual ~LinkPair() { }
        bool isBelongingToSameSubBody;
        DyLink* link[2];
        vector<ConstraintPoint> constraintPoints;
        ContactMaterialExPtr contactMaterial;
        bool isNonContactConstraint;
    };

    unordered_map<IdPair<GeometryHandle>, LinkPair> geometryPairToLinkPairMap;

    double minFrictionCoefficient;
    double maxFrictionCoefficient;
    double defaultContactCullingDistance;
    double defaultContactCullingDepth;
    double defaultCoefficientOfRestitution;
    int defaultMaxNumContactPoints; // 0 means no limit (reduction disabled)

    class ExtraJointLinkPair : public LinkPair
    {
    public:
        // Each constraint pins the relative translation along one axis at one
        // anchor point. The anchor is given as a local point in each link's frame
        // (localPoint[0] in link0, localPoint[1] in link1) and the constraint axis
        // is given in link0's local frame. A ball uses one anchor with three axes;
        // a hinge uses two anchors along the joint axis (the first pinned in all
        // three axes, the second in the two axes orthogonal to the joint axis) so
        // that the axis line is fixed while the rotation about it remains free.
        struct ConstraintInfo
        {
            Vector3 localPoint[2];
            Vector3 axis; // in link0 local frame
        };
        vector<ConstraintInfo> constraintInfos;
    };
    typedef std::shared_ptr<ExtraJointLinkPair> ExtraJointLinkPairPtr;
    vector<ExtraJointLinkPairPtr> extraJointLinkPairs;
    unordered_set<ExtraJoint*> initializedExtraJoints;

    bool is2Dmode;
    DySubBodyPtr subBodyFor2dConstraint;

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

    int numUnconverged;

    typedef Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> MatrixX;
    typedef VectorXd VectorX;

    // The LCP formulation: Mlcp * solution + b   _|_  solution
    //
    // The matrix Mlcp is generally sparse; its element (i, j) can be nonzero
    // only when the link pair owning constraint i and the link pair owning
    // constraint j share a dynamic (non-static) sub-body, because a test
    // force applied at a constraint propagates to the accelerations at
    // another constraint only through a common dynamic sub-body. To exploit
    // this, the matrix is stored in a row-compressed form in which the
    // nonzero column pattern is shared by all the rows of the same link pair.
    // One adjacent link pair that shares a dynamic sub-body with the owner block
    // (the owner itself is included). All the per-adjacent quantities are kept
    // together here so that they are indexed by a single position.
    struct Adjacent
    {
        int pairIndex; // index of the adjacent link pair (the list is sorted by this)

        // position of this adjacent pair's normal / friction column block within
        // the owner block's columns
        int normalOffset;
        int frictionOffset;

        // position of the owner block's normal / friction column block within
        // this adjacent pair's columns
        int reciprocalNormalOffset;
        int reciprocalFrictionOffset;
    };

    struct LinkPairBlock
    {
        int normalTop;    // global index of the first normal constraint
        int numNormals;   // number of normal constraints
        int frictionTop;  // global friction index of the first friction constraint
        int numFrictions; // number of friction constraints

        // the link pairs sharing a dynamic sub-body (sorted by pairIndex, including self)
        std::vector<Adjacent> adjacents;
        int selfPosition; // position of the own pair in adjacents

        // nonzero column indices (ascending) shared by all the rows of this pair
        std::vector<int> columns;
    };
    std::vector<LinkPairBlock> linkPairBlocks; // aligned with constrainedLinkPairs

    // Per matrix row information, indexed by the matrix row
    struct RowInfo
    {
        int linkPairIndex;    // the link pair this row belongs to
        int valueTop;         // top position of this row in matrixValues
        int diagonalPosition; // position of the diagonal element within this row
    };
    std::vector<RowInfo> rowInfos;

    std::vector<double> matrixValues; // nonzero element values stored row by row

    // work buffer reused by buildMatrixStructure to collect adjacent pair indices
    std::vector<int> adjacentPairIndexBuf;

    // work buffers reused by reduceContactConstraintPoints (cleared and refilled
    // each call to avoid per-frame heap allocations)
    std::vector<int> reduceClusterOf;
    std::vector<Vector3> reduceClusterNormals;
    std::vector<ConstraintPoint> reduceResult;
    std::vector<int> reduceMembers;
    std::vector<bool> reduceSelected;
    std::vector<int> reducePicked;

    std::unordered_map<DySubBody*, std::vector<int>> subBodyToLinkPairIndicesMap;

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
    int  numCollisionDetectionThreads;
    double contactCorrectionDepth;
    double contactCorrectionVelocityRatio;

    int numGaussSeidelTotalLoops;
    int numGaussSeidelTotalCalls;
    int numGaussSeidelTotalLoopsMax;

    Impl(DyWorldBase& world);
    ~Impl();
    void clearBodies();
    void initBody(DyBody* body);
    void initSubBody(DySubBody* subBody);
    void initExtraJoint(ExtraJoint* extrajoint);
    void initWorldExtraJoints();
    void init2Dconstraint(DySubBody* subBody);
    void initialize(void);
    void initializeContactMaterials();
    ContactMaterialEx* createContactMaterialFromMaterialPair(int material1, int material2);
    void solve();
    void setConstraintPoints();
    void extractConstraintPoints(const CollisionPair& collisionPair);
    bool addContactConstraintCandidate(LinkPair& linkPair, const Collision& collision);
    void reduceContactConstraintPoints(LinkPair& linkPair, int maxNumPoints);
    void finalizeContactConstraintPoint(LinkPair& linkPair, ConstraintPoint& contact, Vector3 normal);
    void setFrictionVectors(ConstraintPoint& constraintPoint);
    void setExtraJointConstraintPoints(const ExtraJointLinkPairPtr& linkPair);
    void set2dConstraintPoints(const Constrain2dLinkPairPtr& linkPair);
    void putContactPoints();
    void initMatrices();
    void buildMatrixStructure();
    void setAccelCalcSkipInformation();
    void setDefaultAccelerationVector();
    void setAccelerationMatrix();
    void initABMForceElementsWithNoExtForce(DySubBody* subBody);
    void calcABMForceElementsWithTestForce(
        DySubBody* subBody, DyLink* linkToApplyForce, const Vector3& f, const Vector3& tau);
    void calcAccelsABM(DySubBody* subBody, int constraintIndex);
    void calcAccelsMM(DySubBody* bodyData, int constraintIndex);
    void extractRelAccelsOfConstraintPoints(
        LinkPairBlock& block, int localColumnOffset, bool isFrictionColumn, int constraintIndex);
    void extractRelAccelsFromLinkPairCase1(
        LinkPair& linkPair, int columnPosition, int maxConstraintIndexToExtract);
    void extractRelAccelsFromLinkPairCase2(
        LinkPair& linkPair, int iTestForce, int iDefault, int columnPosition, int maxConstraintIndexToExtract);
    void copySymmetricElementsOfAccelerationMatrix();
    void clearSingularPointConstraintsOfClosedLoopConnections();
    void setConstantVectorAndMuBlock();
    void addConstraintForceToLinks();
    void addConstraintForceToLink(LinkPair* linkPair, int ipair);

    double solveGaussSeidelRow(int row, const VectorX& b, const VectorX& x)
    {
        const RowInfo& rowInfo = rowInfos[row];
        const double* values = &matrixValues[rowInfo.valueTop];
        const double diagonal = values[rowInfo.diagonalPosition];
        if(diagonal == numeric_limits<double>::max()){
            return 0.0;
        }
        const LinkPairBlock& block = linkPairBlocks[rowInfo.linkPairIndex];
        const int* columns = block.columns.data();
        const int numColumns = block.columns.size();
        double sum = -diagonal * x(row);
        for(int k=0; k < numColumns; ++k){
            sum += values[k] * x(columns[k]);
        }
        return (-b(row) - sum) / diagonal;
    }

    void solveMCPByProjectedGaussSeidel(const VectorX& b, VectorX& x);
    void solveMCPByProjectedGaussSeidelMainStep(const VectorX& b, VectorX& x);
    void solveMCPByProjectedGaussSeidelInitial(const VectorX& b, VectorX& x, const int numIteration);
    void multiplyMatrixAndVector(const VectorX& x, VectorX& y);
    MatrixX makeDenseMatrix();
    void checkLCPResult(VectorX& b, VectorX& x);
    void checkMCPResult(VectorX& b, VectorX& x);

    ofstream os;

    template<class TMatrix>
    void putMatrix(TMatrix& M, const char *name) {
        if(M.cols() == 1){
            os << "Vector " << name << M << std::endl;
        } else {
            os << "Matrix " << name << ": \n";
            for(int i=0; i < M.rows(); i++){
                for(int j=0; j < M.cols(); j++){
                    os << formatC(" {:.50g} ", M(i, j));
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
};

}


ConstraintForceSolver::Impl::Impl(DyWorldBase& world)
    : world(world),
      randomAngle(0.0, 2.0 * PI)
{
    minFrictionCoefficient = DEFAULT_MIN_FRICTION_COEFFICIENT;
    maxFrictionCoefficient = DEFAULT_MAX_FRICTION_COEFFICIENT;
    defaultContactCullingDistance = DEFAULT_CONTACT_CULLING_DISTANCE;
    defaultContactCullingDepth = DEFAULT_CONTACT_CULLING_DEPTH;
    defaultCoefficientOfRestitution = 0.0;
    defaultMaxNumContactPoints = DEFAULT_MAX_NUM_CONTACT_POINTS;

    maxNumGaussSeidelIteration = DEFAULT_MAX_NUM_GAUSS_SEIDEL_ITERATION;
    numGaussSeidelInitialIteration = DEFAULT_NUM_GAUSS_SEIDEL_INITIAL_ITERATION;
    gaussSeidelErrorCriterion = DEFAULT_GAUSS_SEIDEL_ERROR_CRITERION;
    numCollisionDetectionThreads = 0;
    contactCorrectionDepth = DEFAULT_CONTACT_CORRECTION_DEPTH;
    contactCorrectionVelocityRatio = DEFAULT_CONTACT_CORRECTION_VELOCITY_RATIO;

    bodyIndexToCollisionDetectionModeMap.clear();
    is2Dmode = false;
}


ConstraintForceSolver::Impl::~Impl()
{
    if(CFS_DEBUG){
        os.close();
    }
}


void ConstraintForceSolver::clearBodies()
{
    impl->clearBodies();
}


void ConstraintForceSolver::Impl::clearBodies()
{
    cloneMap.clear();

    bodyCollisionDetector.clearBodies();

    geometryPairToLinkPairMap.clear();
    constrainedLinkPairs.clear();
    extraJointLinkPairs.clear();
    initializedExtraJoints.clear();
    constrain2dLinkPairs.clear();
}    


void ConstraintForceSolver::Impl::initBody(DyBody* body)
{
    for(auto& subBody : body->subBodies()){
        initSubBody(subBody);
    }
    for(int i=0; i < body->numExtraJoints(); ++i){
        initExtraJoint(body->extraJoint(i));
    }
}


void ConstraintForceSolver::Impl::initSubBody(DySubBody* subBody)
{
    subBody->clearExternalForces();
    subBody->hasConstrainedLinks = false;
    subBody->isTestForceBeingApplied = false;
    subBody->hasConstrainedLinks = false;
    
    for(auto& link : subBody->links()){
        link->cfs.dw.setZero();
        link->cfs.dvo.setZero();
        if(link->sensingMode() & Link::LinkContactState){
            subBody->hasContactStateSensingLinks = true;
        }
    }
    if(is2Dmode && !subBody->isStatic()){
        init2Dconstraint(subBody);
    }
}


void ConstraintForceSolver::Impl::init2Dconstraint(DySubBody* subBody)
{
    if(!subBodyFor2dConstraint){
        DyBodyPtr body = new DyBody;
        DyLinkPtr link = new DyLink;
        body->setRootLink(link);
        link->p().setZero();
        link->R().setIdentity();
        // The following code causes a memory leak
        subBodyFor2dConstraint = new DySubBody(link);
        initSubBody(subBodyFor2dConstraint);
    }

    auto rootLink = subBody->rootLink();

    Constrain2dLinkPairPtr linkPair = std::make_shared<Constrain2dLinkPair>();
    linkPair->isBelongingToSameSubBody = false;
    linkPair->isNonContactConstraint = true;
    
    linkPair->constraintPoints.resize(3);
    for(int i=0; i < 3; ++i){
        ConstraintPoint& constraint = linkPair->constraintPoints[i];
        constraint.numFrictionVectors = 0;
        constraint.globalFrictionIndex = numeric_limits<int>::max();
        linkPair->globalYpositions[i] = (rootLink->R() * local2dConstraintPoints[i] + rootLink->p()).y();
    }
        
    linkPair->link[0] = subBodyFor2dConstraint->rootLink();
    linkPair->link[1] = rootLink;
            
    constrain2dLinkPairs.push_back(linkPair);
}


// initialize extra joints for making closed links
void ConstraintForceSolver::Impl::initExtraJoint(ExtraJoint* extraJoint)
{
    if(!extraJoint->link(0) || !extraJoint->link(1)){
        return;
    }
    if(!initializedExtraJoints.insert(extraJoint).second){
        return;
    }

    ExtraJointLinkPairPtr linkPair = std::make_shared<ExtraJointLinkPair>();
    linkPair->isBelongingToSameSubBody = extraJoint->isForLinksOfSameBody();
    linkPair->isNonContactConstraint = true;

    for(int i=0; i < 2; ++i){
        linkPair->link[i] = static_cast<DyLink*>(extraJoint->link(i));
    }

    // The two anchor points (one in each link's local frame) of the joint.
    const Vector3 p0 = extraJoint->point(0);
    const Vector3 p1 = extraJoint->point(1);

    auto& infos = linkPair->constraintInfos;

    const int jointType = extraJoint->type();

    if(jointType == ExtraJoint::Ball){
        // Pin the relative translation at the anchor in all three axes.
        for(int i=0; i < 3; ++i){
            ExtraJointLinkPair::ConstraintInfo info;
            info.localPoint[0] = p0;
            info.localPoint[1] = p1;
            info.axis = Vector3::Unit(i);
            infos.push_back(info);
        }

    } else if(jointType == ExtraJoint::Piston){
        // Pin the two axes orthogonal to the joint axis at the anchor, leaving the
        // axial translation and the rotation about the axis free.
        const Vector3 a = (extraJoint->localRotation(0) * extraJoint->axis()).normalized();
        const Vector3 u = (std::abs(a.dot(Vector3::UnitY())) < 0.9) ? Vector3::UnitY() : Vector3::UnitZ();
        const Vector3 ax1 = a.cross(u).normalized();
        const Vector3 ax2 = a.cross(ax1).normalized();
        for(const auto& axis : {ax1, ax2}){
            ExtraJointLinkPair::ConstraintInfo info;
            info.localPoint[0] = p0;
            info.localPoint[1] = p1;
            info.axis = axis;
            infos.push_back(info);
        }

    } else if(jointType == ExtraJoint::Hinge){
        // Fix the joint axis line and leave only the rotation about it. The first
        // anchor is pinned in all three axes; a second anchor offset along the
        // joint axis is pinned in the two axes orthogonal to the joint axis. This
        // uses five constraints (3 + 2), which is the minimum for a hinge.
        const Vector3 a0 = (extraJoint->localRotation(0) * extraJoint->axis()).normalized();
        const Vector3 a1 = (extraJoint->localRotation(1) * extraJoint->axis()).normalized();
        const Vector3 u = (std::abs(a0.dot(Vector3::UnitY())) < 0.9) ? Vector3::UnitY() : Vector3::UnitZ();
        const Vector3 ax1 = a0.cross(u).normalized();
        const Vector3 ax2 = a0.cross(ax1).normalized();
        const double span = 0.05; // offset of the second anchor along the axis

        // First anchor: pin all three axes.
        for(int i=0; i < 3; ++i){
            ExtraJointLinkPair::ConstraintInfo info;
            info.localPoint[0] = p0;
            info.localPoint[1] = p1;
            info.axis = Vector3::Unit(i);
            infos.push_back(info);
        }
        // Second anchor along the axis: pin the two orthogonal axes only. The
        // anchor is offset consistently in both link frames so that the axis
        // direction is what is being constrained.
        for(const auto& axis : {ax1, ax2}){
            ExtraJointLinkPair::ConstraintInfo info;
            info.localPoint[0] = p0 + span * a0;
            info.localPoint[1] = p1 + span * a1;
            info.axis = axis;
            infos.push_back(info);
        }

    } else {
        // Unsupported extra joint type; add no constraint.
        return;
    }

    linkPair->constraintPoints.resize(infos.size());
    for(auto& constraint : linkPair->constraintPoints){
        constraint.numFrictionVectors = 0;
        constraint.globalFrictionIndex = numeric_limits<int>::max();
    }

    extraJointLinkPairs.push_back(linkPair);
}


void ConstraintForceSolver::Impl::initWorldExtraJoints()
{
    for(auto& orgJoint : world.extraJoints()){
        ExtraJointPtr joint = orgJoint->clone();
        for(int i=0; i < 2; ++i){
            if(auto body = world.body(orgJoint->bodyName(i))){
                if(auto link = body->link(orgJoint->linkName(i))){
                    joint->setLink(i, link);
                }
            }
        }
        initExtraJoint(joint);
    }
}


void ConstraintForceSolver::Impl::initialize(void)
{
    if(CFS_DEBUG || CFS_MCP_DEBUG){
        static int ntest = 0;
        os.close();
        os.open((string("cfs-log-") + std::to_string(ntest++) + ".log").c_str());
        //os << setprecision(50);
    }

    clearBodies();

    if(CFS_MCP_DEBUG){
        numGaussSeidelTotalCalls = 0;
        numGaussSeidelTotalLoops = 0;
        numGaussSeidelTotalLoopsMax = 0;
    }

    if(!bodyCollisionDetector.collisionDetector()){
        bodyCollisionDetector.setCollisionDetector(new AISTCollisionDetector);
    }

    initializeContactMaterials();

    int numBodies = world.numBodies();
    for(int bodyIndex = 0; bodyIndex < numBodies; ++bodyIndex){
        auto body = world.body(bodyIndex);
        initBody(body);

        auto collisionDetectionMode =  bodyIndexToCollisionDetectionModeMap[bodyIndex];
        if(collisionDetectionMode & BodyToBodyCollision){
            bodyCollisionDetector.addBody(body, collisionDetectionMode & SelfCollision);
        }
    }

    initWorldExtraJoints();

    // Enable parallel collision detection when requested. This must be set
    // before makeReady(), which is where the detector fixes its thread count.
    if(numCollisionDetectionThreads > 0){
        if(auto aist = dynamic_cast<AISTCollisionDetector*>(bodyCollisionDetector.collisionDetector())){
            aist->setNumThreads(numCollisionDetectionThreads);
        }
    }

    bodyCollisionDetector.makeReady();

    prevGlobalNumConstraintVectors = 0;
    prevGlobalNumFrictionVectors = 0;
    numUnconverged = 0;

    if(ENABLE_RANDOM_STATIC_FRICTION_BASE){
        randomEngine.seed();
    }
}


void ConstraintForceSolver::Impl::initializeContactMaterials()
{
    if(!orgMaterialTable){
        materialTable = new MaterialTable;
    } else {
        string collisionHandlerName;
        materialTable =
            orgMaterialTable->clone(
                cloneMap,
                [&](const ContactMaterial* org){
                    auto cm = new ContactMaterialEx(*org);
                    cm->cullingDistance = cm->info("cullingDistance", defaultContactCullingDistance);
                    cm->cullingDepth = cm->info("cullingDepth", defaultContactCullingDepth);
                    cm->maxNumContactPoints = cm->info("max_num_contact_points", defaultMaxNumContactPoints);

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

                    cm->setStaticFriction(
                        std::min(std::max(cm->staticFriction(), minFrictionCoefficient), maxFrictionCoefficient));
                    cm->setDynamicFriction(
                        std::min(std::max(cm->dynamicFriction(), minFrictionCoefficient), maxFrictionCoefficient));

                    return cm;
                });
    }
}


ConstraintForceSolver::Impl::ContactMaterialEx*
ConstraintForceSolver::Impl::createContactMaterialFromMaterialPair(int material1, int material2)
{
    Material* m1 = materialTable->material(material1);
    Material* m2 = materialTable->material(material2);

    auto cm = new ContactMaterialEx;
    double friction = sqrt(m1->roughness() * m2->roughness());
    friction = std::min(std::max(friction, minFrictionCoefficient), maxFrictionCoefficient);
    cm->setFriction(friction);
    cm->setRestitution(sqrt((1.0 - m1->viscosity()) * (1.0 - m2->viscosity())));
    cm->cullingDistance = defaultContactCullingDistance;
    cm->cullingDepth = defaultContactCullingDepth;
    cm->maxNumContactPoints = defaultMaxNumContactPoints;
    materialTable->setContactMaterial(material1, material2, cm);

    return cm;
}


void ConstraintForceSolver::Impl::solve()
{
    if(CFS_DEBUG){
        os << "Time: " << world.currentTime() << std::endl;
    }

    for(auto& subBody : world.subBodies()){
        subBody->hasConstrainedLinks = false;
        if(subBody->hasContactStateSensingLinks){
            for(auto& link : subBody->links()){
                link->contactPoints().clear();
            }
        }
    }

    bodyCollisionDetector.updatePositions();

    globalNumConstraintVectors = 0;
    globalNumFrictionVectors = 0;

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

        buildMatrixStructure();

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
            MatrixX M = makeDenseMatrix();
            debugPutMatrix(M, "Mlcp");
            debugPutVector(b.head(globalNumConstraintVectors), "b1");
            debugPutVector(b.segment(globalNumConstraintVectors, globalNumFrictionVectors), "b2");
        }

        if(!USE_PREVIOUS_LCP_SOLUTION || constraintsSizeChanged){
            solution.setZero();
        }
        solveMCPByProjectedGaussSeidel(b, solution);

        if(CFS_DEBUG){
            os << "LCP converged" << std::endl;
        }
        if(CFS_DEBUG_LCPCHECK){
            // checkLCPResult(b, solution);
            checkMCPResult(b, solution);
        }

        addConstraintForceToLinks();
    }

    prevGlobalNumConstraintVectors = globalNumConstraintVectors;
    prevGlobalNumFrictionVectors = globalNumFrictionVectors;
}


void ConstraintForceSolver::Impl::setConstraintPoints()
{
    bodyCollisionDetector.detectCollisions(
        [&](const CollisionPair& collisionPair){
            extractConstraintPoints(collisionPair);
            return false; // Continue checking all collisions
        });

    globalNumContactNormalVectors = globalNumConstraintVectors;

    for(size_t i=0; i < extraJointLinkPairs.size(); ++i){
        setExtraJointConstraintPoints(extraJointLinkPairs[i]);
    }

    for(size_t i=0; i < constrain2dLinkPairs.size(); ++i){
        set2dConstraintPoints(constrain2dLinkPairs[i]);
    }
}


void ConstraintForceSolver::Impl::extractConstraintPoints(const CollisionPair& collisionPair)
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
            auto link = static_cast<DyLink*>(collisionPair.object(i));
            linkPair.link[i] = link;
            material[i] = link->materialId();
        }
        auto subBody0 = linkPair.link[0]->subBody();
        auto subBody1 = linkPair.link[1]->subBody();
        
        linkPair.isBelongingToSameSubBody = (subBody0 == subBody1);
        linkPair.isNonContactConstraint = false;

        linkPair.contactMaterial =
            static_cast<ContactMaterialEx*>(materialTable->contactMaterial(material[0], material[1]));
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
    
    pLinkPair->link[0]->subBody()->hasConstrainedLinks = true;
    pLinkPair->link[1]->subBody()->hasConstrainedLinks = true;

    // First collect the contact points as candidates (depth/proximity culling
    // only), then optionally reduce them to a manifold of at most
    // maxNumContactPoints, and finally assign the global indices and the
    // friction vectors to the surviving points. The candidate collection must
    // precede the index assignment so that the global indices of one link pair
    // stay contiguous, which the sparse matrix structure relies on.
    auto& candidates = pLinkPair->constraintPoints;
    for(auto& collision : collisions){
        addContactConstraintCandidate(*pLinkPair, collision);
    }

    const int maxNumPoints = pLinkPair->contactMaterial->maxNumContactPoints;
    if(maxNumPoints > 0 && static_cast<int>(candidates.size()) > maxNumPoints){
        reduceContactConstraintPoints(*pLinkPair, maxNumPoints);
    }

    for(auto& contact : candidates){
        finalizeContactConstraintPoint(*pLinkPair, contact, contact.normalTowardInside[1]);
    }

    if(!candidates.empty()){
        constrainedLinkPairs.push_back(pLinkPair);
    }
}


/**
   Add a collision as a contact constraint candidate. Only the geometric
   quantities (point, depth, normal) are set here; the global indices and the
   friction vectors are assigned later in finalizeContactConstraintPoint, after
   the optional manifold reduction. The candidate normal is temporarily kept in
   normalTowardInside[1].
   @return true if the point is actually added as a candidate
*/
bool ConstraintForceSolver::Impl::addContactConstraintCandidate(LinkPair& linkPair, const Collision& collision)
{
    // skip the contact which has too much depth
    if(collision.depth > linkPair.contactMaterial->cullingDepth){
        return false;
    }

    auto& constraintPoints = linkPair.constraintPoints;

    // When the manifold reduction is enabled (maxNumContactPoints > 0), the
    // reduction's farthest-point selection already avoids picking near-duplicate
    // points, so the proximity culling here is skipped (it would otherwise discard
    // candidates by arrival order, possibly dropping a better extreme point before
    // the reduction can choose it). When the reduction is disabled, the proximity
    // culling provides the legacy behavior of eliminating dense duplicate points.
    if(linkPair.contactMaterial->maxNumContactPoints == 0){
        for(auto& prev : constraintPoints){
            if((prev.point - collision.point).norm() < linkPair.contactMaterial->cullingDistance){
                return false;
            }
        }
    }

    constraintPoints.push_back(ConstraintPoint());
    ConstraintPoint& contact = constraintPoints.back();
    contact.point = collision.point;
    contact.depth = collision.depth;
    contact.normalTowardInside[1] = collision.normal;

    return true;
}


/**
   Reduce the contact constraint candidates of a link pair to a manifold of at
   most maxNumPoints points per normal cluster, keeping the points that span the
   largest area (the convex-hull extremes of the contact patch) so that the
   support polygon is preserved. Candidates that share a similar normal form one
   cluster; the deepest point of each cluster is always kept.
*/
void ConstraintForceSolver::Impl::reduceContactConstraintPoints(LinkPair& linkPair, int maxNumPoints)
{
    auto& candidates = linkPair.constraintPoints;
    const int numCandidates = candidates.size();

    // Cluster the candidates by normal direction (cos of 15 deg ~= 0.966).
    // Each cluster's representative normal is fixed to the normal of the first
    // point that created it; subsequent points are compared against that fixed
    // representative.
    //
    // Note: this makes the clustering slightly order-dependent. If the normals
    // within one real contact face are spread close to the threshold (e.g. on a
    // curved surface or a coarse mesh whose per-point normals scatter by ~10
    // deg), the order in which points arrive can change where a cluster is split
    // (chained clustering). Updating the representative to the running mean of
    // the cluster's normals (re-normalized) would reduce this order dependence
    // at negligible cost. It is intentionally NOT done here because: (1) for a
    // flat face the per-point normals agree to within a few degrees, so the mean
    // and the fixed representative give the same clusters and the averaging
    // would have no effect; (2) the mean still drifts as the cluster grows, so
    // it only mitigates rather than removes the order dependence. Adopt the mean
    // only if contact chattering or frame-to-frame instability of the reduced
    // manifold is actually observed.
    static const double normalClusterCosThresh = 0.966;
    auto& clusterOf = reduceClusterOf;
    auto& clusterNormals = reduceClusterNormals;
    clusterOf.assign(numCandidates, -1);
    clusterNormals.clear();
    for(int i=0; i < numCandidates; ++i){
        const Vector3& ni = candidates[i].normalTowardInside[1];
        int found = -1;
        for(size_t c=0; c < clusterNormals.size(); ++c){
            if(ni.dot(clusterNormals[c]) >= normalClusterCosThresh){
                found = c;
                break;
            }
        }
        if(found < 0){
            found = clusterNormals.size();
            clusterNormals.push_back(ni);
        }
        clusterOf[i] = found;
    }

    auto& reduced = reduceResult;
    reduced.clear();
    reduced.reserve(maxNumPoints * clusterNormals.size());

    auto& members = reduceMembers;
    auto& selected = reduceSelected;
    auto& picked = reducePicked;
    for(size_t c=0; c < clusterNormals.size(); ++c){
        members.clear();
        for(int i=0; i < numCandidates; ++i){
            if(clusterOf[i] == static_cast<int>(c)){
                members.push_back(i);
            }
        }
        const int numMembers = members.size();
        if(numMembers <= maxNumPoints){
            for(int idx : members){
                reduced.push_back(candidates[idx]);
            }
            continue;
        }

        // Greedy selection: deepest point first, then iteratively the point
        // farthest from the already selected set (maximizing the spanned area).
        selected.assign(numMembers, false);
        picked.clear();

        int deepest = 0;
        for(int m=1; m < numMembers; ++m){
            if(candidates[members[m]].depth > candidates[members[deepest]].depth){
                deepest = m;
            }
        }
        selected[deepest] = true;
        picked.push_back(deepest);

        while(static_cast<int>(picked.size()) < maxNumPoints){
            int best = -1;
            double bestDist = -1.0;
            for(int m=0; m < numMembers; ++m){
                if(selected[m]){
                    continue;
                }
                // distance from member m to the nearest already-picked point
                double nearest = std::numeric_limits<double>::max();
                for(int p : picked){
                    double d = (candidates[members[m]].point - candidates[members[p]].point).squaredNorm();
                    if(d < nearest){
                        nearest = d;
                    }
                }
                if(nearest > bestDist){
                    bestDist = nearest;
                    best = m;
                }
            }
            if(best < 0){
                break;
            }
            selected[best] = true;
            picked.push_back(best);
        }

        for(int m : picked){
            reduced.push_back(candidates[members[m]]);
        }
    }

    candidates.swap(reduced);
}


/**
   Finalize a surviving contact constraint point: assign the global indices, set
   the opposite normal, compute the relative velocity, and build the friction
   vectors. The candidate normal is passed in (it was kept in
   normalTowardInside[1] during the candidate stage).
*/
void ConstraintForceSolver::Impl::finalizeContactConstraintPoint
(LinkPair& linkPair, ConstraintPoint& contact, Vector3 normal)
{
    contact.normalTowardInside[1] = normal;
    contact.normalTowardInside[0] = -normal;
    contact.globalIndex = globalNumConstraintVectors++;

    // check velocities
    Vector3 v[2];
    for(int k=0; k < 2; ++k){
        DyLink* link = linkPair.link[k];
        if(link->isRoot() && link->isFixedJoint()){
            v[k].setZero();
        } else {
            v[k] = link->vo() + link->w().cross(contact.point);

            if(link->jointType() == Link::PseudoContinuousTrackJoint){
                // tentative
                // invalid depths should be fixed
                if (contact.depth > contactCorrectionDepth * 2.0){
                    contact.depth = contactCorrectionDepth * 2.0;
                }
                const Vector3 axis = link->R() * link->a();
                Vector3 direction;
                if(k==0){
                    direction = axis.cross(normal);
                } else {
                    direction = normal.cross(axis);
                }
                direction.normalize();
                v[k] += link->dq_target() * direction;
            }
        }
    }
    contact.relVelocityOn0 = v[1] - v[0];

    contact.normalProjectionOfRelVelocityOn0 = contact.normalTowardInside[1].dot(contact.relVelocityOn0);

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
}


void ConstraintForceSolver::Impl::setFrictionVectors(ConstraintPoint& contact)
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


void ConstraintForceSolver::Impl::setExtraJointConstraintPoints(const ExtraJointLinkPairPtr& linkPair)
{
    auto& constraintPoints = linkPair->constraintPoints;
    auto& constraintInfos = linkPair->constraintInfos;

    DyLink* link0 = linkPair->link[0];
    DyLink* link1 = linkPair->link[1];

    int n = constraintPoints.size();
    for(int i=0; i < n; ++i){
        ConstraintPoint& constraint = constraintPoints[i];
        const auto& info = constraintInfos[i];

        // World position of the anchor as seen from each link.
        Vector3 point[2];
        point[0].noalias() = link0->p() + link0->R() * info.localPoint[0];
        point[1].noalias() = link1->p() + link1->R() * info.localPoint[1];
        Vector3 midPoint = (point[0] + point[1]) / 2.0;
        Vector3 error = midPoint - point[0];

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

        const Vector3 axis = link0->R() * info.axis;
        constraint.point = midPoint;
        constraint.normalTowardInside[0] =  axis;
        constraint.normalTowardInside[1] = -axis;
        constraint.depth = axis.dot(error);
        constraint.globalIndex = globalNumConstraintVectors++;
        constraint.normalProjectionOfRelVelocityOn0 = constraint.normalTowardInside[1].dot(relVelocityOn0);
    }
    linkPair->link[0]->subBody()->hasConstrainedLinks = true;
    linkPair->link[1]->subBody()->hasConstrainedLinks = true;

    constrainedLinkPairs.push_back(linkPair.get());
}


void ConstraintForceSolver::Impl::set2dConstraintPoints(const Constrain2dLinkPairPtr& linkPair)
{
    static const Vector3 yAxis(0.0, 1.0, 0.0);
    
    auto& constraintPoints = linkPair->constraintPoints;
    
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
    linkPair->link[0]->subBody()->hasConstrainedLinks = true;
    linkPair->link[1]->subBody()->hasConstrainedLinks = true;

    constrainedLinkPairs.push_back(linkPair.get());
}



void ConstraintForceSolver::Impl::putContactPoints()
{
    os << "Contact Points\n";
    for(size_t i=0; i < constrainedLinkPairs.size(); ++i){
        LinkPair* linkPair = constrainedLinkPairs[i];
        ExtraJointLinkPair* ejLinkPair = dynamic_cast<ExtraJointLinkPair*>(linkPair);

        if(!ejLinkPair){

            os << " " << linkPair->link[0]->name() << " of " << linkPair->link[0]->body()->modelName();
            os << "<-->";
            os << " " << linkPair->link[1]->name() << " of " << linkPair->link[1]->body()->modelName();
            os << "\n";
            os << " culling thresh: " << linkPair->contactMaterial->cullingDistance << "\n";

            auto& constraintPoints = linkPair->constraintPoints;
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


void ConstraintForceSolver::Impl::initMatrices()
{
    const int n = globalNumConstraintVectors;
    const int m = globalNumFrictionVectors;

    const int dimLCP = n + m;

    b.resize(dimLCP);
    solution.resize(dimLCP);

    frictionIndexToContactIndex.resize(m);
    contactIndexToMu.resize(globalNumContactNormalVectors);
    mcpHi.resize(globalNumContactNormalVectors);

    an0.resize(n);
    at0.resize(m);
}


void ConstraintForceSolver::Impl::buildMatrixStructure()
{
    const int n = globalNumConstraintVectors;
    const int numLinkPairs = constrainedLinkPairs.size();

    if(static_cast<int>(linkPairBlocks.size()) < numLinkPairs){
        linkPairBlocks.resize(numLinkPairs);
    }

    // Set the constraint index blocks of each link pair and collect the link
    // pairs that involve each dynamic sub-body
    subBodyToLinkPairIndicesMap.clear();
    for(int i=0; i < numLinkPairs; ++i){
        LinkPair& linkPair = *constrainedLinkPairs[i];
        LinkPairBlock& block = linkPairBlocks[i];
        auto& points = linkPair.constraintPoints;
        block.normalTop = points.front().globalIndex;
        block.numNormals = points.size();
        block.frictionTop = 0;
        block.numFrictions = 0;
        for(auto& point : points){
            if(point.numFrictionVectors > 0){
                if(block.numFrictions == 0){
                    block.frictionTop = point.globalFrictionIndex;
                }
                block.numFrictions += point.numFrictionVectors;
            }
        }
        DySubBody* subBody0 = linkPair.link[0]->subBody();
        DySubBody* subBody1 = linkPair.link[1]->subBody();
        if(!subBody0->isStatic()){
            subBodyToLinkPairIndicesMap[subBody0].push_back(i);
        }
        if(subBody1 != subBody0 && !subBody1->isStatic()){
            subBodyToLinkPairIndicesMap[subBody1].push_back(i);
        }
    }

    // Determine the adjacent link pairs and the column pattern of each link pair
    for(int i=0; i < numLinkPairs; ++i){
        LinkPair& linkPair = *constrainedLinkPairs[i];
        LinkPairBlock& block = linkPairBlocks[i];

        // Collect the indices of the adjacent pairs (sorted, unique) in a work
        // buffer first, so that the sort/unique operate on plain indices.
        auto& indices = adjacentPairIndexBuf;
        indices.clear();
        indices.push_back(i); // keep the diagonal block even for a pair of static sub-bodies
        for(int k=0; k < 2; ++k){
            DySubBody* subBody = linkPair.link[k]->subBody();
            if(k == 1 && subBody == linkPair.link[0]->subBody()){
                break;
            }
            if(!subBody->isStatic()){
                auto& pairs = subBodyToLinkPairIndicesMap[subBody];
                indices.insert(indices.end(), pairs.begin(), pairs.end());
            }
        }
        std::sort(indices.begin(), indices.end());
        indices.erase(std::unique(indices.begin(), indices.end()), indices.end());

        const int numAdjacents = indices.size();
        auto& adjacents = block.adjacents;
        adjacents.resize(numAdjacents);

        // Build the column pattern: first all the adjacent pairs' normal columns,
        // then all their friction columns, recording each block's offset.
        auto& columns = block.columns;
        columns.clear();
        for(int a=0; a < numAdjacents; ++a){
            Adjacent& adj = adjacents[a];
            adj.pairIndex = indices[a];
            if(indices[a] == i){
                block.selfPosition = a;
            }
            LinkPairBlock& adjacent = linkPairBlocks[indices[a]];
            adj.normalOffset = columns.size();
            for(int j=0; j < adjacent.numNormals; ++j){
                columns.push_back(adjacent.normalTop + j);
            }
        }
        for(int a=0; a < numAdjacents; ++a){
            Adjacent& adj = adjacents[a];
            LinkPairBlock& adjacent = linkPairBlocks[adj.pairIndex];
            adj.frictionOffset = columns.size();
            for(int j=0; j < adjacent.numFrictions; ++j){
                columns.push_back(n + adjacent.frictionTop + j);
            }
        }
    }

    // Resolve the position of each pair's column blocks in the adjacent pairs' patterns
    for(int i=0; i < numLinkPairs; ++i){
        LinkPairBlock& block = linkPairBlocks[i];
        for(auto& adj : block.adjacents){
            LinkPairBlock& adjacent = linkPairBlocks[adj.pairIndex];
            // find the owner pair i within the adjacent pair's adjacents (sorted by pairIndex)
            int lo = 0, hi = adjacent.adjacents.size();
            while(lo < hi){
                int mid = (lo + hi) / 2;
                if(adjacent.adjacents[mid].pairIndex < i){
                    lo = mid + 1;
                } else {
                    hi = mid;
                }
            }
            adj.reciprocalNormalOffset = adjacent.adjacents[lo].normalOffset;
            adj.reciprocalFrictionOffset = adjacent.adjacents[lo].frictionOffset;
        }
    }

    // Set the row information and allocate the value array
    const int dimLCP = n + globalNumFrictionVectors;
    rowInfos.resize(dimLCP);
    for(int i=0; i < numLinkPairs; ++i){
        LinkPairBlock& block = linkPairBlocks[i];
        for(int j=0; j < block.numNormals; ++j){
            rowInfos[block.normalTop + j].linkPairIndex = i;
        }
        for(int j=0; j < block.numFrictions; ++j){
            rowInfos[n + block.frictionTop + j].linkPairIndex = i;
        }
    }
    int valueTop = 0;
    for(int row = 0; row < dimLCP; ++row){
        RowInfo& rowInfo = rowInfos[row];
        LinkPairBlock& block = linkPairBlocks[rowInfo.linkPairIndex];
        const Adjacent& self = block.adjacents[block.selfPosition];
        rowInfo.valueTop = valueTop;
        valueTop += block.columns.size();
        if(row < n){
            rowInfo.diagonalPosition = self.normalOffset + (row - block.normalTop);
        } else {
            rowInfo.diagonalPosition = self.frictionOffset + (row - n - block.frictionTop);
        }
    }
    matrixValues.assign(valueTop, 0.0);
}


void ConstraintForceSolver::Impl::setAccelCalcSkipInformation()
{
    // clear skip check numbers
    for(auto& subBody : world.subBodies()){
        if(subBody->hasConstrainedLinks){
            for(auto& link : subBody->links()){
                link->cfs.numberToCheckAccelCalcSkip = numeric_limits<int>::max();
            }
        }
    }

    // add the number of contact points to skip check numbers of the links from a contact target to the root
    int numLinkPairs = constrainedLinkPairs.size();
    for(int i=0; i < numLinkPairs; ++i){
        LinkPair* linkPair = constrainedLinkPairs[i];
        int constraintIndex = linkPair->constraintPoints.front().globalIndex;
        for(int j=0; j < 2; ++j){
            auto link = linkPair->link[j];
            while(link){
                if(link->cfs.numberToCheckAccelCalcSkip < constraintIndex){
                    break;
                }
                link->cfs.numberToCheckAccelCalcSkip = constraintIndex;
                if(link->isSubBodyRoot()){
                    break;
                }
                link = link->parent();
            }
        }
    }
}


void ConstraintForceSolver::Impl::setDefaultAccelerationVector()
{
    // calculate accelerations with no constraint force
    for(auto& subBody : world.subBodies()){
        if(subBody->hasConstrainedLinks && !subBody->isStatic()){
            if(auto cbm = subBody->forwardDynamicsCBM()){
                cbm->sumExternalForces();
                cbm->solveUnknownAccels();
                calcAccelsMM(subBody, numeric_limits<int>::max());
            } else {
                initABMForceElementsWithNoExtForce(subBody);
                calcAccelsABM(subBody, numeric_limits<int>::max());
            }
        }
    }

    // extract accelerations
    for(size_t i=0; i < constrainedLinkPairs.size(); ++i){

        LinkPair& linkPair = *constrainedLinkPairs[i];
        auto& constraintPoints = linkPair.constraintPoints;

        for(size_t j=0; j < constraintPoints.size(); ++j){
            ConstraintPoint& constraint = constraintPoints[j];

            for(int k=0; k < 2; ++k){
                DyLink* link = linkPair.link[k];
                if(link->subBody()->isStatic()){
                    constraint.defaultAccel[k].setZero();
                } else {
                    constraint.defaultAccel[k] =
                        link->cfs.dvo - constraint.point.cross(link->cfs.dw) +
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


void ConstraintForceSolver::Impl::setAccelerationMatrix()
{
    for(size_t i=0; i < constrainedLinkPairs.size(); ++i){

        LinkPair& linkPair = *constrainedLinkPairs[i];
        LinkPairBlock& block = linkPairBlocks[i];
        int numConstraintsInPair = linkPair.constraintPoints.size();

        for(int j=0; j < numConstraintsInPair; ++j){

            ConstraintPoint& constraint = linkPair.constraintPoints[j];
            int constraintIndex = constraint.globalIndex;

            // apply test normal force
            for(int k=0; k < 2; ++k){
                auto link = linkPair.link[k];
                auto subBody = link->subBody();
                if(!subBody->isStatic()){

                    subBody->isTestForceBeingApplied = true;
                    const Vector3& f = constraint.normalTowardInside[k];

                    if(auto cbm = subBody->forwardDynamicsCBM()){
                        //! \todo This code does not work correctly when the links are in the same body. Fix it.
                        Vector3 arm = constraint.point - subBody->rootLink()->p();
                        Vector3 tau = arm.cross(f);
                        Vector3 tauext = constraint.point.cross(f);
                        if(cbm->solveUnknownAccels(link, f, tauext, f, tau)){
                            calcAccelsMM(subBody, constraintIndex);
                        }
                    } else {
                        Vector3 tau = constraint.point.cross(f);
                        calcABMForceElementsWithTestForce(subBody, link, f, tau);
                        if(!linkPair.isBelongingToSameSubBody || (k > 0)){
                            calcAccelsABM(subBody, constraintIndex);
                        }
                    }
                }
            }
            extractRelAccelsOfConstraintPoints(
                block, constraintIndex - block.normalTop, false, constraintIndex);

            // apply test friction force
            for(int l=0; l < constraint.numFrictionVectors; ++l){
                for(int k=0; k < 2; ++k){
                    auto link = linkPair.link[k];
                    auto subBody = link->subBody();
                    if(!subBody->isStatic()){
                        const Vector3& f = constraint.frictionVector[l][k];

                        if(auto cbm = subBody->forwardDynamicsCBM()){
                            //! \todo This code does not work correctly when the links are in the same body. Fix it.
                            Vector3 arm = constraint.point - subBody->rootLink()->p();
                            Vector3 tau = arm.cross(f);
                            Vector3 tauext = constraint.point.cross(f);
                            if(cbm->solveUnknownAccels(link, f, tauext, f, tau)){
                                calcAccelsMM(subBody, constraintIndex);
                            }
                        } else {
                            Vector3 tau = constraint.point.cross(f);
                            calcABMForceElementsWithTestForce(subBody, link, f, tau);
                            if(!linkPair.isBelongingToSameSubBody || (k > 0)){
                                calcAccelsABM(subBody, constraintIndex);
                            }
                        }
                    }
                }
                extractRelAccelsOfConstraintPoints(
                    block, constraint.globalFrictionIndex + l - block.frictionTop, true, constraintIndex);
            }

            linkPair.link[0]->subBody()->isTestForceBeingApplied = false;
            linkPair.link[1]->subBody()->isTestForceBeingApplied = false;
        }
    }

    if(ASSUME_SYMMETRIC_MATRIX){
        copySymmetricElementsOfAccelerationMatrix();
    }
}


void ConstraintForceSolver::Impl::initABMForceElementsWithNoExtForce(DySubBody* subBody)
{
    subBody->dpf.setZero();
    subBody->dptau.setZero();

    // Cache the inverse of the free root link's 6x6 articulated inertia. The
    // matrix is constant throughout the matrix assembly of this step, so the
    // per-test-force solve in calcAccelsABM becomes a matrix-vector product
    // instead of refactorizing the matrix every time. The matrix is symmetric
    // positive definite (an inertia matrix), so LDLT is used; for a well
    // conditioned 6x6 inertia the explicit inverse agrees with a direct solve
    // to machine precision.
    auto rootLink = subBody->rootLink();
    if(rootLink->isFreeJoint()){
        Eigen::Matrix<double, 6, 6> M;
        M << rootLink->Ivv(), rootLink->Iwv().transpose(),
             rootLink->Iwv(), rootLink->Iww();
        subBody->rootInvInertia = M.ldlt().solve(Eigen::Matrix<double, 6, 6>::Identity());
    }

    const int n = subBody->numLinks();
    for(int i = n-1; i >= 0; --i){
        auto link = subBody->link(i);

        /*
          data.pf0   = link->pf;
          data.ptau0 = link->ptau;
        */
        link->cfs.pf0   = link->pf() - link->f_ext();
        link->cfs.ptau0 = link->ptau() - link->tau_ext();

        for(auto child = link->child(); child; child = child->sibling()){
            if(child->isFreeJoint()){
                continue;
            }
            link->cfs.pf0   += child->cfs.pf0;
            link->cfs.ptau0 += child->cfs.ptau0;

            if(!child->isFixedJoint()){
                double uu_dd = child->cfs.uu0 / child->dd();
                link->cfs.pf0   += uu_dd * child->hhv();
                link->cfs.ptau0 += uu_dd * child->hhw();
            }
        }

        if(i > 0){
            if(!link->isFixedJoint()){
                double u = std::clamp(link->u(), link->u_lower(), link->u_upper());
                link->cfs.uu0  = link->uu() + u - (link->sv().dot(link->cfs.pf0) + link->sw().dot(link->cfs.ptau0));
                link->cfs.uu = link->cfs.uu0;
            }
        }
    }
}


void ConstraintForceSolver::Impl::calcABMForceElementsWithTestForce
(DySubBody* subBody, DyLink* linkToApplyForce, const Vector3& f, const Vector3& tau)
{
    Vector3 dpf   = -f;
    Vector3 dptau = -tau;

    DyLink* link = linkToApplyForce;
    while(!link->isSubBodyRoot()){
        if(!link->isFixedJoint()){
            double duu = -(link->sv().dot(dpf) + link->sw().dot(dptau));
            link->cfs.uu += duu;
            double duudd = duu / link->dd();
            dpf   += duudd * link->hhv();
            dptau += duudd * link->hhw();
        }
        link = link->parent();
    }

    subBody->dpf   += dpf;
    subBody->dptau += dptau;
}


void ConstraintForceSolver::Impl::calcAccelsABM(DySubBody* subBody, int constraintIndex)
{
    auto rootLink = subBody->rootLink();

    if(!rootLink->isFreeJoint()){
        rootLink->cfs.dw .setZero();
        rootLink->cfs.dvo.setZero();
    } else {
        // Solve M * a = -f using the inverse cached in
        // initABMForceElementsWithNoExtForce (M is constant during the assembly).
        Eigen::Matrix<double, 6, 1> f;
        f << (rootLink->cfs.pf0   + subBody->dpf),
             (rootLink->cfs.ptau0 + subBody->dptau);

        Eigen::Matrix<double, 6, 1> a(subBody->rootInvInertia * (-f));

        rootLink->cfs.dvo = a.head<3>();
        rootLink->cfs.dw  = a.tail<3>();
    }

    // reset
    subBody->dpf  .setZero();
    subBody->dptau.setZero();

    int skipCheckNumber = ASSUME_SYMMETRIC_MATRIX ? constraintIndex : (numeric_limits<int>::max() - 1);
    int n = subBody->numLinks();
    for(int linkIndex = 1; linkIndex < n; ++linkIndex){
        auto link = subBody->link(linkIndex);
        if(!SKIP_REDUNDANT_ACCEL_CALC || link->cfs.numberToCheckAccelCalcSkip <= skipCheckNumber){
            auto parent = link->parent();
            if(link->isFixedJoint()){
                link->cfs.ddq = 0.0;
                link->cfs.dvo = parent->cfs.dvo;
                link->cfs.dw  = parent->cfs.dw;
            } else {
                link->cfs.ddq = (link->cfs.uu - (link->hhv().dot(parent->cfs.dvo) + link->hhw().dot(parent->cfs.dw))) / link->dd();
                link->cfs.dvo = parent->cfs.dvo + link->cv() + link->sv() * link->cfs.ddq;
                link->cfs.dw  = parent->cfs.dw  + link->cw() + link->sw() * link->cfs.ddq;
            }
            // reset
            link->cfs.uu = link->cfs.uu0;
        }
    }
}


void ConstraintForceSolver::Impl::calcAccelsMM(DySubBody* subBody, int constraintIndex)
{
    auto rootLink = subBody->rootLink();
    rootLink->cfs.dvo = rootLink->dvo();
    rootLink->cfs.dw  = rootLink->dw();

    const int skipCheckNumber = ASSUME_SYMMETRIC_MATRIX ? constraintIndex : (numeric_limits<int>::max() - 1);
    const int n = subBody->numLinks();
    for(int linkIndex = 1; linkIndex < n; ++linkIndex){
        auto link = subBody->link(linkIndex);
        if(!SKIP_REDUNDANT_ACCEL_CALC || link->cfs.numberToCheckAccelCalcSkip <= skipCheckNumber){
            auto parent = link->parent();
            if(link->isFixedJoint()){
                link->cfs.dvo = parent->cfs.dvo;
                link->cfs.dw  = parent->cfs.dw;
            } else {
                link->cfs.dvo = parent->cfs.dvo + link->cv() + link->ddq() * link->sv();
                link->cfs.dw  = parent->cfs.dw  + link->cw() + link->ddq() * link->sw();
            }
        }
    }
}


void ConstraintForceSolver::Impl::extractRelAccelsOfConstraintPoints
(LinkPairBlock& block, int localColumnOffset, bool isFrictionColumn, int constraintIndex)
{
    int maxConstraintIndexToExtract = ASSUME_SYMMETRIC_MATRIX ? constraintIndex : globalNumConstraintVectors;

    // Only the link pairs sharing a dynamic sub-body with the pair of the test
    // force can have nonzero responses; the other elements are structurally
    // zero and are not stored in the sparse matrix.
    for(auto& adj : block.adjacents){
        LinkPair& linkPair = *constrainedLinkPairs[adj.pairIndex];
        int columnPosition =
            (isFrictionColumn ? adj.reciprocalFrictionOffset : adj.reciprocalNormalOffset)
            + localColumnOffset;
        auto subBody0 = linkPair.link[0]->subBody();
        auto subBody1 = linkPair.link[1]->subBody();
        if(subBody0->isTestForceBeingApplied){
            if(subBody1->isTestForceBeingApplied){
                extractRelAccelsFromLinkPairCase1(linkPair, columnPosition, maxConstraintIndexToExtract);
            } else {
                extractRelAccelsFromLinkPairCase2(linkPair, 0, 1, columnPosition, maxConstraintIndexToExtract);
            }
        } else {
            if(subBody1->isTestForceBeingApplied){
                extractRelAccelsFromLinkPairCase2(linkPair, 1, 0, columnPosition, maxConstraintIndexToExtract);
            }
            // When the test force is not applied to either sub-body, the
            // elements remain zero
        }
    }
}


void ConstraintForceSolver::Impl::extractRelAccelsFromLinkPairCase1
(LinkPair& linkPair, int columnPosition, int maxConstraintIndexToExtract)
{
    auto& constraintPoints = linkPair.constraintPoints;

    for(size_t i=0; i < constraintPoints.size(); ++i){

        ConstraintPoint& constraint = constraintPoints[i];
        int constraintIndex = constraint.globalIndex;

        if(ASSUME_SYMMETRIC_MATRIX && constraintIndex > maxConstraintIndexToExtract){
            break;
        }

        auto link0 = linkPair.link[0];
        auto link1 = linkPair.link[1];

        //! \todo Can the follwoing equations be simplified ?
        Vector3 dv0 =
            link0->cfs.dvo - constraint.point.cross(link0->cfs.dw) +
            link0->w().cross(link0->vo() + link0->w().cross(constraint.point));

        Vector3 dv1 =
            link1->cfs.dvo - constraint.point.cross(link1->cfs.dw) +
            link1->w().cross(link1->vo() + link1->w().cross(constraint.point));

        Vector3 relAccel = dv1 - dv0;

        matrixValues[rowInfos[constraintIndex].valueTop + columnPosition] =
            constraint.normalTowardInside[1].dot(relAccel) - an0(constraintIndex);

        for(int j=0; j < constraint.numFrictionVectors; ++j){
            const int index = constraint.globalFrictionIndex + j;
            matrixValues[rowInfos[globalNumConstraintVectors + index].valueTop + columnPosition] =
                constraint.frictionVector[j][1].dot(relAccel) - at0(index);
        }
    }
}


void ConstraintForceSolver::Impl::extractRelAccelsFromLinkPairCase2
(LinkPair& linkPair, int iTestForce, int iDefault, int columnPosition, int maxConstraintIndexToExtract)
{
    auto& constraintPoints = linkPair.constraintPoints;

    for(size_t i=0; i < constraintPoints.size(); ++i){

        ConstraintPoint& constraint = constraintPoints[i];
        int constraintIndex = constraint.globalIndex;

        if(ASSUME_SYMMETRIC_MATRIX && constraintIndex > maxConstraintIndexToExtract){
            break;
        }

        auto link = linkPair.link[iTestForce];

        Vector3 dv(link->cfs.dvo - constraint.point.cross(link->cfs.dw) + link->w().cross(link->vo() + link->w().cross(constraint.point)));

        if(CFS_DEBUG_VERBOSE_2){
            os << "dv " << constraintIndex << " = " << dv << "\n";
        }

        Vector3 relAccel = constraint.defaultAccel[iDefault] - dv;

        matrixValues[rowInfos[constraintIndex].valueTop + columnPosition] =
            constraint.normalTowardInside[iDefault].dot(relAccel) - an0(constraintIndex);

        for(int j=0; j < constraint.numFrictionVectors; ++j){
            const int index = constraint.globalFrictionIndex + j;
            matrixValues[rowInfos[globalNumConstraintVectors + index].valueTop + columnPosition] =
                constraint.frictionVector[j][iDefault].dot(relAccel) - at0(index);
        }

    }
}


void ConstraintForceSolver::Impl::copySymmetricElementsOfAccelerationMatrix()
{
    const int n = globalNumConstraintVectors;
    const int dimLCP = n + globalNumFrictionVectors;

    for(int row = 0; row < dimLCP; ++row){
        LinkPairBlock& block = linkPairBlocks[rowInfos[row].linkPairIndex];
        const int rowTop = rowInfos[row].valueTop;
        const bool isFrictionRow = (row >= n);
        const int localOffset = isFrictionRow ? (row - n - block.frictionTop) : (row - block.normalTop);
        for(auto& adj : block.adjacents){
            LinkPairBlock& adjacent = linkPairBlocks[adj.pairIndex];
            // position of the column corresponding to this row in the adjacent pair's pattern
            const int mirrorColumnPosition =
                (isFrictionRow ? adj.reciprocalFrictionOffset : adj.reciprocalNormalOffset)
                + localOffset;
            for(int j=0; j < adjacent.numNormals; ++j){
                const int column = adjacent.normalTop + j;
                if(column > row){
                    matrixValues[rowInfos[column].valueTop + mirrorColumnPosition] =
                        matrixValues[rowTop + adj.normalOffset + j];
                }
            }
            for(int j=0; j < adjacent.numFrictions; ++j){
                const int column = n + adjacent.frictionTop + j;
                if(column > row){
                    matrixValues[rowInfos[column].valueTop + mirrorColumnPosition] =
                        matrixValues[rowTop + adj.frictionOffset + j];
                }
            }
        }
    }
}


void ConstraintForceSolver::Impl::clearSingularPointConstraintsOfClosedLoopConnections()
{
    const int n = globalNumConstraintVectors;
    const int dimLCP = n + globalNumFrictionVectors;

    for(int i = 0; i < dimLCP; ++i){
        double& diagonal = matrixValues[rowInfos[i].valueTop + rowInfos[i].diagonalPosition];
        if(diagonal < 1.0e-4){
            // Clear the i-th column. Only the rows of the link pairs adjacent
            // to the pair owning constraint i can have the column.
            LinkPairBlock& block = linkPairBlocks[rowInfos[i].linkPairIndex];
            const bool isFrictionColumn = (i >= n);
            const int localColumnOffset =
                isFrictionColumn ? (i - n - block.frictionTop) : (i - block.normalTop);
            for(auto& adj : block.adjacents){
                LinkPairBlock& adjacent = linkPairBlocks[adj.pairIndex];
                const int columnPosition =
                    (isFrictionColumn ? adj.reciprocalFrictionOffset : adj.reciprocalNormalOffset)
                    + localColumnOffset;
                for(int j=0; j < adjacent.numNormals; ++j){
                    matrixValues[rowInfos[adjacent.normalTop + j].valueTop + columnPosition] = 0.0;
                }
                for(int j=0; j < adjacent.numFrictions; ++j){
                    matrixValues[rowInfos[n + adjacent.frictionTop + j].valueTop + columnPosition] = 0.0;
                }
            }
            diagonal = numeric_limits<double>::max();
        }
    }
}


void ConstraintForceSolver::Impl::setConstantVectorAndMuBlock()
{
    const double dt = world.timeStep();
    double dtinv = 1.0 / dt;
    const int block2 = globalNumConstraintVectors;

    for(size_t i=0; i < constrainedLinkPairs.size(); ++i){

        LinkPair& linkPair = *constrainedLinkPairs[i];
        int numConstraintsInPair = linkPair.constraintPoints.size();
        const double restitution =
            linkPair.isNonContactConstraint ? 0.0 : linkPair.contactMaterial->restitution();

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

                // Restitution: when the contact is an impact (the approaching
                // normal velocity exceeds the threshold), require the post-step
                // normal velocity to be -e * (approaching velocity) instead of
                // zero. This is realized by scaling the velocity term of the
                // LCP constant vector by (1 + e). The impact threshold is
                // derived per contact point from an0, the relative normal
                // acceleration under the constraint-free dynamics, which covers
                // every persistent source of approach velocity regeneration
                // (gravity in any direction, slopes, actuator forces pressing
                // the contact, centrifugal terms).
                double vn = constraint.normalProjectionOfRelVelocityOn0;
                if(restitution > 0.0){
                    const double vth = std::max(
                        IMPACT_VELOCITY_THRESH_RATIO * std::max(-an0(globalIndex), 0.0) * dt,
                        MIN_IMPACT_VELOCITY_THRESH);
                    if(vn < -vth){
                        vn *= (1.0 + restitution);
                    }
                }

                if(ENABLE_CONTACT_DEPTH_CORRECTION){
                    double velOffset;
                    const double depth = constraint.depth - contactCorrectionDepth;
                    if(depth <= 0.0){
                        velOffset = contactCorrectionVelocityRatio * depth;
                    } else {
                        // Saturating correction that stays linear (slope =
                        // contactCorrectionVelocityRatio) near depth = 0, so it
                        // joins the shallow branch smoothly, and tends to the
                        // upper bound contactCorrectionVelocityRatio * s as the
                        // excess depth grows, where s is the characteristic
                        // depth scaled from d0.
                        const double s = CONTACT_CORRECTION_SATURATION_DEPTH_RATIO * contactCorrectionDepth;
                        velOffset = contactCorrectionVelocityRatio * depth / (1.0 + depth / s);
                    }
                    b(globalIndex) = an0(globalIndex) + (vn - velOffset) * dtinv;
                } else {
                    b(globalIndex) = an0(globalIndex) + vn * dtinv;
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

                    // for iterative solver
                    frictionIndexToContactIndex[globalFrictionIndex] = globalIndex;

                    ++globalFrictionIndex;
                }
            }
        }
    }
}


void ConstraintForceSolver::Impl::addConstraintForceToLinks()
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


void ConstraintForceSolver::Impl::addConstraintForceToLink(LinkPair* linkPair, int ipair)
{
    auto& constraintPoints = linkPair->constraintPoints;
    int numConstraintPoints = constraintPoints.size();

    if(numConstraintPoints > 0){
        auto link = linkPair->link[ipair];
        Vector3 f_total   = Vector3::Zero();
        Vector3 tau_total = Vector3::Zero();
        bool doUpdateContactStates = (link->sensingMode() & Link::LinkContactState);

        for(int i=0; i < numConstraintPoints; ++i){

            ConstraintPoint& constraint = constraintPoints[i];
            int globalIndex = constraint.globalIndex;

            Vector3 f = solution(globalIndex) * constraint.normalTowardInside[ipair];
            for(int j=0; j < constraint.numFrictionVectors; ++j){
                f += solution(globalNumConstraintVectors + constraint.globalFrictionIndex + j) * constraint.frictionVector[j][ipair];
            }
            f_total   += f;
            tau_total += constraint.point.cross(f);

            if(doUpdateContactStates){
                link->contactPoints().emplace_back(
                    Link::ContactPoint(
                        constraint.point, constraint.normalTowardInside[ipair], f,
                        (ipair == 0) ? -constraint.relVelocityOn0 : constraint.relVelocityOn0,
                        constraint.depth));
            }
        }
    
        link->f_ext()   += f_total;
        link->tau_ext() += tau_total;

        if(CFS_DEBUG){
            os << "Constraint force to " << link->name() << ": f = " << f_total << ", tau = " << tau_total << std::endl;
        }
    }
}


void ConstraintForceSolver::Impl::solveMCPByProjectedGaussSeidel(const VectorX& b, VectorX& x)
{
    if(numGaussSeidelInitialIteration > 0){
        solveMCPByProjectedGaussSeidelInitial(b, x, numGaussSeidelInitialIteration);
    }

    // When the error criterion is zero (or negative), convergence checking is
    // disabled and the solver always performs the maximum number of iterations.
    // This avoids the per-iteration norm computation and makes the cost of each
    // step predictable, as in a fixed-iteration solver.
    const bool checkConvergence = (gaussSeidelErrorCriterion > 0.0);

    if(CFS_MCP_DEBUG){
        os << "Iteration ";
    }

    double error = 0.0;
    VectorXd x0;
    int i = 0;
    while(i < maxNumGaussSeidelIteration){
        i++;

        if(checkConvergence){
            x0 = x;
        }

        solveMCPByProjectedGaussSeidelMainStep(b, x);

        if(checkConvergence){
            double n = x.norm();
            if(n > THRESH_TO_SWITCH_REL_ERROR){
                error = (x - x0).norm() / n;
            } else {
                error = (x - x0).norm();
            }
            if(error < gaussSeidelErrorCriterion){
                if(CFS_MCP_DEBUG_SHOW_ITERATION_STOP){
                    os << "stopped at " << i << ", error = " << error << endl;
                }
                break;
            }
        }
    }

    if(CFS_MCP_DEBUG){

        if(i == maxNumGaussSeidelIteration){
            os << "not stopped" << ", error = " << error << endl;
        }

        numGaussSeidelTotalLoops += i;
        numGaussSeidelTotalCalls++;
        numGaussSeidelTotalLoopsMax = std::max(numGaussSeidelTotalLoopsMax, i);
        os << ", avarage = " << (numGaussSeidelTotalLoops / numGaussSeidelTotalCalls);
        os << ", max = " << numGaussSeidelTotalLoopsMax;
        os << endl;
    }
}


void ConstraintForceSolver::Impl::solveMCPByProjectedGaussSeidelMainStep(const VectorX& b, VectorX& x)
{
    const int size = globalNumConstraintVectors + globalNumFrictionVectors;

    for(int j=0; j < globalNumContactNormalVectors; ++j){

        double xx = solveGaussSeidelRow(j, b, x);
        if(xx < 0.0){
            x(j) = 0.0;
        } else {
            x(j) = xx;
        }
        mcpHi[j] = contactIndexToMu[j] * x(j);
    }

    for(int j=globalNumContactNormalVectors; j < globalNumConstraintVectors; ++j){
        x(j) = solveGaussSeidelRow(j, b, x);
    }


    if(ENABLE_TRUE_FRICTION_CONE){

        int contactIndex = 0;
        for(int j=globalNumConstraintVectors; j < size; ++j, ++contactIndex){

            double fx0 = solveGaussSeidelRow(j, b, x);
            double& fx = x(j);

            ++j;

            double fy0 = solveGaussSeidelRow(j, b, x);
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

            double xx = solveGaussSeidelRow(j, b, x);

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


void ConstraintForceSolver::Impl::solveMCPByProjectedGaussSeidelInitial
(const VectorX& b, VectorX& x, const int numIteration)
{
    const int size = globalNumConstraintVectors + globalNumFrictionVectors;

    const double rstep = 1.0 / (numIteration * size);
    double r = 0.0;

    for(int i=0; i < numIteration; ++i){

        for(int j=0; j < globalNumContactNormalVectors; ++j){

            double xx = solveGaussSeidelRow(j, b, x);
            if(xx < 0.0){
                x(j) = 0.0;
            } else {
                x(j) = r * xx;
            }
            r += rstep;
            mcpHi[j] = contactIndexToMu[j] * x(j);
        }

        for(int j=globalNumContactNormalVectors; j < globalNumConstraintVectors; ++j){
            x(j) = r * solveGaussSeidelRow(j, b, x);
            r += rstep;
        }

        if(ENABLE_TRUE_FRICTION_CONE){

            int contactIndex = 0;
            for(int j=globalNumConstraintVectors; j < size; ++j, ++contactIndex){

                double fx0 = solveGaussSeidelRow(j, b, x);
                double& fx = x(j);

                ++j;

                double fy0 = solveGaussSeidelRow(j, b, x);
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

                double xx = solveGaussSeidelRow(j, b, x);

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


void ConstraintForceSolver::Impl::multiplyMatrixAndVector(const VectorX& x, VectorX& y)
{
    const int dimLCP = globalNumConstraintVectors + globalNumFrictionVectors;
    y.resize(dimLCP);
    for(int row = 0; row < dimLCP; ++row){
        auto& block = linkPairBlocks[rowInfos[row].linkPairIndex];
        const double* values = &matrixValues[rowInfos[row].valueTop];
        const int numColumns = block.columns.size();
        double sum = 0.0;
        for(int k=0; k < numColumns; ++k){
            sum += values[k] * x(block.columns[k]);
        }
        y(row) = sum;
    }
}


// for debugging
ConstraintForceSolver::Impl::MatrixX ConstraintForceSolver::Impl::makeDenseMatrix()
{
    const int dimLCP = globalNumConstraintVectors + globalNumFrictionVectors;
    MatrixX M = MatrixX::Zero(dimLCP, dimLCP);
    for(int row = 0; row < dimLCP; ++row){
        auto& columns = linkPairBlocks[rowInfos[row].linkPairIndex].columns;
        for(size_t k=0; k < columns.size(); ++k){
            M(row, columns[k]) = matrixValues[rowInfos[row].valueTop + k];
        }
    }
    return M;
}


void ConstraintForceSolver::Impl::checkLCPResult(VectorX& b, VectorX& x)
{
    os << "check LCP result\n";
    os << "-------------------------------\n";

    VectorX z;
    multiplyMatrixAndVector(x, z);
    z += b;

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


void ConstraintForceSolver::Impl::checkMCPResult(VectorX& b, VectorX& x)
{
    os << "check MCP result\n";
    os << "-------------------------------\n";

    VectorX z;
    multiplyMatrixAndVector(x, z);
    z += b;

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


ConstraintForceSolver::ConstraintForceSolver(DyWorldBase& world)
{
    impl = new ConstraintForceSolver::Impl(world);
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


void ConstraintForceSolver::setFrictionCoefficientRange(double minFriction, double maxFriction)
{
    impl->minFrictionCoefficient = minFriction;
    impl->maxFrictionCoefficient = maxFriction;
}


double ConstraintForceSolver::minFrictionCoefficient() const
{
    return impl->minFrictionCoefficient;
}


double ConstraintForceSolver::maxFrictionCoefficient() const
{
    return impl->maxFrictionCoefficient;
}


void ConstraintForceSolver::registerCollisionHandler(const std::string& name, CollisionHandler handler)
{
    ConstraintForceSolver::Impl::CollisionHandlerInfoPtr info = new ConstraintForceSolver::Impl::CollisionHandlerInfo;;
    info->handler = handler;
    impl->collisionHandlerMap[name] = info;
}


bool ConstraintForceSolver::unregisterCollisionHandler(const std::string& name)
{
    return (impl->collisionHandlerMap.erase(name) > 0);
}


void ConstraintForceSolver::setBodyCollisionDetectionMode
(int bodyIndex, bool isBodyToBodyCollisionEnabled, bool isSelfCollisionEnabled)
{
    if(bodyIndex >= impl->bodyIndexToCollisionDetectionModeMap.size()){
        impl->bodyIndexToCollisionDetectionModeMap.resize(bodyIndex + 1, BodyToBodyCollision);
    }
    int mode = 0;
    if(isBodyToBodyCollisionEnabled){
        mode = BodyToBodyCollision;
    }
    if(isSelfCollisionEnabled){
        mode |= SelfCollision;
    }
    impl->bodyIndexToCollisionDetectionModeMap[bodyIndex] = mode;
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


void ConstraintForceSolver::setMaxNumContactPoints(int n)
{
    impl->defaultMaxNumContactPoints = n;
}


int ConstraintForceSolver::maxNumContactPoints() const
{
    return impl->defaultMaxNumContactPoints;
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


void ConstraintForceSolver::setNumCollisionDetectionThreads(int n)
{
    impl->numCollisionDetectionThreads = n;
}


int ConstraintForceSolver::numCollisionDetectionThreads() const
{
    return impl->numCollisionDetectionThreads;
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


void ConstraintForceSolver::enableConstraintForceOutput(bool /* on */)
{

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
    for(auto& body : impl->world.bodies()){
        body->clearExternalForces();
    }
}


shared_ptr<CollisionLinkPairList> ConstraintForceSolver::getCollisions()
{
    return impl->getCollisions();
}


shared_ptr<CollisionLinkPairList> ConstraintForceSolver::Impl::getCollisions()
{
    auto collisionPairs = std::make_shared<CollisionLinkPairList>();
    for(size_t i=0; i < constrainedLinkPairs.size(); ++i){
        LinkPair& source = *constrainedLinkPairs[i];
        auto dest = std::make_shared<CollisionLinkPair>();
        int numConstraintsInPair = source.constraintPoints.size();

        for(int j=0; j < numConstraintsInPair; ++j){
            ConstraintPoint& constraint = source.constraintPoints[j];
            dest->collisions().push_back(Collision());
            Collision& col = dest->collisions().back();
            col.point = constraint.point;
            col.normal = constraint.normalTowardInside[1];
            col.depth = constraint.depth;
        }
        dest->setLinkPair(source.link[0], source.link[1]);
        collisionPairs->push_back(dest);
    }

    return collisionPairs;
}
