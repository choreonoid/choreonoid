/*
 * Copyright (c) 2008, AIST, the University of Tokyo and General Robotix Inc.
 * All rights reserved. This program is made available under the terms of the
 * Eclipse Public License v1.0 which accompanies this distribution, and is
 * available at http://www.eclipse.org/legal/epl-v10.html
 * Contributors:
 * National Institute of Advanced Industrial Science and Technology (AIST)
 */

/** \file
    \brief Implementation of ConstraintForceSolver class
    \author Shin'ichiro Nakaoka
*/

#ifdef __WIN32__
#define NOMINMAX
#endif

#include "ABMWorld.h"
#include "ABMBody.h"
#include "LinkTraverse.h"
#include "ForwardDynamicsCBM.h"
#include "ConstraintForceSolver.h"
#include <cnoid/uBlasCommonTypes>
#include <cnoidCorba/OpenHRPCommon.hh>
#include <cnoid/ColdetModelPair>
#include <fmt/format.h>
#include <boost/random.hpp>
#include <boost/numeric/ublas/triangular.hpp>
#include <boost/numeric/ublas/lu.hpp>
#include <tuple>
#include <limits>


using namespace std;
using namespace tvmet;
using namespace boost::numeric::ublas;
using namespace cnoid;
using namespace OpenHRP;



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

static const int DEFAULT_MAX_NUM_GAUSS_SEIDEL_ITERATION = 500;
static const int DEFAULT_NUM_GAUSS_SEIDEL_ITERATION_BLOCK = 10;
static const int DEFAULT_NUM_GAUSS_SEIDEL_INITIAL_ITERATION = 0;
static const double DEFAULT_GAUSS_SEIDEL_MAX_REL_ERROR = 1.0e-3;
static const bool USE_PREVIOUS_LCP_SOLUTION = true;

static const bool ALLOW_SUBTLE_PENETRATION_FOR_STABILITY = true;

// normal setting
static const double ALLOWED_PENETRATION_DEPTH = 0.0001;
//static const double PENETRATION_A = 500.0;
//static const double PENETRATION_B = 80.0;
static const double NEGATIVE_VELOCITY_RATIO_FOR_PENETRTION = 10.0;


// test for mobile robots with wheels
//static const double ALLOWED_PENETRATION_DEPTH = 0.005;
//static const double PENETRATION_A = 500.0;
//static const double PENETRATION_B = 80.0;
//static const double NEGATIVE_VELOCITY_RATIO_FOR_PENETRTION = 10.0;
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


namespace cnoid
{
class CFSImpl
{
public:

    CFSImpl(WorldBase& world);

    bool addCollisionCheckLinkPair
    (int bodyIndex1, Link* link1, int bodyIndex2, Link* link2, double muStatic, double muDynamic, double culling_thresh, double epsilon);

    void initialize(void);
    void solve(CollisionSequence& corbaCollisionSequence);
    inline void clearExternalForces();

#undef PI
#ifdef __WIN32__       // Visual C++ bug
    static const double PI;
    static const double PI_2;
#elif defined(__APPLE__)
#define PI M_PI
#define PI_2 M_PI/2
#else
    static const double PI   = 3.14159265358979323846;
    static const double PI_2 = 1.57079632679489661923;
#endif

    WorldBase& world;

    bool isConstraintForceOutputMode;
    bool useBuiltinCollisionDetector;

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
        Vector3	dvo;
        Vector3	dw;
        Vector3	pf0;
        Vector3	ptau0;
        double  uu;
        double  uu0;
        double  ddq;
        int     numberToCheckAccelCalcSkip;
        int     parentIndex;
        Link*   link;
    };
    typedef std::vector<LinkData> LinkDataArray;

    struct BodyData
    {
        BodyPtr body;
        bool isStatic;
        bool hasConstrainedLinks;
        bool isTestForceBeingApplied;
        LinkDataArray linksData;

        Vector3 dpf;
        Vector3 dptau;

        /// only used by the ForwardDynamicsMM mode
        Vector3* rootLinkPosRef;

        /**
           If the body includes high-gain mode joints,
           the ForwardDynamisMM object of the body is set to this pointer.
           The pointer is null when all the joints are torque mode and
           the forward dynamics is calculated by ABM.
        */
        ForwardDynamicsMMPtr forwardDynamicsMM;
    };

    std::vector<BodyData> bodiesData;

    class LinkPair : public ColdetModelPair
    {
    public:
        int index;
        bool isSameBodyPair;
        int bodyIndex[2];
        BodyData* bodyData[2];
        Link* link[2];
        LinkData* linkData[2];
        ConstraintPointArray constraintPoints;

        double muStatic;
        double muDynamic;
        double culling_thresh;
        double epsilon;

        Body::LinkConnection* connection;

    };
    typedef std::shared_ptr<LinkPair> LinkPairPtr;
    typedef std::vector<LinkPairPtr> LinkPairArray;

    LinkPairArray collisionCheckLinkPairs;
    LinkPairArray connectedLinkPairs;

    std::vector<LinkPair*> constrainedLinkPairs;

    /**
       globalNumConstraintVectors = globalNumContactNormalVectors + globalNumConnectionVectors
    */
    int globalNumConstraintVectors;

    int globalNumContactNormalVectors;
    int globalNumConnectionVectors;
    int globalNumFrictionVectors;

    int prevGlobalNumConstraintVectors;
    int prevGlobalNumFrictionVectors;

    bool areThereImpacts;
    int numUnconverged;

    // Mlcp * solution + b   _|_  solution

    typedef boost::numeric::ublas::matrix<double, ublas::row_major> rmdmatrix;
    rmdmatrix Mlcp;

    // constant acceleration term when no external force is applied
    dvector an0;
    dvector at0;

    // constant vector of LCP
    dvector b;

    // contact force solution: normal forces at contact points
    dvector solution;

    // random number generator
    variate_generator<mt19937, uniform_real<> > randomAngle;

    // for special version of gauss sidel iterative solver
    std::vector<int> frictionIndexToContactIndex;
    dvector contactIndexToMu;
    dvector mcpHi;

    int  maxNumGaussSeidelIteration;
    int  numGaussSeidelInitialIteration;
    double gaussSeidelMaxRelError;

    int numGaussSeidelTotalLoops;
    int numGaussSeidelTotalCalls;

    void setConstraintPoints(CollisionSequence& collisions);
    void setContactConstraintPoints(LinkPair& linkPair, CollisionPointSequence& collisionPoints);
    void setFrictionVectors(ConstraintPoint& constraintPoint);
    bool setConnectionConstraintPoints(LinkPair& linkPair);
    void putContactPoints();
    void solveImpactConstraints();
    void initMatrices();
    void setAccelCalcSkipInformation();
    void setDefaultAccelerationVector();
    void setAccelerationMatrix();
    void initABMForceElementsWithNoExtForce(BodyData& bodyData);
    void calcABMForceElementsWithTestForce(BodyData& bodyData, Link* linkToApplyForce, const Vector3& f, const Vector3& tau);
    void calcAccelsABM(BodyData& bodyData, int constraintIndex);
    void calcAccelsMM(BodyData& bodyData, int constraintIndex);

    void extractRelAccelsOfConstraintPoints
    (matrix_range<rmdmatrix>& Kxn, matrix_range<rmdmatrix>& Kxt, int testForceIndex, int constraintIndex);

    void extractRelAccelsFromLinkPairCase1
    (matrix_range<rmdmatrix>& Kxn, matrix_range<rmdmatrix>& Kxt, LinkPair& linkPair, int testForceIndex, int constraintIndex);
    void extractRelAccelsFromLinkPairCase2
    (matrix_range<rmdmatrix>& Kxn, matrix_range<rmdmatrix>& Kxt, LinkPair& linkPair, int iTestForce, int iDefault, int testForceIndex, int constraintIndex);
    void extractRelAccelsFromLinkPairCase3
    (matrix_range<rmdmatrix>& Kxn, matrix_range<rmdmatrix>& Kxt, LinkPair& linkPair, int testForceIndex, int constraintIndex);

    void copySymmetricElementsOfAccelerationMatrix
    (matrix_range<rmdmatrix>& Knn, matrix_range<rmdmatrix>& Ktn, matrix_range<rmdmatrix>& Knt, matrix_range<rmdmatrix>& Ktt);

    void clearSingularPointConstraintsOfClosedLoopConnections();
		
    void setConstantVectorAndMuBlock();
    void addConstraintForceToLinks();
    void addConstraintForceToLink(LinkPair* linkPair, int ipair);

    void solveMCPByProjectedGaussSeidel
    (const rmdmatrix& M, const dvector& b, dvector& x);
    void solveMCPByProjectedGaussSeidelInitial
    (const rmdmatrix& M, const dvector& b, dvector& x, const int numIteration);
    void solveMCPByProjectedGaussSeidelMain
    (const rmdmatrix& M, const dvector& b, dvector& x, const int numIteration);
    double solveMCPByProjectedGaussSeidelErrorCheck
    (const rmdmatrix& M, const dvector& b, dvector& x);

    void checkLCPResult(rmdmatrix& M, dvector& b, dvector& x);
    void checkMCPResult(rmdmatrix& M, dvector& b, dvector& x);

#ifdef USE_PIVOTING_LCP
    bool callPathLCPSolver(rmdmatrix& Mlcp, dvector& b, dvector& solution);

    // for PATH solver
    std::vector<double> lb;
    std::vector<double> ub;
    std::vector<int> m_i;
    std::vector<int> m_j;
    std::vector<double> m_ij;
#endif

};
#ifdef __WIN32__

const double CFSImpl::PI   = 3.14159265358979323846;
const double CFSImpl::PI_2 = 1.57079632679489661923;
#endif
};

template<class TMatrix>
static void putMatrix(TMatrix& M, const char *name)
{
    if(M.size2() == 1){
        std::cout << "Vector " << name << M << std::endl;
    } else {
        std::cout << "Matrix " << name << ": \n";
        for(size_t i=0; i < M.size1(); i++){
            for(size_t j=0; j < M.size2(); j++){
                std::cout << fmt::format(" {:6.3f} ", M(i, j));
            }
            std::cout << std::endl;
        }
    }
}

template<class TVector>
static void putVector(TVector& M, const char *name)
{
    std::cout << "Vector " << name << M << std::endl;
}

template<class TMatrix>
static inline void debugPutMatrix(TMatrix& M, const char *name)
{
    if(CFS_DEBUG_VERBOSE) putMatrix(M, name);
}

template<class TVector>
static inline void debugPutVector(TVector& M, const char *name)
{
    if(CFS_DEBUG_VERBOSE) putVector(M, name);
}


CFSImpl::CFSImpl(WorldBase& world) :
    world(world),
    randomAngle(mt19937(), uniform_real<>(0.0, 2.0 * PI))
{
    maxNumGaussSeidelIteration = DEFAULT_MAX_NUM_GAUSS_SEIDEL_ITERATION;
    numGaussSeidelInitialIteration = DEFAULT_NUM_GAUSS_SEIDEL_INITIAL_ITERATION;
    gaussSeidelMaxRelError = DEFAULT_GAUSS_SEIDEL_MAX_REL_ERROR;
    isConstraintForceOutputMode = false;
    useBuiltinCollisionDetector = false;
}


bool CFSImpl::addCollisionCheckLinkPair
(int bodyIndex1, Link* link1, int bodyIndex2, Link* link2, double muStatic, double muDynamic, double culling_thresh, double epsilon)
{
    int index;
    int isRegistered;

    std::tie(index, isRegistered) = world.getIndexOfLinkPairs(link1, link2);

    if(index >= 0){

        int n = collisionCheckLinkPairs.size();
        if(index >= n){
            collisionCheckLinkPairs.resize(index+1);
        }

        LinkPairPtr& linkPair = collisionCheckLinkPairs[index];
        if(!linkPair){
            linkPair = new LinkPair();
        }

        linkPair->set(link1->coldetModel, link2->coldetModel);
        linkPair->isSameBodyPair = (bodyIndex1 == bodyIndex2);
        linkPair->bodyIndex[0] = bodyIndex1;
        linkPair->link[0] = link1;
        linkPair->bodyIndex[1] = bodyIndex2;
        linkPair->link[1] = link2;
        linkPair->index = index;
        linkPair->muStatic = muStatic;
        linkPair->muDynamic = muDynamic;
        linkPair->culling_thresh = culling_thresh;
        linkPair->epsilon = epsilon;
        linkPair->connection = 0;
    }

    return (index >= 0 && !isRegistered);
}



void CFSImpl::initialize(void)
{
    if(CFS_MCP_DEBUG){
        numGaussSeidelTotalCalls = 0;
        numGaussSeidelTotalLoops = 0;
    }

    int numBodies = world.numBodies();

    bodiesData.resize(numBodies);

    connectedLinkPairs.clear();

    for(int bodyIndex=0; bodyIndex < numBodies; ++bodyIndex){

        BodyPtr body = world.body(bodyIndex);

        body->clearExternalForces();
        BodyData& bodyData = bodiesData[bodyIndex];
        bodyData.body = body;
        bodyData.linksData.resize(body->numLinks());
        bodyData.hasConstrainedLinks = false;
        bodyData.isTestForceBeingApplied = false;
        bodyData.isStatic = body->isStaticModel();

        bodyData.forwardDynamicsMM =
            dynamic_pointer_cast<ForwardDynamicsMM>(world.forwardDynamics(bodyIndex));

        LinkDataArray& linksData = bodyData.linksData;

        if(bodyData.isStatic && !(bodyData.forwardDynamicsMM)){
            int n = linksData.size();
            for(int j=0; j < n; ++j){
                LinkData& linkData = linksData[j];
                linkData.dw  = 0.0;
                linkData.dvo = 0.0;
            }
        }

        // initialize link data
        const LinkTraverse& traverse = body->linkTraverse();
        for(int j=0; j < traverse.numLinks(); ++j){
            Link* link = traverse[j];
            linksData[link->index].link = link;
            linksData[link->index].parentIndex = link->parent ? link->parent->index : -1;
        }

        // initialize link connection
        Body::LinkConnectionArray& connections = body->linkConnections;
        for(size_t j=0; j < connections.size(); ++j){

            connectedLinkPairs.push_back(new LinkPair());
            LinkPair& linkPair = *connectedLinkPairs.back();

            Body::LinkConnection& connection = connections[j];
            linkPair.connection = &connection;
            linkPair.isSameBodyPair = true;
            linkPair.constraintPoints.resize(connection.numConstraintAxes);

            for(int k=0; k < connection.numConstraintAxes; ++k){
                ConstraintPoint& constraint = linkPair.constraintPoints[k];
                constraint.numFrictionVectors = 0;
                constraint.globalFrictionIndex = numeric_limits<int>::max();
            }

            for(int k=0; k < 2; ++k){
                linkPair.bodyIndex[k] = bodyIndex;
                linkPair.bodyData[k] = &bodiesData[bodyIndex];
                Link* link = connection.link[k];
                linkPair.link[k] = link;
                linkPair.linkData[k] = &(bodyData.linksData[link->index]);
            }
        }
    }

    int numLinkPairs = collisionCheckLinkPairs.size();
    for(int i=0; i < numLinkPairs; ++i){
        LinkPair& linkPair = *collisionCheckLinkPairs[i];
        for(int j=0; j < 2; ++j){
            BodyData& bodyData = bodiesData[linkPair.bodyIndex[j]];
            linkPair.bodyData[j] = &bodyData;
            linkPair.linkData[j] = &(bodyData.linksData[linkPair.link[j]->index]);
        }
    }

    prevGlobalNumConstraintVectors = 0;
    prevGlobalNumFrictionVectors = 0;
    numUnconverged = 0;

    randomAngle.engine().seed();
}


inline void CFSImpl::clearExternalForces()
{
    for(size_t i=0; i < bodiesData.size(); ++i){
        BodyData& bodyData = bodiesData[i];
        if(bodyData.hasConstrainedLinks){
            bodyData.body->clearExternalForces();
        }
    }
}


void CFSImpl::solve(CollisionSequence& corbaCollisionSequence)
{
    if(CFS_DEBUG)
        std::cout << "Time: " << world.currentTime() << std::endl;

    for(size_t i=0; i < bodiesData.size(); ++i){
        bodiesData[i].hasConstrainedLinks = false;
        if(useBuiltinCollisionDetector){
            bodiesData[i].body->updateLinkColdetModelPositions();
        }
        if(isConstraintForceOutputMode){
            BodyPtr& body = bodiesData[i].body;
            const int n = body->numLinks();
            for(int j=0; j < n; ++j){
                body->link(j)->constraintForces.clear();
            }
        }
    }

    globalNumConstraintVectors = 0;
    globalNumFrictionVectors = 0;
    areThereImpacts = false;
    constrainedLinkPairs.clear();

    setConstraintPoints(corbaCollisionSequence);

    if(CFS_PUT_NUM_CONTACT_POINTS){
        cout << globalNumContactNormalVectors;
    }

    if(globalNumConstraintVectors > 0){

        if(CFS_DEBUG){
            std::cout << "Num Collisions: " << globalNumContactNormalVectors << std::endl;
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
            vector_range<dvector> b1(b, range(0, globalNumConstraintVectors));
            debugPutVector(b1, "b1");
            vector_range<dvector> b2(b, range(globalNumConstraintVectors, globalNumConstraintVectors + globalNumFrictionVectors));
            debugPutVector(b2, "b2");
        }

        bool isConverged;
#ifdef USE_PIVOTING_LCP
        isConverged = callPathLCPSolver(Mlcp, b, solution);
#else
        if(!USE_PREVIOUS_LCP_SOLUTION || constraintsSizeChanged){
            solution.clear();
        }
        solveMCPByProjectedGaussSeidel(Mlcp, b, solution);
        isConverged = true;
#endif

        if(!isConverged){
            ++numUnconverged;
            if(CFS_DEBUG)
                std::cout << "LCP didn't converge" << numUnconverged << std::endl;
        } else {
            if(CFS_DEBUG)
                std::cout << "LCP converged" << std::endl;
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


void CFSImpl::setConstraintPoints(CollisionSequence& collisions)
{
    static const bool enableNormalVisualization = true;

    CollisionPointSequence collisionPoints;
    CollisionPointSequence* pCollisionPoints = 0;

    if(useBuiltinCollisionDetector && enableNormalVisualization){
        collisions.length(collisionCheckLinkPairs.size());
    }
	
    for(size_t colIndex=0; colIndex < collisionCheckLinkPairs.size(); ++colIndex){
        pCollisionPoints = 0;
        LinkPair& linkPair = *collisionCheckLinkPairs[colIndex];

        if( ! useBuiltinCollisionDetector){
            CollisionPointSequence& points = collisions[colIndex].points;
            if(points.length() > 0){
                pCollisionPoints = &points;
            }
        } else {
            if(enableNormalVisualization){
                Collision& collision = collisions[colIndex];
                collision.pair.charName1 = CORBA::string_dup(linkPair.bodyData[0]->body->name().c_str());
                collision.pair.charName2 = CORBA::string_dup(linkPair.bodyData[1]->body->name().c_str());
                collision.pair.linkName1 = CORBA::string_dup(linkPair.link[0]->name.c_str());
                collision.pair.linkName2 = CORBA::string_dup(linkPair.link[1]->name.c_str());
                pCollisionPoints = &collision.points;
            } else {
                pCollisionPoints = &collisionPoints;
            }

            std::vector<collision_data>& cdata = linkPair.detectCollisions();
            
            if(cdata.empty()){
                pCollisionPoints = 0;
            } else {
                int npoints = 0;
                for(int i = 0; i < cdata.size(); i++) {
                    for(int j = 0; j < cdata[i].num_of_i_points; j++){
                        if(cdata[i].i_point_new[j]) npoints++;
                    }
                }
                if(npoints == 0){
                    pCollisionPoints = 0;
                } else {
                    pCollisionPoints->length(npoints);
                    int idx = 0;
                    for (int i = 0; i < cdata.size(); i++) {
                        collision_data& cd = cdata[i];
                        for(int j=0; j < cd.num_of_i_points; j++){
                            if (cd.i_point_new[j]){
                                CollisionPoint& point = (*pCollisionPoints)[idx];
                                for(int k=0; k < 3; k++){
                                    point.position[k] = cd.i_points[j][k];
                                }
                                for(int k=0; k < 3; k++){
                                    point.normal[k] = cd.n_vector[k];
                                }
                                point.idepth = cd.depth;
                                idx++;
                            }
                        }
                    }
                }
            }
        }

        if(pCollisionPoints){
            constrainedLinkPairs.push_back(&linkPair);
            setContactConstraintPoints(linkPair, *pCollisionPoints);
            linkPair.bodyData[0]->hasConstrainedLinks = true;
            linkPair.bodyData[1]->hasConstrainedLinks = true;
        }
    }

    globalNumContactNormalVectors = globalNumConstraintVectors;

    for(size_t i=0; i < connectedLinkPairs.size(); ++i){
        LinkPair& linkPair = *connectedLinkPairs[i];
        constrainedLinkPairs.push_back(&linkPair);
        if(setConnectionConstraintPoints(linkPair)){
            linkPair.bodyData[0]->hasConstrainedLinks = true;
            linkPair.bodyData[1]->hasConstrainedLinks = true;
        } else {
            constrainedLinkPairs.pop_back();
        }
    }
    globalNumConnectionVectors = globalNumConstraintVectors - globalNumContactNormalVectors;
}


void CFSImpl::setContactConstraintPoints(LinkPair& linkPair, CollisionPointSequence& collisionPoints)
{
    ConstraintPointArray& constraintPoints = linkPair.constraintPoints;
    constraintPoints.clear();
    int numExtractedPoints = 0;
    int numContactsInPair = collisionPoints.length();

    for(int j=0; j < numContactsInPair; ++j){

        CollisionPoint& collision = collisionPoints[j];
        constraintPoints.push_back(ConstraintPoint());
        ConstraintPoint& contact = constraintPoints.back();

        getVector3(contact.point, collision.position);
        getVector3(contact.normalTowardInside[1], collision.normal);
        contact.normalTowardInside[0] = -contact.normalTowardInside[1];
        contact.depth = collision.idepth;

        bool isNeighborhood = false;
        
        // dense contact points are eliminated
        for(int k=0; k < numExtractedPoints; ++k){
            if(norm2(constraintPoints[k].point - contact.point) < linkPair.culling_thresh){
                isNeighborhood = true;
                break;
            }
        }

        if(isNeighborhood){
            constraintPoints.pop_back();
        } else {
            numExtractedPoints++;
            contact.globalIndex = globalNumConstraintVectors++;

            // check velocities
            Vector3 v[2];
            for(int k=0; k < 2; ++k){
                Link* link = linkPair.link[k];
                if(link->isRoot() && link->jointType == Link::FIXED_JOINT){
                    v[k] = 0.0;
                } else {
                    v[k] = link->vo + cross(link->w, contact.point);
                }
            }
            contact.relVelocityOn0 = v[1] - v[0];

            contact.normalProjectionOfRelVelocityOn0 = dot(contact.normalTowardInside[1], contact.relVelocityOn0);

            if( ! areThereImpacts){
                if(contact.normalProjectionOfRelVelocityOn0 < -1.0e-6){
                    areThereImpacts = true;
                }
            }

            Vector3 v_tangent(contact.relVelocityOn0 - contact.normalProjectionOfRelVelocityOn0 * contact.normalTowardInside[1]);

            contact.globalFrictionIndex = globalNumFrictionVectors;

            double vt_square = dot(v_tangent, v_tangent);
            static const double vsqrthresh = VEL_THRESH_OF_DYNAMIC_FRICTION * VEL_THRESH_OF_DYNAMIC_FRICTION;
            bool isSlipping = (vt_square > vsqrthresh);
            contact.mu = isSlipping ? linkPair.muDynamic : linkPair.muStatic;

            if( !ONLY_STATIC_FRICTION_FORMULATION && isSlipping){
                contact.numFrictionVectors = 1;
                double vt_mag = sqrt(vt_square);
                Vector3 t1(v_tangent / vt_mag);
                Vector3 t2(cross(contact.normalTowardInside[1], t1));
                Vector3 t3(cross(t2, contact.normalTowardInside[1]));
                contact.frictionVector[0][0] = normalize(t3);
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
    }
}



void CFSImpl::setFrictionVectors(ConstraintPoint& contact)
{
    Vector3 u(0.0);
    int minAxis = 0;
    Vector3& normal = contact.normalTowardInside[0];

    for(int i=1; i < 3; i++){
        if(fabs(normal(i)) < fabs(normal(minAxis))){
            minAxis = i;
        }
    }
    u(minAxis) = 1.0;

    Vector3 t1(cross(normal, u));
    t1 /= norm2(t1);
    Vector3 t2(cross(normal, t1));
    t2 /= norm2(t2);

    if(ENABLE_RANDOM_STATIC_FRICTION_BASE){
        double theta = randomAngle();
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


bool CFSImpl::setConnectionConstraintPoints(LinkPair& linkPair)
{
    ConstraintPointArray& constraintPoints = linkPair.constraintPoints;

    Body::LinkConnection* connection = linkPair.connection;

    Link* link0 = connection->link[0];
    Link* link1 = connection->link[1];

    Vector3 point[2];
    point[0] = link0->p + link0->R * connection->point[0];
    point[1] = link1->p + link1->R * connection->point[1];
    Vector3 midPoint((point[0] + point[1]) / 2.0);
    Vector3 error(midPoint - point[0]);

    if(dot(error, error) > (0.04 * 0.04)){
        return false;
    }

    // check velocities
    Vector3 v[2];
    for(int k=0; k < 2; ++k){
        Link* link = connection->link[k];
        if(link->isRoot() && link->jointType == Link::FIXED_JOINT){
            v[k] = 0.0;
        } else {
            v[k] = link->vo + cross(link->w, point[k]);
        }
    }
    Vector3 relVelocityOn0(v[1] - v[0]);

    for(int i=0; i < connection->numConstraintAxes; ++i){
        ConstraintPoint& constraint = constraintPoints[i];
        constraint.point = midPoint;
        const Vector3 axis(link0->R * connection->constraintAxes[i]);
        constraint.normalTowardInside[0] =  axis;
        constraint.normalTowardInside[1] = -axis;
        constraint.depth = dot(axis, error);
        constraint.globalIndex = globalNumConstraintVectors++;
        constraint.normalProjectionOfRelVelocityOn0 = dot(constraint.normalTowardInside[1], relVelocityOn0);
    }

    return true;
}



void CFSImpl::putContactPoints()
{
    std::cout << "Contact Points\n";
    for(size_t i=0; i < constrainedLinkPairs.size(); ++i){
        LinkPair* linkPair = constrainedLinkPairs[i];

        if(!linkPair->connection){

            std::cout << " " << linkPair->link[0]->name << " of " << linkPair->bodyData[0]->body->modelName();
            std::cout << "<-->";
            std::cout << " " << linkPair->link[1]->name << " of " << linkPair->bodyData[1]->body->modelName();

            ConstraintPointArray& constraintPoints = linkPair->constraintPoints;
            std::cout << "\n";
            for(size_t j=0; j < constraintPoints.size(); ++j){
                ConstraintPoint& contact = constraintPoints[j];
                std::cout << " index " << contact.globalIndex;
                std::cout << " point: " << contact.point;
                std::cout << " normal: " << contact.normalTowardInside[1];
                std::cout << " rel velocity: " << contact.relVelocityOn0;
                std::cout << " tangent: " << contact.frictionVector[0];
                std::cout << "\n";
            }
        }
    }
    std::cout << std::endl;
}


void CFSImpl::solveImpactConstraints()
{
    if(CFS_DEBUG)
        std::cout << "Impacts !" << std::endl;
}


void CFSImpl::initMatrices()
{
    const int n = globalNumConstraintVectors;
    const int m = globalNumFrictionVectors;

    const int dimLCP = usePivotingLCP ? (n + m + m) : (n + m);

    Mlcp.resize(dimLCP, dimLCP, false);
    b.resize(dimLCP, false);
    solution.resize(dimLCP, false);

    range block1(0, n);
    range block2(n, n + m);

    if(usePivotingLCP){

        range block3(n + m, n + m + m);

        project(Mlcp, block1, block3) = dzeromatrix(n, m);
        project(Mlcp, block3, block1) = dzeromatrix(m, n); // clear mu block

        didentity identity(m);
        project(Mlcp, block3, block2) = -identity;

        project(Mlcp, block3, block3) = dzeromatrix(m, m);
        project(Mlcp, block2, block3) =  identity;

        project(b, block3) = dzerovector(m);

    } else {
        frictionIndexToContactIndex.resize(m);
        contactIndexToMu.resize(globalNumContactNormalVectors, false);
        mcpHi.resize(globalNumContactNormalVectors, false);
    }

    an0.resize(n, false);
    at0.resize(m, false);

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
            int linkIndex = linkPair->link[j]->index;
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

            if(bodyData.forwardDynamicsMM){

                bodyData.rootLinkPosRef = &(bodyData.body->rootLink()->p);
                Vector3 zeroForce(0.0);
                bodyData.forwardDynamicsMM->initializeAccelSolver();
                bodyData.forwardDynamicsMM->solveUnknownAccels(zeroForce, zeroForce);
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
                    constraint.defaultAccel[k] = 0.0;
                } else {
                    Link* link = linkPair.link[k];
                    LinkData* linkData = linkPair.linkData[k];
                    constraint.defaultAccel[k] =
                        linkData->dvo - cross(constraint.point, linkData->dw) +
                        cross(link->w, Vector3(link->vo + cross(link->w, constraint.point)));
                }
            }

            Vector3 relDefaultAccel(constraint.defaultAccel[1] - constraint.defaultAccel[0]);
            an0[constraint.globalIndex] = dot(constraint.normalTowardInside[1], relDefaultAccel);

            for(int k=0; k < constraint.numFrictionVectors; ++k){
                at0[constraint.globalFrictionIndex + k] = dot(constraint.frictionVector[k][1], relDefaultAccel);
            }
        }
    }
}


void CFSImpl::setAccelerationMatrix()
{
    const int n = globalNumConstraintVectors;
    const int m = globalNumFrictionVectors;

    range block1(0, n);
    range block2(n, n + m);
    matrix_range<rmdmatrix> Knn(Mlcp, block1, block1);
    matrix_range<rmdmatrix> Ktn(Mlcp, block1, block2);
    matrix_range<rmdmatrix> Knt(Mlcp, block2, block1);
    matrix_range<rmdmatrix> Ktt(Mlcp, block2, block2);

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

                    if(bodyData.forwardDynamicsMM){
                        //! \todo This code does not work correctly when the links are in the same body. Fix it.
                        Vector3 arm(constraint.point - *(bodyData.rootLinkPosRef));
                        Vector3 tau(cross(arm, f));
                        Vector3 tauext = cross(constraint.point, f);
                        bodyData.forwardDynamicsMM->solveUnknownAccels(linkPair.link[k], f, tauext, f, tau);
                        calcAccelsMM(bodyData, constraintIndex);
                    } else {
                        Vector3 tau(cross(constraint.point, f));
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

                        if(bodyData.forwardDynamicsMM){
                            //! \todo This code does not work correctly when the links are in the same body. Fix it.
                            Vector3 arm(constraint.point - *(bodyData.rootLinkPosRef));
                            Vector3 tau(cross(arm, f));
                            Vector3 tauext = cross(constraint.point, f);
                            bodyData.forwardDynamicsMM->solveUnknownAccels(linkPair.link[k], f, tauext, f, tau);
                            calcAccelsMM(bodyData, constraintIndex);
                        } else {
                            Vector3 tau(cross(constraint.point, f));
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
    bodyData.dpf   = 0;
    bodyData.dptau = 0;

    std::vector<LinkData>& linksData = bodyData.linksData;
    const LinkTraverse& traverse = bodyData.body->linkTraverse();
    int n = traverse.numLinks();

    for(int i = n-1; i >= 0; --i){
        Link* link = traverse[i];
        LinkData& data = linksData[i];

        data.pf0   = link->pf;
        data.ptau0 = link->ptau;

        for(Link* child = link->child; child; child = child->sibling){

            LinkData& childData = linksData[child->index];

            data.pf0   += childData.pf0;
            data.ptau0 += childData.ptau0;

            if(child->jointType != Link::FIXED_JOINT ){
                double uu_dd = childData.uu0 / child->dd;
                data.pf0    += uu_dd * child->hhv;
                data.ptau0  += uu_dd * child->hhw;
            }
        }

        if(i > 0){
            if(link->jointType != Link::FIXED_JOINT){
                data.uu0  = link->uu + link->u - (dot(link->sv, data.pf0) + dot(link->sw, data.ptau0));
                data.uu = data.uu0;
            }
        }
    }
}


void CFSImpl::calcABMForceElementsWithTestForce
(BodyData& bodyData, Link* linkToApplyForce, const Vector3& f, const Vector3& tau)
{
    std::vector<LinkData>& linksData = bodyData.linksData;

    Vector3 dpf  (-f);
    Vector3 dptau(-tau);

    Link* link = linkToApplyForce;
    while(link->parent){
        if(link->jointType != Link::FIXED_JOINT){
            LinkData& data = linksData[link->index];
            double duu = -(dot(link->sv, dpf) + dot(link->sw, dptau));
            data.uu += duu;
            double duudd = duu / link->dd;
            dpf   += duudd * link->hhv;
            dptau += duudd * link->hhw;
        }
        link = link->parent;
    }

    bodyData.dpf   += dpf;
    bodyData.dptau += dptau;
}


void CFSImpl::calcAccelsABM(BodyData& bodyData, int constraintIndex)
{
    std::vector<LinkData>& linksData = bodyData.linksData;
    LinkData& rootData = linksData[0];
    Link* rootLink = rootData.link;

    if(rootLink->jointType == Link::FREE_JOINT){

        Vector3 pf  (rootData.pf0   + bodyData.dpf);
        Vector3 ptau(rootData.ptau0 + bodyData.dptau);

        ublas::bounded_matrix<double, 6, 6, ublas::column_major> Ia;
        setMatrix3(rootLink->Ivv, Ia, 0, 0);
        setTransMatrix3(rootLink->Iwv, Ia, 0, 3);
        setMatrix3(rootLink->Iwv, Ia, 3, 0);
        setMatrix3(rootLink->Iww, Ia, 3, 3);

        ublas::bounded_vector<double, 6> p;
        setVector3(pf,   p, 0);
        setVector3(ptau, p, 3);
        p *= -1.0;

        ublas::permutation_matrix<std::size_t> pm(6);
        ublas::lu_factorize (Ia, pm);
        ublas::lu_substitute(Ia, pm, p);

        getVector3(rootData.dvo, p, 0);
        getVector3(rootData.dw,  p, 3);

    } else {
        rootData.dw  = 0.0;
        rootData.dvo = 0.0;
    }

    // reset
    bodyData.dpf   = 0;
    bodyData.dptau = 0;

    int skipCheckNumber = ASSUME_SYMMETRIC_MATRIX ? constraintIndex : (numeric_limits<int>::max() - 1);
    int n = linksData.size();
    for(int linkIndex = 1; linkIndex < n; ++linkIndex){

        LinkData& linkData = linksData[linkIndex];

        if(!SKIP_REDUNDANT_ACCEL_CALC || linkData.numberToCheckAccelCalcSkip <= skipCheckNumber){

            Link* link = linkData.link;
            LinkData& parentData = linksData[linkData.parentIndex];

            if(link->jointType != Link::FIXED_JOINT){
                linkData.ddq = (linkData.uu - (dot(link->hhv, parentData.dvo) + dot(link->hhw, parentData.dw))) / link->dd;
                linkData.dvo = parentData.dvo + link->cv + link->sv * linkData.ddq;
                linkData.dw  = parentData.dw  + link->cw + link->sw * linkData.ddq;
            }else{
                linkData.ddq = 0.0;
                linkData.dvo = parentData.dvo;
                linkData.dw  = parentData.dw;
            }

            // reset
            linkData.uu   = linkData.uu0;
        }
    }
}


void CFSImpl::calcAccelsMM(BodyData& bodyData, int constraintIndex)
{
    std::vector<LinkData>& linksData = bodyData.linksData;

    LinkData& rootData = linksData[0];
    Link* rootLink = rootData.link;
    rootData.dvo = rootLink->dvo;
    rootData.dw  = rootLink->dw;

    int skipCheckNumber = ASSUME_SYMMETRIC_MATRIX ? constraintIndex : (numeric_limits<int>::max() - 1);
    int n = linksData.size();

    for(int linkIndex = 1; linkIndex < n; ++linkIndex){

        LinkData& linkData = linksData[linkIndex];

        if(!SKIP_REDUNDANT_ACCEL_CALC || linkData.numberToCheckAccelCalcSkip <= skipCheckNumber){

            Link* link = linkData.link;
            LinkData& parentData = linksData[linkData.parentIndex];
            if(link->jointType != Link::FIXED_JOINT){
                linkData.dvo = parentData.dvo + link->cv + link->ddq * link->sv;
                linkData.dw  = parentData.dw  + link->cw + link->ddq * link->sw;
            }else{
                linkData.dvo = parentData.dvo;
                linkData.dw = parentData.dw;
            }
        }
    }
}


void CFSImpl::extractRelAccelsOfConstraintPoints
(matrix_range<rmdmatrix>& Kxn, matrix_range<rmdmatrix>& Kxt, int testForceIndex, int constraintIndex)
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
(matrix_range<rmdmatrix>& Kxn, matrix_range<rmdmatrix>& Kxt, LinkPair& linkPair, int testForceIndex, int maxConstraintIndexToExtract)
{
    ConstraintPointArray& constraintPoints = linkPair.constraintPoints;

    for(size_t i=0; i < constraintPoints.size(); ++i){

        ConstraintPoint& constraint = constraintPoints[i];
        int constraintIndex = constraint.globalIndex;

        if(ASSUME_SYMMETRIC_MATRIX && constraintIndex > maxConstraintIndexToExtract){
            break;
        }

        Link* link0 = linkPair.link[0];
        Link* link1 = linkPair.link[1];
        LinkData* linkData0 = linkPair.linkData[0];
        LinkData* linkData1 = linkPair.linkData[1];

        //! \todo Can the follwoing equations be simplified ?
        Vector3 dv0(linkData0->dvo - cross(constraint.point, linkData0->dw) + cross(link0->w, Vector3(link0->vo + cross(link0->w, constraint.point))));
        Vector3 dv1(linkData1->dvo - cross(constraint.point, linkData1->dw) + cross(link1->w, Vector3(link1->vo + cross(link1->w, constraint.point))));

        Vector3 relAccel(dv1 - dv0);

        Kxn(constraintIndex, testForceIndex) =
            dot(constraint.normalTowardInside[1], relAccel) - an0(constraintIndex);

        for(int j=0; j < constraint.numFrictionVectors; ++j){
            const int index = constraint.globalFrictionIndex + j;
            Kxt(index, testForceIndex) = dot(constraint.frictionVector[j][1], relAccel) - at0(index);
        }
    }
}


void CFSImpl::extractRelAccelsFromLinkPairCase2
(matrix_range<rmdmatrix>& Kxn, matrix_range<rmdmatrix>& Kxt, LinkPair& linkPair, int iTestForce, int iDefault, int testForceIndex, int maxConstraintIndexToExtract)
{
    ConstraintPointArray& constraintPoints = linkPair.constraintPoints;

    for(size_t i=0; i < constraintPoints.size(); ++i){

        ConstraintPoint& constraint = constraintPoints[i];
        int constraintIndex = constraint.globalIndex;

        if(ASSUME_SYMMETRIC_MATRIX && constraintIndex > maxConstraintIndexToExtract){
            break;
        }

        Link* link = linkPair.link[iTestForce];
        LinkData* linkData = linkPair.linkData[iTestForce];

        Vector3 dv(linkData->dvo - cross(constraint.point, linkData->dw) + cross(link->w, Vector3(link->vo + cross(link->w, constraint.point))));

        if(CFS_DEBUG_VERBOSE_2)
            std::cout << "dv " << constraintIndex << " = " << dv << "\n";

        Vector3 relAccel(constraint.defaultAccel[iDefault] - dv);

        Kxn(constraintIndex, testForceIndex) =
            dot(constraint.normalTowardInside[iDefault], relAccel) - an0(constraintIndex);

        for(int j=0; j < constraint.numFrictionVectors; ++j){
            const int index = constraint.globalFrictionIndex + j;
            Kxt(index, testForceIndex) =
                dot(constraint.frictionVector[j][iDefault], relAccel) - at0(index);
        }

    }
}


void CFSImpl::extractRelAccelsFromLinkPairCase3
(matrix_range<rmdmatrix>& Kxn, matrix_range<rmdmatrix>& Kxt, LinkPair& linkPair, int testForceIndex, int maxConstraintIndexToExtract)
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
(matrix_range<rmdmatrix>& Knn, matrix_range<rmdmatrix>& Ktn, matrix_range<rmdmatrix>& Knt, matrix_range<rmdmatrix>& Ktt)
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
    for(size_t i = 0; i < Mlcp.size1(); ++i){
        if(Mlcp(i, i) < 1.0e-4){
            for(size_t j=0; j < Mlcp.size1(); ++j){
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

            if(linkPair.connection){
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
                if(ALLOW_SUBTLE_PENETRATION_FOR_STABILITY){
                    double extraNegativeVel;
                    double newDepth = ALLOWED_PENETRATION_DEPTH - constraint.depth;
                    extraNegativeVel = NEGATIVE_VELOCITY_RATIO_FOR_PENETRTION * newDepth;
                    b(globalIndex) = an0(globalIndex) + (constraint.normalProjectionOfRelVelocityOn0 + extraNegativeVel) * dtinv;
                } else {
                    b(globalIndex) = an0(globalIndex) + constraint.normalProjectionOfRelVelocityOn0 * dtinv;
                }

                contactIndexToMu[globalIndex] = constraint.mu;

                int globalFrictionIndex = constraint.globalFrictionIndex;
                for(int k=0; k < constraint.numFrictionVectors; ++k){

                    // constraints for tangent acceleration
                    double tangentProjectionOfRelVelocity = dot(constraint.frictionVector[k][1], constraint.relVelocityOn0);

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
    const bool DO_PENALTY_METHOD_TEST = false;

    //const double kp = 1.0e3;
    //const double kd = 1.0e2;

    //const double kp = 1.0e4;
    //const double kd = 1.0e3;

    const double kp = 1.0e5;
    const double kd = 1.0e3;
    
    Vector3 f_total(0.0);
    Vector3 tau_total(0.0);

    ConstraintPointArray& constraintPoints = linkPair->constraintPoints;
    int numConstraintPoints = constraintPoints.size();
    Link* link = linkPair->link[ipair];

    for(int i=0; i < numConstraintPoints; ++i){

        ConstraintPoint& constraint = constraintPoints[i];
        int globalIndex = constraint.globalIndex;

        if(!DO_PENALTY_METHOD_TEST){
            Vector3 f(solution(globalIndex) * constraint.normalTowardInside[ipair]);

            for(int j=0; j < constraint.numFrictionVectors; ++j){
                f += solution(globalNumConstraintVectors + constraint.globalFrictionIndex + j) * constraint.frictionVector[j][ipair];
            }
            
            f_total   += f;
            tau_total += cross(constraint.point, f);
            
            if(isConstraintForceOutputMode){
                Link::ConstraintForceArray& constraintForces = link->constraintForces;
                constraintForces.resize(constraintForces.size() + 1);
                Link::ConstraintForce& cforce = constraintForces.back();
                cforce.point = constraint.point;
                cforce.force = f;
            }
        } else {
            double vn((ipair == 0) ? constraint.normalProjectionOfRelVelocityOn0 : -constraint.normalProjectionOfRelVelocityOn0);
            double fn(constraint.depth * kp - vn * kd);
            Vector3 f(fn * constraint.normalTowardInside[ipair]);

            if(fn > 0.0){
                //for(int j=0; j < constraint.numFrictionVectors; ++j){
                Vector3 v(constraint.relVelocityOn0 - constraint.normalProjectionOfRelVelocityOn0 * constraint.normalTowardInside[1]);
                //cout << constraint.relVelocityOn0 << " ";
                if(ipair == 0){
                    v = -v;
                }
                double s = norm2(v);
                if(s < 1.0e-3){
                    f += - (v / 1.0e-3) * fn * constraint.mu;
                } else {
                    f += - (v / s) * fn * constraint.mu;
                }
                //}
            }
            
            f_total += f;
            tau_total += cross(constraint.point, f);
        }
    }
    
    link->fext   += f_total;
    link->tauext += tau_total;


    if(CFS_DEBUG){
        std::cout << "Constraint force to " << link->name << ": f = " << f_total << ", tau = " << tau_total << std::endl;
    }
}



void CFSImpl::solveMCPByProjectedGaussSeidel(const rmdmatrix& M, const dvector& b, dvector& x)
{
    static const int loopBlockSize = DEFAULT_NUM_GAUSS_SEIDEL_ITERATION_BLOCK;

    if(numGaussSeidelInitialIteration > 0){
        solveMCPByProjectedGaussSeidelInitial(M, b, x, numGaussSeidelInitialIteration);
    }

    int numBlockLoops = maxNumGaussSeidelIteration / loopBlockSize;
    if(numBlockLoops==0) numBlockLoops = 1;

    if(CFS_MCP_DEBUG) cout << "Iteration ";

    double error = 0.0;
    int i=0;
    while(i < numBlockLoops){
        i++;
        solveMCPByProjectedGaussSeidelMain(M, b, x, loopBlockSize - 1);

        error = solveMCPByProjectedGaussSeidelErrorCheck(M, b, x);
        if(error < gaussSeidelMaxRelError){
            if(CFS_MCP_DEBUG_SHOW_ITERATION_STOP) cout << "stopped at " << i * loopBlockSize << endl;
            break;
        }
    }

    if(CFS_MCP_DEBUG){
        int n = loopBlockSize * i;
        numGaussSeidelTotalLoops += n;
        numGaussSeidelTotalCalls++;
        cout << n;
        cout << ", avarage = " << (numGaussSeidelTotalLoops / numGaussSeidelTotalCalls);
        cout << ", error = " << error;
        cout << endl;
    }
}


void CFSImpl::solveMCPByProjectedGaussSeidelInitial
(const rmdmatrix& M, const dvector& b, dvector& x, const int numIteration)
{
    const int size = globalNumConstraintVectors + globalNumFrictionVectors;

    const double rstep = 1.0 / (numIteration * size);
    double r = 0.0;

    for(int i=0; i < numIteration; ++i){

        for(int j=0; j < globalNumContactNormalVectors; ++j){

            double sum = -M(j, j) * x(j);
            for(int k=0; k < size; ++k){
                sum += M(j, k) * x(k);
            }
            const double xx = (-b(j) - sum) / M(j, j);
            if(xx < 0.0){
                x(j) = 0.0;
            } else {
                x(j) = r * xx;
            }
            r += rstep;
            mcpHi[j] = contactIndexToMu[j] * x(j);
        }

        for(int j=globalNumContactNormalVectors; j < globalNumConstraintVectors; ++j){

            double sum = -M(j, j) * x(j);
            for(int k=0; k < size; ++k){
                sum += M(j, k) * x(k);
            }
            x(j) = r * (-b(j) - sum) / M(j, j);
            r += rstep;
        }

        if(ENABLE_TRUE_FRICTION_CONE){

            int contactIndex = 0;
            for(int j=globalNumConstraintVectors; j < size; ++j, ++contactIndex){

                double sum = -M(j, j) * x(j);
                for(int k=0; k < size; ++k){
                    sum += M(j, k) * x(k);
                }
                const double fx0 = (-b(j) - sum) / M(j, j);
                double& fx = x(j);

                ++j;

                sum = -M(j, j) * x(j);
                for(int k=0; k < size; ++k){
                    sum += M(j, k) * x(k);
                }
                const double fy0 = (-b(j) - sum) / M(j, j);
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

                double sum = -M(j, j) * x(j);
                for(int k=0; k < size; ++k){
                    sum += M(j, k) * x(k);
                }
                const double xx = (-b(j) - sum) / M(j, j);

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


void CFSImpl::solveMCPByProjectedGaussSeidelMain
(const rmdmatrix& M, const dvector& b, dvector& x, const int numIteration)
{
    const int size = globalNumConstraintVectors + globalNumFrictionVectors;

    for(int i=0; i < numIteration; ++i){

        for(int j=0; j < globalNumContactNormalVectors; ++j){

            double sum = -M(j, j) * x(j);
            for(int k=0; k < size; ++k){
                sum += M(j, k) * x(k);
            }
            const double xx = (-b(j) - sum) / M(j, j);
            if(xx < 0.0){
                x(j) = 0.0;
            } else {
                x(j) = xx;
            }
            mcpHi[j] = contactIndexToMu[j] * x(j);
        }

        for(int j=globalNumContactNormalVectors; j < globalNumConstraintVectors; ++j){

            double sum = -M(j, j) * x(j);
            for(int k=0; k < size; ++k){
                sum += M(j, k) * x(k);
            }
            x(j) = (-b(j) - sum) / M(j, j);
        }


        if(ENABLE_TRUE_FRICTION_CONE){

            int contactIndex = 0;
            for(int j=globalNumConstraintVectors; j < size; ++j, ++contactIndex){

                double sum = -M(j, j) * x(j);
                for(int k=0; k < size; ++k){
                    sum += M(j, k) * x(k);
                }
                const double fx0 = (-b(j) - sum) / M(j, j);
                double& fx = x(j);

                ++j;

                sum = -M(j, j) * x(j);
                for(int k=0; k < size; ++k){
                    sum += M(j, k) * x(k);
                }
                const double fy0 = (-b(j) - sum) / M(j, j);
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

                double sum = -M(j, j) * x(j);
                for(int k=0; k < size; ++k){
                    sum += M(j, k) * x(k);
                }
                const double xx = (-b(j) - sum) / M(j, j);

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
}


double CFSImpl::solveMCPByProjectedGaussSeidelErrorCheck
(const rmdmatrix& M, const dvector& b, dvector& x)
{
    const int size = globalNumConstraintVectors + globalNumFrictionVectors;

    double error = 0.0;

    for(int j=0; j < globalNumConstraintVectors; ++j){

        double sum = -M(j, j) * x(j);
        for(int k=0; k < size; ++k){
            sum += M(j, k) * x(k);
        }
        double xx = (-b(j) - sum) / M(j, j);

        if(j < globalNumContactNormalVectors){
            if(xx < 0.0){
                xx = 0.0;
            }
            mcpHi[j] = contactIndexToMu[j] * xx;
        }
        double d = fabs(xx - x(j));
        if(xx > numeric_limits<double>::epsilon()){
            d /= xx;
        }
        if(d > error){
            error = d;
        }
        x(j) = xx;
    }

    if(ENABLE_TRUE_FRICTION_CONE){

        int contactIndex = 0;
        for(int j=globalNumConstraintVectors; j < size; ++j, ++contactIndex){

            double sum = -M(j, j) * x(j);
            for(int k=0; k < size; ++k){
                sum += M(j, k) * x(k);
            }
            double fx0 = (-b(j) - sum) / M(j, j);
            double& fx = x(j);

            ++j;

            sum = -M(j, j) * x(j);
            for(int k=0; k < size; ++k){
                sum += M(j, k) * x(k);
            }
            double fy0 = (-b(j) - sum) / M(j, j);
            double& fy = x(j);

            const double fmax = mcpHi[contactIndex];
            const double fmax2 = fmax * fmax;
            const double fmag2 = fx0 * fx0 + fy0 * fy0;

            if(fmag2 > fmax2){
                const double s = fmax / sqrt(fmag2);
                fx0 *= s;
                fy0 *= s;
            }
			
            double d = fabs(fx0 - fx);
            const double afx0 = fabs(fx0);
            if(afx0 > numeric_limits<double>::epsilon()){
                d /= afx0;
            }
            if(d > error){
                error = d;
            }
            d = fabs(fy0 - fy);
            const double afy0 = fabs(fy0);
            if(afy0 > numeric_limits<double>::epsilon()){
                d /= afy0;
            }
            if(d > error){
                error = d;
            }
            fx = fx0;
            fy = fy0;
        }

    } else {

        int frictionIndex = 0;
        for(int j=globalNumConstraintVectors; j < size; ++j, ++frictionIndex){

            double sum = -M(j, j) * x(j);
            for(int k=0; k < size; ++k){
                sum += M(j, k) * x(k);
            }

            double xx = (-b(j) - sum) / M(j, j);

            const int contactIndex = frictionIndexToContactIndex[frictionIndex];
            const double fmax = mcpHi[contactIndex];
            const double fmin = (STATIC_FRICTION_BY_TWO_CONSTRAINTS ? -fmax : 0.0);

            if(xx < fmin){
                xx = fmin;
            } else if(xx > fmax){
                xx = fmax;
            }
            double d = fabs(xx - x(j));
            if(xx > numeric_limits<double>::epsilon()){
                d /= xx;
            }
            if(d > error){
                error = d;
            }
            x(j) = xx;
        }
    }

    return error;
}



void CFSImpl::checkLCPResult(rmdmatrix& M, dvector& b, dvector& x)
{
    std::cout << "check LCP result\n";
    std::cout << "-------------------------------\n";

    dvector z = prod(M, x) + b;

    int n = x.size();
    for(int i=0; i < n; ++i){
        std::cout << "(" << x(i) << ", " << z(i) << ")";

        if(x(i) < 0.0 || z(i) < 0.0 || x(i) * z(i) != 0.0){
            std::cout << " - X";
        }
        std::cout << "\n";

        if(i == globalNumConstraintVectors){
            std::cout << "-------------------------------\n";
        } else if(i == globalNumConstraintVectors + globalNumFrictionVectors){
            std::cout << "-------------------------------\n";
        }
    }

    std::cout << "-------------------------------\n";


    std::cout << std::endl;
}


void CFSImpl::checkMCPResult(rmdmatrix& M, dvector& b, dvector& x)
{
    std::cout << "check MCP result\n";
    std::cout << "-------------------------------\n";

    dvector z = prod(M, x) + b;

    for(int i=0; i < globalNumConstraintVectors; ++i){
        std::cout << "(" << x(i) << ", " << z(i) << ")";

        if(x(i) < 0.0 || z(i) < -1.0e-6){
            std::cout << " - X";
        } else if(x(i) > 0.0 && fabs(z(i)) > 1.0e-6){
            std::cout << " - X";
        } else if(z(i) > 1.0e-6 && fabs(x(i)) > 1.0e-6){
            std::cout << " - X";
        }


        std::cout << "\n";
    }

    std::cout << "-------------------------------\n";

    int j = 0;
    for(int i=globalNumConstraintVectors; i < globalNumConstraintVectors + globalNumFrictionVectors; ++i, ++j){
        std::cout << "(" << x(i) << ", " << z(i) << ")";

        int contactIndex = frictionIndexToContactIndex[j];
        double hi = contactIndexToMu[contactIndex] * x(contactIndex);

        std::cout << " hi = " << hi;

        if(x(i) < 0.0 || x(i) > hi){
            std::cout << " - X";
        } else if(x(i) == hi && z(i) > -1.0e-6){
            std::cout << " - X";
        } else if(x(i) < hi && x(i) > 0.0 && fabs(z(i)) > 1.0e-6){
            std::cout << " - X";
        }
        std::cout << "\n";
    }

    std::cout << "-------------------------------\n";

    std::cout << std::endl;
}


#ifdef USE_PIVOTING_LCP
bool CFSImpl::callPathLCPSolver(rmdmatrix& Mlcp, dvector& b, dvector& solution)
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


bool ConstraintForceSolver::addCollisionCheckLinkPair
(int bodyIndex1, Link* link1, int bodyIndex2, Link* link2, double muStatic, double muDynamic, double culling_thresh, double epsilon)
{
    return impl->addCollisionCheckLinkPair(bodyIndex1, link1, bodyIndex2, link2, muStatic, muDynamic, culling_thresh, epsilon);
}


void ConstraintForceSolver::clearCollisionCheckLinkPairs()
{
    impl->world.clearCollisionPairs();
    impl->collisionCheckLinkPairs.clear();
}


void ConstraintForceSolver::setGaussSeidelParameters(int maxNumIteration, int numInitialIteration, double maxRelError)
{
    impl->maxNumGaussSeidelIteration = maxNumIteration;
    impl->numGaussSeidelInitialIteration = numInitialIteration;
    impl->gaussSeidelMaxRelError = maxRelError;
}


void ConstraintForceSolver::enableConstraintForceOutput(bool on)
{
    impl->isConstraintForceOutputMode = on;
}


void ConstraintForceSolver::useBuiltinCollisionDetector(bool on)
{
    impl->useBuiltinCollisionDetector = on;
}


void ConstraintForceSolver::initialize(void)
{
    impl->initialize();
}


void ConstraintForceSolver::solve(CollisionSequence& corbaCollisionSequence)
{
    impl->solve(corbaCollisionSequence);
}


void ConstraintForceSolver::clearExternalForces()
{
    impl->clearExternalForces();
}
