#if defined _MSC_VER && !defined(_USE_MATH_DEFINES)
#define _USE_MATH_DEFINES
#endif

#include "PoseSeqInterpolator.h"
#include "BodyKeyPose.h"
#include "PronunSymbol.h"
#include <cnoid/Link>
#include <cnoid/JointPath>
#include <cnoid/LeggedBodyHelper>
#include <cnoid/ValueTree>
#include <cnoid/EigenUtil>
#include <cnoid/Array2D>
#include <cnoid/ConnectionSet>
#include <cnoid/stdx/optional>
#include <list>
#include <vector>
#include <algorithm>
#include <unordered_map>

using namespace std;
using namespace cnoid;

namespace {

const double epsilon = 1.0e-6;

enum SegmentType { UNDETERMINED, INVALID, CUBIC_SPLINE, CUBIC_CONNECTION, MIN_JERK_CONNECTION, LINEAR, ZERO_LENGTH };

// coefficients for interpolation
struct Coeff
{
    double y;    // sample value
    double yp;   // derivative value
    double a;
    double a_end;
    double b;
    double c;
};

struct LinkSample
{
    SegmentType segmentType;
    PoseSeq::iterator poseIter;
    double x;
    Coeff c[6]; // x, y, z, roll, pitch, yaw
    bool isBaseLink;
    bool isEndPoint;
    bool isDirty;
    bool isTouching;
    bool isSlave;
    bool isAux;

    typedef std::list<LinkSample> Seq;
};

struct LinkAuxSample
{
    SegmentType segmentType;
    PoseSeq::iterator poseIter;
    double x;
    Coeff c[1];
    bool isEndPoint;
    bool isDirty;
    bool isTouching;

    typedef std::list<LinkAuxSample> Seq;
};

    
struct LinkInfo
{
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    
    LinkInfo(Body* body, int linkIndex)
    {
        iter = samples.end();
        auxIter = auxSamples.end();
        Link* link = body->link(linkIndex);
        if(link){
            jointId = link->jointId();
        } else {
            jointId = -1;
        }
        jointSpaceBlendingRatio = 0.0;
        isFootLink = false;
    }
    
    int jointId;
    bool isFootLink;
    LinkSample::Seq samples;
    LinkSample::Seq::iterator iter;
    LinkAuxSample::Seq auxSamples;
    LinkAuxSample::Seq::iterator auxIter;

    // Link local coordinate frame used in the interpolation
    stdx::optional<Isometry3> T_offset;
        
    // interpolation state
    Isometry3 T;
    bool isValid;
    double jointSpaceBlendingRatio; // 1.0 = joint space, 0.0 = Cartesian space
};

struct JointSample
{
    JointSample(PoseSeq::iterator it, int jointId, bool useLinearInterpolation)
    {
        poseIter = it;
        if(useLinearInterpolation){
            segmentType = LINEAR;
            isDirty = false;
        } else {
            segmentType = UNDETERMINED;
            isDirty = true;
        }
        x = it->time();
        if(auto pose = it->get<BodyKeyPose>()){
            c[0].y = pose->jointDisplacement(jointId);
            isEndPoint = pose->isJointStationaryPoint(jointId);
        } else {
            c[0].y = 0.0;
            isEndPoint = false;
        }
        c[0].yp = 0.0;
    }

    SegmentType segmentType;
    PoseSeq::iterator poseIter;
    double x;
    Coeff c[1];
    bool isEndPoint;
    bool isDirty;

    typedef std::list<JointSample> Seq;
};

struct JointInfo
{
    JointInfo()
    {
        useLinearInterpolation = false;
        clear();
    }

    void clear()
    {
        samples.clear();
        iter = samples.begin();
        prevSegmentDirectionSign = 0.0;
        prev_q = 0.0;
    }
    
    JointSample::Seq samples;
    JointSample::Seq::iterator iter;
    bool useLinearInterpolation;

    double prevSegmentDirectionSign;
    double prev_q;

    // interpolated state
    stdx::optional<double> q;
};

struct ZmpSample
{
    ZmpSample(PoseSeq::iterator it)
    {
        poseIter = it;
        segmentType = UNDETERMINED;
        x = it->time();

        if(auto pose = it->get<BodyKeyPose>()){
            const Vector3& zmp = pose->zmp();
            for(int i=0; i < 3; ++i){
                Coeff& ci = c[i];
                ci.y = zmp[i];
                ci.yp = 0.0;
            }
            isEndPoint = pose->isZmpStationaryPoint();
        } else {
            for(int i=0; i < 3; ++i){
                Coeff& ci = c[i];
                ci.y = 0.0;
                ci.yp = 0.0;
            }
            isEndPoint = false;
        }

        isDirty = true;
    }

    ZmpSample(double time, const Vector3& p)
    {
        segmentType = UNDETERMINED;
        x = time;
        for(int i=0; i < 3; ++i){
            Coeff& ci = c[i];
            ci.y = p[i];
            ci.yp = 0.0;
        }
        isEndPoint = true;
        isDirty = true;
    }
            
    SegmentType segmentType;
    PoseSeq::iterator poseIter;
    double x;
    Coeff c[3];
    bool isEndPoint;
    bool isDirty;

    typedef std::list<ZmpSample> Seq;
};

}

namespace std {

template<class T> struct hash<std::pair<T, T>>
{
    void hash_combine(std::size_t& seed, const T& v) const {
        std::hash<T> hasher;
        seed ^= hasher(v) + 0x9e3779b9 + (seed<<6) + (seed>>2);
    }
    std::size_t operator()(const std::pair<T, T>& v) const {
        std::size_t seed = 0;
        hash_combine(seed, v.first);
        hash_combine(seed, v.second);
        return seed;
    }
};

}

namespace cnoid {

class PoseSeqInterpolator::Impl
{
public:

    Impl(PoseSeqInterpolator* self);

    PoseSeqInterpolator* self;
    BodyPtr body;
    PoseSeqPtr poseSeq;

    bool needUpdate;

    ConnectionSet poseSeqConnections;

    vector<JointInfo> jointInfos;

    typedef unordered_map<int, LinkInfo, std::hash<int>, std::equal_to<int>,
                          Eigen::aligned_allocator<pair<const int, LinkInfo>>> LinkInfoMap;
    LinkInfoMap ikLinkInfos;
    vector<int> footLinkIndices;
    LeggedBodyHelperPtr legged;
    vector<LinkInfo*> footLinkInfos;

    typedef pair<Link*, Link*> LinkPair;
    typedef unordered_map<LinkPair, shared_ptr<JointPath>> JointPathMap;
    JointPathMap ikJointPathMap;

    bool isAutoZmpAdjustmentMode;
    double minZmpTransitionTime;
    double zmpCenteringTimeThresh;
    double zmpTimeMarginBeforeLifting;
    double zmpMaxDistanceFromCenterSqr;

    int stepTrajectoryAdjustmentMode;
    
    double stealthyHeightRatioThresh;
    double flatLiftingHeight;
    double flatLandingHeight;
    double impactReductionHeight;
    double impactReductionTime;
    double impactReductionVelocity;

    double toeContactTime;
    double toeContactAngle;
            
    ZmpSample::Seq zmpSamples;
    ZmpSample::Seq::iterator zmpIter;

    struct ZmpSampleIterPair {
        ZmpSampleIterPair(PoseSeq::iterator poseIter, ZmpSample::Seq::iterator sampleIter)
            : poseIter(poseIter), sampleIter(sampleIter) { }
        PoseSeq::iterator poseIter;
        ZmpSample::Seq::iterator sampleIter;
    };
    
    vector<ZmpSampleIterPair> orgZmpSampleIterPairs;

    enum SupportPhase {
        LEFT = LeggedBodyHelper::Left,
        RIGHT = LeggedBodyHelper::Right,
        BOTH = 2,
        FLOATING,
        NONE
    };

    enum LipShapeId {
        LS_A, LS_I, LS_U, LS_E, LS_O, LS_N,
        LS_a, LS_i, LS_u, LS_e, LS_o,
        NUM_LIP_SHAPES
    };

    enum mixType { UPPER, LOWER, ADDITION };

    struct LipSyncJoint {
        int jointId;
        int mixType;
        int orgIndex;
    };
    struct LipSyncSample {
        double time;
        int shapeId;
    };

    bool isLipSyncMixEnabled;
    vector<LipSyncJoint> lipSyncJoints;
    vector<int> lipSyncLinkIndices;
    typedef Array2D<double> ShapeArray;
    ShapeArray lipSyncShapes;
    vector<LipSyncSample> lipSyncSeq;
    vector<LipSyncSample>::iterator lipSyncIter;
    double lipSyncMaxTransitionTime;

    double currentTime;
    double timeScaleRatio;
    LinkInfoMap::iterator currentBaseLinkInfoIter;
    vector<bool> validIkLinkFlag;
    Vector3 waistTranslation;

    Signal<void()> sigUpdated;

    void setBody(Body* body0);
    void setLinearInterpolationJoint(int jointId);
    void clearLipSyncShapes();
    void setLipSyncShapes(const Mapping& lipSyncShapeNode);
    void setPoseSeq(PoseSeq* seq);
    void invalidateCurrentInterpolation();
    bool interpolate(double time, int waistLinkIndex, const Vector3& waistTranslation);
    bool mixLipSyncShape();
    void calcIkJointPositions();
    void calcIkJointPositionsSub(Link* link, Link* baseLink, LinkInfo* baseLinkInfo, bool doUpward, Link* prevLink);
    void appendPronun(PoseSeq::iterator poseIter);
    void appendLinkSamples(PoseSeq::iterator poseIter, BodyKeyPose* pose);

    inline bool checkZmp(const Vector3& zmp, const Vector3& centerZmp);
        
    void adjustZmpAndFootKeyPosesForLifting
    (LinkSample::Seq& swingSamples, LinkSample::Seq::iterator pSwing0, LinkSample::Seq::iterator pSwing1,
     LinkAuxSample::Seq& swingAuxSamples, LinkAuxSample::Seq::iterator pSwingAux0, LinkAuxSample::Seq::iterator pSwingAux1,
     ZmpSample::Seq::iterator pZmp0,  const Vector3& zmpOnSupport, bool zmpCenteringDone);

    void adjustZmpAndFootKeyPosesForLanding
    (LinkSample::Seq& swingSamples, LinkSample::Seq::iterator pSwing0, LinkSample::Seq::iterator pSwing1,
     LinkAuxSample::Seq& swingAuxSamples, LinkAuxSample::Seq::iterator pSwingAux0, LinkAuxSample::Seq::iterator pSwingAux1,
     ZmpSample::Seq::iterator pZmp1,  const Vector3& zmp1, const Vector3& zmpOnSupport);

    bool adjustZmpForBothPhase(
        ZmpSample::Seq::iterator& pZmp0, double time0, double time1,
        LinkInfo* leftInfo, LinkSample::Seq::iterator pRight0,
        LinkInfo* rightInfo, LinkSample::Seq::iterator pLeft0,
        SupportPhase prevPhase, SupportPhase nextPhase);

    Vector3 getCenterZmp(LinkInfo* linkInfo, const LinkSample::Seq::iterator& xyzrpy, int which);

    void adjustZmpAndFootKeyPoses();
    void insertZmpSamplesAtTransitionStartPoints();
    void insertAuxKeyPosesForStealthySteps();
    void insertAuxKeyPosesForToeSteps();
    bool update();
    LinkInfo* getIkLinkInfo(int linkIndex);
    void onPoseInserted(PoseSeq::iterator it);
    void onPoseAboutToBeRemoved(PoseSeq::iterator it, bool isMoving);
    void onPoseModified(PoseSeq::iterator it);
};
}


namespace {

template <int dim, class SampleType>
typename SampleType::Seq::iterator updateZeroLengthSegment(typename SampleType::Seq::iterator s)
{
    const typename SampleType::Seq::iterator s0 = s;
    const typename SampleType::Seq::iterator s1 = ++s;
    s0->segmentType = ZERO_LENGTH;
    s0->isDirty = false;
    s0->isEndPoint = true;
    s1->isEndPoint = true;
    return s;
}


template <int dim, class SampleType>
typename SampleType::Seq::iterator updateCubicConnectionSegment(typename SampleType::Seq::iterator s)
{
    const typename SampleType::Seq::iterator s0 = s;
    const typename SampleType::Seq::iterator s1 = ++s;

    if(fabs(s1->x - s0->x) < epsilon){
        return updateZeroLengthSegment<dim, SampleType>(s0);
    }
        
    s0->segmentType = CUBIC_CONNECTION;
    s0->isDirty = false;
    s0->isEndPoint = true;
    s1->isEndPoint = true;
        
    const double h = (s1->x - s0->x);
    const double h2 = h * h;
    const double h3 = h2 * h;

    for(int i=0; i < dim; ++i){
        Coeff& c0 = s0->c[i];
        Coeff& c1 = s1->c[i];
        c0.a = c0.yp;
        c0.b = 3.0 * (c1.y - c0.y) / h2 - (2.0 * c0.yp + c1.yp) / h;
        c0.c = (c0.yp + c1.yp) / h2 + 2.0 * (c0.y - c1.y) / h3;
    }
        
    return s;
}


template <int dim, class SampleType>
typename SampleType::Seq::iterator updateMinJerkConnectionSegment(typename SampleType::Seq::iterator s)
{
    const typename SampleType::Seq::iterator s0 = s;
    const typename SampleType::Seq::iterator s1 = ++s;
    s0->segmentType = MIN_JERK_CONNECTION;
    s0->isDirty = false;
    s0->isEndPoint = true;
    s1->isEndPoint = true;
    return s;
}

    
template <int dim, class SampleType>
typename SampleType::Seq::iterator
updateCubicSplineSegment(typename SampleType::Seq::iterator s, typename SampleType::Seq::iterator end)
{
    typename SampleType::Seq::iterator s0 = s;
    typename SampleType::Seq::iterator s1 = ++s;

    if(fabs(s1->x - s0->x) < epsilon){
        return updateZeroLengthSegment<dim, SampleType>(s0);
    } else {
        typename SampleType::Seq::iterator s2 = s1;
        ++s2;
        if(fabs(s2->x - s1->x) < epsilon){
            return updateCubicConnectionSegment<dim, SampleType>(s0);
        }
    }

    s0->segmentType = CUBIC_SPLINE;
    s0->isEndPoint = true;
    s0->isDirty = false;

    for(int i=0; i < dim; ++i){
        Coeff& c0 = s0->c[i];
        Coeff& c1 = s1->c[i];
        c0.a = -0.5;
        c0.b = (3.0 / (s1->x - s0->x)) * ((c1.y - c0.y) / (s1->x - s0->x) - c0.yp);
    }

    while(true) {
        typename SampleType::Seq::iterator s0 = s; --s0;
        typename SampleType::Seq::iterator s1 = s;
        typename SampleType::Seq::iterator s2 = ++s;

        if(fabs(s2->x - s1->x) < epsilon){
            --s;
            break;
        }
            
        s1->segmentType = CUBIC_SPLINE;
        s1->isDirty = false;
            
        double sig = (s1->x - s0->x) / (s2->x - s0->x);
        for(int i=0; i < dim; ++i){
            Coeff& c0 = s0->c[i];
            Coeff& c1 = s1->c[i];
            Coeff& c2 = s2->c[i];
            double p = sig * c0.a + 2.0;
            c1.a = (sig - 1.0) / p;
            double b = (c2.y - c1.y) / (s2->x - s1->x) - (c1.y - c0.y) / (s1->x - s0->x);
            c1.b = (6.0 * b / (s2->x - s0->x) - sig * c0.b) / p;
        }

        if(s2->isEndPoint || ++s2 == end){
            break;
        }
    }

    typename SampleType::Seq::iterator sf0 = s; --sf0;
    typename SampleType::Seq::iterator sf = s;

    sf->isEndPoint = true;

    double a_save[dim];

    for(int i=0; i < dim; ++i){
        Coeff& cf0 = sf0->c[i];
        Coeff& cf = sf->c[i];
        double qf = 0.5;
        double bf = (3.0 / (sf->x - sf0->x)) * (cf.yp - (cf.y - cf0.y) / (sf->x - sf0->x));
        a_save[i] = cf.a;
        cf.a = (bf - qf * cf0.b) / (qf * cf0.a + 1.0);
    }

    while(s != s0){
        typename SampleType::Seq::iterator s1 = s;
        typename SampleType::Seq::iterator s0 = --s;
        for(int i=0; i < dim; ++i){
            s0->c[i].a = s0->c[i].a * s1->c[i].a + s0->c[i].b;
        }
    }

    for(int i=0; i < dim; ++i){
        sf->c[i].a_end = sf->c[i].a;
        sf->c[i].a = a_save[i];
    }

    return sf;
}


/**
   pre-determined velocity version
*/
template <int dim, class SampleType>
void usePredeterminedVelocities(typename SampleType::Seq& samples)
{
    typename SampleType::Seq::iterator s = samples.begin();

    typename SampleType::Seq::iterator prev = s;

    while(s != samples.end()){

        if(s->segmentType != INVALID){
            typename SampleType::Seq::iterator next = s; ++next;
            while(true){
                if(next == samples.end()){
                    next = s;
                    break;
                }
                if(next->segmentType == INVALID){
                    ++next;
                } else {
                    break;
                }
            }

            double dt0 = s->x - prev->x;
            double dt1 = next->x - s->x;

            for(int i=3; i < dim; ++i){ // for rotation over 180[deg]
                double& y0 = s->c[i].y;
                double& y = next->c[i].y;
                if(fabs(y - y0) > M_PI){
                    double p0 = floor(y0 / (2.0 * M_PI)) * 2.0 * M_PI;
                    double p = floor(y / (2.0 * M_PI)) * 2.0 * M_PI;
                    y = y - p + p0;
                    if(fabs(y - y0) > M_PI){
                        y += 2.0 * M_PI;
                    }
                }
            }

            if(!s->isEndPoint){
                for(int i=0; i < dim; ++i){
                    if(s->c[i].yp == 0.0){
                        double dy0 = (s->c[i].y - prev->c[i].y);
                        double dy1 = (next->c[i].y - s->c[i].y);
                        if(fabs(dy0) < 1.0e-3 || fabs(dy1) < 1.0e-3 || dy0 * dy1 <= 0.0){
                            s->c[i].yp = 0.0;
                        } else {
                            s->c[i].yp = (dy0 / dt0 + dy1 / dt1) / 2.0;
                        }
                    }
                }
                s->isEndPoint = true;
            }
            prev = s;
        }
        ++s;
    }
}
    

template <int dim, class SampleType, bool useJerkMinModel>
void initializeInterpolation(typename SampleType::Seq& samples)
{
    usePredeterminedVelocities<dim, SampleType>(samples);
        
    typename SampleType::Seq::iterator s = samples.begin();

    while(s != samples.end()){

        if(s->segmentType == INVALID){
            ++s;
        } else {

            typename SampleType::Seq::iterator next = s; ++next;
            if(next == samples.end()){
                break;
            }

            if(!next->isEndPoint && (++next != samples.end())){
                s = updateCubicSplineSegment<dim, SampleType>(s, samples.end());
            } else {
                if(useJerkMinModel){
                    s = updateMinJerkConnectionSegment<dim, SampleType>(s);
                } else {
                    s = updateCubicConnectionSegment<dim, SampleType>(s);
                }
            }
        }
    }
}


template <int dim, class SampleType>
bool interpolate(
    typename SampleType::Seq& samples, typename SampleType::Seq::iterator& p, double x, double* out_result)
{
    if(samples.empty()){
        return false;
    }

    if(p == samples.end()){
        --p;
        if(p->x <= x){
            for(int i=0; i < dim; ++i){
                out_result[i] = p->c[i].y;
            }
            return true;
        }
    }

    typename SampleType::Seq::iterator next;
        
    while(true){
        if(x < p->x){
            if(p == samples.begin()){
                for(int i=0; i < dim; ++i){
                    out_result[i] = p->c[i].y;
                }
                return true;
            }
            --p;
            continue;
        }
        while(true){
            next = p; ++next;
            if(next == samples.end()){
                for(int i=0; i < dim; ++i){
                    out_result[i] = p->c[i].y;
                }
                return true;
            }
            if(x < next->x){
                break;
            }
            ++p;
        }
        break;
    }

    const SampleType& s0 = *p;
    const SampleType& s1 = *next;

    if(s0.isDirty){
        return false;
    }

    switch(s0.segmentType){

            
    case UNDETERMINED:
    case INVALID:
        return false;

    case CUBIC_SPLINE:
        for(int i=0; i < dim; ++i){
            const double a_end = (s1.isEndPoint ? s1.c[i].a_end : s1.c[i].a);
            const double h = s1.x - s0.x;
            const double A = (s1.x - x) / h;
            const double B = (x - s0.x) / h;
            out_result[i] =
                A * s0.c[i].y + B * s1.c[i].y + ((A*A*A - A) * s0.c[i].a + (B*B*B - B) * a_end) * (h*h) / 6.0;
        }
        break;

    case CUBIC_CONNECTION:
        for(int i=0; i < dim; ++i){
            const double& a0 = s0.c[i].y;
            const double& a1 = s0.c[i].a;
            const double& a2 = s0.c[i].b;
            const double& a3 = s0.c[i].c;
            const double h = x - s0.x;
            const double h2 = h * h;
            const double h3 = h2 * h;
            out_result[i] = (a0 + a1 * h + a2 * h2 + a3 * h3);
        }
        break;

    case MIN_JERK_CONNECTION:
        for(int i=0; i < dim; ++i){
            double r = (x - s0.x) / (s1.x - s0.x);
            double r2 = r * r;
            double r3 = r2 * r;
            double r4 = r2 * r2;
            double r5 = r4 * r;
            double y0 = s0.c[i].y;
            double y1 = s1.c[i].y;
            out_result[i] = y0 + (y0 - y1) * (15.0 * r4 - 6.0 * r5 - 10.0 * r3);
        }
        break;

    case LINEAR:
        for(int i=0; i < dim; ++i){
            double r = (x - s0.x) / (s1.x - s0.x);
            out_result[i] = (1.0 - r) * s0.c[i].y + r * s1.c[i].y;
        }
        break;

    case ZERO_LENGTH:
        for(int i=0; i < dim; ++i){
            out_result[i] = s1.c[i].y;
        }
        break;
    }
    return true;
}


template <class SampleType>
void insertSampleAtTransitionStartPoint
(typename SampleType::Seq& samples, typename SampleType::Seq::iterator it, const PoseSeq::iterator& poseIter)
{
    if(it != samples.begin()){
        auto prev = it;
        --prev;
        const double time = poseIter->time();
        const double ttime = poseIter->maxTransitionTime();
        if(ttime > 0.0 && time - prev->x > ttime){
            prev->isEndPoint = true;
            auto inserted = samples.insert(it, *prev);
            SampleType& sampleForTransition = *inserted;
            sampleForTransition.x = time - ttime;
            sampleForTransition.isEndPoint = true;
        }
    }
}


template <class SampleType>
void appendSample(typename SampleType::Seq& samples, const SampleType& sample)
{
    insertSampleAtTransitionStartPoint<SampleType>(samples, samples.end(), sample.poseIter);
    samples.push_back(sample);
}

}


PoseSeqInterpolator::PoseSeqInterpolator()
{
    impl = new Impl(this);
}


PoseSeqInterpolator::Impl::Impl(PoseSeqInterpolator* self)
    : self(self)
{
    timeScaleRatio = 1.0;
    isAutoZmpAdjustmentMode = false;
    minZmpTransitionTime = 0.1;
    zmpCenteringTimeThresh = 0.03;
    zmpTimeMarginBeforeLifting = 0.0;
    zmpMaxDistanceFromCenterSqr = 0.015 * 0.015;

    stepTrajectoryAdjustmentMode = NoStepAdjustmentMode;

    stealthyHeightRatioThresh = 2.0;
    flatLiftingHeight = 0.005;
    flatLandingHeight = 0.005;
    impactReductionHeight = 0.012;
    impactReductionTime = 0.3;
    impactReductionVelocity = -2.0 * impactReductionHeight / impactReductionTime;
    
    toeContactTime = 0.1;
    toeContactAngle = radian(10.0);

    isLipSyncMixEnabled = false;
    
    needUpdate = true;
}


void PoseSeqInterpolator::setBody(Body* body)
{
    impl->setBody(body);
}


void PoseSeqInterpolator::Impl::setBody(Body* body0)
{
    body.reset();
    jointInfos.clear();
    ikLinkInfos.clear();
    footLinkIndices.clear();
    ikJointPathMap.clear();
    validIkLinkFlag.clear();
    clearLipSyncShapes();
        
    if(body0){
        body = body0->clone();
        int n = body->numJoints();
        jointInfos.resize(n);
        validIkLinkFlag.resize(body->numLinks(), false);

        legged = getLeggedBodyHelper(body);
        for(int i=0; i < legged->numFeet(); ++i){
            footLinkIndices.push_back(legged->footLink(i)->index());
        }
        
        invalidateCurrentInterpolation();
    }
    
    needUpdate = true;
}


Body* PoseSeqInterpolator::body() const
{
    return impl->body;
}


void PoseSeqInterpolator::setLinearInterpolationJoint(int jointId)
{
    impl->setLinearInterpolationJoint(jointId);
}


void PoseSeqInterpolator::Impl::setLinearInterpolationJoint(int jointId)
{
    if(jointId < (int)jointInfos.size()){
        jointInfos[jointId].useLinearInterpolation = true;
    }
}


void PoseSeqInterpolator::Impl::clearLipSyncShapes()
{
    lipSyncJoints.clear();
    lipSyncLinkIndices.clear();
    lipSyncSeq.clear();
    needUpdate = true;
}


void PoseSeqInterpolator::setLipSyncShapes(const Mapping& info)
{
    impl->setLipSyncShapes(info);
}


/**
   \todo move this into LipSyncTranslator
*/
void PoseSeqInterpolator::Impl::setLipSyncShapes(const Mapping& info)
{
    needUpdate = true;

    clearLipSyncShapes();
    
    if(!info.isValid()){
        return;
    }
    
    const Listing& jointSeq = *info["joints"].toListing();
    const Listing& mixTypeSeq = *info["mixTypes"].toListing();
    int numOrgJoints = jointSeq.size();
    
    for(int i=0; i < numOrgJoints; ++i){
        Link* joint = body->link(jointSeq[i].toString());
        if(joint){
            LipSyncJoint lipSyncJoint;
            int mixType = -1;
            string mixTypeSymbol = mixTypeSeq[i].toString();
            if(mixTypeSymbol == "upper"){
                mixType = UPPER;
            } else if(mixTypeSymbol == "lower"){
                mixType = LOWER;
            } else if(mixTypeSymbol == "addition"){
                mixType = ADDITION;
            }
            if(mixType >= 0){
                lipSyncJoint.jointId = joint->jointId();
                lipSyncJoint.mixType = mixType;
                lipSyncJoint.orgIndex = i;
                lipSyncJoints.push_back(lipSyncJoint);
                lipSyncLinkIndices.push_back(joint->index());
            }
        }
    }
    
    static const char* shapeSymbols[] = {
        "A", "I", "U", "E", "O", "N", "a", "i", "u", "e", "o"
    };
    
    int numShapesRead = 0;
    if(!lipSyncJoints.empty()){
        lipSyncShapes.resize(NUM_LIP_SHAPES, lipSyncJoints.size());
        const Mapping& shapes = *info["shapes"].toMapping();
        for(int i=0; i < NUM_LIP_SHAPES; ++i){
            const Listing& shapeSeq = *shapes.findListing(shapeSymbols[i]);
            if(!shapeSeq.isValid() || shapeSeq.size() != numOrgJoints){
                break;
            }
            for(size_t j=0; j < lipSyncJoints.size(); ++j){
                double q = radian(shapeSeq[lipSyncJoints[j].orgIndex].toDouble());
                lipSyncShapes(i, j) = q;
            }
            numShapesRead++;
        }
    }

    lipSyncMaxTransitionTime = info.get("maxTransitionTime", 0.2);

    if(numShapesRead < NUM_LIP_SHAPES){
        clearLipSyncShapes();
    }
}


const std::vector<int>& PoseSeqInterpolator::lipSyncLinkIndices()
{
    return impl->lipSyncLinkIndices;
}


void PoseSeqInterpolator::Impl::invalidateCurrentInterpolation()
{
    currentTime = std::numeric_limits<double>::max();
    currentBaseLinkInfoIter = ikLinkInfos.end();
}


void PoseSeqInterpolator::setPoseSeq(PoseSeq* seq)
{
    impl->setPoseSeq(seq);
}


void PoseSeqInterpolator::Impl::setPoseSeq(PoseSeq* seq)
{
    poseSeqConnections.disconnect();
    poseSeq = seq;

    // for auto update mode (not implemented yet)
    poseSeqConnections.add(
        seq->sigPoseInserted().connect(
            [this](PoseSeq::iterator it, bool /* isMoving */){
                onPoseInserted(it);
            }));
    poseSeqConnections.add(
        seq->sigPoseAboutToBeRemoved().connect(
            [this](PoseSeq::iterator it, bool isMoving){
                onPoseAboutToBeRemoved(it, isMoving);
            }));
    poseSeqConnections.add(
        seq->sigPoseModified().connect(
            [this](PoseSeq::iterator it){
                onPoseModified(it);
            }));
    
    invalidateCurrentInterpolation();
    needUpdate = true;
}


void PoseSeqInterpolator::setTimeScaleRatio(double ratio)
{
    impl->timeScaleRatio = ratio;
}


double PoseSeqInterpolator::beginningTime() const
{
    return 0.0;
}


double PoseSeqInterpolator::endingTime() const
{
    if(impl->poseSeq){
        return impl->timeScaleRatio * impl->poseSeq->endingTime();
    } else {
        return 0.0;
    }
}


void PoseSeqInterpolator::enableAutoZmpAdjustmentMode(bool on)
{
    impl->isAutoZmpAdjustmentMode = on;
    impl->needUpdate = true;
}


void PoseSeqInterpolator::setZmpAdjustmentParameters
(double minTransitionTime, double centeringTimeThresh, double timeMarginBeforeLifting, double maxDistanceFromCenter)
{
    impl->minZmpTransitionTime = minTransitionTime;
    impl->zmpCenteringTimeThresh = centeringTimeThresh;
    impl->zmpTimeMarginBeforeLifting = timeMarginBeforeLifting;
    impl->zmpMaxDistanceFromCenterSqr = maxDistanceFromCenter * maxDistanceFromCenter;
    impl->needUpdate = true;
}


int PoseSeqInterpolator::stepTrajectoryAdjustmentMode() const
{
    return impl->stepTrajectoryAdjustmentMode;
}


void PoseSeqInterpolator::setStepTrajectoryAdjustmentMode(int mode)
{
    impl->stepTrajectoryAdjustmentMode = mode;
    impl->needUpdate = true;
}


void PoseSeqInterpolator::setStealthyStepParameters
(double heightRatioThresh,
 double flatLiftingHeight, double flatLandingHeight,
 double impactReductionHeight, double impactReductionTime)
{
    impl->stealthyHeightRatioThresh = heightRatioThresh;
    impl->flatLiftingHeight = flatLiftingHeight;
    impl->flatLandingHeight = flatLandingHeight;
    impl->impactReductionHeight = impactReductionHeight;
    impl->impactReductionTime = impactReductionTime;
    impl->impactReductionVelocity = -2.0 * impactReductionHeight / impactReductionTime;
    impl->needUpdate = true;
}


void PoseSeqInterpolator::setToeStepParameters(double toeContactAngle, double toeContactTime)
{
    impl->toeContactAngle = toeContactAngle;
    impl->toeContactTime = toeContactTime;
    impl->needUpdate = true;
}


void PoseSeqInterpolator::enableLipSyncMix(bool on)
{
    impl->isLipSyncMixEnabled = on;
}


bool PoseSeqInterpolator::update()
{
    return impl->update();
}


SignalProxy<void()> PoseSeqInterpolator::sigUpdated()
{
    return impl->sigUpdated;
}


bool PoseSeqInterpolator::interpolate(double time)
{
    return impl->interpolate(time, -1, Vector3::Zero());
}


bool PoseSeqInterpolator::seek(double time)
{
    return impl->interpolate(time, -1, Vector3::Zero());
}


/**
   @param waistLinkIndex A link that is translated for maintaing the dynamic balance
   @param waistTranslation translation of the balancing link usually provided by the waist balance filter
*/
bool PoseSeqInterpolator::interpolate(double time, int waistLinkIndex, const Vector3& waistTranslation)
{
    return impl->interpolate(time, waistLinkIndex, waistTranslation);
}


/**
   A virtual function of the WaistTranslator interface
*/
bool PoseSeqInterpolator::seek(double time, int waistLinkIndex, const Vector3& waistTranslation)
{
    return impl->interpolate(time, waistLinkIndex, waistTranslation);
}


static void getLinkPosition(LinkInfo& info, const Vector6& xyzrpy, Isometry3& out_T)
{
    out_T.translation() = xyzrpy.head<3>();
    out_T.linear() = rotFromRpy(xyzrpy.tail<3>());
    if(info.T_offset){
        out_T = out_T * info.T_offset->inverse();
    }
}


static void getLinkPosition(LinkInfo& info, const LinkSample::Seq::iterator& it, Isometry3& out_T)
{
    Vector6 xyzrpy;
    for(int i=0; i < 6; ++i){
        xyzrpy[i] = it->c[i].y;
    }
    getLinkPosition(info, xyzrpy, out_T);
}


/**
   \todo Skip interpolation when only waistTranslation changes
*/
bool PoseSeqInterpolator::Impl::interpolate(double time, int waistLinkIndex, const Vector3& waistTranslation)
{
    if(!body){
        return false;
    }

    if(needUpdate){
        if(!update()){
            return false;
        }
    }

    time /= timeScaleRatio; // This should be applied to the process of appending samples

    if(time == currentTime && waistTranslation == this->waistTranslation){
        return true;
    }

    currentTime = time;
    const auto validIkLinkFlagSize = validIkLinkFlag.size();
    validIkLinkFlag.clear();
    validIkLinkFlag.resize(validIkLinkFlagSize, false);
    
    currentBaseLinkInfoIter = ikLinkInfos.end();
    double baseLinkTime = -std::numeric_limits<double>::max();
    LinkInfoMap::iterator subBaseLinkInfoIter = ikLinkInfos.end();
    double subBaseLinkTime = -std::numeric_limits<double>::max();
    
    bool waistTranslationDone = false;
    this->waistTranslation = waistTranslation;
    
    Vector6 xyzrpy;

    for(LinkInfoMap::iterator it = ikLinkInfos.begin(); it != ikLinkInfos.end(); ++it){

        LinkInfo& info = it->second;

        info.isValid = ::interpolate<6, LinkSample>(info.samples, info.iter, currentTime, xyzrpy.data());

        if(info.isValid){

            /// \todo check this before the interpolation (after seeking only)
            /// \todo skip aux samples for detecting the blending segment
            info.jointSpaceBlendingRatio = 0.0;
            if(info.iter != info.samples.end()){
                LinkSample::Seq::iterator prevNonAuxIter = info.iter;
                while(prevNonAuxIter->isAux){
                    --prevNonAuxIter;
                }
                double prevNonAuxTime = prevNonAuxIter->x;
                LinkSample::Seq::iterator nextIter = info.iter;
                nextIter++;
                LinkSample::Seq::iterator nextNonAuxIter = nextIter;
                while(nextNonAuxIter != info.samples.end() && nextNonAuxIter->isAux){
                    ++nextNonAuxIter;
                }
                
                if(prevNonAuxIter->isSlave){
                    if(nextNonAuxIter == info.samples.end() || nextNonAuxIter->isSlave){
                        info.jointSpaceBlendingRatio = 1.0;
                        continue; // not Cartesian space at all ! skip to the next
                    } else {
                        double r;
                        if(nextNonAuxIter->isTouching){
                            if(info.iter->isAux){
                                r =0.0;
                            } else {
                                r = (nextIter->x - currentTime) / (nextIter->x - prevNonAuxTime);
                            }
                        } else {
                            r = (nextNonAuxIter->x - currentTime) / (nextNonAuxIter->x - prevNonAuxTime);
                        }
                        info.jointSpaceBlendingRatio = r * r * (3.0 - 2.0 * r);
                    }
                } else {
                    if(nextNonAuxIter != info.samples.end() && nextNonAuxIter->isSlave){
                        double r;
                        if(prevNonAuxIter->isTouching){
                            if(nextIter == nextNonAuxIter){
                                r = (currentTime - info.iter->x) / (nextIter->x - info.iter->x);
                            } else {
                                r = 0.0;
                            }
                        } else {
                            r = (currentTime - prevNonAuxTime) / (nextNonAuxIter->x - prevNonAuxTime);
                        }
                        info.jointSpaceBlendingRatio = r * r * (3.0 - 2.0 * r);
                    }
                }
            }

            if(info.isFootLink){
                if(stepTrajectoryAdjustmentMode == StealthyStepMode){
                    // Translation z interpolation
                    ::interpolate<1, LinkAuxSample>(info.auxSamples, info.auxIter, currentTime, &xyzrpy[2]);
                } else if(stepTrajectoryAdjustmentMode == ToeStepMode){
                    // Pithc interpolation
                    ::interpolate<1, LinkAuxSample>(info.auxSamples, info.auxIter, currentTime, &xyzrpy[4]);
                }
            }

            Isometry3 T;
            getLinkPosition(info, xyzrpy, T);

            const int linkIndex = it->first;
            validIkLinkFlag[linkIndex] = true;
            if(linkIndex == waistLinkIndex){
                info.T.translation() = T.translation() + waistTranslation;
                waistTranslationDone = true;
            } else {
                info.T.translation() = T.translation() + info.jointSpaceBlendingRatio * waistTranslation;
            }
            
            info.T.linear() = T.linear();
            if(info.iter->isBaseLink){
                if(info.iter->x > baseLinkTime){
                    currentBaseLinkInfoIter = it;
                    baseLinkTime = info.iter->x;
                }
            } else {
                if(info.iter->x > subBaseLinkTime){
                    subBaseLinkInfoIter = it;
                    subBaseLinkTime = info.iter->x;
                }
            }

        }
    }

    if(currentBaseLinkInfoIter == ikLinkInfos.end()){
        currentBaseLinkInfoIter = subBaseLinkInfoIter;
    }

    for(size_t i=0; i < jointInfos.size(); ++i){
        jointInfos[i].q = stdx::nullopt;
    }

    calcIkJointPositions();

    if(waistLinkIndex >= 0 && !waistTranslationDone){
        /// \todo write a code here to translate the waist when the waist link is not an ik link
        return false;
    }

    if(isLipSyncMixEnabled){
        mixLipSyncShape();
    }

    return true;
}


bool PoseSeqInterpolator::Impl::mixLipSyncShape()
{
    if(lipSyncSeq.empty()){
        return false;
    }

    vector<LipSyncSample>::iterator next = lipSyncSeq.end();
    
    if(lipSyncIter == lipSyncSeq.end()){
        --lipSyncIter;
        if(lipSyncIter->time <= currentTime){
            goto mix;
        }
    }

    while(true){
        if(currentTime < lipSyncIter->time){
            if(lipSyncIter == lipSyncSeq.begin()){
                goto mix;
            }
            --lipSyncIter;
            continue;
        }
        while(true){
            next = lipSyncIter; ++next;
            if(next == lipSyncSeq.end()){
                goto mix;
            }
            if(currentTime < next->time){
                break;
            }
            ++lipSyncIter;
        }
        break;
    }

mix:

    for(size_t i=0; i < lipSyncJoints.size(); ++i){
        LipSyncJoint& lipSyncJoint = lipSyncJoints[i];
        double q0 = lipSyncShapes(lipSyncIter->shapeId, i);
        double q;
        if(next == lipSyncSeq.end()){
            q = q0;
        } else {
            double q1 = lipSyncShapes(next->shapeId, i);
            double t = (currentTime - lipSyncIter->time) / (next->time - lipSyncIter->time);
            q = (1.0 - t) * q0 + t * q1;
        }

        int jointId = lipSyncJoint.jointId;
        JointInfo& jointInfo = jointInfos[jointId];
        auto qorg = self->jointPosition(jointId);

        if(!qorg){
            jointInfo.q = q;
        } else {
            switch(lipSyncJoint.mixType){
            case UPPER:
                jointInfo.q = std::max(*qorg, q);
                break;
            case LOWER:
                jointInfo.q = std::min(*qorg, q);
                break;
            case ADDITION: {
                q += *qorg;
                Link* joint = body->joint(jointId);
                jointInfo.q = std::max(joint->q_lower(), std::min(joint->q_upper(), q));
            }
                break;
            default:
                break;
            }
        }
    }

    return true;
}


void PoseSeqInterpolator::Impl::calcIkJointPositions()
{
    Link* baseLink;
    LinkInfo* baseLinkInfo;
    
    if(currentBaseLinkInfoIter != ikLinkInfos.end()){
        baseLink = body->link(currentBaseLinkInfoIter->first);
        baseLinkInfo = &currentBaseLinkInfoIter->second;
    } else {
        baseLink = body->rootLink();
        baseLinkInfo = nullptr;
    }

    calcIkJointPositionsSub(baseLink, baseLink, baseLinkInfo, true, nullptr);
}


/**
   \todo search an analytical IK path even if the base link of the path is not an ik link
*/
void PoseSeqInterpolator::Impl::calcIkJointPositionsSub
(Link* link, Link* baseLink, LinkInfo* baseLinkInfo, bool doUpward, Link* prevLink)
{
    if(link != baseLink && validIkLinkFlag[link->index()]){
        LinkInfo* endLinkInfo = getIkLinkInfo(link->index());
        if(baseLinkInfo && endLinkInfo){

            shared_ptr<JointPath> jointPath;
            LinkPair linkPair(baseLink, link);
            auto iter = ikJointPathMap.find(linkPair);
            if(iter != ikJointPathMap.end()){
                jointPath = iter->second;
            } else {
                jointPath = JointPath::getCustomPath(baseLink, link);
                ikJointPathMap[linkPair] = jointPath;
            }

            bool doIK = true; // tmp
            
            if(!jointPath->hasCustomIK()){

                // tmp
                if(jointPath->numJoints() != 6){
                    doIK = false;
                }
                
                for(int i=0; i < jointPath->numJoints(); ++i){
                    Link* joint = jointPath->joint(i);
                    JointInfo& jointInfo = jointInfos[joint->jointId()];
                    double q;
                    if(::interpolate<1, JointSample>(jointInfo.samples, jointInfo.iter, currentTime, &q)){
                        jointInfo.q = q;
                        joint->q() = q;
                    }
                }
            }

            if(doIK){ // tmp

                jointPath->setBaseLinkGoal(baseLinkInfo->T);
                bool ikSolved = jointPath->setBaseLinkGoal(baseLinkInfo->T)
                    .calcInverseKinematics(endLinkInfo->T);
                
                if(!ikSolved){
                    double len = waistTranslation.norm();
                    double low = 0.0;
                    double hi = len * (1.0 - endLinkInfo->jointSpaceBlendingRatio);
                    while((hi - low) > 1.0e-4){
                        double current = (low + hi) / 2.0;
                        Isometry3 T_end(endLinkInfo->T);
                        T_end.translation() += waistTranslation * (current / len);
                        if(jointPath->setBaseLinkGoal(baseLinkInfo->T).calcInverseKinematics(T_end)){
                            ikSolved = true;
                            hi = current;
                        } else {
                            low = current;
                        }
                    }
                }
                
                if(ikSolved){
                    for(int i=0; i < jointPath->numJoints(); ++i){
                        Link* joint = jointPath->joint(i);
                        JointInfo& jointInfo = jointInfos[joint->jointId()];
                        const double& r = endLinkInfo->jointSpaceBlendingRatio;
                        if(r == 0.0){
                            jointInfo.q = joint->q();
                        } else {
                            double qtmp;
                            if(::interpolate<1, JointSample>(jointInfo.samples, jointInfo.iter, currentTime, &qtmp)){
                                jointInfo.q = r * qtmp + (1.0 - r) * joint->q();
                            } else {
                                jointInfo.q = joint->q();
                            }
                        }
                    }
                }
            }
        }
        baseLink = link;
        baseLinkInfo = endLinkInfo;
    }

    if(doUpward && link->parent()){
        calcIkJointPositionsSub(link->parent(), baseLink, baseLinkInfo, true, link);
    }
    for(Link* childLink = link->child(); childLink; childLink = childLink->sibling()){
        if(childLink != prevLink){
            calcIkJointPositionsSub(childLink, baseLink, baseLinkInfo, false, 0);
        }
    }    
}


int PoseSeqInterpolator::baseLinkIndex() const
{
    if(impl->currentBaseLinkInfoIter != impl->ikLinkInfos.end()){
        return impl->currentBaseLinkInfoIter->first;
    }
    return -1;
}


bool PoseSeqInterpolator::getBaseLinkPosition(Isometry3& out_T) const
{
    if(impl->currentBaseLinkInfoIter != impl->ikLinkInfos.end()){
        const LinkInfo& info = impl->currentBaseLinkInfoIter->second;
        out_T = info.T;
        return true;
    }
    return false;
}


stdx::optional<double> PoseSeqInterpolator::jointPosition(int jointId) const
{
    JointInfo& info = impl->jointInfos[jointId];
    if(!info.q){
        double qtmp;
        if(::interpolate<1, JointSample>(info.samples, info.iter, impl->currentTime, &qtmp)){
            info.q = qtmp;
        }
    }
    return info.q;
}


void PoseSeqInterpolator::getJointDisplacements(std::vector<stdx::optional<double>>& out_q) const
{
    const int n = impl->jointInfos.size();
    out_q.resize(n);
    for(int i=0; i < n; ++i){
        out_q[i] = jointPosition(i);
    }
}


stdx::optional<Vector3> PoseSeqInterpolator::ZMP() const
{
    Vector3 p;
    if(::interpolate<3, ZmpSample>(impl->zmpSamples, impl->zmpIter, impl->currentTime, p.data())){
        return p;
    }
    return stdx::nullopt;
}


bool PoseSeqInterpolator::Impl::update()
{
    if(!body || !poseSeq){
        return false;
    }
    
    for(size_t i=0; i < jointInfos.size(); ++i){
        jointInfos[i].clear();
    }
    ikLinkInfos.clear();
    zmpSamples.clear();
    orgZmpSampleIterPairs.clear();
    lipSyncSeq.clear();

    footLinkInfos.clear();
    if(isAutoZmpAdjustmentMode || stepTrajectoryAdjustmentMode != NoStepAdjustmentMode){
        for(size_t i=0; i < footLinkIndices.size(); ++i){
            if(auto info = getIkLinkInfo(footLinkIndices[i])){
                info->isFootLink = true;
                if(stepTrajectoryAdjustmentMode == ToeStepMode){
                    info->T_offset = legged->toeOffset(i);
                } else {
                    info->T_offset = stdx::nullopt;
                }
                footLinkInfos.push_back(info);
            }
        }
    }

    for(PoseSeq::iterator poseIter = poseSeq->begin(); poseIter != poseSeq->end(); ++poseIter){

        auto pose = poseIter->get<BodyKeyPose>();

        if(!pose){
            PronunSymbolPtr pronun = poseIter->get<PronunSymbol>();
            if(pronun){
                appendPronun(poseIter);
            }
        } else {
            appendLinkSamples(poseIter, pose);

            const int n = std::min(pose->numJoints(), (int)jointInfos.size());

            for(int i=0; i < n; ++i){
                JointInfo& jointInfo = jointInfos[i];
                if(pose->isJointValid(i)){

                    // make a flipping point stationary point
                    double q = pose->jointDisplacement(i);
                    double sign = q - jointInfo.prev_q;
                    if(jointInfo.prevSegmentDirectionSign * sign <= 0.0){
                        if(!jointInfo.samples.empty()){
                            jointInfo.samples.back().isEndPoint = true;
                        }
                    }
                    jointInfo.prevSegmentDirectionSign = sign;
                    jointInfo.prev_q = q;

                    appendSample(jointInfo.samples, JointSample(poseIter, i, jointInfo.useLinearInterpolation));
                }
            }
            if(pose->isZmpValid()){
                if(isAutoZmpAdjustmentMode){
                    zmpSamples.push_back(ZmpSample(poseIter));
                    /*
                       Note that the sample at the start of the transition is not inserted here,
                       but will be inserted later with the insertZmpSamplesAtTransitionStartPoints
                       function if necessary to avoid the conflict with ZMP samples inserted by the
                       automatic ZMP adjustment.
                    */
                    orgZmpSampleIterPairs.emplace_back(poseIter, --zmpSamples.end());
                } else {
                    appendSample(zmpSamples, ZmpSample(poseIter));
                }
            }
        }            
    }

    if(!footLinkInfos.empty()){
        if(isAutoZmpAdjustmentMode && footLinkInfos.size() == 2){
            adjustZmpAndFootKeyPoses();
            insertZmpSamplesAtTransitionStartPoints();
        }
        if(stepTrajectoryAdjustmentMode == StealthyStepMode){
            insertAuxKeyPosesForStealthySteps();
        } else if(stepTrajectoryAdjustmentMode == ToeStepMode){
            insertAuxKeyPosesForToeSteps();
        }
    }

    for(size_t i=0; i < jointInfos.size(); ++i){
        JointInfo& info = jointInfos[i];
        if(!info.useLinearInterpolation){
            initializeInterpolation<1, JointSample, false>(info.samples);
        }
        info.iter = info.samples.begin();
    }
    for(auto& kv : ikLinkInfos){
        LinkInfo& info = kv.second;
        initializeInterpolation<6, LinkSample, false>(info.samples);
        info.iter = info.samples.begin();
        if(info.isFootLink){
            initializeInterpolation<1, LinkAuxSample, false>(info.auxSamples);
            info.auxIter = info.auxSamples.begin();
        }
    }
    
    initializeInterpolation<3, ZmpSample, false>(zmpSamples);
    zmpIter = zmpSamples.begin();

    lipSyncIter = lipSyncSeq.begin();

    invalidateCurrentInterpolation();
    needUpdate = false;

    sigUpdated();

    return true;
}


void PoseSeqInterpolator::Impl::appendLinkSamples(PoseSeq::iterator poseIter, BodyKeyPose* pose)
{
    for(auto it = pose->ikLinkBegin(); it != pose->ikLinkEnd(); ++it){
        const int linkIndex = it->first;
        LinkInfo* linkInfo = getIkLinkInfo(linkIndex);
        if(linkInfo){
            const BodyKeyPose::LinkInfo& ikLinkInfo = it->second;
    
            LinkSample::Seq& samples = linkInfo->samples;
            insertSampleAtTransitionStartPoint<LinkSample>(samples, samples.end(), poseIter);

            samples.emplace_back();
            LinkSample& sample = samples.back();

            sample.segmentType = UNDETERMINED;
            sample.poseIter = poseIter;
            sample.x = poseIter->time();

            const Isometry3* T;
            Isometry3 T_fixed;
            if(linkInfo->T_offset){
                T_fixed = ikLinkInfo.T() * (*linkInfo->T_offset);
                T = &T_fixed;
            } else {
                T = &ikLinkInfo.T();
            }
            Vector6 xyzrpy;
            xyzrpy.head<3>() = T->translation();
            xyzrpy.tail<3>() = rpyFromRot(T->linear());
            for(int i=0; i < 6; ++i){
                sample.c[i].y = xyzrpy[i];
                sample.c[i].yp = 0.0;
            }
            
            sample.isBaseLink = ikLinkInfo.isBaseLink();
            sample.isTouching = ikLinkInfo.isTouching();
            sample.isEndPoint = ikLinkInfo.isStationaryPoint() || sample.isTouching;
            sample.isDirty = true;
            sample.isSlave = ikLinkInfo.isSlave() && !ikLinkInfo.isTouching();
            sample.isAux = false;

            if(linkInfo->isFootLink && stepTrajectoryAdjustmentMode != NoStepAdjustmentMode){
                LinkAuxSample::Seq& auxSamples = linkInfo->auxSamples;
                insertSampleAtTransitionStartPoint<LinkAuxSample>(auxSamples, auxSamples.end(), poseIter);
                auxSamples.emplace_back();
                LinkAuxSample& sample = auxSamples.back();
                sample.segmentType = UNDETERMINED;
                sample.poseIter = poseIter;
                sample.x = poseIter->time();
                int auxElement = (stepTrajectoryAdjustmentMode == StealthyStepMode) ? 2 /* Z */ : 4 /* Pitch */;
                sample.c[0].y = xyzrpy[auxElement];
                sample.c[0].yp = 0.0;
                sample.isTouching = ikLinkInfo.isTouching();
                sample.isEndPoint = ikLinkInfo.isStationaryPoint() || sample.isTouching;
                sample.isDirty = true;
            }
        }
    }
}


inline bool PoseSeqInterpolator::Impl::checkZmp(const Vector3& zmp, const Vector3& centerZmp)
{
    return (zmp - centerZmp).squaredNorm() <= zmpMaxDistanceFromCenterSqr;
}


void PoseSeqInterpolator::Impl::adjustZmpAndFootKeyPosesForLifting
(LinkSample::Seq& swingSamples, LinkSample::Seq::iterator pSwing0, LinkSample::Seq::iterator pSwing1,
 LinkAuxSample::Seq& swingAuxSamples, LinkAuxSample::Seq::iterator pSwingAux0, LinkAuxSample::Seq::iterator pSwingAux1,
 ZmpSample::Seq::iterator pZmp0,  const Vector3& zmpOnSupport, bool zmpCenteringDone)
{
    Vector3 zmp0(pZmp0->c[0].y, pZmp0->c[1].y, pZmp0->c[2].y);

    if(!checkZmp(zmp0, zmpOnSupport)){

        double ttime0 = pSwing0->x - pZmp0->x;
        
        if(!zmpCenteringDone && ttime0 < minZmpTransitionTime){
            double auxKeyTime = std::min(pZmp0->x + minZmpTransitionTime, (pSwing0->x + pSwing1->x) / 2.0);
            double zmpTime = std::max((pZmp0->x + auxKeyTime) / 2.0, auxKeyTime - zmpTimeMarginBeforeLifting);
            zmpSamples.insert(++pZmp0, ZmpSample(zmpTime, zmpOnSupport));
            LinkSample::Seq::iterator pAux = swingSamples.insert(pSwing1, LinkSample(*pSwing0));
            pAux->x = auxKeyTime;
            LinkAuxSample::Seq::iterator pZAux = swingAuxSamples.insert(pSwingAux1, LinkAuxSample(*pSwingAux0));
            pZAux->x = auxKeyTime;
        } else {
            double zmpTime = std::max((pZmp0->x + pSwing0->x) / 2.0, pSwing0->x - zmpTimeMarginBeforeLifting);
            ZmpSample::Seq::iterator pAuxZmp = zmpSamples.insert(++pZmp0, ZmpSample(zmpTime, zmpOnSupport));
            /*
              if(ttime0 >= minZmpTransitionTime * 2.0){
              zmpSamples.insert(pAuxZmp, ZmpSample(pSwing0->x - minZmpTransitionTime * 2.0, zmp0));
              }
            */
        }
    }
}


void PoseSeqInterpolator::Impl::adjustZmpAndFootKeyPosesForLanding
(LinkSample::Seq& swingSamples, LinkSample::Seq::iterator pSwing0, LinkSample::Seq::iterator pSwing1,
 LinkAuxSample::Seq& swingAuxSamples, LinkAuxSample::Seq::iterator pSwingAux0, LinkAuxSample::Seq::iterator pSwingAux1,
 ZmpSample::Seq::iterator pZmp1,  const Vector3& zmp1, const Vector3& zmpOnSupport)
{
    if(!checkZmp(zmp1, zmpOnSupport)){
        if(pZmp1 == zmpSamples.end()){
            zmpSamples.insert(pZmp1, ZmpSample(pSwing1->x, zmpOnSupport));
        } else {
            if((pZmp1->x - pSwing1->x) < minZmpTransitionTime){
                double auxKeyTime = std::max(pZmp1->x - minZmpTransitionTime, (pSwing0->x + pSwing1->x) / 2.0);
                zmpSamples.insert(pZmp1, ZmpSample(auxKeyTime, zmpOnSupport));
                LinkSample::Seq::iterator pAux = swingSamples.insert(pSwing1, LinkSample(*pSwing1));
                pAux->x = auxKeyTime;
                LinkAuxSample::Seq::iterator pZAux = swingAuxSamples.insert(pSwingAux1, LinkAuxSample(*pSwingAux1));
                pZAux->x = auxKeyTime;
            } else {
                zmpSamples.insert(pZmp1, ZmpSample(pSwing1->x, zmpOnSupport));
            }
        }
    }
}


/**
   @return true if centering ZMP is inserted
*/
bool PoseSeqInterpolator::Impl::adjustZmpForBothPhase
(ZmpSample::Seq::iterator& pZmp0, double time0, double time1,
 LinkInfo* leftInfo, LinkSample::Seq::iterator pRight0,
 LinkInfo* rightInfo, LinkSample::Seq::iterator pLeft0,
 SupportPhase prevPhase, SupportPhase nextPhase)
{
    double len = time1 - time0;
    if(len < zmpCenteringTimeThresh){
        return false;
    }

    Isometry3 T0;
    getLinkPosition(*rightInfo, pRight0, T0);
    Isometry3 T1;
    getLinkPosition(*leftInfo, pLeft0, T1);
    
    double thresh = minZmpTransitionTime * 2.0;

    if(prevPhase != nextPhase){
        thresh *= 2.0;
    }

    const Vector3& c0 = legged->centerOfSoleLocal(0);
    const Vector3& c1 = legged->centerOfSoleLocal(1);
    
    if(len > thresh){
        Vector3 zmp = (T0 * c0 + T1 * c1) / 2.0;
        zmp[2] = 0.0;
        zmpSamples.insert(++pZmp0, ZmpSample(time0 + minZmpTransitionTime, zmp));
        pZmp0 = zmpSamples.insert(pZmp0, ZmpSample(time1 - minZmpTransitionTime, zmp));

    } else if(prevPhase == nextPhase){
        double r = 0.5;
        double thresh2 = (2.0 * minZmpTransitionTime) * 0.6;
        if(len < thresh2){
            r = 0.5 * (len / thresh2);
            if(prevPhase == RIGHT){
                r = 1.0 - r;
            }
        }
        Vector3 zmp = (T0 * c0) * r + (T1 * c1) * (1.0 - r);
        zmp[2] = 0.0;
        pZmp0 = zmpSamples.insert(++pZmp0, ZmpSample(time0 + len / 2.0, zmp));
    }

    return true;
}


Vector3 PoseSeqInterpolator::Impl::getCenterZmp
(LinkInfo* linkInfo, const LinkSample::Seq::iterator& it, int which)
{
    Isometry3 T_foot;
    getLinkPosition(*linkInfo, it, T_foot);
    Vector3 zmp = T_foot * legged->centerOfSoleLocal(which);
    zmp.z() = 0.0;
    return zmp;
}


void PoseSeqInterpolator::Impl::adjustZmpAndFootKeyPoses()
{
    // actual left and right may be exchanged
    LinkInfo* leftInfo = footLinkInfos[LEFT];
    LinkInfo* rightInfo = footLinkInfos[RIGHT];
    LinkSample::Seq& leftSamples = leftInfo->samples;
    LinkSample::Seq& rightSamples = rightInfo->samples;
    LinkAuxSample::Seq& leftAuxSamples = leftInfo->auxSamples;
    LinkAuxSample::Seq& rightAuxSamples = rightInfo->auxSamples;

    if(leftSamples.empty() || rightSamples.empty()){
        return;
    }

    LinkSample::Seq::iterator pLeft0, pLeft, pLeftNext, pRight0, pRight, pRightNext;
    LinkAuxSample::Seq::iterator pLeftZ0, pLeftZ, pLeftZNext, pRightZ0, pRightZ, pRightZNext;
    pLeft = pLeftNext = leftSamples.begin();
    pRight = pRightNext = rightSamples.begin();
    pLeftZ = pLeftZNext = leftAuxSamples.begin();
    pRightZ = pRightZNext = rightAuxSamples.begin();

    // insert initial zmp
    if(zmpSamples.empty()){
        Vector3 zmp0 = Vector3::Zero();
        int n = 0;
        if(pLeft->isTouching){
            zmp0 += getCenterZmp(leftInfo, pLeft, LEFT);
            n++;
        }
        if(pRight->isTouching){
            zmp0 += getCenterZmp(rightInfo, pRight, RIGHT);
            n++;
        }
        if(n > 0){
            zmp0 /= n;
        }
        zmpSamples.push_back(ZmpSample(0.0, zmp0));
    }

    ZmpSample::Seq::iterator pZmp0 = zmpSamples.begin();

    bool isLeftTouching = true;
    bool isRightTouching = true;

    SupportPhase prevPhase = NONE;
    double time = 0.0;
    double time0 = 0.0;

    SupportPhase phaseBeforeBothPhase = NONE;
    ZmpSample::Seq::iterator pZmpInBothPhase = zmpSamples.end();
    double bothPhaseTime0 = 0.0;

    bool doContinue = true;
    
    while(doContinue){

        if(pLeftNext != leftSamples.end()){
            if(pRightNext != rightSamples.end()){
                if(pLeftNext->x < pRightNext->x){
                    pLeft = pLeftNext++;
                    pLeftZ = pLeftZNext++;
                    isLeftTouching = pLeft->isTouching;
                    time = pLeft->x;
                } else if(pLeftNext->x == pRightNext->x){
                    pLeft = pLeftNext++;
                    pLeftZ = pLeftZNext++;
                    isLeftTouching = pLeft->isTouching;
                    pRight = pRightNext++;
                    pRightZ = pRightZNext++;
                    isRightTouching = pRight->isTouching;
                    time = pLeft->x;
                } else {
                    pRight = pRightNext++;
                    pRightZ = pRightZNext++;
                    isRightTouching = pRight->isTouching;
                    time = pRight->x;
                }
            } else {
                pLeft = pLeftNext++;
                pLeftZ = pLeftZNext++;
                isLeftTouching = pLeft->isTouching;
                time = pLeft->x;
            }
        } else if(pRightNext != rightSamples.end()){
            pRight = pRightNext++;
            pRightZ = pRightZNext++;
            isRightTouching = pRight->isTouching;
            time = pRight->x;
        } else {
            doContinue = false;
        }

        SupportPhase phase;

        if(!doContinue){
            phase = NONE;
        } else {
            if(isLeftTouching){
                if(isRightTouching){
                    phase = BOTH;
                } else {
                    phase = LEFT;
                }
            } else {
                if(isRightTouching){
                    phase = RIGHT;
                } else {
                    phase = FLOATING;
                }
            }
        }

        while(++pZmp0 != zmpSamples.end()){
            if(pZmp0->x > time0){
                break;
            }
        }
        --pZmp0;

        if(prevPhase == BOTH){

            if(pZmp0->x > bothPhaseTime0){
                pZmpInBothPhase = pZmp0;
            }
            if(phase == LEFT || phase == RIGHT){

                bool zmpCenteringDone = false;
                
                if(pZmpInBothPhase == zmpSamples.end()){
                    zmpCenteringDone =
                        adjustZmpForBothPhase(
                            pZmp0, bothPhaseTime0, time0,
                            rightInfo, pRight0, leftInfo, pLeft0,
                            phaseBeforeBothPhase, phase);
                }
                if(phase == LEFT){
                    adjustZmpAndFootKeyPosesForLifting(
                        rightSamples, pRight0, pRight, rightAuxSamples, pRightZ0, pRightZ,
                        pZmp0, getCenterZmp(leftInfo, pLeft, LEFT), zmpCenteringDone);
                } else if(phase == RIGHT){
                    adjustZmpAndFootKeyPosesForLifting(
                        leftSamples, pLeft0, pLeft, leftAuxSamples, pLeftZ0, pLeftZ,
                        pZmp0, getCenterZmp(rightInfo, pRight, RIGHT), zmpCenteringDone);
                }
            }
                
        } else if(phase == BOTH){
            if(prevPhase == LEFT || prevPhase == RIGHT){
                ZmpSample::Seq::iterator pZmp1 = pZmp0;
                while(pZmp1 != zmpSamples.end() && pZmp1->x < time){
                    ++pZmp1;
                }
                Vector3 zmp1;
                if(pZmp1 != zmpSamples.end() && pZmp1->x == time){
                    zmp1 << pZmp1->c[0].y, pZmp1->c[1].y, pZmp1->c[2].y;
                } else {
                    //ZmpSample::Seq::iterator pZmpLast = --zmpSamples.end();
                    //zmp1 = pZmpLast->c[0].y, pZmpLast->c[1].y, pZmpLast->c[2].y;
                    zmp1.fill(numeric_limits<double>::max());
                }
                if(prevPhase == LEFT){
                    adjustZmpAndFootKeyPosesForLanding(
                        rightSamples, pRight0, pRight, rightAuxSamples, pRightZ0, pRightZ,
                        pZmp1, zmp1, getCenterZmp(leftInfo, pLeft, LEFT));
                } else if(prevPhase == RIGHT){
                    adjustZmpAndFootKeyPosesForLanding(
                        leftSamples, pLeft0, pLeft, leftAuxSamples, pLeftZ0, pLeftZ,
                        pZmp1, zmp1, getCenterZmp(rightInfo, pRight, RIGHT));
                }
                phaseBeforeBothPhase = prevPhase;
                bothPhaseTime0 = time;
                pZmpInBothPhase = zmpSamples.end();
            }
        }
        
        prevPhase = phase;
        time0 = time;
        pRight0 = pRight;
        pLeft0 = pLeft;
        pRightZ0 = pRightZ;
        pLeftZ0 = pLeftZ;
    }

    // insert last ZMP
    if(++pZmp0 == zmpSamples.end()){
        Vector3 zmpf = Vector3::Zero();
        int n = 0;
        if(pLeft->isTouching){
            zmpf += getCenterZmp(leftInfo, pLeft, LEFT);
            n++;
        }
        if(pRight->isTouching){
            zmpf += getCenterZmp(rightInfo, pRight, RIGHT);
            n++;
        }
        if(n >0 ){
            zmpf /= n;
        }
        zmpSamples.push_back(ZmpSample(time + minZmpTransitionTime, zmpf));
    }
}


void PoseSeqInterpolator::Impl::insertZmpSamplesAtTransitionStartPoints()
{
    for(auto& iterPair : orgZmpSampleIterPairs){
        insertSampleAtTransitionStartPoint<ZmpSample>(zmpSamples, iterPair.sampleIter, iterPair.poseIter);
    }
}


void PoseSeqInterpolator::Impl::insertAuxKeyPosesForStealthySteps()
{
    for(size_t i=0; i < footLinkInfos.size(); ++i){

        LinkInfo* linkInfo = footLinkInfos[i];
        LinkSample::Seq& samples = linkInfo->samples;
        LinkAuxSample::Seq& auxSamples = linkInfo->auxSamples;

        if(!samples.empty()){
            LinkSample::Seq::iterator pprev = samples.begin();
            LinkSample::Seq::iterator p = pprev;
            ++p;
            LinkAuxSample::Seq::iterator pprevZ = auxSamples.begin();
            LinkAuxSample::Seq::iterator pZ = pprevZ;
            ++pZ;
            while(p != samples.end()){
                if(pprev->isTouching && !p->isTouching){ // lifting
                    if(flatLiftingHeight > 0.0){
                        double height = pZ->c[0].y - pprevZ->c[0].y;
                        if(height >= stealthyHeightRatioThresh * flatLiftingHeight){
                            LinkSample::Seq::iterator paux = samples.insert(p, *pprev);
                            paux->x += (flatLiftingHeight / height) * (p->x - pprev->x);
                        }
                    }
                } else if(!pprev->isTouching && p->isTouching){ // landing
                    if(flatLandingHeight > 0.0){
                        double touchingHeight = pZ->c[0].y;
                        double height = pprevZ->c[0].y - touchingHeight;
                        if(height >= stealthyHeightRatioThresh * flatLandingHeight){
                            LinkSample::Seq::iterator paux = samples.insert(p, LinkSample(*p));
                            const double fallingTime = p->x - pprev->x;
                            paux->isAux = true;
                            paux->x -= (flatLandingHeight / height) * fallingTime;

                            if(impactReductionHeight > 0.0 && impactReductionTime < fallingTime / 2.0){
                                const double h = fallingTime;
                                const double h2 = h * h;
                                const double h3 = h2 * h;
                                const double a2 = 3.0 * (pZ->c[0].y - pprevZ->c[0].y) / h2;
                                const double a3 = 2.0 * (pprevZ->c[0].y - pZ->c[0].y) / h3;
                                const double s = fallingTime - impactReductionTime;
                                const double v = 2.0 * a2 * s + 3.0 * a3 * s * s;
                                if(v < impactReductionVelocity){
                                    LinkAuxSample::Seq::iterator pZaux = auxSamples.insert(pZ, LinkAuxSample(*pZ));
                                    pZaux->x -= impactReductionTime;
                                    pZaux->c[0].y += impactReductionHeight;
                                    pZaux->c[0].yp = impactReductionVelocity;
                                }
                            }
                        }
                    }
                }
                pprev = p++;
                pprevZ = pZ++;
            }
        }
    }
}


void PoseSeqInterpolator::Impl::insertAuxKeyPosesForToeSteps()
{
    for(size_t i=0; i < footLinkInfos.size(); ++i){

        LinkInfo* linkInfo = footLinkInfos[i];
        LinkSample::Seq& samples = linkInfo->samples;
        LinkAuxSample::Seq& auxSamples = linkInfo->auxSamples;

        if(!samples.empty()){
            LinkSample::Seq::iterator pprev = samples.begin();
            LinkSample::Seq::iterator p = pprev;
            ++p;
            LinkAuxSample::Seq::iterator pPrevAux = auxSamples.begin();
            LinkAuxSample::Seq::iterator pAux = pPrevAux;
            ++pAux;
            while(p != samples.end()){
                if(pprev->isTouching && !p->isTouching){ // lifting
                    double liftingTime = p->x - pprev->x;
                    if(toeContactTime < liftingTime / 2.0){
                        auto inserted = samples.insert(p, *pprev);
                        inserted->isAux = true;
                        inserted->x += toeContactTime;
                        auto auxInserted = auxSamples.insert(pAux, *pPrevAux);
                        auxInserted->x = inserted->x;
                        auxInserted->c[0].y = toeContactAngle;
                    }

                } else if(!pprev->isTouching && p->isTouching){ // landing
                    double fallingTime = p->x - pprev->x;
                    if(toeContactTime < fallingTime / 2.0){
                        auto inserted = samples.insert(p, *p);
                        inserted->isAux = true;
                        inserted->x -= toeContactTime;
                        auto auxInserted = auxSamples.insert(pAux, *pAux);
                        auxInserted->x -= toeContactTime;
                        auxInserted->c[0].y = toeContactAngle;
                    }
                }
                pprev = p++;
                pPrevAux = pAux++;
            }
        }
    }
}


void PoseSeqInterpolator::Impl::appendPronun(PoseSeq::iterator poseIter)
{
    PronunSymbol* pronum = poseIter->get<PronunSymbol>();
    if(!pronum){
        return;
    }
    const string& symbol = pronum->symbol();

    if(symbol.empty()){
        return;
    }

    int vowel = -1;

    switch(tolower(symbol[symbol.size()-1])){

    case 'a': vowel = LS_A; break;
    case 'i': vowel = LS_I; break;
    case 'u': vowel = LS_U; break;
    case 'e': vowel = LS_E; break;
    case 'o': vowel = LS_O; break;
    case 'n': vowel = LS_N; break;
    case ',': vowel = LS_N; break;
    case '.': vowel = LS_N; break;

    default: 
        break;
    }

    if(vowel < 0){
        return;
    }

    LipSyncSample sample0;
    sample0.shapeId = -1;
    LipSyncSample sample1;
    sample1.shapeId = -1;

    if(vowel != LS_N && symbol.size() >= 2){
        int consonantChar = tolower(symbol[0]);
        if(consonantChar == 'm' || consonantChar == 'b' || consonantChar == 'p'){
            sample0.shapeId = LS_N;
        } else {
            if(!lipSyncSeq.empty()){
                int prevVowel = lipSyncSeq.back().shapeId;
                if(vowel == prevVowel){
                    sample0.shapeId = vowel + LS_a;
                }
            }
        }
    }

    if(sample0.shapeId < 0){
        sample0.shapeId = vowel;
    } else {
        sample1.shapeId = vowel;
    }
    
    double time = poseIter->time();

    while(!lipSyncSeq.empty()){
        double prevTime = lipSyncSeq.back().time;
        double ttime = time - prevTime;
        if(ttime <= 0.0){
            lipSyncSeq.pop_back();
            continue;
        }
        if(ttime > lipSyncMaxTransitionTime){
            lipSyncSeq.push_back(lipSyncSeq.back());
            lipSyncSeq.back().time = time - lipSyncMaxTransitionTime;
        }
        break;
    }

    sample0.time = time;
    lipSyncSeq.push_back(sample0);

    if(sample1.shapeId >= 0){
        sample1.time = time + 0.05;
        lipSyncSeq.push_back(sample1);
    }
}


LinkInfo* PoseSeqInterpolator::Impl::getIkLinkInfo(int linkIndex)
{
    auto it = ikLinkInfos.find(linkIndex);
    if(it == ikLinkInfos.end()){
        if(linkIndex >= 0 && linkIndex < body->numLinks()){
            it = ikLinkInfos.insert(make_pair(linkIndex, LinkInfo(body, linkIndex))).first;
        } else {
            return nullptr;
        }
    }
    return &it->second;
}


void PoseSeqInterpolator::Impl::onPoseInserted(PoseSeq::iterator /* pose */)
{
    needUpdate = true;
}


void PoseSeqInterpolator::Impl::onPoseAboutToBeRemoved(PoseSeq::iterator /* pose */, bool /* isMoving */)
{
    needUpdate = true;
}


void PoseSeqInterpolator::Impl::onPoseModified(PoseSeq::iterator /* pose */)
{
    needUpdate = true;
}
