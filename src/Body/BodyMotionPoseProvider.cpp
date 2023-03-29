#include "BodyMotionPoseProvider.h"
#include "JointPath.h"
#include "LeggedBodyHelper.h"

using namespace std;
using namespace cnoid;

#if defined(_MSC_VER) && _MSC_VER < 1800
namespace {
inline long lround(double x) {
    return static_cast<long>((x > 0.0) ? floor(x + 0.5) : ceil(x -0.5));
}
}
#endif


BodyMotionPoseProvider::BodyMotionPoseProvider()
{
    isReady = false;
    T_waist.setIdentity();
    ZMP_.setZero();
}


bool BodyMotionPoseProvider::setMotion(Body* body__, std::shared_ptr<BodyMotion> motion)
{
    body_ = body__->clone();
    positionSeq = motion->positionSeq();

    footLinks.clear();
    ikPaths.clear();
    footLinkPositions.clear();

    LeggedBodyHelperPtr legged = getLeggedBodyHelper(body_);
    if(legged->isValid()){
        for(int i=0; i < legged->numFeet(); ++i){
            Link* link = legged->footLink(i);
            auto ikPath = JointPath::getCustomPath(body_->rootLink(), link);
            if(ikPath){
                if(ikPath->hasCustomIK() || ikPath->numJoints() == 6){
                    footLinks.push_back(link);
                    ikPaths.push_back(ikPath);
                }
            }
        }
        ZMP_ = legged->homeCopOfSoles();
    } else {
        ZMP_.setZero();
    }

    zmpSeq = getZMPSeq(*motion);

    return updateMotion();
}


bool BodyMotionPoseProvider::updateMotion()
{
    int numLinkPositions = std::min(body_->numLinks(), positionSeq->numLinkPositionsHint());
    if(numLinkPositions < 1){
        isReady = false;
        return false;
    }
    bool doForwardKinematics = false;
    for(auto& footLink : footLinks){
        if(footLink->index() >= numLinkPositions){
            doForwardKinematics = true;
            break;
        }
    }
    const int numFrames = positionSeq->numFrames();
    footLinkPositions.setDimension(numFrames, footLinks.size());
    footLinkPositions.setFrameRate(positionSeq->frameRate());

    numJointDisplacements = std::min(body_->numJoints(), positionSeq->numJointDisplacementsHint());
    qTranslated.resize(numJointDisplacements);
    
    if(!doForwardKinematics){
        for(int frameIndex = 0; frameIndex < numFrames; ++frameIndex){
            auto& frame = positionSeq->frame(frameIndex);
            for(size_t i=0; i < footLinks.size(); ++i){
                auto footLink = footLinks[i];
                Isometry3& T_foot = footLinkPositions(frameIndex, i);
                auto position = frame.linkPosition(footLink->index());
                T_foot.translation() = position.translation();
                T_foot.linear() = position.rotation().toRotationMatrix();
            }
        }
    } else {
        auto rootLink = body_->rootLink();
        for(int frameIndex = 0; frameIndex < numFrames; ++frameIndex){
            auto& frame = positionSeq->frame(frameIndex);
            auto position = frame.linkPosition(0);
            rootLink->setTranslation(position.translation());
            rootLink->setRotation(position.rotation());

            auto displacements = frame.jointDisplacements();
            for(int i=0; i < numJointDisplacements; ++i){
                body_->joint(i)->q() = displacements[i];
            }
            body_->calcForwardKinematics();

            for(size_t i=0; i < footLinks.size(); ++i){
                auto footLink = footLinks[i];
                Isometry3& T_foot = footLinkPositions(frameIndex, i);
                T_foot.translation() = footLink->translation();
                T_foot.linear() = footLink->R();
            }
        }
    }

    isReady = true;
    return true;
}


Body* BodyMotionPoseProvider::body() const
{
    return body_;
}


double BodyMotionPoseProvider::beginningTime() const
{
    return 0.0;
}


double BodyMotionPoseProvider::endingTime() const
{
    if(isReady){
        return (positionSeq->numFrames() - 1) / positionSeq->frameRate();
    }
    return 0.0;
}


bool BodyMotionPoseProvider::seek
(double time, int waistLinkIndex, const Vector3& waistTranslation, bool applyWaistTranslation)
{
    if(!isReady || waistLinkIndex != 0){
        return false;
    }
    
    int frameIndex = lround(time * positionSeq->frameRate());
    if(frameIndex >= positionSeq->numFrames()){
        frameIndex = positionSeq->numFrames() - 1;
    }

    auto& frame = positionSeq->frame(frameIndex);
    auto displacements = frame.jointDisplacements();
    for(int i=0; i < numJointDisplacements; ++i){
        qTranslated[i] = displacements[i];
    }

    auto waistPosition = frame.linkPosition(0);
    T_waist.translation() = waistPosition.translation();
    T_waist.linear() = waistPosition.rotation().toRotationMatrix();
    
    if(applyWaistTranslation){
        T_waist.translation() += waistTranslation;
        for(size_t i=0; i < footLinks.size(); ++i){
            auto ikPath = ikPaths[i];
            const Isometry3& T_foot = footLinkPositions(frameIndex, i);
            ikPath->setBaseLinkGoal(T_waist).calcInverseKinematics(T_foot);
            for(int j=0; j < ikPath->numJoints(); ++j){
                Link* joint = ikPath->joint(j);
                qTranslated[joint->jointId()] = joint->q();
            }
        }
    }

    if(zmpSeq){
        if(zmpSeq->isRootRelative()){
            ZMP_.noalias() = T_waist.linear() * zmpSeq->at(frameIndex) + T_waist.translation();
        } else {
            ZMP_.noalias() = zmpSeq->at(frameIndex);
        }
    }
    
    return true;
}


bool BodyMotionPoseProvider::seek(double time)
{
    return seek(time, 0, Vector3::Zero(), false);
}


bool BodyMotionPoseProvider::seek(double time, int waistLinkIndex, const Vector3& waistTranslation)
{
    return seek(time, waistLinkIndex, waistTranslation, true);
}


int BodyMotionPoseProvider::baseLinkIndex() const
{
    return 0;
}


bool BodyMotionPoseProvider::getBaseLinkPosition(Isometry3& out_T) const
{
    out_T = T_waist;
    return true;
}


void BodyMotionPoseProvider::getJointDisplacements(std::vector<stdx::optional<double>>& out_q) const
{
    int n = qTranslated.size();
    out_q.resize(n);
    for(int i=0; i < n; ++i){
        out_q[i] = qTranslated[i];
    }
}


stdx::optional<Vector3> BodyMotionPoseProvider::ZMP() const
{
    return ZMP_;
}
