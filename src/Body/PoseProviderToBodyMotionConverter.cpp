#include "PoseProviderToBodyMotionConverter.h"
#include "Body.h"
#include "LinkPath.h"
#include "BodyMotion.h"
#include "ZMPSeq.h"
#include "PoseProvider.h"

using namespace std;
using namespace cnoid;


PoseProviderToBodyMotionConverter::PoseProviderToBodyMotionConverter()
{
    setFullTimeRange();
    allLinkPositionOutputMode = true;
}

    
void PoseProviderToBodyMotionConverter::setTimeRange(double lower, double upper)
{
    lowerTime = std::max(0.0, lower);
    upperTime = std::max(lowerTime, upper);
}

    
void PoseProviderToBodyMotionConverter::setFullTimeRange()
{
    lowerTime = 0.0;
    upperTime = std::numeric_limits<double>::max();
}


void PoseProviderToBodyMotionConverter::setAllLinkPositionOutput(bool on)
{
    allLinkPositionOutputMode = on;
}


bool PoseProviderToBodyMotionConverter::convert(Body* body, PoseProvider* provider, BodyMotion& motion)
{
    const double frameRate = motion.frameRate();
    const int beginningFrame = static_cast<int>(frameRate * std::max(provider->beginningTime(), lowerTime));
    const int endingFrame = static_cast<int>(frameRate * std::min(provider->endingTime(), upperTime));
    const int numJoints = body->numJoints();
    const int numLinkPositions = (allLinkPositionOutputMode ? body->numLinks() : 1);

    auto pseq = motion.positionSeq();
    pseq->setNumLinkPositionsHint(numLinkPositions);
    pseq->setNumJointDisplacementsHint(numJoints);
    motion.setNumFrames(endingFrame + 1, true);

    ZMPSeq& zmpSeq = *getOrCreateZMPSeq(motion);
    bool isZmpValid = false;

    Link* rootLink = body->rootLink();
    Link* baseLink = rootLink;

    std::shared_ptr<LinkTraverse> fkTraverse;
    if(allLinkPositionOutputMode){
        fkTraverse = make_shared<LinkTraverse>(baseLink, true, true);
    } else {
        fkTraverse = make_shared<LinkPath>(baseLink, rootLink);
    }

    // store the original state
    vector<double> orgJointDisplacements(numJoints);
    for(int i=0; i < numJoints; ++i){
        orgJointDisplacements[i] = body->joint(i)->q();
    }
    Vector3 p0 = rootLink->p();
    Matrix3 R0 = rootLink->R();

    std::vector<stdx::optional<double>> srcJointDisplacements(numJoints);

    for(int frameIndex = beginningFrame; frameIndex <= endingFrame; ++frameIndex){

        provider->seek(frameIndex / frameRate);

        const int baseLinkIndex = provider->baseLinkIndex();
        if(baseLinkIndex >= 0){
            if(baseLinkIndex != baseLink->index()){
                baseLink = body->link(baseLinkIndex);
                if(allLinkPositionOutputMode){
                    fkTraverse->find(baseLink, true, true);
                } else {
                    static_pointer_cast<LinkPath>(fkTraverse)->setPath(baseLink, rootLink);
                }
            }
            provider->getBaseLinkPosition(baseLink->T());
        }

        auto& frame = pseq->allocateFrame(frameIndex);

        provider->getJointDisplacements(srcJointDisplacements);
        auto displacements = frame.jointDisplacements();
        for(int i=0; i < numJoints; ++i){
            const auto& q = srcJointDisplacements[i];
            body->joint(i)->q() = displacements[i] = q ? *q : 0.0;
        }

        if(allLinkPositionOutputMode || baseLink != rootLink){
            fkTraverse->calcForwardKinematics();
        }

        for(int i=0; i < numLinkPositions; ++i){
            frame.linkPosition(i).set(body->link(i)->position());
        }

        if(auto zmp = provider->ZMP()){
            zmpSeq[frameIndex] = *zmp;
            isZmpValid = true;
        }

    }

    if(!isZmpValid){
        //bodyMotionItem->clearRelativeZmpSeq();
    }

    // restore the original state
    for(int i=0; i < numJoints; ++i){
        body->joint(i)->q() = orgJointDisplacements[i];
    }
    rootLink->p() = p0;
    rootLink->R() = R0;
    body->calcForwardKinematics();

    return true;
}
