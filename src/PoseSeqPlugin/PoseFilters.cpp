#include "PoseFilters.h"
#include "BodyKeyPose.h"
#include <cnoid/Body>
#include <cnoid/Link>
#include <cnoid/ValueTree>

using namespace std;
using namespace cnoid;

namespace {

class StepAdjuster
{
public:
    StepAdjuster(PoseSeq* seq, const vector<int>& footLinkIndices);
    bool adjustStepPositions(PoseSeq::iterator origin);

private:
    PoseSeqPtr seq;
    const vector<int>& footLinkIndices;
    map<int, BodyKeyPose::LinkInfo*> supportingLinks;
    Vector3 stepAdjustmentTranslation;
    double stepAdjustmentYawDiff;
    Matrix3 stepAdjustmentRotation;

    bool adjustStepPosition(PoseSeq::iterator poseIter);
};


StepAdjuster::StepAdjuster(PoseSeq* seq, const vector<int>& footLinkIndices)
    : seq(seq),
      footLinkIndices(footLinkIndices)
{
    supportingLinks.clear();
    stepAdjustmentTranslation.setZero();
    stepAdjustmentYawDiff = 0.0;
    stepAdjustmentRotation.setIdentity();
}


bool StepAdjuster::adjustStepPositions(PoseSeq::iterator origin)
{
    bool modified = false;

    PoseSeq::iterator it;
    for(auto it = origin; it != seq->end(); ++it){
        if(adjustStepPosition(it)){
            modified = true;
        }
    }

    supportingLinks.clear();
    stepAdjustmentTranslation.setZero();
    stepAdjustmentYawDiff = 0.0;
    stepAdjustmentRotation.setIdentity();

    it = origin;
    
    while(true){
        if(adjustStepPosition(it)){
            modified = true;
        }
        if(it == seq->begin()){
            break;
        }
        it--;
    }
    
    return modified;
}
    

bool StepAdjuster::adjustStepPosition(PoseSeq::iterator it)
{
    auto pose = it->get<BodyKeyPose>();
    if(!pose){
        return false;
    }

    seq->beginPoseModification(it);

    bool modified = false;
    Vector3 dp = Vector3::Zero();
    double da = 0.0;
    for(size_t i=0; i < footLinkIndices.size(); ++i){
        int linkIndex = footLinkIndices[i];
        BodyKeyPose::LinkInfo* info = pose->ikLinkInfo(linkIndex);
        if(info){
            auto it = supportingLinks.find(linkIndex);
            if(it != supportingLinks.end()){
                if(!info->isTouching()){
                    supportingLinks.erase(it);
                } else {
                    BodyKeyPose::LinkInfo* prev = it->second;
                    if(prev->p() != info->p()){
                        dp += prev->p() - info->p();
                        info->p() = prev->p();
                        modified = true;
                    }
                    if(prev->R() != info->R()){
                        const Matrix3 R = info->R().transpose() * prev->R();
                        da += atan2(R(1,0),R(0,0));
                        info->R() = prev->R();
                        modified = true;
                    }
                    it->second = info;
                }
            }
        }
    }
    
    double ns = supportingLinks.size();
    if(modified && ns > 0){
        stepAdjustmentTranslation[0] = dp[0] / ns;
        stepAdjustmentTranslation[1] = dp[1] / ns;
        stepAdjustmentYawDiff = da / ns;
        stepAdjustmentRotation = AngleAxisd(stepAdjustmentYawDiff, Vector3::UnitZ());
    }
    
    bool isDifferent = (stepAdjustmentTranslation != Vector3::Zero() || stepAdjustmentYawDiff != 0.0);
    
    if(isDifferent){
        modified = true;
    }
    
    for(auto it = pose->ikLinkBegin(); it != pose->ikLinkEnd(); ++it){
        int linkIndex = it->first;
        if(supportingLinks.find(linkIndex) == supportingLinks.end()){
            BodyKeyPose::LinkInfo& info = it->second;
            if(isDifferent){
                info.p() += stepAdjustmentTranslation;
                info.R() = stepAdjustmentRotation * info.R();
            }
            if(info.isTouching()){
                supportingLinks.insert(make_pair(linkIndex, &info));
            }
        }
    }
    if(pose->isZmpValid() && isDifferent){
        pose->setZmp(pose->zmp() + stepAdjustmentTranslation);
    }
    
    if(modified){
        seq->endPoseModification(it);
    }

    return modified;
}

}


/**
   \return true if step positions are actually modified.
*/
bool cnoid::adjustStepPositions(PoseSeq* seq, const vector<int>& footLinkIndices, PoseSeq::iterator origin)
{
    StepAdjuster adjuster(seq, footLinkIndices);
    return adjuster.adjustStepPositions(origin);
}


namespace {

struct JointFlipInfo
{
    int counterPartJointId;
    double jointPositionSign;
};
    
class FlipFilter
{
    BodyPtr body;

    typedef map<int, JointFlipInfo> JointFlipInfoMap;
    JointFlipInfoMap jointFlipInfoMap;

    typedef map<int, int> LinkFlipMap;
    LinkFlipMap linkFlipMap;

    bool flipPose(BodyKeyPose* pose);

public:
    FlipFilter(Body* body);
    bool flip(PoseSeq* seq);
};
}


FlipFilter::FlipFilter(Body* body)
    : body(body)
{
    const Listing& sjoints = *body->info()->findListing({ "symmetric_joints", "symmetricJoints" });
    if(sjoints.isValid() && !sjoints.empty()){
        for(int i=0; i < sjoints.size(); ++i){
            const Listing& jointPair = *sjoints[i].toListing();
            if(jointPair.size() == 1){
                Link* link = body->link(jointPair[0].toString());
                if(link){
                    JointFlipInfo& info = jointFlipInfoMap[link->jointId()];
                    info.counterPartJointId = link->jointId();
                    info.jointPositionSign = -1.0;
                }
            } else if(jointPair.size() >= 2){
                Link* joint[2];
                joint[0] = body->link(jointPair[0].toString());
                joint[1] = body->link(jointPair[1].toString());
                if(joint[0] && joint[1] && joint[0]->jointId() >= 0 && joint[1]->jointId() >= 0){
                    for(int j=0; j < 2; ++j){
                        int other = 1 - j;
                        JointFlipInfo& info = jointFlipInfoMap[joint[j]->jointId()];
                        info.counterPartJointId = joint[other]->jointId();
                        info.jointPositionSign = 1.0;
                        if(jointPair.size() >= 3){
                            info.jointPositionSign = jointPair[2].toDouble();
                        }
                    }
                }
            }
        }
    }

    const Listing& slinks = *body->info()->findListing({ "symmetric_ik_links", "symmetricIkLinks" });
    if(slinks.isValid() && !slinks.empty()){
        for(int i=0; i < slinks.size(); ++i){
            const Listing& linkPair = *slinks[i].toListing();
            if(linkPair.size() == 1){
                Link* link = body->link(linkPair[0].toString());
                if(link){
                    linkFlipMap[link->index()] = link->index();
                }
            } else if(linkPair.size() >= 2){
                Link* link[2];
                link[0] = body->link(linkPair[0].toString());
                link[1] = body->link(linkPair[1].toString());
                if(link[0] && link[1]){
                    for(int j=0; j < 2; ++j){
                        int other = 1 - j;
                        linkFlipMap[link[j]->index()] = link[other]->index();
                    }
                }
            }
        }
    }
}


bool FlipFilter::flip(PoseSeq* seq)
{
    bool modified = false;
    for(auto it = seq->begin(); it != seq->end(); ++it){
        if(auto pose = it->get<BodyKeyPose>()){
            seq->beginPoseModification(it);
            if(flipPose(pose)){
                seq->endPoseModification(it);
                modified = true;
            }
        }
    }
    return modified;
}


bool FlipFilter::flipPose(BodyKeyPose* pose)
{
    bool modified = false;
    
    BodyKeyPosePtr orgPose = pose->clone();

    pose->setNumJoints(0);
    for(int i=0; i < orgPose->numJoints(); ++i){
        if(orgPose->isJointValid(i)){
            double q = orgPose->jointDisplacement(i);
            auto it = jointFlipInfoMap.find(i);
            if(it == jointFlipInfoMap.end()){
                pose->setJointDisplacement(i, q);
            } else {
                JointFlipInfo& flip = it->second;
                pose->setJointDisplacement(flip.counterPartJointId, q * flip.jointPositionSign);
                modified = true;
            }
        }
    }

    pose->clearIkLinks();
    for(auto ikLinkIter = orgPose->ikLinkBegin(); ikLinkIter != orgPose->ikLinkEnd(); ++ikLinkIter){
        int index = ikLinkIter->first;
        BodyKeyPose::LinkInfo& orgInfo = ikLinkIter->second;

        BodyKeyPose::LinkInfo* info;
        auto it = linkFlipMap.find(index);
        if(it != linkFlipMap.end()){
            index = it->second;
            info = pose->getOrCreateIkLink(index);
            info->p() = orgInfo.p();
            info->p().y() = -info->p().y();
            auto R = orgInfo.R();
            info->R() <<
                R(0, 0), -R(0, 1),  R(0, 2),
                -R(1, 0),  R(1, 1), -R(1, 2),
                R(2, 0), -R(2, 1),  R(2, 2);

            modified = true;

        } else {
            info = pose->getOrCreateIkLink(index);
            info->setPosition(orgInfo.position());
        }
        info->setStationaryPoint(orgInfo.isStationaryPoint());
        if(orgInfo.isTouching()){
            std::vector<Vector3> contactPoints = orgInfo.contactPoints();
            for(auto& pointVector : contactPoints){
                pointVector.y() *= -1;
            }
            info->setTouching(orgInfo.partingDirection(), contactPoints);
        }
        info->setSlave(orgInfo.isSlave());

        if(orgInfo.isBaseLink()){
            pose->setBaseLink(index);
        }
    }

    if(orgPose->isZmpValid()){
        Vector3 zmp = orgPose->zmp();
        zmp.y() = -zmp.y();
        pose->setZmp(zmp);
        pose->setZmpStationaryPoint(orgPose->isZmpStationaryPoint());
        modified = true;
    }

    return modified;
}


/**
   \return true if step positions are actually modified.
*/
bool cnoid::flipPoses(PoseSeq* seq, Body* body)
{
    FlipFilter filiter(body);
    return filiter.flip(seq);
}
    

void cnoid::rotateYawOrientations
(PoseSeq* seq, PoseSeq::iterator begin, const Vector3& center, double angle)
{
    const Matrix3 Rz(AngleAxisd(angle, Vector3::UnitZ()));
    
    for(auto it = begin; it != seq->end(); ++it){

        if(auto pose = it->get<BodyKeyPose>()){

            if(pose->numIkLinks() > 0 || pose->isZmpValid()){

                seq->beginPoseModification(it);
            
                for(auto it = pose->ikLinkBegin(); it != pose->ikLinkEnd(); ++it){
                    BodyKeyPose::LinkInfo& linkInfo = it->second;
                    linkInfo.p() = Rz * (linkInfo.p() - center) + center;
                    linkInfo.R() = Rz * linkInfo.R();
                }
                
                if(pose->isZmpValid()){
                    pose->setZmp(Rz * (pose->zmp() - center) + center);
                }

                seq->endPoseModification(it);
            }
        }
    }
}
