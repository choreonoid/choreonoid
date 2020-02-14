/**
   @file
   @author Shin'ichiro NAKAOKA
*/

#include "PoseFilters.h"
#include <cnoid/ValueTree>
#include <cnoid/Link>

using namespace std;
using namespace cnoid;

namespace {

class StepAdjuster
{
public:
    PoseSeqPtr seq;
    const vector<int>& footLinkIndices;
    map<int,Pose::LinkInfo*> supportingLinks;
    Vector3 stepAdjustmentTranslation;
    double stepAdjustmentYawDiff;
    Matrix3 stepAdjustmentRotation;

    StepAdjuster(PoseSeqPtr seq, const vector<int>& footLinkIndices, PoseSeq::iterator origin)
        : seq(seq),
          footLinkIndices(footLinkIndices) {
            
        supportingLinks.clear();
        stepAdjustmentTranslation.setZero();
        stepAdjustmentYawDiff = 0.0;
        stepAdjustmentRotation.setIdentity();

        PoseSeq::iterator poseIter;
        for(poseIter = origin; poseIter != seq->end(); ++poseIter){
            adjustStepPosition(poseIter);
        }

        supportingLinks.clear();
        stepAdjustmentTranslation.setZero();
        stepAdjustmentYawDiff = 0.0;
        stepAdjustmentRotation.setIdentity();

        poseIter = origin;
        while(true){
            adjustStepPosition(poseIter);
            if(poseIter == seq->begin()){
                break;
            }
            poseIter--;
        }
    }
        
    void adjustStepPosition(PoseSeq::iterator poseIter) {

        PosePtr pose = poseIter->get<Pose>();
        if(pose){
                
            seq->beginPoseModification(poseIter);
        
            bool modified = false;
            Vector3 dp = Vector3::Zero();
            double da = 0.0;
            for(size_t i=0; i < footLinkIndices.size(); ++i){
                int linkIndex = footLinkIndices[i];
                Pose::LinkInfo* info = pose->ikLinkInfo(linkIndex);
                if(info){
                    map<int,Pose::LinkInfo*>::iterator p = supportingLinks.find(linkIndex);
                    if(p != supportingLinks.end()){
                        if(!info->isTouching()){
                            supportingLinks.erase(p);
                        } else {
                            Pose::LinkInfo* prev = p->second;
                            if(prev->p != info->p){
                                dp += prev->p - info->p;
                                info->p = prev->p;
                                modified = true;
                            }
                            if(prev->R != info->R){
                                const Matrix3 R = info->R.transpose() * prev->R;
                                da += atan2(R(1,0),R(0,0));
                                info->R = prev->R;
                                modified = true;
                            }
                            p->second = info;
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
                
            for(Pose::LinkInfoMap::iterator it = pose->ikLinkBegin(); it != pose->ikLinkEnd(); ++it){
                int linkIndex = it->first;
                if(supportingLinks.find(linkIndex) == supportingLinks.end()){
                    Pose::LinkInfo& info = it->second;
                    if(isDifferent){
                        info.p += stepAdjustmentTranslation;
                        info.R = stepAdjustmentRotation * info.R;
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
                seq->endPoseModification(poseIter);
            }
        }
    }
};
}


void cnoid::adjustStepPositions(PoseSeqPtr seq, const vector<int>& footLinkIndices, PoseSeq::iterator origin)
{
    StepAdjuster adjuster(seq, footLinkIndices, origin);
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

    bool flipPose(PosePtr pose);

public:
    FlipFilter(BodyPtr body);
    void flip(PoseSeqPtr seq);
};
}


FlipFilter::FlipFilter(BodyPtr body)
    : body(body)
{
    const Listing& sjoints = *body->info()->findListing("symmetricJoints");    
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

    const Listing& slinks = *body->info()->findListing("symmetricIkLinks");    
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


void FlipFilter::flip(PoseSeqPtr seq)
{
    PoseSeq::iterator poseIter;
    for(poseIter = seq->begin(); poseIter != seq->end(); ++poseIter){
        PosePtr pose = poseIter->get<Pose>();
        if(pose){
            seq->beginPoseModification(poseIter);
            if(flipPose(pose)){
                seq->endPoseModification(poseIter);
            }
        }
    }
}


bool FlipFilter::flipPose(PosePtr pose)
{
    bool modified = false;
    
    PosePtr orgPose = static_cast<Pose*>(pose->duplicate());

    pose->setNumJoints(0);
    for(int i=0; i < orgPose->numJoints(); ++i){
        if(orgPose->isJointValid(i)){
            double q = orgPose->jointPosition(i);
            JointFlipInfoMap::iterator p = jointFlipInfoMap.find(i);
            if(p == jointFlipInfoMap.end()){
                pose->setJointPosition(i, q);
            } else {
                JointFlipInfo& flip = p->second;
                pose->setJointPosition(flip.counterPartJointId, q * flip.jointPositionSign);
                modified = true;
            }
        }
    }

    pose->clearIkLinks();
    for(Pose::LinkInfoMap::iterator p = orgPose->ikLinkBegin(); p != orgPose->ikLinkEnd(); ++p){
        int index = p->first;
        Pose::LinkInfo& orgInfo = p->second;

        Pose::LinkInfo* info;
        LinkFlipMap::iterator q = linkFlipMap.find(index);
        if(q != linkFlipMap.end()){
            index = q->second;
            info = pose->addIkLink(index);
            info->p = orgInfo.p;
            info->p.y() = -info->p.y();
            Matrix3& R = orgInfo.R;
            info->R <<
                R(0, 0), -R(0, 1),  R(0, 2),
                -R(1, 0),  R(1, 1), -R(1, 2),
                R(2, 0), -R(2, 1),  R(2, 2);

            modified = true;

        } else {
            info = pose->addIkLink(index);
            info->p = orgInfo.p;
            info->R = orgInfo.R;
        }
        info->setStationaryPoint(orgInfo.isStationaryPoint());
        if(orgInfo.isTouching()){
            std::vector<Vector3> contactPoints = orgInfo.contactPoints();
            for(auto& pointVector : contactPoints) pointVector.y() *= -1;
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


void cnoid::flipPoses(PoseSeqPtr seq, BodyPtr body)
{
    FlipFilter filiter(body);
    filiter.flip(seq);
}
    

void cnoid::rotateYawOrientations
(PoseSeqPtr seq, PoseSeq::iterator begin, const Vector3& center, double angle)
{
    const Matrix3 Rz(AngleAxisd(angle, Vector3::UnitZ()));
    
    PoseSeq::iterator poseIter;
    for(poseIter = begin; poseIter != seq->end(); ++poseIter){

        PosePtr pose = poseIter->get<Pose>();
        if(pose){

            if(pose->numIkLinks() > 0 || pose->isZmpValid()){

                seq->beginPoseModification(poseIter);
            
                for(Pose::LinkInfoMap::iterator p = pose->ikLinkBegin(); p != pose->ikLinkEnd(); ++p){
                    Pose::LinkInfo& linkInfo = p->second;
                    linkInfo.p = Rz * (linkInfo.p - center) + center;
                    linkInfo.R = Rz * linkInfo.R;
                }
                
                if(pose->isZmpValid()){
                    pose->setZmp(Rz * (pose->zmp() - center) + center);
                }

                seq->endPoseModification(poseIter);
            }
        }
    }
}
