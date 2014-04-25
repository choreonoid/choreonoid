/**
   @file
   @author Shin'ichiro Nakaoka
*/

#include "BodySymmetry.h"
#include <cnoid/Link>

using namespace std;
using namespace cnoid;


BodySymmetry::BodySymmetry(BodyPtr body)
    : body(body)
{
    const YamlSequence& sjoints = *body->info()->findSequence("symmetricJoints");    
    if(sjoints.isValid() && !sjoints.empty()){
        for(int i=0; i < sjoints.size(); ++i){
            const YamlSequence& jointPair = *sjoints[i].toSequence();
            if(jointPair.size() == 1){
                Link* link = body->link(jointPair[0].toString());
                if(link){
                    JointSymmetry& symmetry = jointSymmetryMap[link->jointId];
                    symmetry.counterPartJointId = link->jointId;
                    symmetry.jointPositionSign = -1.0;
                    symmetry.offset = 0.0;
                }
            } else if(jointPair.size() >= 2){
                Link* joint[2];
                joint[0] = body->link(jointPair[0].toString());
                joint[1] = body->link(jointPair[1].toString());
                if(joint[0] && joint[1] && joint[0]->jointId >= 0 && joint[1]->jointId >= 0){
                    for(int j=0; j < 2; ++j){
                        int other = 1 - j;
                        JointSymmetry& symmetry = jointSymmetryMap[joint[j]->jointId];
                        symmetry.counterPartJointId = joint[other]->jointId;
                        symmetry.sign = (jointPair.size() >= 3) ? jointPair[2].toDouble() : 1.0;
                        symmetry.offset = (jointPair.size() >= 4) ? jointPair[3].toDouble() : 0.0;
                    }
                }
            }
        }
    }

    const YamlSequence& slinks = *body->info()->findSequence("symmetricIkLinks");    
    if(slinks.isValid() && !slinks.empty()){
        for(int i=0; i < slinks.size(); ++i){
            const YamlSequence& linkPair = *slinks[i].toSequence();
            if(linkPair.size() == 1){
                Link* link = body->link(linkPair[0].toString());
                if(link){
                    LinkSymmetry& symmetry = linkFlipMap[link->index];
                    symmetry.counterPartIndex = link->index;
                }
            } else if(linkPair.size() >= 2){
                Link* link[2];
                link[0] = body->link(linkPair[0].toString());
                link[1] = body->link(linkPair[1].toString());
                if(link[0] && link[1]){
                    for(int j=0; j < 2; ++j){
                        int other = 1 - j;
                        LinkSymmetry& symmetry = linkFlipMap[link[j]->index];
                        symmetry.counterPartIndex = link[other]->index;
                    }
                }
            }
        }
    }
}


void SymmetricPoseChanger::apply(OperationType op)
{
    JointSymmetryMap::iterator p = jointSymmetryMap.begin();
    while(p != jointSymmetryMap.end()){
        
        


        ++p;
    }


}


bool modified = false;
    
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
        info->setTouching(orgInfo.partingDirection());
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
