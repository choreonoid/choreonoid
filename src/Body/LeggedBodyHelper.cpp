/**
   \file
   \author Shin'ichiro Nakaoka
*/

#include "LeggedBodyHelper.h"
#include "Link.h"
#include "JointPath.h"
#include "CompositeIK.h"
#include <cnoid/EigenArchive>

using namespace std;
using namespace cnoid;


namespace cnoid {

LeggedBodyHelper* getLeggedBodyHelper(Body* body)
{
    LeggedBodyHelper* legged = body->getOrCreateCache<LeggedBodyHelper>("LeggedBodyHelper");
    if(!legged->body()){
        legged->resetBody(body);
    }
    return legged;
}
}


LeggedBodyHelper::LeggedBodyHelper()
{
    isValid_ = false;
}


LeggedBodyHelper::LeggedBodyHelper(Body* body)
{
    resetBody(body);
}


LeggedBodyHelper::LeggedBodyHelper(const LeggedBodyHelper& org)
{
    resetBody(org.body_);
}


LeggedBodyHelper::~LeggedBodyHelper()
{

}


bool LeggedBodyHelper::resetBody(Body* body)
{
    body_ = body;

    footInfos.clear();
    
    const Listing& footLinkNodes = *body_->info()->findListing("footLinks");

    if(footLinkNodes.isValid()){
    
        for(int i=0; i < footLinkNodes.size(); ++i){
            FootInfo footInfo;
            const Mapping& footLinkNode = *footLinkNodes[i].toMapping();
            footInfo.link = body_->link(footLinkNode["link"].toString());
            if(footInfo.link){
                readEx(footLinkNode, "soleCenter", footInfo.soleCenter);
                
                if(!read(footLinkNode, "homeCop", footInfo.homeCop)){
                    footInfo.homeCop = footInfo.soleCenter;
                }
                
                string kneePitchJointLabel;
                if(footLinkNode.read("kneePitchJoint", kneePitchJointLabel)){
                    footInfo.kneePitchJoint = body_->link(kneePitchJointLabel);
                }
                
                footInfos.push_back(footInfo);
            }
        }
    }

    isValid_ = !footInfos.empty();

    return isValid_;
}


std::shared_ptr<InverseKinematics> LeggedBodyHelper::getFootBasedIK(Link* targetLink)
{
    shared_ptr<InverseKinematics> ik;
    if(isValid_){
        auto composite = std::make_shared<CompositeIK>(body_, targetLink);
        ik = composite;
        for(size_t i=0; i < footInfos.size(); ++i){
            if(!composite->addBaseLink(footInfos[i].link)){
                ik.reset();
                break;
            }
        }
    }
    return ik;
}
    

bool LeggedBodyHelper::doLegIkToMoveCm(const Vector3& c, bool onlyProjectionToFloor)
{
    if(!isValid_){
        return false;
    }

    static const int MAX_ITERATION = 100;
    static const double ERROR_THRESH_SQR = 1.0e-6 * 1.0e-6;
    
    Link* baseFoot = footInfos[0].link;
    Link* waist = body_->rootLink();
    LinkTraverse fkTraverse(waist);
    Vector3 c0 = body_->calcCenterOfMass();
    bool failed = false;
    int loopCounter = 0;

    while(true){
        Vector3 e = c - c0;
        if(onlyProjectionToFloor){
            e.z() = 0.0;
        }
        if(e.squaredNorm() < ERROR_THRESH_SQR){
            break;
        }
        size_t numDone = 0;
        auto baseToWaist = getCustomJointPath(body_, baseFoot, waist);
        if(baseToWaist){
            Position T = waist->T();
            T.translation() += e;
            if(baseToWaist->calcInverseKinematics(T)){
                numDone++;
                for(size_t j=1; j < footInfos.size(); ++j){
                    Link* foot = footInfos[j].link;
                    auto waistToFoot = getCustomJointPath(body_, waist, foot);
                    if(waistToFoot){
                        bool ikDone;
                        if(waistToFoot->hasAnalyticalIK()){
                            ikDone = waistToFoot->calcInverseKinematics(foot->T());
                        } else {
                            waistToFoot->calcForwardKinematics();
                            ikDone = waistToFoot->calcInverseKinematics(foot->T());
                        }
                        if(ikDone){
                            numDone++;
                        } else {
                            break;
                        }
                    }
                }
            }
        }
        if(numDone < footInfos.size()){
            failed = true;
            break;
        }
        if(++loopCounter < MAX_ITERATION){
            fkTraverse.calcForwardKinematics();
            c0 = body_->calcCenterOfMass();
        } else {
            break;
        }
    }

    return !failed;
}


bool LeggedBodyHelper::setStance(double width, Link* baseLink)
{
    if(footInfos.size() != 2){
        return false;
    }

    bool result = false;
    Link* foot[2];
    double sign;
    
    if(footInfos[1].link == baseLink){
        foot[0] = footInfos[1].link;
        foot[1] = footInfos[0].link;
        sign = -1.0;
    } else {
        foot[0] = footInfos[0].link;
        foot[1] = footInfos[1].link;
        sign = 1.0;
    }

    Link* waist = body_->rootLink();
        
    auto ikPath = getCustomJointPath(body_, foot[0], waist);

    if(ikPath){
        Position T = waist->T();
        const Matrix3& R0 = foot[0]->R();
        const Vector3 baseY(R0(0,1), sign * R0(1,1), 0.0);
        foot[1]->p() = foot[0]->p() + baseY * width;
        Vector3 wp = (foot[0]->p() + foot[1]->p()) / 2.0;
        T.translation() << wp.x(), wp.y();
        
        if(ikPath->calcInverseKinematics(T)){
            ikPath = getCustomJointPath(body_, waist, foot[1]);
            if(ikPath && ikPath->calcInverseKinematics(foot[1]->T())){
                LinkTraverse fkTraverse(baseLink);
                fkTraverse.calcForwardKinematics();
                result = true;
            }
        }
    }

    return result;
}


Vector3 LeggedBodyHelper::centerOfSole(int footIndex) const
{
    const FootInfo& info = footInfos[footIndex];
    return info.link->p() + info.link->R() * info.soleCenter;
}


Vector3 LeggedBodyHelper::centerOfSoles() const
{
    Vector3 p = Vector3::Zero();
    int n = footInfos.size();
    if(n == 0){
        throw "LeggedBodyHelper::centerOfSoles(): not foot information";
    } else {
        for(int i=0; i < n; ++i){
            const FootInfo& info = footInfos[i];
            p.noalias() += info.link->p() + info.link->R() * info.soleCenter;
        }
    }
    return p / n;
}


Vector3 LeggedBodyHelper::homeCopOfSole(int footIndex) const
{
    const FootInfo& info = footInfos[footIndex];
    return info.link->p() + info.link->R() * info.homeCop;
}


Vector3 LeggedBodyHelper::homeCopOfSoles() const
{
    Vector3 p = Vector3::Zero();
    int n = footInfos.size();
    if(n == 0){
        throw "LeggedBodyHelper::homeCopOfSoles(): not foot information";
    } else {
        for(size_t i=0; i < footInfos.size(); ++i){
            const FootInfo& info = footInfos[i];
            p.noalias() += info.link->p() + info.link->R() * info.homeCop;
        }
    }
    return p / n;
}
