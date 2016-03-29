
#ifndef CNOID_BODY_JACOBIAN_H
#define CNOID_BODY_JACOBIAN_H

#include "Body.h"
#include "Link.h"
#include "JointPath.h"
#include "exportdecl.h"

namespace cnoid {

CNOID_EXPORT void calcCMJacobian(Body* body, Link* base, Eigen::MatrixXd& J);

CNOID_EXPORT void calcAngularMomentumJacobian(Body* body, Link* base, Eigen::MatrixXd& H);

template<int elementMask, int rowOffset, int colOffset, bool useTargetLinkLocalPos>
void setJacobian(const JointPath& path, Link* targetLink, const Vector3& targetLinkLocalPos,
                 MatrixXd& out_J) {

    const bool isTranslationValid = (elementMask & 0x7);

    int n = path.numJoints();
    int i = 0;
    while(i < n){
		
        Link* link = path.joint(i);
        int row = rowOffset;

        MatrixXd::ColXpr Ji = out_J.col(i + colOffset);
                
        switch(link->jointType()){
                
        case Link::ROTATIONAL_JOINT:
        {
            Vector3 omega = link->R() * link->a();
            if(!path.isJointDownward(i)){
                omega = -omega;
            }
            if(isTranslationValid){
                Vector3 arm;
                if(useTargetLinkLocalPos){
                    arm = targetLink->p() + targetLink->R() * targetLinkLocalPos - link->p();
                } else {
                    arm = targetLink->p() - link->p();
                }
                const Vector3 dp = omega.cross(arm);
                if(elementMask & 0x1) Ji(row++) = dp.x();
                if(elementMask & 0x2) Ji(row++) = dp.y();
                if(elementMask & 0x4) Ji(row++) = dp.z();
            }
            if(elementMask & 0x8)  Ji(row++) = omega.x();
            if(elementMask & 0x10) Ji(row++) = omega.y();
            if(elementMask & 0x20) Ji(row  ) = omega.z();
        }
        break;
            
        case Link::SLIDE_JOINT:
        {
            if(isTranslationValid){
                Vector3 dp = link->R() * link->d();
                if(!path.isJointDownward(i)){
                    dp = -dp;
                }
                if(elementMask & 0x1) Ji(row++) = dp.x();
                if(elementMask & 0x2) Ji(row++) = dp.y();
                if(elementMask & 0x4) Ji(row++) = dp.z();
            }
            if(elementMask & 0x8)  Ji(row++) = 0.0;
            if(elementMask & 0x10) Ji(row++) = 0.0;
            if(elementMask & 0x20) Ji(row  ) = 0.0;
        }
        break;
            
        default:
            if(elementMask & 0x1)  Ji(row++) = 0.0;
            if(elementMask & 0x2)  Ji(row++) = 0.0;
            if(elementMask & 0x4)  Ji(row++) = 0.0;
            if(elementMask & 0x8)  Ji(row++) = 0.0;
            if(elementMask & 0x10) Ji(row++) = 0.0;
            if(elementMask & 0x20) Ji(row  ) = 0.0;
        }

        ++i;
            
        if(link == targetLink){
            break;
        }
    }

    while(i < n){
        MatrixXd::ColXpr Ji = out_J.col(i + colOffset);
        int row = rowOffset;
        if(elementMask & 0x1)  Ji(row++) = 0.0;
        if(elementMask & 0x2)  Ji(row++) = 0.0;
        if(elementMask & 0x4)  Ji(row++) = 0.0;
        if(elementMask & 0x8)  Ji(row++) = 0.0;
        if(elementMask & 0x10) Ji(row++) = 0.0;
        if(elementMask & 0x20) Ji(row  ) = 0.0;
        ++i;
    }
}

template<int elementMask, int rowOffset, int colOffset>
void setJacobian(const JointPath& path, Link* targetLink, MatrixXd& out_J) {
    static const Vector3 targetLinkLocalPos(Vector3::Zero());
    setJacobian<elementMask, rowOffset, colOffset, false>(
        path, targetLink, targetLinkLocalPos, out_J);
}

}

#endif
