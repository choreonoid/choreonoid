
#include "Jacobian.h"
#include "Link.h"
#include "JointPath.h"
#include <iostream>

using namespace std;
using namespace cnoid;

namespace {

Matrix3d D(Vector3d r)
{
    Matrix3d r_cross;
    r_cross <<
        0.0,  -r(2), r(1),
        r(2),    0.0,  -r(0),
        -r(1), r(0),    0.0;
    return r_cross.transpose() * r_cross;
}

struct SubMass
{
    double m;
    Vector3 mwc;
    Matrix3d Iw;
    SubMass& operator+=(const SubMass& rhs){
        m += rhs.m;
        mwc += rhs.mwc;
        Iw += rhs.Iw;
        return *this;
    }
};

void calcSubMass(Link* link, vector<SubMass>& subMasses, bool calcIw)
{
    Matrix3d R = link->R();
    SubMass& sub = subMasses[link->index()];
    sub.m = link->m();
    sub.mwc = link->m() * link->wc();

    for(Link* child = link->child(); child; child = child->sibling()){
        calcSubMass(child, subMasses, calcIw);
        SubMass& childSub = subMasses[child->index()];
        sub.m += childSub.m;
        sub.mwc += childSub.mwc;
    }
    
    if(calcIw){
        sub.Iw = R * link->I() * R.transpose() + link->m() * D( link->wc() - sub.mwc/sub.m );
        for(Link* child = link->child(); child; child = child->sibling()){
            SubMass& childSub = subMasses[child->index()];
            sub.Iw += childSub.Iw + childSub.m * D( childSub.mwc/childSub.m - sub.mwc/sub.m );
        }
    }
}

}

namespace cnoid {

/**
   @brief compute CoM Jacobian
   @param base link fixed to the environment
   @param J CoM Jacobian
   @note Link::wc must be computed by calcCM() before calling
*/
void calcCMJacobian(Body* body, Link* base, Eigen::MatrixXd& J)
{
    // prepare subm, submwc

    const int nj = body->numJoints();
    vector<SubMass> subMasses(body->numLinks());
    Link* rootLink = body->rootLink();
    std::vector<int> sgn(nj, 1);
        
    if(!base){
        calcSubMass(rootLink, subMasses, false);
        J.resize(3, nj + 6);

    } else {
        JointPath path(rootLink, base);
        Link* skip = path.joint(0);
        SubMass& sub = subMasses[skip->index()];
        sub.m = rootLink->m();
        sub.mwc = rootLink->m() * rootLink->wc();
            
        for(Link* child = rootLink->child(); child; child = child->sibling()){
            if(child != skip){
                calcSubMass(child, subMasses, false);
                const SubMass& c = subMasses[child->index()];
                sub.m += c.m;
                sub.mwc += c.mwc;
            }
        }
            
        // assuming there is no branch between base and root
        for(int i=1; i < path.numJoints(); i++){
            Link* joint = path.joint(i);
            const Link* parent = joint->parent();
            SubMass& sub = subMasses[joint->index()];
            sub.m = parent->m();
            sub.mwc = parent->m() * parent->wc();
            const SubMass& p = subMasses[parent->index()];
            sub.m += p.m;
            sub.mwc += p.mwc;
        }
            
        J.resize(3, nj);

        for(int i=0; i < path.numJoints(); i++){
            sgn[path.joint(i)->jointId()] = -1;
        }
    }
        
    // compute Jacobian
    for(int i=0; i < nj; i++){
        Link* joint = body->joint(i);
        if(joint->isRotationalJoint()){
            const Vector3 omega = sgn[joint->jointId()] * joint->R() * joint->a();
            const SubMass& sub = subMasses[joint->index()];
            const Vector3 arm = (sub.mwc - sub.m * joint->p()) / body->mass();
            const Vector3 dp = omega.cross(arm);
            J.col(joint->jointId()) = dp;
        } else {
            std::cerr << "calcCMJacobian() : unsupported jointType("
                      << joint->jointType() << std::endl;
        }
    }

    if(!base){
        const int c = nj;
        J.block(0, c, 3, 3).setIdentity();

        const Vector3 dp = subMasses[0].mwc / body->mass() - rootLink->p();

        J.block(0, c + 3, 3, 3) <<
            0.0,  dp(2), -dp(1),
            -dp(2),    0.0,  dp(0),
            dp(1), -dp(0),    0.0;
    }
}

/**
   @brief compute Angular Momentum Jacobian
   @param base link fixed to the environment
   @param H Angular Momentum Jacobian
   @note Link::wc must be computed by calcCM() before calling
*/
void calcAngularMomentumJacobian(Body* body, Link* base, Eigen::MatrixXd& H)
{

    // prepare subm, submwc

    const int nj = body->numJoints();
    std::vector<SubMass> subMasses(body->numLinks());
    Link* rootLink = body->rootLink();
    std::vector<int> sgn(nj, 1);

    MatrixXd M;
    calcCMJacobian( body, base, M );
    M.conservativeResize(3, nj);
    M *= body->mass();

    if(!base){
        calcSubMass(rootLink, subMasses, true);
        H.resize(3, nj + 6);

    } else {
        JointPath path(rootLink, base);
        Link* skip = path.joint(0);
        SubMass& sub = subMasses[skip->index()];
        sub.m = rootLink->m();
        sub.mwc = rootLink->m() * rootLink->wc();
            
        for(Link* child = rootLink->child(); child; child = child->sibling()){
            if(child != skip){
                calcSubMass(child, subMasses, true);
                sub += subMasses[child->index()];
            }
        }
            
        // assuming there is no branch between base and root
        for(int i=1; i < path.numJoints(); i++){
            Link* joint = path.joint(i);
            const Link* parent = joint->parent();
            SubMass& sub = subMasses[joint->index()];
            sub.m = parent->m();
            sub.mwc = parent->m() * parent->wc();
            sub += subMasses[parent->index()];
        }
            
        H.resize(3, nj);

        for(int i=0; i < path.numJoints(); i++){
            sgn[path.joint(i)->jointId()] = -1;
        }
    }

    // compute Jacobian
    for(int i=0; i < nj; ++i){
        Link* joint = body->joint(i);
        if(joint->isRotationalJoint()){
            const Vector3 omega = sgn[joint->jointId()] * joint->R() * joint->a();
            const SubMass& sub = subMasses[joint->index()];
            const Vector3 Mcol = M.col(joint->jointId());
            const Vector3 dp = (sub.mwc/sub.m).cross(Mcol) + sub.Iw * omega;
            H.col(joint->jointId()) = dp;
        } else {
            std::cerr << "calcAngularMomentumJacobian() : unsupported jointType("
                      << joint->jointType() << std::endl;
        }
    }


    if(!base){
        const int c = nj;
        H.block(0, c, 3, 3).setIdentity();

        Vector3 cm = body->calcCenterOfMass();
        Matrix3d cm_cross;
        cm_cross <<
            0.0,  -cm(2), cm(1),
            cm(2),    0.0,  -cm(0),
            -cm(1), cm(0),    0.0;
        H -= cm_cross * M;

    }
    
}

}
