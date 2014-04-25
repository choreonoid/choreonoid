
#include "Jacobian.h"
#include "Link.h"
#include "JointPath.h"
#include <iostream>

using namespace std;
using namespace cnoid;

namespace {

struct SubMass
{
    double m;
    Vector3 mwc;
    SubMass& operator+=(const SubMass& rhs){
        m += rhs.m;
        mwc += rhs.mwc;
        return *this;
    }
};

void calcSubMass(Link* link, vector<SubMass>& subMasses)
{
    SubMass& sub = subMasses[link->index()];
    sub.m = link->m();
    sub.mwc = link->m() * link->wc();

    for(Link* child = link->child(); child; child = child->sibling()){
        calcSubMass(child, subMasses);
        SubMass& childSub = subMasses[child->index()];
        sub.m += childSub.m;
        sub.mwc += childSub.mwc;
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
void calcCMJacobian(const BodyPtr& body, Link* base, Eigen::MatrixXd& J)
{
    // prepare subm, submwc

    const int nj = body->numJoints();
    vector<SubMass> subMasses(body->numLinks());
    Link* rootLink = body->rootLink();
        
    JointPath path;
    if(!base){
        calcSubMass(rootLink, subMasses);
        J.resize(3, nj + 6);

    } else {
        path.setPath(rootLink, base);
        Link* skip = path.joint(0);
        SubMass& sub = subMasses[skip->index()];
        sub.m = rootLink->m();
        sub.mwc = rootLink->m() * rootLink->wc();
            
        for(Link* child = rootLink->child(); child; child = child->sibling()){
            if(child != skip){
                calcSubMass(child, subMasses);
                subMasses[skip->index()] += subMasses[child->index()];
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
            
        J.resize(3, nj);
    }
        
    // compute Jacobian
    std::vector<int> sgn(nj, 1);
    for(int i=0; i < path.numJoints(); i++){
        sgn[path.joint(i)->jointId()] = -1;
    }
    
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
}
