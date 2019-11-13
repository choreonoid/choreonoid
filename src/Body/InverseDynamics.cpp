/**
   @author Shin'ichiro Nakaoka
*/

#include "InverseDynamics.h"
#include "Link.h"
#include <cnoid/EigenUtil>

using namespace cnoid;

namespace {

/**
   see Kajita et al. Humanoid Robot Ohm-sha,  p.210
*/
Vector6 calcInverseDynamicsSub(Link* link, const Vector3& vo_parent, const Vector3& dvo_parent)
{
    Vector3 dvo, sv, sw;

    Link* parent = link->parent();
    if(!parent){
        dvo = link->dv() - link->dw().cross(link->p()) - link->w().cross(link->v());
        sv.setZero();
        sw.setZero();

    } else {
        switch(link->jointType()){
        case Link::ROTATIONAL_JOINT:
            sw.noalias() = link->R() * link->a();
            sv.noalias() = link->p().cross(sw);
            break;
        case Link::SLIDE_JOINT:
            sw.setZero();
            sv.noalias() = link->R() * link->d();
            break;
        case Link::FIXED_JOINT:
        default:
            sw.setZero();
            sv.setZero();
            break;
        }
        const Vector3 dsv = parent->w().cross(sv) + vo_parent.cross(sw);
        const Vector3 dsw = parent->w().cross(sw);

        link->dw() = parent->dw() + dsw * link->dq() + sw * link->ddq();
        dvo = dvo_parent + dsv * link->dq() + sv * link->ddq();
    }

    const Vector3 c = link->R() * link->c() + link->p();
    Matrix3 I = link->R() * link->I() * link->R().transpose();
    const Matrix3 c_hat = hat(c);
    I.noalias() += link->m() * c_hat * c_hat.transpose();
    const Vector3 vo = link->v() - link->w().cross(link->p());
    const Vector3 P = link->m() * (vo + link->w().cross(c));
    const Vector3 L = link->m() * c.cross(vo) + I * link->w();

    Vector6 f;
    f.head<3>() = link->m() * (dvo + link->dw().cross(c)) + link->w().cross(P);
    f.tail<3>() = link->m() * c.cross(dvo) + I * link->dw() + vo.cross(P) + link->w().cross(L);

    for(Link* child = link->child(); child; child = child->sibling()){
        f += calcInverseDynamicsSub(child, vo, dvo);
    }
    f -= link->F_ext();
    
    link->u() = sv.dot(f.head<3>()) + sw.dot(f.tail<3>()) + link->ddq() * link->Jm2() /* rotor inertia */;

    return f;
}

}
    
namespace cnoid {

Vector6 calcInverseDynamics(Link* link)
{
    // spatial velocity and acceleration of the parent link
    Vector3 vo, dvo;

    Link* parent = link->parent();
        
    if(!parent){
        vo.setZero();
        dvo.setZero();
    } else {
        vo = parent->v() - parent->w().cross(parent->p());
        dvo = parent->dv() - parent->dw().cross(parent->p()) - parent->w().cross(parent->v());
    }
    return calcInverseDynamicsSub(link, vo, dvo);
}

}
