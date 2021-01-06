/**
   \file
   \author Shin'ichiro Nakaoka
*/

#include "ForwardDynamicsABM.h"
#include "DyBody.h"
#include <cnoid/EigenUtil>

using namespace std;
using namespace cnoid;

static const bool debugMode = false;
static const bool rootAttitudeNormalizationEnabled = false;


ForwardDynamicsABM::ForwardDynamicsABM(DySubBody* subBody) :
    ForwardDynamics(subBody),
    q0(subBody->numLinks()),
    dq0(subBody->numLinks()),
    dq(subBody->numLinks()),
    ddq(subBody->numLinks())
{

}


ForwardDynamicsABM::~ForwardDynamicsABM()
{

}


void ForwardDynamicsABM::initialize()
{
    auto root = subBody->rootLink();
    
    root->sw().setZero();
    root->sv().setZero();
    root->cv().setZero();
    root->cw().setZero();
    root->hhv().setZero();
    root->hhw().setZero();
    root->uu() = 0.0;
    root->dd() = 0.0;

    initializeSensors();
    calcABMFirstHalf();
}


inline void ForwardDynamicsABM::calcABMFirstHalf()
{
    calcABMPhase1(true);
    calcABMPhase2Part1();
}


inline void ForwardDynamicsABM::calcABMLastHalf()
{
    calcABMPhase2Part2();
    calcABMPhase3();
}


void ForwardDynamicsABM::refreshState()
{
    calcABMFirstHalf();
}


void ForwardDynamicsABM::calcNextState()
{
    switch(integrationMode){

    case EULER_METHOD:
        calcMotionWithEulerMethod();
        break;
		
    case RUNGEKUTTA_METHOD:
        calcMotionWithRungeKuttaMethod();
        break;
    }

    if(rootAttitudeNormalizationEnabled){
        normalizeRotation(subBody->rootLink()->T());
    }

    calcABMFirstHalf();

    if(sensorsEnabled){
        sensorHelper.updateGyroAndAccelerationSensors();
    }
}


void ForwardDynamicsABM::calcMotionWithEulerMethod()
{
    calcABMLastHalf();

    if(!subBody->forceSensors().empty()){
        updateForceSensors();
    }

    auto root = subBody->rootLink();
    
    if(!root->isFixedJoint()){
        root->dv() =
            root->dvo() - root->p().cross(root->dw())
            + root->w().cross(root->vo() + root->w().cross(root->p()));
        Isometry3 T;
        SE3exp(T, root->T(), root->w(), root->vo(), timeStep);
        root->T() = T;
        root->vo() += root->dvo() * timeStep;
        root->w()  += root->dw()  * timeStep;
    }

    int n = subBody->numLinks();
    for(int i=1; i < n; ++i){
        DyLink* link = subBody->link(i);
        link->q()  += link->dq()  * timeStep;
        link->dq() += link->ddq() * timeStep;
    }
}


void ForwardDynamicsABM::integrateRungeKuttaOneStep(double r, double dt)
{
    auto root = subBody->rootLink();

    if(!root->isFixedJoint()){

        SE3exp(root->T(), T0, root->w(), root->vo(), dt);
        root->vo().noalias() = vo0 + root->dvo() * dt;
        root->w().noalias()  = w0  + root->dw()  * dt;

        vo  += r * root->vo();
        w   += r * root->w();
        dvo += r * root->dvo();
        dw  += r * root->dw();
    }

    int n = subBody->numLinks();
    for(int i=1; i < n; ++i){

        DyLink* link = subBody->link(i);

        link->q()  =  q0[i] + dt * link->dq();
        link->dq() = dq0[i] + dt * link->ddq();

        dq[i]  += r * link->dq();
        ddq[i] += r * link->ddq();
    }
}


void ForwardDynamicsABM::calcMotionWithRungeKuttaMethod()
{
    auto root = subBody->rootLink();

    if(!root->isFixedJoint()){
        T0 = root->T();
        vo0 = root->vo();
        w0  = root->w();
    }

    vo.setZero();
    w.setZero();
    dvo.setZero();
    dw.setZero();

    int n = subBody->numLinks();
    for(int i=1; i < n; ++i){
        auto link = subBody->link(i);
        q0[i]  = link->q();
        dq0[i] = link->dq();
        dq[i]  = 0.0;
        ddq[i] = 0.0;
    }

    calcABMLastHalf();

    if(!subBody->forceSensors().empty()){
        updateForceSensors();
    }

    integrateRungeKuttaOneStep(1.0 / 6.0, timeStep / 2.0);
    calcABMPhase1(false);
    calcABMPhase2();
    calcABMPhase3();

    integrateRungeKuttaOneStep(2.0 / 6.0, timeStep / 2.0);
    calcABMPhase1(false);
    calcABMPhase2();
    calcABMPhase3();

    integrateRungeKuttaOneStep(2.0 / 6.0, timeStep);
    calcABMPhase1(false);
    calcABMPhase2();
    calcABMPhase3();

    if(!root->isFixedJoint()){
        root->dvo() = dvo + root->dvo() / 6.0;
        root->dw() = dw  + root->dw()  / 6.0;
        root->dv() =
            root->dvo() - T0.translation().cross(root->dw()) + w0.cross(vo0 + w0.cross(T0.translation()));
        root->vo() = vo0 + root->dvo() * timeStep;
        root->w()  = w0  + root->dw() * timeStep;
        SE3exp(root->T(), T0, w0, vo0, timeStep);
    }

    for(int i=1; i < n; ++i){
        auto link = subBody->link(i);
        link->q()  =  q0[i] + ( dq[i] + link->dq()  / 6.0) * timeStep;
        link->dq() = dq0[i] + (ddq[i] + link->ddq() / 6.0) * timeStep;
    }
}


/**
   \note v, dv, dw are not used in the forward dynamics, but are calculated
   for forward dynamics users.
*/
void ForwardDynamicsABM::calcABMPhase1(bool updateNonSpatialVariables)
{
    const int n = subBody->numLinks();

    for(int i=0; i < n; ++i){
        auto link = subBody->link(i);
        DyLink* parent;
        if(i == 0 && link->isFreeJoint()){
            parent = nullptr;
        } else {
            parent = link->parent();
        }

        if(parent){
            
            switch(link->jointType()){
                
            case Link::ROTATIONAL_JOINT:
            {
                const Vector3 arm = parent->R() * link->b();
                link->R().noalias() = parent->R() * link->Rb() * AngleAxisd(link->q(), link->a());
                link->p().noalias() = arm + parent->p();
                link->sw().noalias() = parent->R() * (link->Rb() * link->a());
                link->sv().noalias() = link->p().cross(link->sw());
                link->w().noalias() = link->dq() * link->sw() + parent->w();
                if(updateNonSpatialVariables){
                    link->dw().noalias() =
                        parent->dw() + link->dq() * parent->w().cross(link->sw()) + (link->ddq() * link->sw());
                    link->dv().noalias() =
                        parent->dv() + parent->w().cross(parent->w().cross(arm)) + parent->dw().cross(arm);
                }
                break;
            }
                
            case Link::SLIDE_JOINT:
                link->p().noalias() = parent->R() * (link->b() + link->Rb() * (link->q() * link->d())) + parent->p();
                link->R().noalias() = parent->R() * link->Rb();
                link->sw().setZero();
                link->sv().noalias() = parent->R() * (link->Rb() * link->d());
                link->w() = parent->w();
                if(updateNonSpatialVariables){
                    link->dw() = parent->dw();
                    const Vector3 arm = parent->R() * link->b();
                    link->dv().noalias() =
                        parent->dv() + parent->w().cross(parent->w().cross(arm)) + parent->dw().cross(arm)
                        + 2.0 * link->dq() * parent->w().cross(link->sv()) + link->ddq() * link->sv();
                }
                break;
                
            case Link::FIXED_JOINT:
            default:
                link->p().noalias() = parent->R() * link->b() + parent->p();
                link->R().noalias() = parent->R() * link->Rb();
                link->w() = parent->w();
                link->vo() = parent->vo();
                link->sw().setZero();
                link->sv().setZero();
                link->cv().setZero();
                link->cw().setZero();
                if(updateNonSpatialVariables){
                    link->dw() = parent->dw();
                    const Vector3 arm = parent->R() * link->b();
                    link->dv().noalias() = parent->dv() +
                        parent->w().cross(parent->w().cross(arm)) + parent->dw().cross(arm);
                }
                
                goto COMMON_CALCS_FOR_ALL_JOINT_TYPES;
            }
            
            // Common for ROTATE and SLIDE
            link->vo().noalias() = link->dq() * link->sv() + parent->vo();
            const Vector3 dsv = parent->w().cross(link->sv()) + parent->vo().cross(link->sw());
            const Vector3 dsw = parent->w().cross(link->sw());
            link->cv().noalias() = link->dq() * dsv;
            link->cw().noalias() = link->dq() * dsw;
        }
        
COMMON_CALCS_FOR_ALL_JOINT_TYPES:

        if(updateNonSpatialVariables){
            link->v().noalias() = link->vo() + link->w().cross(link->p());
        }
        link->wc().noalias() = link->R() * link->c() + link->p();
        
        // compute I^s (Eq.(6.24) of Kajita's textbook))
        const Matrix3 Iw = link->R() * link->I() * link->R().transpose();
        
        const double m = link->m();
        const Matrix3 c_hat = hat(link->wc());
        link->Iww().noalias() = m * c_hat * c_hat.transpose() + Iw;

        link->Ivv() <<
            m,  0.0, 0.0,
            0.0,  m,  0.0,
            0.0, 0.0,  m;
        
        link->Iwv() = m * c_hat;
        
        // compute P and L (Eq.(6.25) of Kajita's textbook)
        const Vector3 P = m * (link->vo() + link->w().cross(link->wc()));
        const Vector3 L = link->Iww() * link->w() + m * link->wc().cross(link->vo());
        
        link->pf().noalias() = link->w().cross(P);
        link->ptau().noalias() = link->vo().cross(P) + link->w().cross(L);
        
        const Vector3 fg = m * g;
        const Vector3 tg = link->wc().cross(fg);
        
        link->pf() -= fg;
        link->ptau() -= tg;
    }
}


void ForwardDynamicsABM::calcABMPhase2()
{
    const int n = subBody->numLinks();

    for(int i = n-1; i >= 0; --i){
        auto link = subBody->link(i);

        link->pf()   -= link->f_ext();
        link->ptau() -= link->tau_ext();

        // compute articulated inertia (Eq.(6.48) of Kajita's textbook)
        for(DyLink* child = link->child(); child; child = child->sibling()){
            if(child->isFreeJoint()){
                continue;
            } else if(child->isFixedJoint()){
                link->Ivv() += child->Ivv();
                link->Iwv() += child->Iwv();
                link->Iww() += child->Iww();

            } else {
                const Vector3 hhv_dd = child->hhv() / child->dd();
                link->Ivv().noalias() += child->Ivv() - child->hhv() * hhv_dd.transpose();
                link->Iwv().noalias() += child->Iwv() - child->hhw() * hhv_dd.transpose();
                link->Iww().noalias() += child->Iww() - child->hhw() * (child->hhw() / child->dd()).transpose();
            }
            link->pf()  .noalias() += child->Ivv() * child->cv() + child->Iwv().transpose() * child->cw() + child->pf();
            link->ptau().noalias() += child->Iwv() * child->cv() + child->Iww() * child->cw() + child->ptau();

            if(!child->isFixedJoint()){
                const double uu_dd = child->uu() / child->dd();
                link->pf()   += uu_dd * child->hhv();
                link->ptau() += uu_dd * child->hhw();
            }
        }

        if(i > 0){
            if(!link->isFixedJoint()){
                // hh = Ia * s
                link->hhv().noalias() = link->Ivv() * link->sv() + link->Iwv().transpose() * link->sw();
                link->hhw().noalias() = link->Iwv() * link->sv() + link->Iww() * link->sw();
                // dd = Ia * s * s^T
                link->dd() = link->sv().dot(link->hhv()) + link->sw().dot(link->hhw()) + link->Jm2();
                // uu = u - hh^T*c + s^T*pp
                link->uu() = link->u() -
                    (link->hhv().dot(link->cv()) + link->hhw().dot(link->cw()) +
                     link->sv().dot(link->pf()) + link->sw().dot(link->ptau()));
            }
        }
    }
}


// A part of phase 2 (inbound loop) that can be calculated before external forces are given
void ForwardDynamicsABM::calcABMPhase2Part1()
{
    const int n = subBody->numLinks();

    for(int i = n-1; i >= 0; --i){
        auto link = subBody->link(i);
        for(DyLink* child = link->child(); child; child = child->sibling()){
            if(child->isFreeJoint()){
                continue;
            } else if(child->isFixedJoint()){
                link->Ivv() += child->Ivv();
                link->Iwv() += child->Iwv();
                link->Iww() += child->Iww();
            } else {
                const Vector3 hhv_dd = child->hhv() / child->dd();
                link->Ivv().noalias() += child->Ivv() - child->hhv() * hhv_dd.transpose();
                link->Iwv().noalias() += child->Iwv() - child->hhw() * hhv_dd.transpose();
                link->Iww().noalias() += child->Iww() - child->hhw() * (child->hhw() / child->dd()).transpose();
            }
            link->pf()  .noalias() += child->Ivv() * child->cv() + child->Iwv().transpose() * child->cw();
            link->ptau().noalias() += child->Iwv() * child->cv() + child->Iww() * child->cw();
        }

        if(i > 0){
            if(!link->isFixedJoint()){
                link->hhv().noalias() = link->Ivv() * link->sv() + link->Iwv().transpose() * link->sw();
                link->hhw().noalias() = link->Iwv() * link->sv() + link->Iww() * link->sw();
                link->dd() = link->sv().dot(link->hhv()) + link->sw().dot(link->hhw()) + link->Jm2();
                link->uu() = -(link->hhv().dot(link->cv()) + link->hhw().dot(link->cw()));
            }
        }
    }
}


// A remaining part of phase 2 that requires external forces
void ForwardDynamicsABM::calcABMPhase2Part2()
{
    const int n = subBody->numLinks();

    for(int i = n-1; i >= 0; --i){
        auto link = subBody->link(i);

        link->pf()   -= link->f_ext();
        link->ptau() -= link->tau_ext();

        for(DyLink* child = link->child(); child; child = child->sibling()){
            if(child->isFreeJoint()){
                continue;
            }
            link->pf()   += child->pf();
            link->ptau() += child->ptau();

            if(!child->isFixedJoint()){
                const double uu_dd = child->uu() / child->dd();
                link->pf()   += uu_dd * child->hhv();
                link->ptau() += uu_dd * child->hhw();
            }
        }

        if(i > 0){
            if(!link->isFixedJoint()){
                link->uu() += link->u() - (link->sv().dot(link->pf()) + link->sw().dot(link->ptau()));
            }
        }
    }
}


void ForwardDynamicsABM::calcABMPhase3()
{
    auto root = subBody->rootLink();
    if(!root->isFreeJoint()){
        root->dvo().setZero();
        root->dw().setZero();
    } else {
        // - | Ivv  trans(Iwv) | * | dvo | = | pf   |
        //   | Iwv     Iww     |   | dw  |   | ptau |

        Eigen::Matrix<double, 6, 6> M;
        M << root->Ivv(), root->Iwv().transpose(),
            root->Iwv(), root->Iww();
        
        Eigen::Matrix<double, 6, 1> f;
        f << root->pf(),
            root->ptau();
        f *= -1.0;

        Eigen::Matrix<double, 6, 1> a(M.colPivHouseholderQr().solve(f));

        root->dvo() = a.head<3>();
        root->dw() = a.tail<3>();
    }

    const int n = subBody->numLinks();
    for(int i=1; i < n; ++i){
        auto link = subBody->link(i);
        auto parent = link->parent();
        if(link->isFixedJoint()){
            link->ddq() = 0.0;
            link->dvo() = parent->dvo();
            link->dw()  = parent->dw(); 
        } else {
            link->ddq() = (link->uu() - (link->hhv().dot(parent->dvo()) + link->hhw().dot(parent->dw()))) / link->dd();
            link->dvo().noalias() = parent->dvo() + link->cv() + link->sv() * link->ddq();
            link->dw().noalias()  = parent->dw()  + link->cw() + link->sw() * link->ddq();
        }
    }
}


void ForwardDynamicsABM::updateForceSensors()
{
    for(auto& sensor : subBody->forceSensors()){
        const DyLink* link = static_cast<DyLink*>(sensor->link());

        //    | f   | = | Ivv  trans(Iwv) | * | dvo | + | pf   |
        //    | tau |   | Iwv     Iww     |   | dw  |   | ptau |
        const Vector3 f = -(link->Ivv() * link->dvo() + link->Iwv().transpose() * link->dw() + link->pf());
        const Vector3 tau = -(link->Iwv() * link->dvo() + link->Iww() * link->dw() + link->ptau());
        
        const Matrix3 R = link->R() * sensor->R_local();
        const Vector3 p = link->p() + link->R() * sensor->p_local();
        sensor->f().noalias()   = R.transpose() * f;
        sensor->tau().noalias() = R.transpose() * (tau - p.cross(f));
        sensor->notifyStateChange();
    }
}
