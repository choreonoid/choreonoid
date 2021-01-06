/**
   \file
   \author Shin'ichiro Nakaoka
*/

#include "ForwardDynamicsCBM.h"
#include "DyBody.h"
#include <cnoid/EigenUtil>
#include <iostream>

using namespace std;
using namespace cnoid;

static const bool CALC_ALL_JOINT_TORQUES = false;
static const bool ROOT_ATT_NORMALIZATION_ENABLED = false;

static const bool debugMode = false;

#include <fmt/format.h>

template<class TMatrix>
static void putMatrix(TMatrix& M, char* name)
{
    if(M.cols() == 1){
        std::cout << "Vector " << name << M << std::endl;
    } else {
        std::cout << "Matrix " << name << ": \n";
        for(size_t i=0; i < M.rows(); i++){
            for(size_t j=0; j < M.cols(); j++){
                std::cout << fmt::format(" {:6.3f} ", M(i, j));
            }
            std::cout << std::endl;
        }
    }
}

template<class TVector>
static void putVector(TVector& M, char* name)
{
    std::cout << "Vector " << name << M << std::endl;
}


ForwardDynamicsCBM::ForwardDynamicsCBM(DySubBody* subBody) :
    ForwardDynamics(subBody)
{

}


ForwardDynamicsCBM::~ForwardDynamicsCBM()
{

}


void ForwardDynamicsCBM::initialize()
{
    auto root = subBody->rootLink();
    bool isRootActuated = root->actuationMode() == Link::LinkPosition;
    unknown_rootDof = (root->isFreeJoint() && !isRootActuated) ? 6 : 0;
    given_rootDof = (root->isFreeJoint() && isRootActuated) ? 6 : 0;

    const int numLinks = subBody->numLinks();
    torqueModeJoints.clear();
    highGainModeJoints.clear();

    for(int i=1; i < numLinks; ++i){
        DyLink* link = subBody->link(i);
        if(link->isRevoluteJoint() || link->isPrismaticJoint()){
            if(link->actuationMode() == Link::JointDisplacement ||
               link->actuationMode() == Link::JointVelocity){
                highGainModeJoints.push_back(link);
            } else {
                torqueModeJoints.push_back(link);
            }
        }
    }

    const int n = unknown_rootDof + torqueModeJoints.size();
    const int m = highGainModeJoints.size();
    
    isNoUnknownAccelMode = (n == 0);

    M11.resize(n, n);
    M12.resize(n, given_rootDof + m);
    b1. resize(n, 1);
    c1. resize(n);
    d1. resize(n);

    qGiven.  resize(m);
    dqGiven. resize(m);
    ddqGiven.resize(given_rootDof + m);

    qGivenPrev. resize(m);
    dqGivenPrev.resize(m);

    ddqorg.resize(numLinks);
    uorg.  resize(numLinks);

    calcPositionAndVelocityFK();

    if(!isNoUnknownAccelMode){
        calcMassMatrix();
    }

    ddqGivenCopied = false;

    initializeSensors();

    if(integrationMode == RUNGEKUTTA_METHOD){
        q0. resize(numLinks);
        dq0.resize(numLinks);
        dq. resize(numLinks);
        ddq.resize(numLinks);
    }

    preserveHighGainModeJointState();
}


void ForwardDynamicsCBM::complementHighGainModeCommandValues()
{
    for(size_t i=0; i < highGainModeJoints.size(); ++i){
        Link* joint = highGainModeJoints[i];
        if(joint->actuationMode() == Link::JointDisplacement){
            joint->q() = joint->q_target();
            joint->dq() = (joint->q() - qGivenPrev[i]) / timeStep;
            joint->ddq() = (joint->dq() - dqGivenPrev[i]) / timeStep;
        } else if(joint->actuationMode() == Link::JointVelocity){
            joint->dq() = joint->dq_target();
            joint->q() = joint->q() + joint->dq() * timeStep;
            joint->ddq() = (joint->dq() - dqGivenPrev[i]) / timeStep;
        }
    }
}


void ForwardDynamicsCBM::solveUnknownAccels()
{
    if(!isNoUnknownAccelMode){
        initializeAccelSolver();
        solveUnknownAccels(fextTotal, tauextTotal);
    }
}


inline void ForwardDynamicsCBM::calcAccelFKandForceSensorValues()
{
    Vector3 f, tau;
    calcAccelFKandForceSensorValues(subBody->rootLink(), f, tau, true);
}


void ForwardDynamicsCBM::refreshState()
{

}


void ForwardDynamicsCBM::calcNextState()
{
    complementHighGainModeCommandValues();
    
    if(isNoUnknownAccelMode && !sensorHelper.isActive()){

        calcPositionAndVelocityFK();

    } else {

        switch(integrationMode){

        case EULER_METHOD:
            calcMotionWithEulerMethod();
            break;
			
        case RUNGEKUTTA_METHOD:
            calcMotionWithRungeKuttaMethod();
            break;
        }
		
        if(ROOT_ATT_NORMALIZATION_ENABLED && unknown_rootDof){
            normalizeRotation(subBody->rootLink()->T());
        }

        preserveHighGainModeJointState();
    }
		
    if(sensorHelper.isActive()){
        updateForceSensors();
        sensorHelper.updateGyroAndAccelerationSensors();
    }
	
    ddqGivenCopied = false;
}


void ForwardDynamicsCBM::calcMotionWithEulerMethod()
{
    sumExternalForces();
    solveUnknownAccels();
    calcAccelFKandForceSensorValues();

    DyLink* root = subBody->rootLink();

    if(unknown_rootDof){
        Isometry3 T;
        SE3exp(T, root->T(), root->w(), root->vo(), timeStep);
        root->T() = T;
        root->vo() += root->dvo() * timeStep;
        root->w()  += root->dw()  * timeStep;
    }

    const int n = torqueModeJoints.size();
    for(int i=0; i < n; ++i){
        DyLink* link = torqueModeJoints[i];
        link->q()  += link->dq()  * timeStep;
        link->dq() += link->ddq() * timeStep;
    }

    calcPositionAndVelocityFK();
    calcMassMatrix();
}


void ForwardDynamicsCBM::calcMotionWithRungeKuttaMethod()
{
    const int numHighGainJoints = highGainModeJoints.size();
    DyLink* root = subBody->rootLink();

    if(given_rootDof){
        pGiven = root->p();
        RGiven = root->R();
        voGiven = root->vo();
        wGiven = root->w();
        root->p() = pGivenPrev;
        root->R() = RGivenPrev;
        root->vo() = voGivenPrev;
        root->w() = wGivenPrev;
    }

    for(int i=0; i < numHighGainJoints; ++i){
        DyLink* link = highGainModeJoints[i];
        qGiven [i] = link->q();
        dqGiven[i] = link->dq();
        link->q()  = qGivenPrev[i];
        link->dq() = dqGivenPrev[i];
    }
  
    if(unknown_rootDof || given_rootDof){
        T0  = root->T();
        vo0 = root->vo();
        w0  = root->w();
    }

    vo.setZero();
    w.setZero();
    dvo.setZero();
    dw.setZero();

    const int numLinks = subBody->numLinks();
    for(int i=1; i < numLinks; ++i){
        const DyLink* link = subBody->link(i);
        q0 [i] = link->q();
        dq0[i] = link->dq();
        dq [i] = 0.0;
        ddq[i] = 0.0;
    }

    sumExternalForces();

    solveUnknownAccels();
    calcAccelFKandForceSensorValues();

    integrateRungeKuttaOneStep(1.0 / 6.0, timeStep / 2.0);

    calcPositionAndVelocityFK();
    calcMassMatrix();
    solveUnknownAccels();

    integrateRungeKuttaOneStep(2.0 / 6.0, timeStep / 2.0);

    calcPositionAndVelocityFK();
    calcMassMatrix();
    solveUnknownAccels();

    integrateRungeKuttaOneStep(2.0 / 6.0, timeStep);

    calcPositionAndVelocityFK();
    calcMassMatrix();
    solveUnknownAccels();

    if(unknown_rootDof){
        SE3exp(root->T(), T0, w0, vo0, timeStep);
        root->vo() = vo0 + (dvo + root->dvo() / 6.0) * timeStep;
        root->w()  = w0  + (dw  + root->dw()  / 6.0) * timeStep;
    }
    if(given_rootDof){
        root->p() = pGiven;
        root->R() = RGiven;
        root->vo() = voGiven;
        root->w() = wGiven;
    }

    for(size_t i=0; i < torqueModeJoints.size(); ++i){
        DyLink* link = torqueModeJoints[i];
        const int index = link->index();
        link->q()  = q0 [index] + (dq [index] + link->dq()  / 6.0) * timeStep;
        link->dq() = dq0[index] + (ddq[index] + link->ddq() / 6.0) * timeStep;
    }
    for(size_t i=0; i < highGainModeJoints.size(); ++i){
        DyLink* link = highGainModeJoints[i];
        link->q()  = qGiven [i];
        link->dq() = dqGiven[i];
    }

    calcPositionAndVelocityFK();
    calcMassMatrix();
}


void ForwardDynamicsCBM::integrateRungeKuttaOneStep(double r, double dt)
{
    auto root = subBody->rootLink();

    if(unknown_rootDof || given_rootDof){
        SE3exp(root->T(), T0, root->w(), root->vo(), dt);
        root->vo() = vo0 + root->dvo() * dt;
        root->w()  = w0  + root->dw()  * dt;

        vo  += r * root->vo();
        w   += r * root->w();
        dvo += r * root->dvo();
        dw  += r * root->dw();
    }

    const int n = subBody->numLinks();
    for(int i=1; i < n; ++i){
        auto link = subBody->link(i);
        link->q()  = q0 [i] + dt * link->dq();
        link->dq() = dq0[i] + dt * link->ddq();
        dq [i] += r * link->dq();
        ddq[i] += r * link->ddq();
    }
}


void ForwardDynamicsCBM::preserveHighGainModeJointState()
{
    if(given_rootDof){
        auto root = subBody->rootLink();
        pGivenPrev = root->p();
        RGivenPrev = root->R();
        voGivenPrev = root->vo();
        wGivenPrev = root->w();
    }
    for(size_t i=0; i < highGainModeJoints.size(); ++i){
        const DyLink* link = highGainModeJoints[i];
        qGivenPrev [i] = link->q();
        dqGivenPrev[i] = link->dq();
    }
}


void ForwardDynamicsCBM::calcPositionAndVelocityFK()
{
    auto root = subBody->rootLink();
    root_w_x_v.noalias() = root->w().cross(root->vo() + root->w().cross(root->p()));
    if(given_rootDof){
        root->vo().noalias() = root->v() - root->w().cross(root->p());
    }

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

            case Link::SLIDE_JOINT:
                link->p().noalias() = parent->R() * (link->b() + link->Rb() * (link->q() * link->d())) + parent->p();
                link->R().noalias() = parent->R() * link->Rb();
                link->sw().setZero();
                link->sv().noalias() = parent->R() * (link->Rb() * link->d());
                link->w() = parent->w();
                break;

            case Link::ROTATIONAL_JOINT:
                link->R().noalias() = parent->R() * link->Rb() * AngleAxisd(link->q(), link->a());
                link->p().noalias() = parent->R() * link->b() + parent->p();
                link->sw().noalias() = parent->R() * (link->Rb() * link->a());
                link->sv().noalias() = link->p().cross(link->sw());
                link->w().noalias() = link->dq() * link->sw() + parent->w();
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
                goto COMMON_CALCS_FOR_ALL_JOINT_TYPES;
            }

            link->vo().noalias() = link->dq() * link->sv() + parent->vo();

            const Vector3 dsv = parent->w().cross(link->sv()) + parent->vo().cross(link->sw());
            const Vector3 dsw = parent->w().cross(link->sw());
            link->cv().noalias() = link->dq() * dsv;
            link->cw().noalias() = link->dq() * dsw;
        }

COMMON_CALCS_FOR_ALL_JOINT_TYPES:

        link->v().noalias() = link->vo() + link->w().cross(link->p());

        link->wc().noalias() = link->R() * link->c() + link->p();
        const Matrix3 Iw = link->R() * link->I() * link->R().transpose();
        const Matrix3 c_hat = hat(link->wc());
        link->Iww().noalias() = link->m() * c_hat * c_hat.transpose() + Iw;
        link->Iwv().noalias() = link->m() * c_hat;

        const Vector3 P = link->m() * (link->vo() + link->w().cross(link->wc()));
        const Vector3 L = link->Iww() * link->w() + link->m() * link->wc().cross(link->vo());

        link->pf().noalias() = link->w().cross(P);
        link->ptau().noalias() = link->vo().cross(P) + link->w().cross(L);
    }
}


/**
   calculate the mass matrix using the unit vector method
   \todo replace the unit vector method here with
   a more efficient method that only requires O(n) computation time
*/
void ForwardDynamicsCBM::calcMassMatrix()
{
    auto root = subBody->rootLink();
    const int numLinks = subBody->numLinks();

    // preserve and clear the joint accelerations
    for(int i=1; i < numLinks; ++i){
        auto link = subBody->link(i);
        ddqorg[i] = link->ddq();
        uorg  [i] = link->u();
        link->ddq() = 0.0;
    }

    // preserve and clear the root link acceleration
    dvoorg = root->dvo();
    dworg  = root->dw();
    root->dvo() = -g - root_w_x_v;   // dv = g, dw = 0
    root->dw().setZero();
	
    setColumnOfMassMatrix(b1, 0);

    if(unknown_rootDof){
        for(int i=0; i < 3; ++i){
            root->dvo()[i] += 1.0;
            setColumnOfMassMatrix(M11, i);
            root->dvo()[i] -= 1.0;
        }
        for(int i=0; i < 3; ++i){
            root->dw()[i] = 1.0;
            Vector3 dw_x_p = root->dw().cross(root->p());
            root->dvo() -= dw_x_p;
            setColumnOfMassMatrix(M11, i + 3);
            root->dvo() += dw_x_p;
            root->dw()[i] = 0.0;
        }
    }
    if(given_rootDof){
        for(int i=0; i < 3; ++i){
            root->dvo()[i] += 1.0;
            setColumnOfMassMatrix(M12, i);
            root->dvo()[i] -= 1.0;
        }
        for(int i=0; i < 3; ++i){
            root->dw()[i] = 1.0;
            Vector3 dw_x_p = root->dw().cross(root->p());
            root->dvo() -= dw_x_p;
            setColumnOfMassMatrix(M12, i + 3);
            root->dvo() += dw_x_p;
            root->dw()[i] = 0.0;
        }
    }

    for(size_t i=0; i < torqueModeJoints.size(); ++i){
        DyLink* link = torqueModeJoints[i];
        link->ddq() = 1.0;
        int j = i + unknown_rootDof;
        setColumnOfMassMatrix(M11, j);
        M11(j, j) += link->Jm2(); // motor inertia
        link->ddq() = 0.0;
    }
    for(size_t i=0; i < highGainModeJoints.size(); ++i){
        DyLink* link = highGainModeJoints[i];
        link->ddq() = 1.0;
        int j = i + given_rootDof;
        setColumnOfMassMatrix(M12, j);
        link->ddq() = 0.0;
    }

    // subtract the constant term
    for(int i=0; i < M11.cols(); ++i){
        M11.col(i) -= b1;
    }
    for(int i=0; i < M12.cols(); ++i){
        M12.col(i) -= b1;
    }

    for(int i=1; i < numLinks; ++i){
        DyLink* link = subBody->link(i);
        link->ddq() = ddqorg[i];
        link->u()   = uorg  [i];
    }
    root->dvo() = dvoorg;
    root->dw()  = dworg;

    accelSolverInitialized = false;
}


void ForwardDynamicsCBM::setColumnOfMassMatrix(MatrixXd& M, int column)
{
    Vector3 f;
    Vector3 tau;
    auto root = subBody->rootLink();
    calcInverseDynamics(root, f, tau, true);

    MatrixXd::ColXpr col = M.col(column);

    if(unknown_rootDof){
        tau -= root->p().cross(f);
        col << f, tau;
        /*
          col.head(3) = f;
          col.segment(3, 3) = tau;
        */
    }

    for(size_t i=0; i < torqueModeJoints.size(); ++i){
        col(i + unknown_rootDof) = torqueModeJoints[i]->u();
    }
}


void ForwardDynamicsCBM::calcInverseDynamics(DyLink* link, Vector3& out_f, Vector3& out_tau, bool isSubBodyRoot)
{
    if(!isSubBodyRoot){
        auto parent = link->parent();
        link->dvo() = parent->dvo() + link->cv() + link->sv() * link->ddq();
        link->dw()  = parent->dw()  + link->cw() + link->sw() * link->ddq();
    }

    out_f = link->pf();
    out_tau = link->ptau();

    auto child = link->child();
    if(child && !child->isFreeJoint()){
        Vector3 f_c;
        Vector3 tau_c;
        calcInverseDynamics(child, f_c, tau_c, false);
        out_f += f_c;
        out_tau += tau_c;
    }

    out_f  .noalias() += link->m()   * link->dvo() + link->Iwv().transpose() * link->dw();
    out_tau.noalias() += link->Iwv() * link->dvo() + link->Iww()             * link->dw();

    link->u() = link->sv().dot(out_f) + link->sw().dot(out_tau);

    if(!isSubBodyRoot){
        auto sibling = link->sibling();
        if(sibling && !sibling->isFreeJoint()){
            Vector3 f_s;
            Vector3 tau_s;
            calcInverseDynamics(link->sibling(), f_s, tau_s, false);
            out_f += f_s;
            out_tau += tau_s;
        }
    }
}


void ForwardDynamicsCBM::sumExternalForces()
{
    fextTotal.setZero();
    tauextTotal.setZero();

    const int n = subBody->numLinks();
    for(int i=0; i < n; ++i){
        auto link = subBody->link(i);
        fextTotal   += link->f_ext();
        tauextTotal += link->tau_ext();
    }

    tauextTotal -= subBody->rootLink()->p().cross(fextTotal);
}

void ForwardDynamicsCBM::calcd1(DyLink* link, Vector3& out_f, Vector3& out_tau, bool isSubBodyRoot)
{
    out_f = -link->f_ext();
    out_tau = -link->tau_ext();

    auto child = link->child();
    if(child && !child->isFreeJoint()){
        Vector3 f_c;
        Vector3 tau_c;
        calcd1(child, f_c, tau_c, false);
        out_f += f_c;
        out_tau += tau_c;
    }

    link->u() = link->sv().dot(out_f) + link->sw().dot(out_tau);

    if(!isSubBodyRoot){
        auto sibling = link->sibling();
        if(sibling && !sibling->isFreeJoint()){
            Vector3 f_s;
            Vector3 tau_s;
            calcd1(sibling, f_s, tau_s, false);
            out_f += f_s;
            out_tau += tau_s;
        }
    }
}

void ForwardDynamicsCBM::initializeAccelSolver()
{
    if(!accelSolverInitialized){

        if(!ddqGivenCopied){
            if(given_rootDof){
                auto root = subBody->rootLink();
                root->dvo() = root->dv() - root->dw().cross(root->p()) - root->w().cross(root->v());
                ddqGiven.head(3) = root->dvo();
                ddqGiven.segment(3, 3) = root->dw();
            }
            for(size_t i=0; i < highGainModeJoints.size(); ++i){
                ddqGiven(given_rootDof + i) = highGainModeJoints[i]->ddq();
            }
            ddqGivenCopied = true;
        }

        b1.noalias() += M12 * ddqGiven;
        
        for(int i=1; i < subBody->numLinks(); ++i){
            uorg[i] = subBody->link(i)->u();
        }
        Vector3 f, tau;
        calcd1(subBody->rootLink(), f, tau, true);
        for(int i=0; i < unknown_rootDof; i++){
            d1(i, 0) = 0;
        }
        for(size_t i=0; i < torqueModeJoints.size(); ++i){
            d1(i + unknown_rootDof, 0) = torqueModeJoints[i]->u();
        }
        for(int i=1; i < subBody->numLinks(); ++i){
            subBody->link(i)->u() = uorg[i];
        }

        accelSolverInitialized = true;
    }
}

/**
   This is called from ConstraintForceSolver.
   @return This function returns true if the accelerations can be calculated for the axes of unkown motion.
   If there is no axis of unkown motion, the function returns false.
*/
bool ForwardDynamicsCBM::solveUnknownAccels
(DyLink* link, const Vector3& fext, const Vector3& tauext, const Vector3& rootfext, const Vector3& roottauext)
{
    if(isNoUnknownAccelMode){
        return false;
    }
    
    const Vector3 fextorg = link->f_ext();
    const Vector3 tauextorg = link->tau_ext();
    link->f_ext() = fext;
    link->tau_ext() = tauext;
    for(int i=1; i < subBody->numLinks(); ++i){
        uorg[i] = subBody->link(i)->u();
    }
    Vector3 f, tau;
    calcd1(subBody->rootLink(), f, tau, true);
    for(int i=0; i < unknown_rootDof; i++){
        d1(i, 0) = 0;
    }
    for(size_t i=0; i < torqueModeJoints.size(); ++i){
        d1(i + unknown_rootDof, 0) = torqueModeJoints[i]->u();
    }
    for(int i=1; i < subBody->numLinks(); ++i){
        subBody->link(i)->u() = uorg[i];
    }
    link->f_ext() = fextorg;
    link->tau_ext() = tauextorg;

    solveUnknownAccels(rootfext, roottauext);

    return true;
}

void ForwardDynamicsCBM::solveUnknownAccels(const Vector3& fext, const Vector3& tauext)
{
    if(unknown_rootDof){
        c1.head(3) = fext;
        c1.segment(3, 3) = tauext;
    }

    for(size_t i=0; i < torqueModeJoints.size(); ++i){
        c1(i + unknown_rootDof) = torqueModeJoints[i]->u();
    }

    c1 -= d1;
    c1 -= b1.col(0);

    const VectorXd a(M11.colPivHouseholderQr().solve(c1));
    
    if(unknown_rootDof){
        auto root = subBody->rootLink();
        root->dw() = a.segment(3, 3);
        const Vector3 dv = a.head(3);
        root->dvo() = dv - root->dw().cross(root->p()) - root_w_x_v;
    }

    for(size_t i=0; i < torqueModeJoints.size(); ++i){
        torqueModeJoints[i]->ddq() = a(i + unknown_rootDof);
    }
}


void ForwardDynamicsCBM::calcAccelFKandForceSensorValues(DyLink* link, Vector3& out_f, Vector3& out_tau, bool isSubBodyRoot)
{
    if(!isSubBodyRoot){
        auto parent = link->parent();
        link->dvo() = parent->dvo() + link->cv() + link->sv() * link->ddq();
        link->dw()  = parent->dw()  + link->cw() + link->sw() * link->ddq();
    }

    out_f   = link->pf();
    out_tau = link->ptau();

    for(DyLink* child = link->child(); child; child = child->sibling()){
        if(!child->isFreeJoint()){
            Vector3 f, tau;
            calcAccelFKandForceSensorValues(child, f, tau, false);
            out_f   += f;
            out_tau += tau;
        }
    }

    auto& cbm = link->cbm;

    if(CALC_ALL_JOINT_TORQUES || cbm->hasForceSensorsAbove){

        Vector3 fg(link->m() * g);
        Vector3 tg(link->wc().cross(fg));

        out_f   -= fg;
        out_tau -= tg;

        out_f   -= link->f_ext();
        out_tau -= link->tau_ext();

        out_f  .noalias() += link->m()   * link->dvo() + link->Iwv().transpose() * link->dw();
        out_tau.noalias() += link->Iwv() * link->dvo() + link->Iww()             * link->dw();

        if(CALC_ALL_JOINT_TORQUES && link->actuationMode() == Link::JOINT_DISPLACEMENT){
            link->u() = link->sv().dot(out_f) + link->sw().dot(out_tau);
        }

        if(cbm->hasForceSensor){
            cbm->f = -out_f;
            cbm->tau = -out_tau;
        }
    }
}


void ForwardDynamicsCBM::initializeSensors()
{
    ForwardDynamics::initializeSensors();

    for(auto& link : subBody->links()){
        link->cbm.reset(new DyLink::ForwardDynamicsCbmData);
    }

    if(sensorsEnabled){
        for(auto& sensor : subBody->forceSensors()){
            static_cast<DyLink*>(sensor->link())->cbm->hasForceSensor = true;
        }
        auto rootLink = subBody->rootLink();
        rootLink->cbm->hasForceSensorsAbove = rootLink->cbm->hasForceSensor;
        for(int i=1; i < subBody->numLinks(); ++i){
            auto link = subBody->link(i);
            link->cbm->hasForceSensorsAbove = link->parent()->cbm->hasForceSensorsAbove;
        }
    }
}


void ForwardDynamicsCBM::updateForceSensors()
{
    for(auto& sensor : subBody->forceSensors()){
        auto link = static_cast<DyLink*>(sensor->link());
        const Matrix3 R = link->R() * sensor->R_local();
        const Vector3 p = link->p() + link->R() * sensor->p_local();
        sensor->f()   = R.transpose() * link->cbm->f;
        sensor->tau() = R.transpose() * (link->cbm->tau - p.cross(link->cbm->f));
        sensor->notifyStateChange();
    }
}
