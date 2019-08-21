/*!
  @file
  @author Shin'ichiro Nakaoka
*/

#include "PinDragIK.h"
#include "Body.h"
#include "JointPath.h"
#include <cnoid/EigenUtil>
#include <map>
#include <iostream>
#include <algorithm>

using namespace std;
using namespace cnoid;

namespace {

const bool SOLVE_CONSTRAINTS_BY_SR_INVERSE = false;
const bool SOLVE_CONSTRAINTS_BY_SVD = !SOLVE_CONSTRAINTS_BY_SR_INVERSE;
    
double calcLU(int n, MatrixXd& a, vector<int>& pivots);
void solveByLU(int n, MatrixXd& a, vector<int>& pivots, const MatrixXd::ColXpr& x, const VectorXd& b);
bool makeInverseMatrix(int n, MatrixXd& org, MatrixXd& inv, double minValidDet);
bool makePseudoInverseType1(int m, int n, const MatrixXd& J, MatrixXd& Jinv, MatrixXd& JJ, MatrixXd& JJinv, double minValidDet);
bool makePseudoInverseType2(int m, int n, const MatrixXd& J, MatrixXd& Jinv, MatrixXd& JJ, MatrixXd& JJinv,
                            const VectorXd& weights, double minValidDet);
bool makeSRInverseMatrix(int m, int n, const MatrixXd& J, MatrixXd& Jinv, MatrixXd& JJ, MatrixXd& JJ2, MatrixXd& JJinv,
                         vector<int>& pivots, double srk0, double srw0);
}


namespace cnoid {

class PinDragIKImpl
{
public:
    PinDragIKImpl(Body* body);

    BodyPtr body_;

    int maxIteration;

    //! calculation is iterated until the erro is less than this value
    double ikErrorSqrThresh;

    // number of joints (body->numJoints())
    int NJ;
            
    // dimension of joint space (this can includes elements of 6-DOF free root)
    int N; 
    // dimension of target space. 3 means position only, 6 means position and orientation.
    int M; 

    Link* baseLink;
    Link* targetLink;

    bool isJointRangeConstraintsEnabled;

    VectorXd q_org;
    Vector3 base_p_org;
    Matrix3 base_R_org;

    MatrixXd J;       // Jacobian Matrix (M x N)
    MatrixXd Jinv;    // J^-1 (N x N) or pseudo inverse of J (N x M)
            
    // temporary matrix variables
    MatrixXd JJ;      // (J^T * J) (N < M, N x N) or (J * J^T) (N > M, M x M)
    MatrixXd JJinv;   // JJ^-1

    VectorXd dr_p; // d(x, y, z, w_x, w_y, w_z) / dt
            
    double minValidDet;

    // dimension of constrained variables (vector p_aux)
    int C;
            
    // Matrices
    MatrixXd Jaux;       // Jacobian Matrix (C x N)
    MatrixXd Jauxinv;    
    MatrixXd W;       
            
    MatrixXd S;        // (C x N)
    MatrixXd Sinv;    
            
    MatrixXd JJ2;    
            
    // Joint space vector to solve (size N)
    // This vector includes elements of 6-DOF root joint (dx, dy, dz, OmegaX, OmegaY, OmegaZ)
    // in the tail when root free model is enabled.
    VectorXd dq;
            
    // weight of joint space vector (size N)
    // this includes 6-DOF root joint like dq
    VectorXd qWeights;
            
    VectorXd y;   // size N
            
    vector<int> pivots;
            
    bool isBaseLinkFreeMode;
    bool isTargetAttitudeEnabled;

    LinkTraverse fkTraverse;
    shared_ptr<JointPath> targetJointPath;
            
    struct PinProperty {
        double weight;
        PinDragIK::AxisSet axes;
        shared_ptr<JointPath> jointPath;
        Vector3 p;
        Matrix3 R;
        Vector3 prevStep_p;
        Matrix3 prevStep_R;
        PinProperty() : jointPath(new JointPath()) { }
    };
            
    typedef map<Link*, PinProperty> PinPropertyMap;
            
    PinPropertyMap pinPropertyMap;
    vector<double> constraintWeightsSqrt; // size C

    struct JointConstrain
    {
        JointConstrain(int jointId, double dq) : jointId(jointId), dq(dq) { }
        int jointId;
        double dq;
    };
    vector<JointConstrain> jointConstraints;

    VectorXd dPaux; // size C

    double srk0; // k of the singular point
    double srw0; // threshold value to calc k

    enum IKStepResult { ERROR, PINS_NOT_CONVERGED, PINS_CONVERGED };

    void setBaseLink(Link* baseLink);
    void setFreeRootWeight(double translation, double rotation);
    void setTargetLink(Link* targetLink, bool isAttitudeEnabled);
    void setJointWeight(int jointId, double weight);
    void setPin(Link* link, PinDragIK::AxisSet axes, double weight);
    PinDragIK::AxisSet pinAxes(Link* link);
    void setIKErrorThresh(double e);
    void setSRInverseParameters(double k0, double w0);
    void enableJointRangeConstraints(bool on);
    bool initialize();
            
    bool calcInverseKinematics(const Position& T);
            
    IKStepResult calcOneStep(const Vector3& v, const Vector3& omega);
    void solveConstraints();
    void setJacobianForOnePath(MatrixXd& J, int row, JointPath& jointPath, int axes);
    void setJacobianForFreeRoot(MatrixXd& J, int row, JointPath& jointPath, int axes);
    void addPinConstraints();
    void addJointRangeConstraints();
    int solveLinearEquationWithSVD(MatrixXd& A, VectorXd& b, VectorXd& x, double sv_ratio);
};
}


PinDragIK::PinDragIK(Body* body)
{
    impl = new PinDragIKImpl(body);
}


PinDragIKImpl::PinDragIKImpl(Body* body)
    : body_(body),
      NJ(body->numJoints()),
      q_org(NJ),
      qWeights(NJ + 6)
{
    M = 0;
    N = 0;
    
    maxIteration = 50;
    //minValidDet = 1.0e-12;
    //minValidDet = 1.0e-5
    minValidDet = 1.0e-9;
    
    setBaseLink(0);
    setTargetLink(0, false);

    qWeights.head(qWeights.size() - 6).fill(1.0);
    
    setFreeRootWeight(30.0, 1.0);

    setIKErrorThresh(1.0e-5);
    setSRInverseParameters(0.1, 0.001);

    //enableJointRangeConstraints(true);
    enableJointRangeConstraints(false);
}


PinDragIK::~PinDragIK()
{
    delete impl;
}


Body* PinDragIK::body() const
{
    return impl->body_;
}


void PinDragIK::setBaseLink(Link* baseLink)
{
    impl->setBaseLink(baseLink);
}


//! if root is zero, root is virtual free 6-DOF link on the top link
void PinDragIKImpl::setBaseLink(Link* baseLink)
{
    if(baseLink){
        this->baseLink = baseLink;
        isBaseLinkFreeMode = false;
    } else {
        this->baseLink = body_->rootLink();
        isBaseLinkFreeMode = true;
    }
}


void PinDragIK::setFreeRootWeight(double translation, double rotation)
{
    impl->setFreeRootWeight(translation, rotation);
}


void PinDragIKImpl::setFreeRootWeight(double translation, double rotation)
{
    for(int i=0; i < 3; i++){
        qWeights(NJ + i) = translation;
        qWeights(NJ + 3 + i) = rotation;
    }
}


void PinDragIK::setTargetLink(Link* targetLink, bool isAttitudeEnabled)
{
    impl->setTargetLink(targetLink, isAttitudeEnabled);
}


void PinDragIKImpl::setTargetLink(Link* targetLink, bool isAttitudeEnabled)
{
    this->targetLink = targetLink;
    isTargetAttitudeEnabled = isAttitudeEnabled;
    M = isAttitudeEnabled ? 6 : 3;
}


void PinDragIK::setJointWeight(int jointId, double weight)
{
    impl->setJointWeight(jointId, weight);
}


void PinDragIKImpl::setJointWeight(int jointId, double weight)
{
    qWeights(jointId) = weight;
}


void PinDragIK::setPin(Link* link, PinDragIK::AxisSet axes, double weight)
{
    impl->setPin(link, axes, weight);
}


void PinDragIKImpl::setPin(Link* link, PinDragIK::AxisSet axes, double weight)
{
    if(link){
        if(axes == PinDragIK::NO_AXES){
            pinPropertyMap.erase(link);
        } else {
            PinProperty& property = pinPropertyMap[link];
            property.axes = axes;
            property.weight = weight;
        }
    }
}


PinDragIK::AxisSet PinDragIK::pinAxes(Link* link)
{
    return impl->pinAxes(link);
}


PinDragIK::AxisSet PinDragIKImpl::pinAxes(Link* link)
{
    PinPropertyMap::iterator p = pinPropertyMap.find(link);
    if(p == pinPropertyMap.end()){
        return PinDragIK::NO_AXES;
    }
    return p->second.axes;
}


void PinDragIK::clearPins()
{
    impl->pinPropertyMap.clear();
}


int PinDragIK::numPinnedLinks()
{
    return impl->pinPropertyMap.size();
}


void PinDragIK::setIKErrorThresh(double e)
{
    impl->setIKErrorThresh(e);
}


void PinDragIKImpl::setIKErrorThresh(double e)
{
    ikErrorSqrThresh = e * e;
}


bool PinDragIK::hasAnalyticalIK()
{
    return false;
}


PinDragIK::AxisSet PinDragIK::targetAxes() const
{
    return impl->isTargetAttitudeEnabled ? TRANSFORM_6D : TRANSLATION_3D;
}


void PinDragIK::setSRInverseParameters(double k0, double w0)
{
    impl->setSRInverseParameters(k0, w0);
}


void PinDragIKImpl::setSRInverseParameters(double k0, double w0)
{
    srk0 = k0;
    srw0 = w0;
}


void PinDragIK::enableJointRangeConstraints(bool on)
{
    impl->enableJointRangeConstraints(on);
}


void PinDragIKImpl::enableJointRangeConstraints(bool on)
{
    isJointRangeConstraintsEnabled = on;
}


bool PinDragIK::initialize()
{
    return impl->initialize();
}


bool PinDragIKImpl::initialize()
{
    if(!targetLink){
        return false;
    }

    N = body_->numJoints();
    
    if(baseLink == targetLink){
        isBaseLinkFreeMode = true;
    }
    if(isBaseLinkFreeMode){
        N += 6;
    }
    
    targetJointPath = make_shared<JointPath>(baseLink, targetLink);

    pinPropertyMap.erase(targetLink);

    C = 0;

    constraintWeightsSqrt.clear();
    PinPropertyMap::iterator p = pinPropertyMap.begin();
    while(p != pinPropertyMap.end()){
        Link* link = p->first;
        PinProperty& property = p->second;
        property.jointPath = make_shared<JointPath>(baseLink, link);
        if(property.jointPath->empty()){
            return false;
        }

        double weightSqrt = sqrt(property.weight);

        if(property.axes & PinDragIK::TRANSLATION_3D){
            property.p = link->p();
            for(int i=0; i < 3; i++){
                constraintWeightsSqrt.push_back(weightSqrt);
            }
            C += 3;
        }

        if(property.axes & PinDragIK::ROTATION_3D){
            property.R = link->R();
            for(int i=0; i < 3; i++){
                constraintWeightsSqrt.push_back(weightSqrt);
            }
            C += 3;
        }
        p++;
    }

    dq.resize(N);
    y.resize(N);
    
    dr_p.resize(M);

    J.resize(M, N);
    Jinv.resize(N, M);
    
    J.setZero();

    jointConstraints.clear();

    Jaux.resize(C, N);
    Jauxinv.resize(N, C);
    S.resize(C, N);
    Sinv.resize(N, C);

    Jaux.setZero();

    dPaux.resize(C);

    int JJsize;
    if(C <= N && M <= N){
        JJsize = C > M ? C : M;
    } else {
        JJsize = N;
    }

    JJ.resize(JJsize, JJsize);
    JJ2.resize(JJsize, JJsize);
    JJinv.resize(JJsize, JJsize);

    W.resize(N, N);
  
    pivots.resize(C);  

    fkTraverse.find(baseLink, true, true);

    return true;
}


bool PinDragIK::calcInverseKinematics(const Position& T)
{
    return impl->calcInverseKinematics(T);
}


bool PinDragIKImpl::calcInverseKinematics(const Position& T)
{
    for(int i=0; i < NJ; i++){
        q_org[i] = body_->joint(i)->q();
    }
    if(isBaseLinkFreeMode){
        base_p_org = baseLink->p();
        base_R_org = baseLink->R();
    }
    
    for(PinPropertyMap::iterator p = pinPropertyMap.begin(); p != pinPropertyMap.end(); p++){
        Link* link = p->first;
        PinProperty& property = p->second;
        property.prevStep_p = link->p();
        property.prevStep_R = link->R();
    }

    IKStepResult result = (C > 0) ? PINS_CONVERGED : PINS_NOT_CONVERGED;

    int i;
    for(i=0; i < maxIteration; i++){
        
        const Vector3 dp = T.translation() - targetLink->p();
        const Vector3 omega = targetLink->R() * omegaFromRot(targetLink->R().transpose() * T.linear());
        
        if((dp.squaredNorm() + omega.squaredNorm()) < ikErrorSqrThresh && result == PINS_CONVERGED){
            break;
        }

        result = calcOneStep(dp, omega);

        if(result == ERROR){
            for(int i=0; i < NJ; i++){
                body_->joint(i)->q() = q_org[i];
            }
            if(isBaseLinkFreeMode){
                baseLink->p() = base_p_org;
                baseLink->R() = base_R_org;
            }
            fkTraverse.calcForwardKinematics();
            break;
        }
    }

    return (result != ERROR);
}


PinDragIKImpl::IKStepResult PinDragIKImpl::calcOneStep(const Vector3& v, const Vector3& omega)
{
    // make Jacobian matrix for the target link
    int axes = PinDragIK::TRANSLATION_3D;
    if(isTargetAttitudeEnabled){
        axes |= PinDragIK::ROTATION_3D;
    }
    setJacobianForOnePath(J, 0, *targetJointPath, axes);
    if(isBaseLinkFreeMode){
        setJacobianForFreeRoot(J, 0, *targetJointPath, axes);
    }

    bool isOk;
    if(N >= M){
        isOk = makePseudoInverseType2(M, N, J, Jinv, JJ, JJinv, qWeights, minValidDet);
    } else {
        isOk = makePseudoInverseType1(M, N, J, Jinv, JJ, JJinv, minValidDet);
    }
    if(!isOk){
        return ERROR;
    }

    // dq0 = J# dr_p
    dr_p[0] = v[0]; dr_p[1] = v[1]; dr_p[2] = v[2];
    if(isTargetAttitudeEnabled){
        dr_p[3] = omega[0]; dr_p[4] = omega[1]; dr_p[5] = omega[2];
    }
    dq.noalias() = Jinv * dr_p;

    if(C > 0){
        solveConstraints();
    }

    double maxq = 0.0;
    for(int i=0; i < N; ++i){
        maxq = std::max(dq[i], maxq);
    }

    double thresh = 0.1;
    if(maxq > thresh){
        dq *= (thresh / maxq);
    }
    
    for(int i=0; i < NJ; i++){
        body_->joint(i)->q() += dq[i];
    }

    if(isBaseLinkFreeMode){
    
        base_p_org = baseLink->p();
        base_R_org = baseLink->R();

        for(int i=0; i < 3; i++){
            baseLink->p()[i] += dq[NJ+i];
        }
        baseLink->R() = rotFromRpy(dq[NJ+3], dq[NJ+4], dq[NJ+5]) * baseLink->R();

        normalizeRotation(baseLink->T());
    }

    fkTraverse.calcForwardKinematics();

    double maxErrorSqr = 0.0;
    for(PinPropertyMap::iterator p = pinPropertyMap.begin(); p != pinPropertyMap.end(); p++){
        Link* link = p->first;
        PinProperty& property = p->second;
        double errsqr = 0.0;
        if(property.axes & PinDragIK::TRANSLATION_3D){
            const Vector3 dp = property.prevStep_p - link->p();
            errsqr += dp.squaredNorm();
            property.prevStep_p = link->p();
        }
        if(property.axes & PinDragIK::ROTATION_3D){
            const Vector3 omega = omegaFromRot(link->R().transpose() * property.prevStep_R);
            errsqr += omega.squaredNorm();
            property.prevStep_R = link->R();
        }
        maxErrorSqr = std::max(errsqr, maxErrorSqr);
    }
    
    return (maxErrorSqr < ikErrorSqrThresh) ? PINS_CONVERGED : PINS_NOT_CONVERGED;
}


void PinDragIKImpl::solveConstraints()
{
    // W = (E - J# J)  (size N x N)
    W.noalias() = MatrixXd::Identity(N, N) - Jinv * J;
    
    // normalize W for weighted theta
    /*
      for(int i=0; i < N; i++){
      double norm2 = 0.0;
      for(int j=0; j < N; j++){
      double a = W(j,i);
      norm2 += qWeights[j] * a * a;
      }
      double norm = sqrt(norm2);
      if(norm > 0.0001){
      for(int j=0; j < N; j++){
      W(j,i) /= norm;
      }
      }
      }
    */

    addPinConstraints();

    if(isJointRangeConstraintsEnabled){
        addJointRangeConstraints();
    }

    // deltaPaux = dPaux - dPaux0, (dPaux0 = Jaux dq0)
    dPaux.noalias() -= Jaux * dq;
    VectorXd& deltaPaux = dPaux;

    S.noalias() = Jaux * W;

    // normalize S and deltaPaux for weighted targets (weights of constraned positions)
    /*
      for(int i=0; i < C; i++){
      double w = constraintWeightsSqrt[i];
      for(int j=0; j < N; j++){
      S(i,j) *= w;
      }
      deltaPaux[i] *= w;
      }
    */

    if(SOLVE_CONSTRAINTS_BY_SR_INVERSE){
        makeSRInverseMatrix(C, N, S, Sinv, JJ, JJ2, JJinv, pivots, srk0, srw0);
        y.noalias() = Sinv * deltaPaux;

    } else if(SOLVE_CONSTRAINTS_BY_SVD){
        y = Eigen::JacobiSVD<MatrixXd>(S, Eigen::ComputeThinU | Eigen::ComputeThinV).solve(deltaPaux);
    }

    // dq = dq0 + W y
    dq.noalias() += W * y;
}


void PinDragIKImpl::setJacobianForOnePath(MatrixXd& J, int row, JointPath& jointPath, int axes)
{
    int col;
    const int n = jointPath.numJoints();

    if(n > 0){
        Link* target = jointPath.endLink();

        for(int i=0; i < n; i++){

            Link* link = jointPath.joint(i);
            col = link->jointId();

            Vector3 omega(link->R() * link->a());
            if(!jointPath.isJointDownward(i)){
                omega = -omega;
            }
   
            int r = row;
            if(axes & PinDragIK::TRANSLATION_3D){
                const Vector3 dp = omega.cross(target->p() - link->p());
                J(r++, col) = dp(0);
                J(r++, col) = dp(1);
                J(r++, col) = dp(2);
            }
    
            if(axes & PinDragIK::ROTATION_3D){
                J(r++, col) = omega(0);
                J(r++, col) = omega(1);
                J(r++, col) = omega(2);
            }
        }
    }
}


void PinDragIKImpl::setJacobianForFreeRoot(MatrixXd& J, int row, JointPath& jointPath, int axes)
{
    Link* target;
    if(jointPath.numJoints() > 0){
        target = jointPath.endLink();
    } else {
        target = baseLink;
    }

    int col = NJ;

    if(axes & PinDragIK::TRANSLATION_3D){
        Vector3 omega = Vector3::Zero();
        for(int i=0; i < 3; i++){
            omega[i] = 1.0;
            const Vector3 dp = omega.cross(target->p() - baseLink->p());
            
            for(int j=0; j < 3; j++){
                if(j == i){
                    J(row + j, col + i) = 1.0;
                }
                J(row + j, col + i + 3) = dp(j);
            }
            omega[i] = 0.0;
        }
        row += 3;
    }
    
    if(axes & PinDragIK::ROTATION_3D){
        for(int i=0; i < 3; i++){
            J(row + i, col + i + 3) = 1.0;
        }
    }
}


void PinDragIKImpl::addPinConstraints()
{
    int row = 0;

    for(PinPropertyMap::iterator p = pinPropertyMap.begin(); p != pinPropertyMap.end(); p++){

        Link* link = p->first;
        PinProperty& property = p->second;

        setJacobianForOnePath(Jaux, row, *property.jointPath, property.axes);
        if(isBaseLinkFreeMode){
            setJacobianForFreeRoot(Jaux, row, *property.jointPath, property.axes);
        }

        if(property.axes & PinDragIK::TRANSLATION_3D){
            const Vector3 dp = property.p - link->p();
            dPaux[row++] = dp[0];
            dPaux[row++] = dp[1];
            dPaux[row++] = dp[2];
        }
    
        if(property.axes & PinDragIK::ROTATION_3D){
            const Vector3 omega = link->R() * omegaFromRot(link->R().transpose() * property.R);
            dPaux[row++] = omega[0];
            dPaux[row++] = omega[1];
            dPaux[row++] = omega[2];
        }
    }
}


void PinDragIKImpl::addJointRangeConstraints()
{
    jointConstraints.clear();
    
    for(int i=0; i < NJ; ++i){
        Link* link = body_->joint(i);
        if(link->q() < link->q_lower()){
            jointConstraints.push_back(JointConstrain(i, link->q_lower() - link->q()));
        } else if(link->q() > link->q_upper()){
            jointConstraints.push_back(JointConstrain(i, link->q_upper() - link->q()));
        }
    }

    const int C2 = jointConstraints.size();

    Jaux.conservativeResize(C + C2, N);
    dPaux.conservativeResize(C + C2);
    S.resize(C + C2, N);

    for(int i=0; i < C2; ++i){
        int r = C + i;
        for(int j=0; j < N; ++j){
            Jaux(r, j) = 0.0;
        }
        JointConstrain& limit = jointConstraints[i];
        Jaux(r, limit.jointId) = 1.0;
        dPaux[r] = limit.dq;
    }
}


namespace {

/**
   \param pivots row exchange index in LU decomposition (size = n)
*/
double calcLU(int n, MatrixXd& a, vector<int>& pivots)
{
    double det = 1.0; 
    vector<double> weight(n);    
        
    // get the max element and a scaling value in each row 
    double v, max;
    for(int k = 0; k < n; k++) { 
        pivots[k] = k;             
        max = 0.0;                 
        for(int j = 0; j < n; j++) {
            v = fabs(a(k,j));  if (v > max) max = v;
        }
        if(max == 0.0) return 0.0; // Factorization is impossible !
        weight[k] = 1.0 / max;     
    }

    for(int k = 0; k < n; k++) {  
    
        // get the next pivot row
        max = -1;
        int maxrow = 0;
        for(int i = k; i < n; i++) {  
            int ix = pivots[i];           
            v = fabs(a(ix,k)) * weight[ix];
            if(v > max){
                max = v; 
                maxrow = i;
            }
        }
        int ik = pivots[maxrow];
        if(maxrow != k) {
            pivots[maxrow] = pivots[k];
            pivots[k] = ik;  
            det = -det;
        }

        double d = a(ik,k);
        det *= d;
        if(d == 0) return det;

        for(int i = k + 1; i < n; i++) {  
            int ix = pivots[i];
            double t = (a(ix,k) /= d);
            for (int j = k + 1; j < n; j++){
                a(ix,j) -= t * a(ik,j);
            }
        }
    }

    return det;         
}


/**
   Solve Ax = b for x 
*/
void solveByLU(int n, MatrixXd& a, vector<int>& pivots, const MatrixXd::ColXpr& x, const VectorXd& b)
{
    int ix;
    double t;
    
    for(int i = 0; i < n; i++) {       
        ix = pivots[i];
        t = b(ix);
        for (int j = 0; j < i; j++){
            t -= a(ix, j) * x(j);
        }
        const_cast<MatrixXd::ColXpr&>(x)(i) = t;
    }
    
    for(int i = n - 1; i >= 0; i--) {  
        t = x(i);
        ix = pivots[i];
        for (int j = i + 1; j < n; j++){
            t -= a(ix, j) * x(j);
        }
        const_cast<MatrixXd::ColXpr&>(x)(i) = t / a(ix, i);
    }
}


bool makeInverseMatrix(int n, MatrixXd& org, MatrixXd& inv, double minValidDet)
{
    vector<int> pivots(n);
    VectorXd unitVector(n);
    unitVector.setZero();

    double det = calcLU(n, org, pivots);
    if(det > minValidDet || det < -minValidDet){
        for(int i=0; i < n; i++){
            unitVector[i] = 1.0;
            solveByLU(n, org, pivots, inv.col(i), unitVector);
            unitVector[i] = 0.0;
        }
        return true;
    } else {
        return false;
    }
}


// N < M
bool makePseudoInverseType1
(int m, int n, const MatrixXd& J, MatrixXd& Jinv, MatrixXd& JJ, MatrixXd& JJinv, double minValidDet)
{
    // JJ = J^T * J
    JJ.noalias() = J.transpose() * J;

    // Jinv = (J^T * J)^-1 * J^T
    if(makeInverseMatrix(n, JJ, JJinv, minValidDet)){
        Jinv.noalias() = JJinv * J.transpose();
        return true;
    }
        
    return false;
}


// N > M
bool makePseudoInverseType2
(int m, int n, const MatrixXd& J, MatrixXd& Jinv, MatrixXd& JJ, MatrixXd& JJinv,
 const VectorXd& weights, double minValidDet)
{
    // JJ = J * W^-1 * J^T
    for(int i=0; i < m; i++){
        for(int j=0; j < m; j++){
            JJ(i, j) = 0.0;
            for(int k=0; k < n; k++){
                JJ(i,j) += J(i,k) * (J(j,k) / weights(k));
            }
        }
    }

    // Jinv = W^-1 * J^T * (J * W^-1 * J^T)^-1
    if(makeInverseMatrix(m, JJ, JJinv, minValidDet)){
        for(int i=0; i < n; i++){
            for(int j=0; j < m; j++){
                Jinv(i,j) = 0.0;
                for(int k=0; k < m; k++){
                    Jinv(i,j) += J(k,i) * JJinv(k,j);
                }
                Jinv(i,j) /= weights(i);
            }
        }
        return true;
    }

    return false;
}


// calculate J^T(J J^T + kI)^-1
bool makeSRInverseMatrix
(int m, int n, const MatrixXd& J, MatrixXd& Jinv, MatrixXd& JJ, MatrixXd& JJ2, MatrixXd& JJinv,
 vector<int>& pivots, double srk0, double srw0)
{
    // JJ = J J^T
    JJ.noalias() = J * J.transpose();

    JJ2 = JJ;
    double det = calcLU(m, JJ2, pivots);
    double w = sqrt(det);
    
    double k;
    if(w < srw0){
        double a = 1.0 - (w / srw0);
        k = srk0 * a * a;
        static int counter = 0;
        cout << "srk0 enabled(" << counter++ << ")" << endl;
    } else {
        k = 0.0;
    }
    
    // JJ' = JJ + kI
    for(int i=0; i < m; i++){
        JJ(i,i) += k;
    }

    // J^T JJ'^-1
    if(makeInverseMatrix(m, JJ, JJinv, 0.0)){
        Jinv.noalias() = J.transpose() * JJinv;
        return true;
    }
    
    return false;
}

}
