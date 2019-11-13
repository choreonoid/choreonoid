/**
   \author Shin'ichiro Nakaoka
*/

#include "MassMatrix.h"
#include "Link.h"
#include "InverseDynamics.h"

using namespace cnoid;

namespace {

template<typename Derived>
void setColumnOfMassMatrix(Body* body, Eigen::MatrixBase<Derived>& out_M, int column)
{
    Link* rootLink = body->rootLink();
    Vector6 f = calcInverseDynamics(rootLink);
        
    if(!rootLink->isFixedJoint()){
        f.tail<3>() -= rootLink->p().cross(f.head<3>());
        out_M. Eigen::MatrixBase<Derived>::template block<6, 1>(0, column) = f;
    }
    const int n = body->numJoints();
    for(int i = 0; i < n; ++i){
        Link* joint = body->joint(i);
        out_M(i + 6, column) = joint->u();
    }
}

}

namespace cnoid {

/**
   calculate the mass matrix using the unit vector method
   \todo replace the unit vector method here with a more efficient method
       
   The motion equation (dv != dvo)
   |       |   | dv  |   |   |   | fext      |
   | out_M | * | dw  | + | b | = | tauext    |
   |       |   | ddq |   |   |   | u         |
*/
void calcMassMatrix(Body* body, const Vector3& g, Eigen::MatrixXd& out_M, VectorXd& out_b)
{
    const int nj = body->numJoints();
    Link* rootLink = body->rootLink();
    const int totaldof = rootLink->isFixedJoint() ? nj : nj + 6;

    out_M.resize(totaldof, totaldof);
        
    // preserve and clear the joint accelerations
    VectorXd ddqorg(nj);
    VectorXd uorg(nj);
    for(int i = 0; i < nj; ++i){
        Link* joint = body->joint(i);
        ddqorg[i] = joint->ddq();
        uorg  [i] = joint->u();
        joint->ddq() = 0.0;
    }

    // preserve and clear the root link acceleration
    const Vector3 dvorg = rootLink->dv();
    const Vector3 dworg  = rootLink->dw();

    rootLink->dv() = g;
    rootLink->dw().setZero();

    out_b.resize(totaldof);
    setColumnOfMassMatrix(body, out_b, 0);

    if(!rootLink->isFixedJoint()){
        for(int i=0; i < 3; ++i){
            rootLink->dv()[i] += 1.0;
            setColumnOfMassMatrix(body, out_M, i);
            rootLink->dv()[i] -= 1.0;
        }
        for(int i=0; i < 3; ++i){
            rootLink->dw()[i] = 1.0;
            setColumnOfMassMatrix(body, out_M, i + 3);
            rootLink->dw()[i] = 0.0;
        }
    }

    for(int i = 0; i < nj; ++i){
        Link* joint = body->joint(i);
        joint->ddq() = 1.0;
        int j = i + 6;
        setColumnOfMassMatrix(body, out_M, j);
        out_M(j, j) += joint->Jm2(); // motor inertia
        joint->ddq() = 0.0;
    }

    // subtract the constant term
    for(int i = 0; i < out_M.cols(); ++i){
        out_M.col(i) -= out_b;
    }
    
    // recover state
    for(int i = 0; i < nj; ++i){
        Link* joint = body->joint(i);
        joint->ddq()  = ddqorg[i];
        joint->u()    = uorg  [i];
    }
    rootLink->dv() = dvorg;
    rootLink->dw() = dworg;
}

void calcMassMatrix(Body* body, MatrixXd& out_M)
{
    VectorXd b;
    calcMassMatrix(body, Vector3::Zero(), out_M, b);
}

}

