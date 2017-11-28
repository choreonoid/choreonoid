/**
   \author Shin'ichiro Nakaoka
*/

#include "MassMatrix.h"
#include "Link.h"
#include "InverseDynamics.h"

using namespace cnoid;

namespace {

void setColumnOfMassMatrix(Body* body, MatrixXd& out_M, int column)
{
    Link* rootLink = body->rootLink();
    Vector6 f = calcInverseDynamics(rootLink);
        
    if(!rootLink->isFixedJoint()){
        f.tail<3>() -= rootLink->p().cross(f.head<3>());
        out_M.block<6, 1>(0, column) = f;
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
   |       |   | dv   |   |    |   | fext      |
   | out_M | * | dw   | + | b1 | = | tauext    |
   |       |   |ddq   |   |    |   | u         |
*/
void calcMassMatrix(Body* body, const Vector3& g, Eigen::MatrixXd& out_M)
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

    //const Vector3 root_w_x_v = rootLink->w().cross(rootLink->vo() + rootLink->w().cross(rootLink->p()));

    rootLink->dv() = g;
    rootLink->dw().setZero();
	
    MatrixXd b1(totaldof, 1);
        
    setColumnOfMassMatrix(body, b1, 0);

    if(!rootLink->isFixedJoint()){
        for(int i=0; i < 3; ++i){
            rootLink->dv()[i] += 1.0;
            setColumnOfMassMatrix(body, out_M, i);
            rootLink->dv()[i] -= 0.0;
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
        out_M.col(i) -= b1;
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


void calcMassMatrix(const BodyPtr& body, MatrixXd& out_M)
{
    const Vector3 g(0, 0, 9.8);
    calcMassMatrix(body, g, out_M);
}


void calcMassMatrix(Body* body, MatrixXd& out_M)
{
    const Vector3 g(0, 0, 9.8);
    calcMassMatrix(body, g, out_M);
}

}

