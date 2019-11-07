/**
   \file
   \author Shin'ichiro Nakaoka
*/

#ifndef CNOID_BODY_FORWARD_DYNAMICS_CBM_H
#define CNOID_BODY_FORWARD_DYNAMICS_CBM_H

#include "ForwardDynamics.h"
#include "exportdecl.h"

namespace cnoid
{
class DyLink;
class ForceSensorDevice;

/**
   The ForwardDynamicsCBM class calculates the forward dynamics using
   the motion equation based on the generalized mass matrix.
   The class is mainly used for a body that has high-gain mode joints.
   If all the joints of a body are the torque mode, the ForwardDynamicsABM,
   which uses the articulated body method, is more efficient.
*/
class CNOID_EXPORT ForwardDynamicsCBM : public ForwardDynamics
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

    ForwardDynamicsCBM(DyBody* body);
    ~ForwardDynamicsCBM();

    virtual void initialize();
    virtual void calcNextState();

    void complementHighGainModeCommandValues();

    void initializeAccelSolver();
    void sumExternalForces();
    void solveUnknownAccels();
    void solveUnknownAccels(const Vector3& fext, const Vector3& tauext);
    bool solveUnknownAccels(DyLink* link, const Vector3& fext, const Vector3& tauext, const Vector3& rootfext, const Vector3& roottauext);

    //! \deprecated Use Link::setActuationMode() instead.
    void setHighGainModeForAllJoints();
    //! \deprecated Use Link::setActuationMode() instead.
    void setHighGainMode(int linkIndex, bool on = true);
        
private:
        
    /*
      Elements of the motion equation
		   
      |     |     |   | dv         |   | b1 |   | 0  |   | totalfext      |
      | M11 | M12 |   | dw         |   |    |   | 0  |   | totaltauext    |
      |     |     | * |ddq (unkown)| + |    | + | d1 | = | u (known)      |
      |-----+-----|   |------------|   |----|   |----|   |----------------|
      | M21 | M22 |   | given ddq  |   | b2 |   | d2 |   | u2             |
		
                   |fext  |
      d1 = trans(s)|      |
                   |tauext|

    */
		
    MatrixXd M11;
    MatrixXd M12;
    MatrixXd b1;
    VectorXd d1;
    VectorXd c1;

    std::vector<DyLink*> torqueModeJoints;
    std::vector<DyLink*> highGainModeJoints;

    //int rootDof; // dof of dv and dw (0 or 6)
    int unknown_rootDof;
    int given_rootDof;

    bool isNoUnknownAccelMode;

    VectorXd qGiven;
    VectorXd dqGiven;
    VectorXd ddqGiven;
    Vector3 pGiven;
    Matrix3 RGiven;
    Vector3 voGiven;
    Vector3 wGiven;

    bool accelSolverInitialized;
    bool ddqGivenCopied;

    VectorXd qGivenPrev;
    VectorXd dqGivenPrev;
    Vector3 pGivenPrev;
    Matrix3 RGivenPrev;
    Vector3 voGivenPrev;
    Vector3 wGivenPrev;

    Vector3 fextTotal;
    Vector3 tauextTotal;

    Vector3 root_w_x_v;

    // buffers for the unit vector method
    VectorXd ddqorg;
    VectorXd uorg;
    Vector3 dvoorg;
    Vector3 dworg;
		
    struct ForceSensorInfo
    {
        bool hasSensor;
        bool hasSensorsAbove;
        Vector3 f;
        Vector3 tau;
        ForceSensorInfo()
            : hasSensor(false),
              hasSensorsAbove(false),
              f(Vector3::Zero()),
              tau(Vector3::Zero())
            { }
    };

    std::vector<ForceSensorInfo> forceSensorInfo;

    // Buffers for the Runge Kutta Method
    Position T0;
    Vector3 vo0;
    Vector3 w0;
    std::vector<double> q0;
    std::vector<double> dq0;
		
    Vector3 vo;
    Vector3 w;
    Vector3 dvo;
    Vector3 dw;
    std::vector<double> dq;
    std::vector<double> ddq;

    virtual void initializeSensors();

    void calcMotionWithEulerMethod();
    void calcMotionWithRungeKuttaMethod();
    void integrateRungeKuttaOneStep(double r, double dt);
    void preserveHighGainModeJointState();
    void calcPositionAndVelocityFK();
    void calcMassMatrix();
    void setColumnOfMassMatrix(MatrixXd& M, int column);
    void calcInverseDynamics(DyLink* link, Vector3& out_f, Vector3& out_tau);
    void calcd1(DyLink* link, Vector3& out_f, Vector3& out_tau);
    inline void calcAccelFKandForceSensorValues();
    void calcAccelFKandForceSensorValues(DyLink* link, Vector3& out_f, Vector3& out_tau);
    void updateForceSensorInfo(DyLink* link, bool hasSensorsAbove);
    void updateForceSensors();
};

}

#endif
