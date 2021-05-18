/**
   \file
   \author Shin'ichiro Nakaoka
*/

#ifndef CNOID_BODY_FORWARD_DYNAMICS_H
#define CNOID_BODY_FORWARD_DYNAMICS_H

#include "BasicSensorSimulationHelper.h"
#include "exportdecl.h"

namespace cnoid {

class DySubBody;
typedef ref_ptr<DySubBody> DySubBodyPtr;

/**
   This class calculates the forward dynamics of a Body object
   by using the Featherstone's articulated body algorithm.
   The class also integrates motion using the Euler method or RungeKutta method.
*/
class CNOID_EXPORT ForwardDynamics {

public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        
    ForwardDynamics(DySubBody* subBody);
    virtual ~ForwardDynamics();
        
    void setGravityAcceleration(const Vector3& g);
    void setEulerMethod();
    void setRungeKuttaMethod();
    void setTimeStep(double timeStep);
    void enableSensors(bool on);
    void setOldAccelSensorCalcMode(bool on);

    virtual void initialize() = 0;
    virtual void calcNextState() = 0;
    virtual void refreshState() = 0;

protected:
    virtual void initializeSensors();

    /**
       @brief update position/orientation using spatial velocity
       @param out_T T(t+dt)
       @param T0 T(t)
       @param w angular velocity
       @param vo spatial velocity
       @param dt time step[s]
    */
    static void SE3exp(Isometry3& out_T, const Isometry3& T0, const Vector3& w, const Vector3& vo, double dt);
		
    DySubBodyPtr subBody;
    Vector3 g;
    double timeStep;
    bool sensorsEnabled;
    BasicSensorSimulationHelper sensorHelper;

    enum { EULER_METHOD, RUNGEKUTTA_METHOD } integrationMode;
};

}

#endif
