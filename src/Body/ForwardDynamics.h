/**
   \file
   \author Shin'ichiro Nakaoka
*/

#ifndef CNOID_BODY_FORWARD_DYNAMICS_H
#define CNOID_BODY_FORWARD_DYNAMICS_H

#include "BasicSensorSimulationHelper.h"
#include "Link.h"
#include "exportdecl.h"

namespace cnoid {

class DyBody;
typedef ref_ptr<DyBody> DyBodyPtr;

/**
   This class calculates the forward dynamics of a Body object
   by using the Featherstone's articulated body algorithm.
   The class also integrates motion using the Euler method or RungeKutta method.
*/
class CNOID_EXPORT ForwardDynamics {

public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        
    ForwardDynamics(DyBody* body);
    virtual ~ForwardDynamics();
        
    void setGravityAcceleration(const Vector3& g);
    void setEulerMethod();
    void setRungeKuttaMethod();
    void setTimeStep(double timeStep);
    void enableSensors(bool on);
    void setOldAccelSensorCalcMode(bool on);

    virtual void initialize() = 0;
    virtual void calcNextState() = 0;

protected:

    virtual void initializeSensors();

    /**
       @brief update position/orientation using spatial velocity
       @param out_p p(t+dt)
       @param out_R R(t+dt)
       @param p0 p(t)
       @param R0 R(t)
       @param w angular velocity
       @param v0 spatial velocity
       @param dt time step[s]
    */
    static void SE3exp(Position& out_T, const Position& T0, const Vector3& w, const Vector3& vo, double dt);
		
    DyBodyPtr body;
    Vector3 g;
    double timeStep;
    bool sensorsEnabled;
    BasicSensorSimulationHelper sensorHelper;

    enum { EULER_METHOD, RUNGEKUTTA_METHOD } integrationMode;
};

}

#endif
