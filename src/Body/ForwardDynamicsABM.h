/**
   \file
   \author Shin'ichiro Nakaoka
*/

#ifndef CNOID_BODY_FORWARD_DYNAMICS_ABM_H
#define CNOID_BODY_FORWARD_DYNAMICS_ABM_H

#include "ForwardDynamics.h"
#include "exportdecl.h"

namespace cnoid
{
/**
   Forward dynamics calculation using Featherstone's Articulated Body Method (ABM)
*/
class CNOID_EXPORT ForwardDynamicsABM : public ForwardDynamics
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        
    ForwardDynamicsABM(DySubBody* subBody);
    ~ForwardDynamicsABM();
        
    virtual void initialize();
    virtual void calcNextState();
    virtual void refreshState();
    
private:
        
    void calcMotionWithEulerMethod();
    void integrateRungeKuttaOneStep(double r, double dt);
    void calcMotionWithRungeKuttaMethod();

    /**
       compute position/orientation/velocity
    */
    void calcABMPhase1(bool updateNonSpatialVariables);

    /**
       compute articulated inertia
    */
    void calcABMPhase2();
    void calcABMPhase2Part1();
    void calcABMPhase2Part2();

    /**
       compute joint acceleration/spatial acceleration
    */
    void calcABMPhase3();

    inline void calcABMFirstHalf();
    inline void calcABMLastHalf();

    void updateForceSensors();

    // Buffers for the Runge Kutta Method
    Isometry3 T0;
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
};
	
}

#endif
