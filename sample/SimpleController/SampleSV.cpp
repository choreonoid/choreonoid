#include <cnoid/SimpleController>
#include <cnoid/BodyState>
#include <cnoid/JointPath>
#include <cnoid/EigenUtil>
#include "Interpolator.h"
#include <iostream>

using namespace std;
using namespace cnoid;
using namespace boost;


#define DOF (4)
#define STEERING_ID 0
#define WHEEL_ID 1

#define STEERING_FILE "etc/steer.dat"

#define STEERING_P_GAIN 100.0
#define STEERING_D_GAIN 1.0
#define WHEEL_P_GAIN 100.0
#define WHEEL_D_GAIN 0.5
#define WHEEL_REF_VEL 6  // [rad/s]

#define TIMESTEP 0.001

class SampleSVController : public cnoid::SimpleController
{
    double time;
    double wheel_ref;
    double steer_ref;
    double torque[2];

public:

    virtual bool initialize() {
        wheel_ref = 0.0;
        steer_ref = 0.0;
        time = 0.0;

        const BodyPtr& io = ioBody();
        for(int i=0; i<DOF; i++)
            io->joint(2)->u() = 0.0;

        return true;
    }
    
    virtual bool control() {

        if(time >= 20.0 && time <21.0)
            steer_ref -= radian(30*TIMESTEP);
        else if(time >= 21.0 && time <23.0)
            steer_ref += radian(30*TIMESTEP);
        else if ( time >= 23.0 && time < 24.0)
            steer_ref -= radian(30*TIMESTEP);
        else if(time >=24.0 && time < 40.0)
            ;
        else if(time >= 40.0 && time <41.0)
            steer_ref += radian(30*TIMESTEP);
        else if(time >= 41.0 && time <42.0)
            steer_ref -= radian(30*TIMESTEP);


        time += TIMESTEP;

        const BodyPtr& io = ioBody();

        double q = io->joint(STEERING_ID)->q();
        double dq = io->joint(STEERING_ID)->dq();
        torque[STEERING_ID] = (steer_ref - q) * STEERING_P_GAIN - dq * STEERING_D_GAIN;
        io->joint(STEERING_ID)->u() = torque[STEERING_ID];

        q = io->joint(WHEEL_ID)->q();
        dq = io->joint(WHEEL_ID)->dq();
        torque[WHEEL_ID] = (wheel_ref - q) * WHEEL_P_GAIN + (WHEEL_REF_VEL - dq) * WHEEL_D_GAIN;
        io->joint(WHEEL_ID)->u() = torque[WHEEL_ID];

        wheel_ref += WHEEL_REF_VEL * TIMESTEP;

          return true;
    }

};


CNOID_IMPLEMENT_SIMPLE_CONTROLLER_FACTORY(SampleSVController);
