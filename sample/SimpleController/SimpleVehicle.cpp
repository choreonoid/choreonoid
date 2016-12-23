#include <cnoid/SimpleController>
#include <cnoid/BodyState>
#include <cnoid/JointPath>
#include <cnoid/EigenUtil>
#include <iostream>

using namespace std;
using namespace cnoid;

#define DOF (4)
#define STEERING_ID 0
#define WHEEL_ID 1

#define STEERING_FILE "etc/steer.dat"

#define STEERING_P_GAIN 100.0
#define STEERING_D_GAIN 1.0
#define WHEEL_P_GAIN 100.0
#define WHEEL_D_GAIN 0.5
#define WHEEL_REF_VEL 6.28  // [rad/s]


class SampleSVController : public cnoid::SimpleController
{
    Body* body;
    double time;
    double wheel_ref;
    double steer_ref;
    double torque[2];
    double dt;

public:

    virtual bool initialize(SimpleControllerIO* io) {

        io->setJointOutput(JOINT_TORQUE);
        io->setJointInput(JOINT_ANGLE | JOINT_VELOCITY);

        wheel_ref = 0.0;
        steer_ref = 0.0;
        time = 0.0;

        body = io->body();
        for(int i=0; i<DOF; i++)
            body->link(i)->u() = 0.0;

        dt = io->timeStep();

        return true;
    }
    
    virtual bool control() {
#define T 4

        if(time >= T && time <T+1)
            steer_ref -= radian(60*dt);
        else if(time >= T+1 && time <T+2)
            steer_ref += radian(60*dt);
        else if ( time >= T+2 && time < T+4)
            ;
        else if(time >=T+4 && time < T+5)
            steer_ref -= radian(60*dt);
        else if(time >= T+5 && time <T+6)
            steer_ref += radian(60*dt);

        time += dt;

        double q = body->joint(STEERING_ID)->q();
        double dq = body->joint(STEERING_ID)->dq();
        torque[STEERING_ID] = (steer_ref - q) * STEERING_P_GAIN - dq * STEERING_D_GAIN;
        body->joint(STEERING_ID)->u() = torque[STEERING_ID];

        dq = body->joint(WHEEL_ID)->dq();
        torque[WHEEL_ID] = (WHEEL_REF_VEL - dq) * WHEEL_D_GAIN;
        body->joint(WHEEL_ID)->u() = torque[WHEEL_ID];


          return true;
    }

};


CNOID_IMPLEMENT_SIMPLE_CONTROLLER_FACTORY(SampleSVController);
