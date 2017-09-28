#include <cnoid/SimpleController>
#include <cnoid/EigenUtil>

using namespace cnoid;

class SampleSVController : public SimpleController
{
    Body* body;
    Link* steeringJoint;
    Link* frontWheelJoint;
    double currentTime;
    double steer_ref;
    double dt;
    double err0;

public:

    virtual bool initialize(SimpleControllerIO* io)
    {
        body = io->body();

        steeringJoint = body->link("STEERING");
        io->enableIO(steeringJoint);
        
        frontWheelJoint = body->link("FRONT_WHEEL");
        io->enableInput(frontWheelJoint, JOINT_VELOCITY);
        io->enableOutput(frontWheelJoint);
        
        dt = io->timeStep();
        steer_ref = 0.0;
        currentTime = 0.0;
        err0 = 0;

        return true;
    }
    
    virtual bool control()
    {
        const double T0 = 4.0;

        if(currentTime >= T0 && currentTime < T0 + 1.0){
            steer_ref -= radian(60 * dt);
        } else if(currentTime >= T0 + 1.0 && currentTime < T0 + 2.0){
            steer_ref += radian(60 * dt);
        } else if(currentTime >= T0 + 4.0 && currentTime < T0 + 5.0){
            steer_ref -= radian(60 * dt);
        } else if(currentTime >= T0 + 5.0 && currentTime < T0 + 6.0){
            steer_ref += radian(60 * dt);
        }

        currentTime += dt;

        const double STEERING_P_GAIN = 100.0;
        const double STEERING_D_GAIN = 1.0;
        const double WHEEL_GAIN = 0.5;
        const double WHEEL_REF_VEL = 6.28; // [rad/s]

        double q = steeringJoint->q();
        double err = steer_ref - q;
        double derr = (err - err0) / dt;
        steeringJoint->u() = err * STEERING_P_GAIN + derr * STEERING_D_GAIN;
        err0 = err;

        frontWheelJoint->u() = (WHEEL_REF_VEL - frontWheelJoint->dq()) * WHEEL_GAIN;

        return true;
    }
};


CNOID_IMPLEMENT_SIMPLE_CONTROLLER_FACTORY(SampleSVController);
