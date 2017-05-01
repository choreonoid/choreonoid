
#include <cnoid/SimpleController>
#include <cnoid/Joystick>

using namespace cnoid;

class TankController2 : public SimpleController
{ 
    Link* joints[2];
    double qref[2];
    double qold[2];
    double dt;
    Joystick joystick;

public:
    virtual bool initialize(SimpleControllerIO* io)
    {
        Body* body = io->body();
        joints[0] = body->link("CANNON_Y");
        joints[1] = body->link("CANNON_P");
        for(int i=0; i < 2; ++i){
            Link* joint = joints[i];
            qref[i] = qold[i] = joint->q();
            io->setLinkOutput(joint, JOINT_TORQUE);
            io->setLinkInput(joint, JOINT_ANGLE);
        }

        dt = io->timeStep();
        
        return true;
    }

    virtual bool control()
    {
        joystick.readCurrentState();

        static const int cannonAxis[] = { 3, 4 };
        static const double cannonAxisRatio[] = { -0.002, 0.002 };
            
        static const double P = 200.0;
        static const double D = 50.0;

        for(int i=0; i < 2; ++i){
            Link* joint = joints[i];
            double q = joint->q();
            double dq = (q - qold[i]) / dt;
            double dqref = 0.0;

            double pos = joystick.getPosition(cannonAxis[i]);
            if(fabs(pos) > 0.2){
                double deltaq = cannonAxisRatio[i] * pos;
                qref[i] += deltaq;
                dqref = deltaq / dt;
            }
            
            joint->u() = P * (qref[i] - q) + D * (dqref - dq);
            qold[i] = q;
        }

        return true;
    }
};

CNOID_IMPLEMENT_SIMPLE_CONTROLLER_FACTORY(TankController2)
