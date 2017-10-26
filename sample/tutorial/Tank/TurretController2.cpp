#include <cnoid/SimpleController>
#include <cnoid/Joystick>

using namespace cnoid;

class TurretController2 : public SimpleController
{ 
    Link* joints[2];
    double q_ref[2];
    double q_prev[2];
    double dt;
    Joystick joystick;

public:
    virtual bool initialize(SimpleControllerIO* io) override
    {
        joints[0] = io->body()->link("TURRET_Y");
        joints[1] = io->body()->link("TURRET_P");

        for(int i=0; i < 2; ++i){
            Link* joint = joints[i];
            joint->setActuationMode(Link::JOINT_TORQUE);
            io->enableIO(joint);
            q_ref[i] = q_prev[i] = joint->q();
        }

        dt = io->timeStep();
        
        return true;
    }

    virtual bool control() override
    {
        static const double P = 200.0;
        static const double D = 50.0;
        static const int axisID[] = { 2, 3 };

        joystick.readCurrentState();

        for(int i=0; i < 2; ++i){
            Link* joint = joints[i];
            double q = joint->q();
            double dq = (q - q_prev[i]) / dt;
            double dq_ref = 0.0;

            double pos = joystick.getPosition(axisID[i]);
            if(fabs(pos) > 0.25){
                double deltaq = 0.002 * pos;
                q_ref[i] += deltaq;
                dq_ref = deltaq / dt;
            }
            
            joint->u() = P * (q_ref[i] - q) + D * (dq_ref - dq);
            q_prev[i] = q;
        }

        return true;
    }
};

CNOID_IMPLEMENT_SIMPLE_CONTROLLER_FACTORY(TurretController2)
