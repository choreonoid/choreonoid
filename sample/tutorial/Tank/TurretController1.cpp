#include <cnoid/SimpleController>

using namespace cnoid;

class TurretController1 : public SimpleController
{
    Link* joint;
    double q_ref;
    double q_prev;
    double dt;

public:
    virtual bool initialize(SimpleControllerIO* io) override
    {
        joint = io->body()->link("TURRET_P");
        joint->setActuationMode(Link::JOINT_TORQUE);
        io->enableIO(joint);
        q_ref = q_prev = joint->q();

        dt = io->timeStep();

        return true;
    }

    virtual bool control() override
    {
        // PD gains
        static const double P = 200.0;
        static const double D = 50.0;

        double q = joint->q(); // input
        double dq = (q - q_prev) / dt;
        double dq_ref = 0.0;
        joint->u() = P * (q_ref - q) + D * (dq_ref - dq); // output
        q_prev = q;
        
        return true;
    }
};

CNOID_IMPLEMENT_SIMPLE_CONTROLLER_FACTORY(TurretController1)
