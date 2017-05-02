#include <cnoid/SimpleController>

using namespace cnoid;

class TurretController1 : public SimpleController
{
    Link* joint;
    double qref;
    double qold;
    double dt;

public:
    virtual bool initialize(SimpleControllerIO* io)
    {
        joint = io->body()->link("TURRET_P");

        io->setLinkOutput(joint, JOINT_TORQUE);
        io->setLinkInput (joint, JOINT_ANGLE);

        qref = qold = joint->q();

        dt = io->timeStep();

        return true;
    }

    virtual bool control()
    {
        // PD gains
        static const double P = 200.0;
        static const double D = 50.0;

        double q = joint->q(); // input
        double dq = (q - qold) / dt;
        double dqref = 0.0;
        joint->u() = P * (qref - q) + D * (dqref - dq); // output
        qold = q;
        
        return true;
    }
};

CNOID_IMPLEMENT_SIMPLE_CONTROLLER_FACTORY(TurretController1)
