#include <cnoid/SimpleController>

using namespace cnoid;

class TankController1 : public SimpleController
{
    Link* pitchJoint;
    double qref;
    double qold;
    double dt;

public:
    virtual bool initialize(SimpleControllerIO* io)
    {
        pitchJoint = io->body()->link("TURRET_P");

        io->setLinkOutput(pitchJoint, JOINT_TORQUE);
        io->setLinkInput (pitchJoint, JOINT_ANGLE);

        qref = qold = pitchJoint->q();

        dt = io->timeStep();

        return true;
    }

    virtual bool control()
    {
        static const double P = 200.0;
        static const double D = 50.0;

        double q = pitchJoint->q();
        double dq = (q - qold) / dt;
        double dqref = 0.0;
        pitchJoint->u() = P * (qref - q) + D * (dqref - dq);
        qold = q;
        
        return true;
    }
};

CNOID_IMPLEMENT_SIMPLE_CONTROLLER_FACTORY(TankController1)
