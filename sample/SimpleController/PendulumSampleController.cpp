/**
   Sample controller swinging up a pendulum
   @author Yuki Onishi
*/

#include <cnoid/SimpleController>
#include <cnoid/JointPath>
#include <cnoid/EigenUtil>
#include "Interpolator.h"

#include <iostream>

using namespace std;
using namespace cnoid;

namespace {

const double pgain = 10.0;
const double dgain = 10.0;

class PendulumSampleController : public SimpleController
{
    Body* ioBody;
    Link* joint;

    double dt;
    double qPrev;

public:

    virtual bool initialize(SimpleControllerIO* io) override
    {
        ioBody = io->body();
        joint = ioBody->joint("ARM");
        joint->setActuationMode(Link::JointTorque);
        io->enableIO(joint);

        dt = io->timeStep();
        qPrev = joint->q();

        return true;
    }

    virtual bool control() override
    {
        const double q = joint->q();
        const double dq = (q - qPrev) / dt;
        const double u = - pgain * q - dgain * dq;
        joint->u() = u;
        std::cout << q << ", " << dq << ", " << u << std::endl;

        qPrev = q;

        return true;
    }
};

}

CNOID_IMPLEMENT_SIMPLE_CONTROLLER_FACTORY(PendulumSampleController)
