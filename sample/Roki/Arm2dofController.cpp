#include <cnoid/SimpleController>
#include <cnoid/BodyState>
#include <cnoid/EigenUtil>
#include "Interpolator.h"

using namespace std;
using namespace cnoid;

#define ROKISAMPLE
#define DOF (2)
#define K 12000
#define C 120

class Arm2dofController : public cnoid::SimpleController
{

public:
    Body* ioBody;
    Interpolator<VectorXd> interpolator;
    VectorXd qref;
    double time;

    virtual bool initialize(SimpleControllerIO* io) override
    {
        ioBody = io->body();

        for(int i=0; i<DOF; i++){
            Link* joint = ioBody->joint(i);
            joint->setActuationMode(Link::JOINT_TORQUE);
            io->enableIO(joint);
            joint->u() = 0.0;
        }

#ifdef ROKISAMPLE
        qref.resize(DOF);
#else
        time = 0.0;
        qref.resize(DOF);
        interpolator.clear();
        VectorXd qf = VectorXd::Zero(DOF);
        interpolator.appendSample(0.0, qf);
        qf[0] = radian(60);
        interpolator.appendSample(1.0, qf);
        qf[0] = radian(-60);
        interpolator.appendSample(3.0, qf);
        qf[0] = 0.0;
        qf[1] = radian(60);
        interpolator.appendSample(5.0, qf);
        qf[1] = radian(-60);
        interpolator.appendSample(7.0, qf);
        interpolator.update();
#endif
        return true;
    }
    
    virtual bool control() {

#ifdef ROKISAMPLE
        qref[0] = qref[1] = radian(90);
#else
        qref = interpolator.interpolate(time);
#endif

        for(int i=0; i<DOF; i++){
            double q = ioBody->joint(i)->q();
            double dq = ioBody->joint(i)->dq();
            ioBody->joint(i)->u() = -K*(q-qref[i]) - C*dq;
        }

#ifndef ROKISAMPLE
        double dt = timeStep();
        time += dt;
#endif
        
        return true;
    }

};


CNOID_IMPLEMENT_SIMPLE_CONTROLLER_FACTORY(Arm2dofController);
