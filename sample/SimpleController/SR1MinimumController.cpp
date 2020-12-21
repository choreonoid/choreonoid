/**
   @author Shin'ichiro Nakaoka
*/

#include <cnoid/SimpleController>
#include <vector>

using namespace cnoid;

const double pgain[] = {
    8000.0, 8000.0, 8000.0, 8000.0, 8000.0, 8000.0,
    3000.0, 3000.0, 3000.0, 3000.0, 3000.0, 3000.0, 3000.0, 
    8000.0, 8000.0, 8000.0, 8000.0, 8000.0, 8000.0,
    3000.0, 3000.0, 3000.0, 3000.0, 3000.0, 3000.0, 3000.0, 
    8000.0, 8000.0, 8000.0 };
    
const double dgain[] = {
    100.0, 100.0, 100.0, 100.0, 100.0, 100.0,
    100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0,
    100.0, 100.0, 100.0, 100.0, 100.0, 100.0,
    100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0,
    100.0, 100.0, 100.0 };

class SR1MinimumController : public SimpleController
{
    BodyPtr ioBody;
    double dt;
    std::vector<double> qref;
    std::vector<double> qold;

public:

    virtual bool initialize(SimpleControllerIO* io) override
    {
        ioBody = io->body();
        dt = io->timeStep();

        for(int i=0; i < ioBody->numJoints(); ++i){
            Link* joint = ioBody->joint(i);
            joint->setActuationMode(Link::JointTorque);
            io->enableIO(joint);
            qref.push_back(joint->q());
        }
        qold = qref;

        return true;
    }

    virtual bool control() override
    {
        for(int i=0; i < ioBody->numJoints(); ++i){
            Link* joint = ioBody->joint(i);
            double q = joint->q();
            double dq = (q - qold[i]) / dt;
            double u = (qref[i] - q) * pgain[i] + (0.0 - dq) * dgain[i];
            qold[i] = q;
            joint->u() = u;
        }
        return true;
    }
};

CNOID_IMPLEMENT_SIMPLE_CONTROLLER_FACTORY(SR1MinimumController)
