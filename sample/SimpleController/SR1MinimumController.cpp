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
    BodyPtr body;
    double dt;
    std::vector<double> qref;
    std::vector<double> qold;

public:

    virtual bool initialize()
    {
        body = ioBody();
        dt = timeStep();

        for(int i=0; i < body->numJoints(); ++i){
            qref.push_back(body->joint(i)->q());
        }
        qold = qref;
        
        return true;
    }

    virtual bool control()
    {
        for(int i=0; i < body->numJoints(); ++i){
            Link* joint = body->joint(i);
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
