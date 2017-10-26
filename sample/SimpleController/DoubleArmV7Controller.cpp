#include <cnoid/SimpleController>
#include <cnoid/Joystick>

using namespace std;
using namespace cnoid;

class DoubleArmV7Controller : public cnoid::SimpleController
{
    Body* body;
    double dt;

    vector<Link*> joints;
    vector<double> qref;
    vector<double> qold;
    vector<double> pgain;
    vector<double> dgain;

    Link* trackL;
    Link* trackR;
    bool hasPseudoContinuousTracks;

    Joystick joystick;

public:
    DoubleArmV7Controller()
    {
        pgain = {
            /* MFRAME */ 200000, /* BLOCK */ 150000, /* BOOM */ 150000, /* ARM  */ 100000,
            /* PITCH  */  30000, /* ROLL  */  20000, /* TIP1 */    500, /* TIP2 */    500,
            /* UFRAME */ 150000, /* SWING */ 100000, /* BOOM */ 100000, /* ARM  */  80000,
            /* ELBOW */   30000, /* YAW   */  20000, /* HAND */    500, /* ROD  */  50000
        };
        dgain = {
            /* MFRAME */ 20000, /* BLOCK */ 15000, /* BOOM */ 10000, /* ARM  */ 5000,
            /* PITCH  */   500, /* ROLL  */   500, /* TIP1 */    50, /* TIP2 */   50,
            /* UFRAME */ 15000, /* SWING */  2000, /* BOOM */  3000, /* ARM  */ 2000,
            /* ELBOW */    500, /* YAW   */   500, /* HAND */    20, /* ROD  */ 5000
        };
        
    }        
    
    virtual bool initialize(SimpleControllerIO* io)
    {
        body = io->body();
        dt = io->timeStep();

        hasPseudoContinuousTracks = false;
        trackL = body->link("TRACK_L");
        trackR = body->link("TRACK_R");
        if(trackL && trackR){
            if(trackL->actuationMode() == Link::JOINT_SURFACE_VELOCITY &&
               trackR->actuationMode() == Link::JOINT_SURFACE_VELOCITY   ){
                io->enableOutput(trackL);
                io->enableOutput(trackR);
                hasPseudoContinuousTracks = true;
            }
        }

        for(auto joint : body->joints()){
            if(joint->isRevoluteJoint() || joint->isPrismaticJoint()){
                if(joint->jointId() >= 0){
                    joint->setActuationMode(Link::JOINT_EFFORT);
                    io->enableIO(joint);
                    joints.push_back(joint);
                    qref.push_back(joint->q());
                }
            }
        }
        qold = qref;

        return true;
    }

    virtual bool control() override
    {
        joystick.readCurrentState();
        
        double pos[2];
        for(int i=0; i < 2; ++i){
            pos[i] = joystick.getPosition(i);
            if(fabs(pos[i]) < 0.2){
                pos[i] = 0.0;
            }
        }
        // set the velocity of each track
        if(hasPseudoContinuousTracks){
            double k = 1.0;
            trackL->dq() = k * (-2.0 * pos[1] + pos[0]);
            trackR->dq() = k * (-2.0 * pos[1] - pos[0]);
        }

        for(size_t i=0; i < joints.size(); ++i){
            Link* joint = joints[i];
            double q = joint->q();
            double dq = (q - qold[i]) / dt;
            joint->u() = (qref[i] - q) * pgain[i] + (0.0 - dq) * dgain[i];
            qold[i] = q;
        }

        return true;
    }
};

CNOID_IMPLEMENT_SIMPLE_CONTROLLER_FACTORY(DoubleArmV7Controller)
