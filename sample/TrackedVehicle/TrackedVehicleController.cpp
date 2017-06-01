/**
   @author Shizuko Hattori
*/

#include <cnoid/SimpleController>
#include <cnoid/Joystick>

using namespace std;
using namespace cnoid;

class TrackedVehicleController : public SimpleController
{
    enum { TORQUE_MODE, VELOCITY_MODE } mode;
    Link* wheelR[4];
    Link* wheelL[4];
    Joystick joystick;

public:

    virtual bool initialize(SimpleControllerIO* io)
    {
        mode = TORQUE_MODE;
        vector<string> options = io->options();
        for(vector<string>::iterator p = options.begin(); p != options.end(); ++p){
            if(*p == "velocity"){
                mode = VELOCITY_MODE;
            }
        }

        if(mode == TORQUE_MODE){
            io->setJointOutput(JOINT_TORQUE);
            io->setJointInput(JOINT_ANGLE);
        } else if(mode == VELOCITY_MODE){
            io->setJointOutput(JOINT_VELOCITY);
        }

        Body* ioBody = io->body();

        wheelR[0] = ioBody->link("WHEEL_R0");
        wheelR[1] = ioBody->link("WHEEL_R1");
        wheelR[2] = ioBody->link("WHEEL_R2");
        wheelR[3] = ioBody->link("WHEEL_R3");
        wheelL[0] = ioBody->link("WHEEL_L0");
        wheelL[1] = ioBody->link("WHEEL_L1");
        wheelL[2] = ioBody->link("WHEEL_L2");
        wheelL[3] = ioBody->link("WHEEL_L3");
        
        return true;
    }

    virtual bool control()
    {
        joystick.readCurrentState();

        double pos[2];
        for(int i=0; i < 2; ++i){
            pos[i] = joystick.getPosition(i);
            if(fabs(pos[i]) < 0.2){
                pos[i] = 0.0;
            }
        }

        for(int i=0; i < 4; ++i){
            wheelR[i]->dq() = -5*(2.0 * pos[1] + pos[0]);
            wheelL[i]->dq() = -5*(2.0 * pos[1] - pos[0]);
        }

        return true;
    }
};

CNOID_IMPLEMENT_SIMPLE_CONTROLLER_FACTORY(TrackedVehicleController)
