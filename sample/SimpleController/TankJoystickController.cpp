/**
   Tank Controller
   @author Shin'ichiro Nakaoka
*/

#include <cnoid/SimpleController>
#include <cnoid/Light>
#include <cnoid/Joystick>

using namespace std;
using namespace cnoid;

namespace {

const int turretAxis[] = { 3, 4 };
const int buttonIds[] = { 0, 1, 2, 3, 4, 5 };

}

class TankJoystickController : public cnoid::SimpleController
{ 
    Link* trackL;
    Link* trackR;
    Link* turretJoint[2];
    double qref[2];
    double qprev[2];
    double dt;
    LightPtr light;
    bool prevLightButtonState;
    Joystick joystick;

public:
    
    virtual bool initialize(SimpleControllerIO* io)
    {
        ostream& os = io->os();
        
        Body* body = io->body();
        trackL = body->link("TRACK_L");
        trackR = body->link("TRACK_R");
        if(!trackL || !trackR){
            os << "The tracks are not found." << endl;
            return false;
        }
        io->setLinkOutput(trackL, JOINT_VELOCITY);
        io->setLinkOutput(trackR, JOINT_VELOCITY);
        
        turretJoint[0] = body->link("TURRET_Y");
        turretJoint[1] = body->link("TURRET_P");
        for(int i=0; i < 2; ++i){
            Link* joint = turretJoint[i];
            if(!joint){
                os << "Turret joint " << i << " is not found." << endl;
                return false;
            }
            qref[i] = qprev[i] = joint->q();
            io->setLinkOutput(joint, JOINT_TORQUE);
            io->setLinkInput(joint, JOINT_ANGLE);
        }

        dt = io->timeStep();
        
        DeviceList<Light> lights(body->devices());
        if(!lights.empty()){
            light = lights.front();
        }
        prevLightButtonState = false;

        if(!joystick.isReady()){
            os << "Joystick is not ready: " << joystick.errorMessage() << endl;
        }
        if(joystick.numAxes() < 5){
            os << "The number of the joystick axes is not sufficient for controlling the robot." << endl;
        }
        if(joystick.numButtons() < 1){
            os << "The number of the joystick buttons is not sufficient for controlling the robot." << endl;
        }

        return true;
    }

    virtual bool control()
    {
        joystick.readCurrentState();
        
        static const double P = 200.0;
        static const double D = 50.0;

        for(int i=0; i < 2; ++i){
            Link* joint = turretJoint[i];
            double q = joint->q();
            double dq = (q - qprev[i]) / dt;
            double dqref = 0.0;
            double pos = joystick.getPosition(turretAxis[i]);
            if(fabs(pos) > 0.25){
                double deltaq = 0.002 * pos;
                qref[i] += deltaq;
                dqref = deltaq / dt;
            }
            joint->u() = P * (qref[i] - q) + D * (dqref - dq);
            qprev[i] = q;
        }

        double pos[2];
        for(int i=0; i < 2; ++i){
            pos[i] = joystick.getPosition(i);
            if(fabs(pos[i]) < 0.2){
                pos[i] = 0.0;
            }
        }
        // set the velocity of each tracks
        trackL->dq() = -2.0 * pos[1] + pos[0];
        trackR->dq() = -2.0 * pos[1] - pos[0];
        
        if(light){
            bool lightButtonState = joystick.getButtonState(buttonIds[0]);
            if(lightButtonState){
                if(!prevLightButtonState){
                    light->on(!light->on());
                    light->notifyStateChange();
                }
            }
            prevLightButtonState = lightButtonState;
        }

        return true;
    }
};

CNOID_IMPLEMENT_SIMPLE_CONTROLLER_FACTORY(TankJoystickController)
