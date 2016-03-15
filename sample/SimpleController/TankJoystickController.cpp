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

const int cannonAxis[] = { 3, 4 };
const double cannonAxisRatio[] = { -1.0, 1.0 };
const int buttonIds[] = { 0, 1, 2, 3, 4, 5 };

}

class TankJoystickController : public cnoid::SimpleController
{ 
    Link* crawlerL;
    Link* crawlerR;
    Link* cannonJoint[2];
    double qref[2];
    double qprev[2];
    LightPtr light;
    bool prevLightButtonState;
    Joystick joystick;

public:
    
    virtual bool initialize(SimpleControllerIO* io) {

        ostream& os = io->os();
        
        Body* body = ioBody();
        crawlerL = body->link("CRAWLER_TRACK_L");
        crawlerR = body->link("CRAWLER_TRACK_R");
        if(!crawlerL || !crawlerR){
            os << "The crawlers are not found." << endl;
            return false;
        }
        cannonJoint[0] = body->link("CANNON_Y");
        cannonJoint[1] = body->link("CANNON_P");
        for(int i=0; i < 2; ++i){
            if(!cannonJoint[i]){
                os << "Cannon joint " << i << " is not found." << endl;
                return false;
            }
            double q = cannonJoint[i]->q();
            qref[i] = q;
            qprev[i] = q;
        }

        DeviceList<Light> lights(body->devices());
        if(!lights.empty()){
            light = lights.front();
        }
        prevLightButtonState = false;

        io->setJointOutput(JOINT_TORQUE);
        io->setJointInput(JOINT_ANGLE);

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

    virtual bool control() {

        joystick.readCurrentState();
        
        static const double P = 200.0;
        static const double D = 50.0;

        for(int i=0; i < 2; ++i){
            Link* joint = cannonJoint[i];
            double q = joint->q();
            double dq = (q - qprev[i]) / timeStep();
            double dqref = 0.0;
            double command = cannonAxisRatio[i] * joystick.getPosition(cannonAxis[i]);
            if(fabs(command) > 0.2){
                double deltaq = command * 0.002;
                qref[i] += deltaq;
                dqref = deltaq / timeStep();
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
        // set the velocity of each crawlers
        crawlerL->u() = -2.0 * pos[1] + pos[0];
        crawlerR->u() = -2.0 * pos[1] - pos[0];
        
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
