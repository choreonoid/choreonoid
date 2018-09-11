/**
   Tank Controller
   @author Shin'ichiro Nakaoka
*/

#include <cnoid/SimpleController>
#include <cnoid/SpotLight>
#include <cnoid/Joystick>

using namespace std;
using namespace cnoid;

namespace {

const int axisID[] = { 0, 1, 2, 3 };
const int buttonID[] = { 0, 2, 3 };

}

class TankJoystickController : public SimpleController
{
    bool usePseudoContinousTrackMode;
    Link::ActuationMode turretActuationMode;
    Link* trackL;
    Link* trackR;
    Link* turretJoint[2];
    double qref[2];
    double qprev[2];
    double dt;
    LightPtr light;
    SpotLightPtr spotLight;
    bool prevLightButtonState;
    Joystick joystick;

public:
    
    virtual bool initialize(SimpleControllerIO* io) override
    {
        ostream& os = io->os();
        Body* body = io->body();

        usePseudoContinousTrackMode = true;
        turretActuationMode = Link::ActuationMode::JOINT_TORQUE;
        for(auto opt : io->options()){
            if(opt == "wheels"){
                usePseudoContinousTrackMode = false;
            }
            if(opt == "velocity"){
                turretActuationMode = Link::ActuationMode::JOINT_VELOCITY;
            }
        }

        if(usePseudoContinousTrackMode){
            trackL = body->link("TRACK_L");
            trackR = body->link("TRACK_R");

        } else {
            trackL = body->link("WHEEL_L0");
            trackR = body->link("WHEEL_R0");
        }

        if(!trackL || !trackR){
            os << "The tracks are not found." << endl;
            return false;
        }

        if(usePseudoContinousTrackMode){
            trackL->setActuationMode(Link::JOINT_SURFACE_VELOCITY);
            trackR->setActuationMode(Link::JOINT_SURFACE_VELOCITY);
        } else {
            trackL->setActuationMode(Link::JOINT_VELOCITY);
            trackR->setActuationMode(Link::JOINT_VELOCITY);
        }
        io->enableOutput(trackL);
        io->enableOutput(trackR);
        
        turretJoint[0] = body->link("TURRET_Y");
        turretJoint[1] = body->link("TURRET_P");
        for(int i=0; i < 2; ++i){
            Link* joint = turretJoint[i];
            if(!joint){
                os << "Turret joint " << i << " is not found." << endl;
                return false;
            }
            qref[i] = qprev[i] = joint->q();
            joint->setActuationMode(turretActuationMode);
            io->enableIO(joint);
        }

        dt = io->timeStep();
        
        DeviceList<Light> lights(body->devices());
        if(!lights.empty()){
            light = lights.front();
            spotLight = dynamic_pointer_cast<SpotLight>(light);
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

    virtual bool control() override
    {
        joystick.readCurrentState();
        
        double pos[2];
        for(int i=0; i < 2; ++i){
            pos[i] = joystick.getPosition(axisID[i]);
            if(fabs(pos[i]) < 0.2){
                pos[i] = 0.0;
            }
        }
        // set the velocity of each tracks
        if(usePseudoContinousTrackMode){
            double k = 1.0;
            trackL->dq_target() = k * (-2.0 * pos[1] + pos[0]);
            trackR->dq_target() = k * (-2.0 * pos[1] - pos[0]);

        }else{
            double k = 4.0;
            trackL->dq_target() = k * (-pos[1] + pos[0]);
            trackR->dq_target() = k * (-pos[1] - pos[0]);
        }

        static const double P = 200.0;
        static const double D = 50.0;

        for(int i=0; i < 2; ++i){

            Link* joint = turretJoint[i];
            double pos = joystick.getPosition(axisID[i + 2]);
            if(fabs(pos) < 0.15){
                pos = 0.0;
            }

            if(turretActuationMode == Link::JOINT_VELOCITY){
                joint->dq_target() = pos;

            } else if(turretActuationMode == Link::JOINT_TORQUE){
                double q = joint->q();
                double dq = (q - qprev[i]) / dt;
                double dqref = 0.0;
                double deltaq = 0.002 * pos;
                qref[i] += deltaq;
                dqref = deltaq / dt;
                joint->u() = P * (qref[i] - q) + D * (dqref - dq);
                qprev[i] = q;
            }
        }

        if(light){
            bool changed = false;
            
            bool lightButtonState = joystick.getButtonState(buttonID[0]);
            if(lightButtonState){
                if(!prevLightButtonState){
                    light->on(!light->on());
                    changed = true;
                }
            }
            prevLightButtonState = lightButtonState;

            if(spotLight){
                if(joystick.getButtonState(buttonID[1])){
                    spotLight->setBeamWidth(std::max(0.1f, spotLight->beamWidth() - 0.001f));
                    changed = true;
                } else if(joystick.getButtonState(buttonID[2])){
                    spotLight->setBeamWidth(std::min(0.7854f, spotLight->beamWidth() + 0.001f));
                    changed = true;
                }
            }

            if(changed){
                light->notifyStateChange();
            }
        }

        return true;
    }
};

CNOID_IMPLEMENT_SIMPLE_CONTROLLER_FACTORY(TankJoystickController)
