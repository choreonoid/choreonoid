/**
   Tank Controller
   @author Shin'ichiro Nakaoka
*/

#include <cnoid/SimpleController>
#include <cnoid/SpotLight>
#include <cnoid/RangeCamera>
#include <cnoid/RangeSensor>
#include <cnoid/Joystick>

using namespace std;
using namespace cnoid;

class TankJoystickController : public SimpleController
{
    SimpleControllerIO* io;
    bool usePseudoContinousTrackMode;
    int turretActuationMode;
    Link* trackL;
    Link* trackR;
    Link* turretJoint[2];
    double qref[2];
    double qprev[2];
    double dt;

    struct DeviceInfo {
        DevicePtr device;
        int buttonId;
        bool prevButtonState;
        bool stateChanged;
        DeviceInfo(Device* device, int buttonId)
            : device(device),
              buttonId(buttonId),
              prevButtonState(false),
              stateChanged(false)
        { }
    };
    vector<DeviceInfo> devices;
    SpotLightPtr spotLight;
    
    Joystick joystick;

public:
    
    virtual bool initialize(SimpleControllerIO* io) override
    {
        this->io = io;
        ostream& os = io->os();
        Body* body = io->body();

        usePseudoContinousTrackMode = true;
        turretActuationMode = Link::JointEffort;
        for(auto opt : io->options()){
            if(opt == "wheels"){
                usePseudoContinousTrackMode = false;
            }
            if(opt == "position"){
                turretActuationMode = Link::JointDisplacement;
                os << "The joint-position command mode is used." << endl;
            }
            if(opt == "velocity"){
                turretActuationMode = Link::JointVelocity;
                os << "The joint-velocity command mode is used." << endl;
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

        io->enableOutput(trackL, JointVelocity);
        io->enableOutput(trackR, JointVelocity);
        
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

        devices = {
            { body->findDevice<SpotLight>("Light"),    Joystick::A_BUTTON },
            { body->findDevice<RangeCamera>("Kinect"), Joystick::B_BUTTON },
            { body->findDevice<Camera>("Theta"),       Joystick::X_BUTTON },
            { body->findDevice<RangeSensor>("VLP-16"), Joystick::Y_BUTTON }
        };
        spotLight = dynamic_pointer_cast<SpotLight>(devices[0].device);

        // Turn on all the devices
        for(auto& device : devices){
            device.device->on(true);
            device.device->notifyStateChange();
        }

        return true;
    }

    virtual bool control() override
    {
        joystick.readCurrentState();
        
        double pos[2];
        for(int i=0; i < 2; ++i){
            pos[i] = joystick.getPosition(
                i==0 ? Joystick::L_STICK_H_AXIS : Joystick::L_STICK_V_AXIS);
            if(fabs(pos[i]) < 0.2){
                pos[i] = 0.0;
            }
        }
        // set the velocity of each tracks
        if(usePseudoContinousTrackMode){
            double k = 1.0;
            trackL->dq_target() = k * (-2.0 * pos[1] + pos[0]);
            trackR->dq_target() = k * (-2.0 * pos[1] - pos[0]);
        } else {
            double k = 4.0;
            trackL->dq_target() = k * (-pos[1] + pos[0]);
            trackR->dq_target() = k * (-pos[1] - pos[0]);
        }

        static const double P = 200.0;
        static const double D = 50.0;

        for(int i=0; i < 2; ++i){
            Link* joint = turretJoint[i];
            double pos = joystick.getPosition(
                i==0 ? Joystick::R_STICK_H_AXIS : Joystick::R_STICK_V_AXIS);
            if(fabs(pos) < 0.15){
                pos = 0.0;
            }

            if(turretActuationMode == Link::JointDisplacement){
                joint->q_target() = joint->q() + pos * dt;

            } else if(turretActuationMode == Link::JointVelocity){
                joint->dq_target() = pos;

            } else if(turretActuationMode == Link::JointEffort){
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

        for(auto& info : devices){
            if(info.device){
                bool stateChanged = false;
                bool buttonState = joystick.getButtonState(info.buttonId);
                if(buttonState && !info.prevButtonState){
                    info.device->on(!info.device->on());
                    stateChanged = true;
                }
                auto spotLight = dynamic_pointer_cast<SpotLight>(info.device);
                if(spotLight){
                    if(joystick.getPosition(Joystick::R_TRIGGER_AXIS) > 0.1){
                        spotLight->setBeamWidth(
                            std::max(0.1f, spotLight->beamWidth() - 0.001f));
                        stateChanged = true;
                    } else if(joystick.getButtonState(Joystick::R_BUTTON)){
                        spotLight->setBeamWidth(
                            std::min(0.7854f, spotLight->beamWidth() + 0.001f));
                        stateChanged = true;
                    }
                }
                info.prevButtonState = buttonState;
                if(stateChanged){
                    info.device->notifyStateChange();
                }
            }
        }

        return true;
    }
};

CNOID_IMPLEMENT_SIMPLE_CONTROLLER_FACTORY(TankJoystickController)
