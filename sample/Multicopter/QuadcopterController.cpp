/**
   Copyright (c) 2018 Japan Atomic Energy Agency (JAEA).
   The original version is implemented as an RT-component.
   This is a simple controller version modified by AIST.
*/

#include <cnoid/SimpleController>
#include <cnoid/RotorDevice>
#include <cnoid/SharedJoystick>
#include <cnoid/EigenUtil>

using namespace std;
using namespace cnoid;

namespace {

const string propname[] = { "PROP0", "PROP1", "PROP2", "PROP3" };
const std::string rotorname[] = { "RotorDevice0", "RotorDevice1", "RotorDevice2", "RotorDevice3" };

const int rotorAxis[] = {
    Joystick::L_STICK_V_AXIS,
    Joystick::R_STICK_H_AXIS,
    Joystick::R_STICK_V_AXIS,
    Joystick::L_STICK_H_AXIS
};

const int cameraAxis = Joystick::DIRECTIONAL_PAD_V_AXIS;
const int powerButton = Joystick::A_BUTTON;

const double sign[4][4] = {
    { 1.0, -1.0, -1.0,  1.0 },
    { 1.0,  1.0, -1.0, -1.0 },
    { 1.0,  1.0,  1.0,  1.0 },
    { 1.0, -1.0,  1.0, -1.0 }
};
const double dir[] = { 1.0, -1.0, 1.0, -1.0 };

static const double KP[] = { 0.4, 0.4, 0.4, 1.0 };
static const double KD[] = { 0.02, 1.0, 1.0, 0.05 };
const double RATE[] = { -1.0, 0.1, -0.1, -0.5 };

// For the stable mode
const double KPX[] = { 0.4, 0.4 };
const double KDX[] = { 0.4, 0.4 };
const double RATEX[] = { -1.0, -1.0 };

class QuadcopterController : public SimpleController
{
public:
    SharedJoystickPtr joystick;
    int targetMode;
    BodyPtr ioBody;
    ostream* os;
    Link* prop[4];
    Multicopter::RotorDevice* rotor[4];
    Link* cameraT;
    double timeStep;

    Vector4 zrpyref;
    Vector4 zrpyprev;
    Vector4 dzrpyref;
    Vector4 dzrpyprev;

    // For the stable mode
    bool isStableMode;
    bool prevModeButtonState;
    Vector2 xyref;
    Vector2 xyprev;
    Vector2 dxyref;
    Vector2 dxyprev;
    
    double qref;
    double qprev;
    bool power;
    bool powerprev;
    bool rotorswitch;

    virtual bool initialize(SimpleControllerIO* io) override;
    Vector4 getZRPY();
    Vector2 getXY();
    virtual bool control() override;
};

}


bool QuadcopterController::initialize(SimpleControllerIO* io)
{
    ioBody = io->body();
    os = &io->os();
    timeStep = io->timeStep();

    io->enableInput(ioBody->rootLink(), LINK_POSITION);

    cameraT = ioBody->link("CAMERA_T");
    cameraT->setActuationMode(Link::JOINT_TORQUE);
    io->enableIO(cameraT);
    qref = qprev = cameraT->q();

    for(int i = 0; i < 4; i++) {
        prop[i] = ioBody->link(propname[i]);
        prop[i]->setActuationMode(Link::JOINT_TORQUE);
        io->enableInput(prop[i], JOINT_VELOCITY);
        io->enableOutput(prop[i]);

        rotor[i] = ioBody->findDevice<Multicopter::RotorDevice>(rotorname[i]);
        io->enableInput(rotor[i]);
    }

    zrpyref = zrpyprev = getZRPY();
    dzrpyref = dzrpyprev = Vector4::Zero();

    isStableMode = false;
    prevModeButtonState = false;
    xyref = xyprev = getXY();
    dxyref = dxyprev = Vector2::Zero();

    power = powerprev = false;

    joystick = io->getOrCreateSharedObject<SharedJoystick>("joystick");
    targetMode = joystick->addMode();
    rotorswitch = false;

    return true;
}


Vector4 QuadcopterController::getZRPY()
{
    auto T = ioBody->rootLink()->position();
    double z = T.translation().z();
    Vector3 rpy = rpyFromRot(T.rotation());
    return Vector4(z, rpy[0], rpy[1], rpy[2]);
}
    

Vector2 QuadcopterController::getXY()
{
    auto T = ioBody->rootLink()->position();
    double x = T.translation().x();
    double y = T.translation().y();
    return Vector2(y, x);
}


bool QuadcopterController::control()
{
    joystick->updateState(targetMode);

    //control rotors
    Vector4 zrpy = getZRPY();
    Vector2 xy = getXY();
    double cc = cos(zrpy[1]) * cos(zrpy[2]);
    double gfcoef = 1.0 * 9.80665 / 4 / cc ;
    Vector4 force = Vector4::Zero();
    Vector4 torque = Vector4::Zero();
    
    power = joystick->getButtonState(targetMode, powerButton);
    if(power == false) {
        powerprev = false;
    } else if((power == true) && (powerprev != true)) {
        rotorswitch = !rotorswitch;
        powerprev = true;
    }

    bool modeButtonState = joystick->getButtonState(Joystick::R_STICK_BUTTON);
    if(modeButtonState){
        if(!prevModeButtonState){
            isStableMode = !isStableMode;
        }
    }
    prevModeButtonState = modeButtonState;

    if(rotorswitch) {
        Vector4 f;
        for(int i = 0; i < 4; ++i) {
            double dzrpy = (zrpy[i] - zrpyprev[i]) / timeStep;
            if(fabs(dzrpy) > (M_PI / timeStep)) {
                dzrpy = 0.0;
            }
            double ddzrpy = (dzrpy - dzrpyprev[i]) / timeStep;
            double pos = joystick->getPosition(targetMode, rotorAxis[i]);

            if((i == 0) || (i == 3)) {
                if(fabs(pos) > 0.25) {
                    dzrpyref[i] = RATE[i] * pos;
                } else {
                    dzrpyref[i] = 0.0;
                }
                f[i] = KP[i] * (dzrpyref[i] - dzrpy) + KD[i] * (0.0 - ddzrpy);

            } else {

                if(isStableMode){
                    if(fabs(pos) > 0.25) {
                        dxyref[i - 1] = RATEX[i - 1] * pos;
                    } else {
                        dxyref[i - 1] = 0.0;
                    }

                    double dxy = (xy[i - 1] - xyprev[i - 1]) /timeStep;
                    double ddxy = (dxy - dxyprev[i - 1]) /timeStep;
                    
                    zrpyref[i] = KPX[i - 1] * (dxyref[i - 1] - dxy) + KDX[i - 1] * (0.0 - ddxy);
                    if(i == 1) {
                        zrpyref[i] *= -1.0;
                    }
                    xyprev[i - 1] = xy[i - 1];
                    dxyprev[i - 1] = dxy;

                } else {                    
                    if(fabs(pos) > 0.25) {
                        zrpyref[i] = RATE[i] * pos;
                    } else {
                        zrpyref[i] = 0.0;
                    }
                }
                f[i] = KP[i] * (zrpyref[i] - zrpy[i]) + KD[i] * (0.0 - dzrpy);
            }
            zrpyprev[i] = zrpy[i];
            dzrpyprev[i] = dzrpy;
        }
        for(int i = 0; i < 4; ++i) {
            double fi = 0.0;
            fi += gfcoef;
            fi += sign[i][0] * f[0];
            fi += sign[i][1] * f[1];
            fi += sign[i][2] * f[2];
            fi += sign[i][3] * f[3];
            force[i] = fi;
            torque[i] = dir[i] * fi;
        }
    }

    for(int i = 0; i < 4; ++i) {
        double tau = torque[i];
        rotor[i]->setTorque(tau);
        if(tau != 0.0) {
            prop[i]->u() = tau * 0.001;
        } else {
            double dq = prop[i]->dq();
            prop[i]->u() = 0.0005 * (0.0 - dq);
        }
        rotor[i]->setValue(force[i]);
        rotor[i]->notifyStateChange();
    }

    //control camera
    double q = cameraT->q();
    static const double P = 0.00002;
    static const double D = 0.00004;
    double dq = (q - qprev) / timeStep;
    double pos = joystick->getPosition(targetMode, cameraAxis) * -1.0;
    double dqref = 0.0;
    if(fabs(pos) > 0.25) {
        double deltaq = 0.002 * pos;
        qref += deltaq;
        if(qref > 0) {
            qref = 0.0;
        } else if(qref < -M_PI) {
            qref = -M_PI;
        }
        dqref = deltaq / timeStep;
    }
    cameraT->u() = P * (qref - q) + D * (dqref - dq);
    qprev = q;

    return true;
}


CNOID_IMPLEMENT_SIMPLE_CONTROLLER_FACTORY(QuadcopterController)
