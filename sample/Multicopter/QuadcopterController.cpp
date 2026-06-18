/**
   Copyright (c) 2018 Japan Atomic Energy Agency (JAEA).
   The original version is implemented as an RT-component.
   This is a simple controller version modified by AIST.
*/

#include <cnoid/SimpleController>
#include <cnoid/RotorDevice>
#include <cnoid/SharedJoystick>
#include <cnoid/EigenUtil>
#include <cmath>

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
const double maxRotorForce = 20.0;
const double yawTorqueCoef = 0.02;
const double maxPropSpeed = 80.0;
const double idlePropSpeed = 20.0;
const double propSpeedGain = 1.0e-4;
const double stickDeadband = 0.25;

class QuadcopterController : public SimpleController
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

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
    bool isTakeoffStarted;

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
    cameraT->setActuationMode(Link::JointTorque);
    io->enableIO(cameraT);
    qref = qprev = cameraT->q();

    for(int i = 0; i < 4; i++) {
        prop[i] = ioBody->link(propname[i]);
        prop[i]->setActuationMode(Link::JointTorque);
        io->enableInput(prop[i], JOINT_VELOCITY);
        io->enableOutput(prop[i]);

        rotor[i] = ioBody->findDevice<Multicopter::RotorDevice>(rotorname[i]);
        rotor[i]->setValue(0.0);
        rotor[i]->setTorque(0.0);
        rotor[i]->notifyStateChange();
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
    isTakeoffStarted = false;

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
    auto p = ioBody->rootLink()->translation();
    return Vector2(p.x(), p.y());
}


bool QuadcopterController::control()
{
    joystick->updateState(targetMode);

    //control rotors
    Vector4 zrpy = getZRPY();
    double cc = cos(zrpy[1]) * cos(zrpy[2]);
    if(cc < 0.2){
        cc = 0.2;
    }
    double gfcoef = 1.0 * 9.80665 / 4 / cc ;
    Vector4 force = Vector4::Zero();
    Vector4 torque = Vector4::Zero();
    
    power = joystick->getButtonState(targetMode, powerButton);
    if(power == false) {
        powerprev = false;
    } else if((power == true) && (powerprev != true)) {
        rotorswitch = !rotorswitch;
        isTakeoffStarted = false;
        powerprev = true;
    }

    bool modeButtonState = joystick->getButtonState(Joystick::R_STICK_BUTTON);
    if(modeButtonState){
        if(!prevModeButtonState){
            isStableMode = !isStableMode;
        }
    }
    prevModeButtonState = modeButtonState;

    double zInput = joystick->getPosition(targetMode, rotorAxis[0]);
    double dzCommand = 0.0;
    if(fabs(zInput) > stickDeadband){
        dzCommand = RATE[0] * zInput;
    }
    if(rotorswitch && dzCommand > 0.0){
        isTakeoffStarted = true;
    } else if(!rotorswitch){
        isTakeoffStarted = false;
    }

    if(rotorswitch && isTakeoffStarted) {
        Vector4 f;

        Vector4 dzrpy = (zrpy - zrpyprev) / timeStep;
        Vector4 ddzrpy = (dzrpy - dzrpyprev) / timeStep;

        // For the stable mode
        Vector2 xy = getXY();
        Vector2 dxy = (xy - xyprev) / timeStep;
        Vector2 ddxy = (dxy - dxyprev) / timeStep;
        Vector2 dxy_local = Eigen::Rotation2Dd(-zrpy[3]) * dxy;
        Vector2 ddxy_local = Eigen::Rotation2Dd(-zrpy[3]) * ddxy;

        for(int axis = 0; axis < 4; ++axis) {
            double pos = joystick->getPosition(targetMode, rotorAxis[axis]);

            if((axis == 0) || (axis == 3)) {
                if(fabs(pos) > stickDeadband) {
                    dzrpyref[axis] = RATE[axis] * pos;
                } else {
                    dzrpyref[axis] = 0.0;
                }
                f[axis] = KP[axis] * (dzrpyref[axis] - dzrpy[axis]) + KD[axis] * (0.0 - ddzrpy[axis]);
            } else {
                if(!isStableMode){
                    if(fabs(pos) > stickDeadband) {
                        zrpyref[axis] = RATE[axis] * pos;
                    } else {
                        zrpyref[axis] = 0.0;
                    }
                } else {
                    int axis_xy = axis - 1;
                    if(fabs(pos) > stickDeadband) {
                        dxyref[axis_xy] = RATEX[axis_xy] * pos;
                    } else {
                        dxyref[axis_xy] = 0.0;
                    }
                    zrpyref[axis] =
                        KPX[axis_xy] * (dxyref[axis_xy] - dxy_local[1 - axis_xy]) +
                        KDX[axis_xy] * (0.0 - ddxy_local[1 - axis_xy]);

                    if(axis == 1) {
                        zrpyref[axis] *= -1.0;
                    }
                }
                f[axis] = KP[axis] * (zrpyref[axis] - zrpy[axis]) + KD[axis] * (0.0 - dzrpy[axis]);
            }
        }
        zrpyprev = zrpy;
        dzrpyprev = dzrpy;

        // For the stable mode
        xyprev = xy;
        dxyprev = dxy;
        
        Vector4 force0;
        double targetTotal = 0.0;
        for(int i = 0; i < 4; ++i) {
            double fi = gfcoef;
            fi += sign[i][0] * f[0];
            fi += sign[i][1] * f[1];
            fi += sign[i][2] * f[2];
            fi += sign[i][3] * f[3];
            force0[i] = fi;
            targetTotal += fi;
        }
        if(targetTotal < 0.0){
            targetTotal = 0.0;
        }

        double clampedTotal = 0.0;
        for(int i = 0; i < 4; ++i) {
            double fi = force0[i];
            if(fi < 0.0){
                fi = 0.0;
            } else if(fi > maxRotorForce){
                fi = maxRotorForce;
            }
            force[i] = fi;
            clampedTotal += fi;
        }

        if(clampedTotal > targetTotal && clampedTotal > 0.0){
            force *= targetTotal / clampedTotal;
        }

        for(int i = 0; i < 4; ++i) {
            torque[i] = yawTorqueCoef * dir[i] * force[i];
        }
    } else {
        zrpyprev = zrpy;
        dzrpyprev = Vector4::Zero();
        Vector2 xy = getXY();
        xyprev = xy;
        dxyprev = Vector2::Zero();
    }

    for(int i = 0; i < 4; ++i) {
        double tau = torque[i];
        rotor[i]->setTorque(tau);
        rotor[i]->setValue(force[i]);
        rotor[i]->notifyStateChange();

        double dqTarget = 0.0;
        if(rotorswitch){
            if(isTakeoffStarted){
                dqTarget = dir[i] * maxPropSpeed * std::sqrt(force[i] / maxRotorForce);
            } else {
                dqTarget = dir[i] * idlePropSpeed;
            }
        }
        prop[i]->u() = propSpeedGain * (dqTarget - prop[i]->dq());
    }

    //control camera
    double q = cameraT->q();
    static const double P = 0.00002;
    static const double D = 0.00004;
    double dq = (q - qprev) / timeStep;
    double pos = joystick->getPosition(targetMode, cameraAxis) * -1.0;
    double dqref = 0.0;
    if(fabs(pos) > stickDeadband) {
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
