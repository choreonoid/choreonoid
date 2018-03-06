#include "ModeJoystick.h"
#include <cnoid/SimpleController>
#include <boost/format.hpp>

using namespace std;
using namespace cnoid;
using boost::format;

class Jaco2Controller : public SimpleController
{
    Body* body;
    double dt;

    Link::ActuationMode mainActuationMode;

    struct JointInfo {
        Link* joint;
        double qref;
        double qold;
        double kp;
        double kd;
    };

    vector<JointInfo> jointInfos;

    struct JointSpec {
        string name;
        double kp_torque;
        double kd_torque;
        double kp_velocity;
    };

    enum {
        SHOULDER,
        ARM,
        FOREARM,
        WRIST1,
        WRIST2,
        HAND,
        FINGER1,
        FINGER1_TIP,
        FINGER2,
        FINGER2_TIP,
        FINGER3,
        FINGER3_TIP,
        NUM_JOINTS
    };

    ModeJoystickPtr joystick;
    int targetMode;

public:
    virtual bool initialize(SimpleControllerIO* io) override;
    bool initializeJoints(SimpleControllerIO* io, vector<JointSpec>& specs, const string& prefix);
    virtual bool control() override;
    void updateTargetJointAngles();
    void setTargetJointAngle(int jointID, Joystick::AxisID stickID, double k);
    void setTargetJointAngle(int jointID, Joystick::ButtonID button1, Joystick::ButtonID button2, double k);
    void controlJointsWithTorque();
    void controlJointsWithVelocity();
    void controlJointsWithPosition();
};


bool Jaco2Controller::initialize(SimpleControllerIO* io)
{
    body = io->body();
    dt = io->timeStep();

    mainActuationMode = Link::JOINT_TORQUE;
    string prefix;
    
    for(auto& option : io->options()){
        if(option == "velocity"){
            mainActuationMode = Link::JOINT_VELOCITY;
            io->os() << "velocity mode" << endl;
        } else if(option == "position"){
            mainActuationMode = Link::JOINT_ANGLE;
            io->os() << "position mode" << endl;
        } else if(option == "torque"){
            mainActuationMode = Link::JOINT_TORQUE;
            io->os() << "torque mode" << endl;
        } else {
            prefix = option;
            io->os() << "prefix: " << prefix << endl;
        }
    }

    jointInfos.clear();

    const double P_GAIN_VELOCITY = 0.5;
    
    vector<JointSpec> specs(NUM_JOINTS);

    if(io->timeStep() < 0.002){
        //                                     P      D      P (vel)
        specs[SHOULDER    ] = { "SHOULDER",  1000.0, 100,  P_GAIN_VELOCITY };
        specs[ARM         ] = { "ARM",       1000.0, 100,  P_GAIN_VELOCITY };
        specs[FOREARM     ] = { "FOREARM",   600.0,   60,  P_GAIN_VELOCITY };
        specs[WRIST1      ] = { "WRIST1",    300.0,   30,  P_GAIN_VELOCITY };
        specs[WRIST2      ] = { "WRIST2",    300.0,   30,  P_GAIN_VELOCITY };
        specs[HAND        ] = { "HAND",      250.0,   25,  P_GAIN_VELOCITY };
        specs[FINGER1     ] = { "FINGER1",     30,     3,  P_GAIN_VELOCITY };
        specs[FINGER1_TIP ] = { "FINGER1_TIP", 20,     2,  P_GAIN_VELOCITY };
        specs[FINGER2     ] = { "FINGER2",     30,     3,  P_GAIN_VELOCITY };
        specs[FINGER2_TIP ] = { "FINGER2_TIP", 20,     2,  P_GAIN_VELOCITY };
        specs[FINGER3     ] = { "FINGER3",     30,     3,  P_GAIN_VELOCITY };
        specs[FINGER3_TIP ] = { "FINGER3_TIP", 20,     2,  P_GAIN_VELOCITY };
    } else {
        //                                     P      D      P (vel)
        specs[SHOULDER    ] = { "SHOULDER",   400.0, 30.0,  P_GAIN_VELOCITY };
        specs[ARM         ] = { "ARM",        400.0, 30.0,  P_GAIN_VELOCITY };
        specs[FOREARM     ] = { "FOREARM",    150.0, 15.0,  P_GAIN_VELOCITY };
        specs[WRIST1      ] = { "WRIST1",      60.0,  5.0,  P_GAIN_VELOCITY };
        specs[WRIST2      ] = { "WRIST2",      60.0,  5.0,  P_GAIN_VELOCITY };
        specs[HAND        ] = { "HAND",        60.0,  5.0,  P_GAIN_VELOCITY };
        specs[FINGER1     ] = { "FINGER1",     5.0,  0.5,  P_GAIN_VELOCITY };
        specs[FINGER1_TIP ] = { "FINGER1_TIP", 3.0,  0.3,  P_GAIN_VELOCITY };
        specs[FINGER2     ] = { "FINGER2",     5.0,  0.5,  P_GAIN_VELOCITY };
        specs[FINGER2_TIP ] = { "FINGER2_TIP", 3.0,  0.3,  P_GAIN_VELOCITY };
        specs[FINGER3     ] = { "FINGER3",     5.0,  0.5,  P_GAIN_VELOCITY };
        specs[FINGER3_TIP ] = { "FINGER3_TIP", 3.0,  0.3,  P_GAIN_VELOCITY };
    }
    
    if(!initializeJoints(io, specs, prefix)){
        return false;
    }

    joystick = io->getOrCreateSharedObject<ModeJoystick>("joystick");
    targetMode = joystick->addMode();

    return true;
}


bool Jaco2Controller::initializeJoints(SimpleControllerIO* io, vector<JointSpec>& specs, const string& prefix)
{
    for(auto& spec : specs){
        string name = prefix + spec.name;
        auto joint = body->link(name);
        if(!joint){
            io->os() << format("%1% of %2% is not found") % name % body->name() << endl;
            return false;
        }
        joint->setActuationMode(mainActuationMode);
        io->enableIO(joint);

        JointInfo info;
        info.joint = joint;
        info.qref = info.qold = joint->q();

        if(mainActuationMode == Link::JOINT_VELOCITY){
            info.kp = spec.kp_velocity;
        } else if(mainActuationMode == Link::JOINT_TORQUE){
            info.kp = spec.kp_torque;
            info.kd = spec.kd_torque;
        }
        
        jointInfos.push_back(info);
    }

    return true;
}


bool Jaco2Controller::control()
{
    joystick->updateState(targetMode);

    updateTargetJointAngles();

    switch(mainActuationMode){
    case Link::JOINT_TORQUE:
        controlJointsWithTorque();
        break;
    case Link::JOINT_VELOCITY:
        controlJointsWithVelocity();
        break;
    case Link::JOINT_ANGLE:
        controlJointsWithPosition();
        break;
    default:
        break;
    }

    return true;
}


void Jaco2Controller::updateTargetJointAngles()
{
    static const double K = 0.8;

    setTargetJointAngle(SHOULDER, Joystick::L_STICK_H_AXIS, -K);
    setTargetJointAngle(ARM,      Joystick::L_STICK_V_AXIS,  K);
    setTargetJointAngle(FOREARM,  Joystick::R_STICK_V_AXIS, -K);
    setTargetJointAngle(WRIST1,   Joystick::R_STICK_H_AXIS, -K);
    
    setTargetJointAngle(WRIST2, Joystick::B_BUTTON, Joystick::A_BUTTON, K);
    setTargetJointAngle(HAND,   Joystick::R_BUTTON, Joystick::L_BUTTON, 1.2 * K);

    double dq_finger = 0.0;
    double lt = joystick->getPosition(targetMode, Joystick::L_TRIGGER_AXIS);
    if(lt > -0.9){
        dq_finger += dt * 0.2 * (lt + 1.0);
    }
    double rt = joystick->getPosition(targetMode, Joystick::R_TRIGGER_AXIS);
    if(rt > -0.9){
        dq_finger -= dt * 0.2 * (rt + 1.0);
    }
    for(int i = FINGER1; i <= FINGER3_TIP; ++i){
        jointInfos[i].qref += dq_finger;
    }
}


void Jaco2Controller::setTargetJointAngle(int jointID, Joystick::AxisID stickID, double k)
{
    jointInfos[jointID].qref += dt * k * joystick->getPosition(targetMode, stickID, 0.1);
}


void Jaco2Controller::setTargetJointAngle(int jointID, Joystick::ButtonID button1, Joystick::ButtonID button2, double k)
{
    double dq = 0.0;
    if(joystick->getButtonState(targetMode, button1)){
        dq -= dt * k;
    }
    if(joystick->getButtonState(targetMode, button2)){
        dq += dt * k;
    }
    jointInfos[jointID].qref += dq;
}


void Jaco2Controller::controlJointsWithTorque()
{
    for(auto& info : jointInfos){
        auto joint = info.joint;
        double q = joint->q();
        double dq = (q - info.qold) / dt;
        joint->u() = info.kp * (info.qref - q) + info.kd * (0.0 - dq);
        info.qold = q;
    }
}


void Jaco2Controller::controlJointsWithVelocity()
{
    for(auto& info : jointInfos){
        auto joint = info.joint;
        double q = joint->q();
        joint->dq() = info.kp * (info.qref - q) / dt;
    }
}


void Jaco2Controller::controlJointsWithPosition()
{
    for(auto& info : jointInfos){
        info.joint->q() = info.qref;
    }
}


CNOID_IMPLEMENT_SIMPLE_CONTROLLER_FACTORY(Jaco2Controller)
