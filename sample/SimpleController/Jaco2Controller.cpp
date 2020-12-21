#include <cnoid/SimpleController>
#include <cnoid/SharedJoystick>
#include <cnoid/EigenUtil>
#include <fmt/format.h>

using namespace std;
using namespace cnoid;

class Jaco2Controller : public SimpleController
{
    Body* body;
    double dt;

    int mainActuationMode;

    struct JointInfo {
        Link* joint;
        double q_ref;
        double q_old;
        double kp;
        double kd;
    };

    vector<JointInfo> jointInfos;

    double fingerJointTargetAngle;

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
        FINGER2,
        FINGER3,
        //FINGER1_TIP,
        //FINGER2_TIP,
        //FINGER3_TIP,
        NUM_JOINTS
    };

    SharedJoystickPtr joystick;
    int targetMode;

public:
    virtual bool initialize(SimpleControllerIO* io) override;
    bool initializeJoints(SimpleControllerIO* io, vector<JointSpec>& specs, const string& prefix);
    virtual bool control() override;
    void updateTargetJointAngles();
    void setTargetJointAngle(int jointID, Joystick::AxisID stickID, double k);
    void setFingerTargetJointAngles();
    void clampTargetJointAngle(int jointId);
    void setTargetJointAngle(int jointID, Joystick::ButtonID button1, Joystick::ButtonID button2, double k);
    
    void controlJointsWithPosition();
    void controlJointsWithVelocity();
    void controlJointsWithTorque();
};


bool Jaco2Controller::initialize(SimpleControllerIO* io)
{
    body = io->body();
    dt = io->timeStep();

    mainActuationMode = Link::JointTorque;
    string prefix;
    
    for(auto& option : io->options()){
        if(option == "velocity"){
            mainActuationMode = Link::JointVelocity;
            io->os() << "velocity mode" << endl;
        } else if(option == "position"){
            mainActuationMode = Link::JointAngle;
            io->os() << "position mode" << endl;
        } else if(option == "torque"){
            mainActuationMode = Link::JointTorque;
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
        specs[FINGER2     ] = { "FINGER2",     30,     3,  P_GAIN_VELOCITY };
        specs[FINGER3     ] = { "FINGER3",     30,     3,  P_GAIN_VELOCITY };
        //specs[FINGER1_TIP ] = { "FINGER1_TIP", 20,     2,  P_GAIN_VELOCITY };
        //specs[FINGER2_TIP ] = { "FINGER2_TIP", 20,     2,  P_GAIN_VELOCITY };
        //specs[FINGER3_TIP ] = { "FINGER3_TIP", 20,     2,  P_GAIN_VELOCITY };
    } else {
        //                                     P      D      P (vel)
        specs[SHOULDER    ] = { "SHOULDER",   400.0, 30.0,  P_GAIN_VELOCITY };
        specs[ARM         ] = { "ARM",        400.0, 30.0,  P_GAIN_VELOCITY };
        specs[FOREARM     ] = { "FOREARM",    150.0, 15.0,  P_GAIN_VELOCITY };
        specs[WRIST1      ] = { "WRIST1",      60.0,  5.0,  P_GAIN_VELOCITY };
        specs[WRIST2      ] = { "WRIST2",      60.0,  5.0,  P_GAIN_VELOCITY };
        specs[HAND        ] = { "HAND",        60.0,  5.0,  P_GAIN_VELOCITY };
        specs[FINGER1     ] = { "FINGER1",      5.0,  0.5,  P_GAIN_VELOCITY };
        specs[FINGER2     ] = { "FINGER2",      5.0,  0.5,  P_GAIN_VELOCITY };
        specs[FINGER3     ] = { "FINGER3",      5.0,  0.5,  P_GAIN_VELOCITY };
        //specs[FINGER1_TIP ] = { "FINGER1_TIP", 3.0,  0.3,  P_GAIN_VELOCITY };
        //specs[FINGER2_TIP ] = { "FINGER2_TIP", 3.0,  0.3,  P_GAIN_VELOCITY };
        //specs[FINGER3_TIP ] = { "FINGER3_TIP", 3.0,  0.3,  P_GAIN_VELOCITY };
    }
    
    if(!initializeJoints(io, specs, prefix)){
        return false;
    }

    joystick = io->getOrCreateSharedObject<SharedJoystick>("joystick");
    targetMode = joystick->addMode();

    return true;
}


bool Jaco2Controller::initializeJoints(SimpleControllerIO* io, vector<JointSpec>& specs, const string& prefix)
{
    for(auto& spec : specs){
        JointInfo info;
        string name = prefix + spec.name;
        auto joint = body->link(name);
        if(!joint){
            io->os() << fmt::format("{0} of {1} is not found", name, body->name()) << endl;
            return false;
        } else {
            joint->setActuationMode(mainActuationMode);
            io->enableIO(joint);

            info.joint = joint;
            info.q_ref = info.q_old = joint->q();

            if(mainActuationMode == Link::JointVelocity){
                info.kp = spec.kp_velocity;
            } else if(mainActuationMode == Link::JointTorque){
                info.kp = spec.kp_torque;
                info.kd = spec.kd_torque;
            }
        }
        jointInfos.push_back(info);
    }

    fingerJointTargetAngle = jointInfos[FINGER1].joint->q();

    return true;
}


bool Jaco2Controller::control()
{
    joystick->updateState(targetMode);

    updateTargetJointAngles();

    switch(mainActuationMode){
    case Link::JointTorque:
        controlJointsWithTorque();
        break;
    case Link::JointVelocity:
        controlJointsWithVelocity();
        break;
    case Link::JointAngle:
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
    setTargetJointAngle(ARM,      Joystick::L_STICK_V_AXIS, -K);
    setTargetJointAngle(FOREARM,  Joystick::R_STICK_V_AXIS,  K);
    setTargetJointAngle(WRIST1,   Joystick::R_STICK_H_AXIS, -K);
    
    setTargetJointAngle(WRIST2, Joystick::B_BUTTON, Joystick::X_BUTTON, K);
    setTargetJointAngle(HAND,   Joystick::R_BUTTON, Joystick::L_BUTTON, 1.2 * K);

    setFingerTargetJointAngles();
}


void Jaco2Controller::setTargetJointAngle(int jointID, Joystick::AxisID stickID, double k)
{
    jointInfos[jointID].q_ref += dt * k * joystick->getPosition(targetMode, stickID, 0.1);
    clampTargetJointAngle(jointID);
}


void Jaco2Controller::setTargetJointAngle(int jointID, Joystick::ButtonID button1, Joystick::ButtonID button2, double k)
{
    if(joystick->getButtonState(targetMode, button1)){
        jointInfos[jointID].q_ref -= dt * k;
    }
    if(joystick->getButtonState(targetMode, button2)){
        jointInfos[jointID].q_ref += dt * k;
    }
    clampTargetJointAngle(jointID);
}


void Jaco2Controller::setFingerTargetJointAngles()
{
    double lt = joystick->getPosition(targetMode, Joystick::L_TRIGGER_AXIS);
    if(lt > 0.1){
        fingerJointTargetAngle -= dt * 0.4 * lt;
    }
    double rt = joystick->getPosition(targetMode, Joystick::R_TRIGGER_AXIS);
    if(rt > 0.1){
        fingerJointTargetAngle += dt * 0.4 * rt;
    }
    double q_max = -100.0;
    double q_min = 100.0;

    for(auto& id : vector<int>{ FINGER1, FINGER2, FINGER3 }){
        auto& info = jointInfos[id];
        info.q_ref = fingerJointTargetAngle;
        clampTargetJointAngle(id);
        q_max = std::max(info.q_ref, q_max);
        q_min = std::min(info.q_ref, q_min);
    }
    if(fingerJointTargetAngle > q_max){
        fingerJointTargetAngle = q_max;
    } else if(fingerJointTargetAngle < q_min){
        fingerJointTargetAngle = q_min;
    }
}


void Jaco2Controller::clampTargetJointAngle(int jointID)
{
    auto& info = jointInfos[jointID];
    auto joint = info.joint;
    auto& q_ref = info.q_ref;
    static const double maxerror = radian(3.0);
    double q_current = joint->q();
    double q_lower = std::max(q_current - maxerror, joint->q_lower());
    double q_upper = std::min(q_current + maxerror, joint->q_upper());
    if(q_ref < q_lower){
        q_ref = q_lower;
    } else if(q_ref > q_upper){
        q_ref = q_upper;
    }
}
    

void Jaco2Controller::controlJointsWithPosition()
{
    for(auto& info : jointInfos){
        info.joint->q_target() = info.q_ref;
    }
}


void Jaco2Controller::controlJointsWithVelocity()
{
    for(auto& info : jointInfos){
        auto joint = info.joint;
        double q = joint->q();
        joint->dq_target() = info.kp * (info.q_ref - q) / dt;
    }
}


void Jaco2Controller::controlJointsWithTorque()
{
    for(auto& info : jointInfos){
        auto joint = info.joint;
        double q = joint->q();
        double dq = (q - info.q_old) / dt;
        joint->u() = info.kp * (info.q_ref - q) + info.kd * (0.0 - dq);
        info.q_old = q;
    }
}


CNOID_IMPLEMENT_SIMPLE_CONTROLLER_FACTORY(Jaco2Controller)
