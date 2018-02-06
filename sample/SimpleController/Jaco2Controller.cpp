#include <cnoid/SimpleController>
#include <cnoid/Joystick>
#include <boost/format.hpp>

using namespace std;
using namespace cnoid;
using boost::format;

class Jaco2Controller : public cnoid::SimpleController
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

    Joystick joystick;

public:
    virtual bool initialize(SimpleControllerIO* io) override;
    bool initializeJoints(SimpleControllerIO* io, vector<JointSpec>& specs, const string& prefix);
    virtual bool control() override;
    void updateTargetJointAngles();
    void setTargetJointAngle(int jointID, Joystick::AxisID stickID, double k);
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

    //                                    P      D      P (vel)
    specs[SHOULDER    ] = { "SHOULDER",   100.0, 10.0,  P_GAIN_VELOCITY };
    specs[ARM         ] = { "ARM",        100.0, 10.0,  P_GAIN_VELOCITY };
    specs[FOREARM     ] = { "FOREARM",     50.0,  5.0,  P_GAIN_VELOCITY };
    specs[WRIST1      ] = { "WRIST1",      20.0,  2.0,  P_GAIN_VELOCITY };
    specs[WRIST2      ] = { "WRIST2",      20.0,  2.0,  P_GAIN_VELOCITY };
    specs[HAND        ] = { "HAND",        10.0,  1.0,  P_GAIN_VELOCITY };
    specs[FINGER1     ] = { "FINGER1",     10.0,  1.0,  P_GAIN_VELOCITY };
    specs[FINGER1_TIP ] = { "FINGER1_TIP", 10.0,  1.0,  P_GAIN_VELOCITY };
    specs[FINGER2     ] = { "FINGER2",     10.0,  1.0,  P_GAIN_VELOCITY };
    specs[FINGER2_TIP ] = { "FINGER2_TIP", 10.0,  1.0,  P_GAIN_VELOCITY };
    specs[FINGER3     ] = { "FINGER3",     10.0,  1.0,  P_GAIN_VELOCITY };
    specs[FINGER3_TIP ] = { "FINGER3_TIP", 10.0,  1.0,  P_GAIN_VELOCITY };
    
    return initializeJoints(io, specs, prefix);
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
    joystick.readCurrentState();

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
    if(!joystick.getButtonState(Joystick::R_BUTTON)){
        return;
    }
    
    static const double K = 0.6;

    setTargetJointAngle(SHOULDER, Joystick::DIRECTIONAL_PAD_H_AXIS, K * 0.8);
    setTargetJointAngle(ARM,      Joystick::DIRECTIONAL_PAD_V_AXIS, K * 0.8);
    setTargetJointAngle(FOREARM,  Joystick::L_STICK_H_AXIS,         K);
    setTargetJointAngle(WRIST1,   Joystick::L_STICK_V_AXIS,         K);
    setTargetJointAngle(WRIST2,   Joystick::R_STICK_H_AXIS,         K);
    setTargetJointAngle(HAND,     Joystick::R_STICK_V_AXIS,         K);

    double dq_finger = 0.0;
    double lt = joystick.getPosition(Joystick::L_TRIGGER_AXIS);
    if(lt > -0.9){
        dq_finger += dt * 0.2 * (lt + 1.0);
    }
    double rt = joystick.getPosition(Joystick::R_TRIGGER_AXIS);
    if(rt > -0.9){
        dq_finger -= dt * 0.2 * (rt + 1.0);
    }
    for(int i = FINGER1; i <= FINGER3_TIP; ++i){
        jointInfos[i].qref += dq_finger;
    }
}


void Jaco2Controller::setTargetJointAngle(int jointID, Joystick::AxisID stickID, double k)
{
    jointInfos[jointID].qref += dt * k * joystick.getPosition(stickID, 0.1);
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
