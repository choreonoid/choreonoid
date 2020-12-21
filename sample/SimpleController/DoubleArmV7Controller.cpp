#include <cnoid/SimpleController>
#include <cnoid/SharedJoystick>
#include <cnoid/EigenUtil>

using namespace std;
using namespace cnoid;

class DoubleArmV7Controller : public cnoid::SimpleController
{
public:
    Body* body;
    double dt;

    int mainActuationMode;

    enum TrackType { NO_TRACKS = 0, CONTINOUS_TRACKS, PSEUDO_TRACKS };
    int trackType;
    Link* trackL;
    Link* trackR;
    double trackgain;

    vector<int> armJointIdMap;
    vector<Link*> armJoints;
    vector<double> q_ref;
    vector<double> q_prev;
    vector<double> pgain;
    vector<double> dgain;
    double* q_tip1;
    double* q_tip2;

    SharedJoystickPtr joystick;
    int arm1Mode;
    int arm2Mode;
    int currentJoystickMode;
    const int SHIFT_BUTTON = Joystick::L_BUTTON;
    int shiftState;

    enum AxisType { STICK, BUTTON };

    struct OperationAxis {
        Link* joint;
        AxisType type;
        int id;
        double ratio;
        int shift;
        OperationAxis(Link* joint, AxisType type, int id, double ratio, int shift = 0)
            : joint(joint), type(type), id(id), ratio(ratio), shift(shift) { }
    };

    vector<vector<OperationAxis>> operationAxes;
    int operationSetIndex;

    DoubleArmV7Controller();
    virtual bool initialize(SimpleControllerIO* io) override;
    bool initContinuousTracks(SimpleControllerIO* io);
    bool initPseudoContinuousTracks(SimpleControllerIO* io);
    void initArms(SimpleControllerIO* io);
    void initPDGain();
    void initJoystickKeyBind();
    virtual bool control() override;
    void controlTracks();
    void setTargetArmPositions();
    void controlArms();
    void controlArmsWithTorque();
    void controlArmsWithVelocity();
    void controlArmsWithPosition();

    Link* link(const char* name) { return body->link(name); }
};


DoubleArmV7Controller::DoubleArmV7Controller()
{
    mainActuationMode = Link::JointEffort;
    trackType = NO_TRACKS;
}


bool DoubleArmV7Controller::initialize(SimpleControllerIO* io)
{
    body = io->body();
    dt = io->timeStep();

    io->os() << "The actuation mode of " << io->controllerName() << " is ";
    string option = io->optionString();
    if(option == "velocity"){
        mainActuationMode = Link::JointVelocity;
        io->os() << "JOINT_VELOCITY";
    } else if(option  == "position"){
        mainActuationMode = Link::JointDisplacement;
        io->os() << "JOINT_DISPLACEMENT";
    } else {
        mainActuationMode = Link::JointEffort;
        io->os() << "JOINT_EFFORT";
    }
    io->os() << "." << endl;

    initContinuousTracks(io) || initPseudoContinuousTracks(io);
    
    initArms(io);
    initPDGain();

    joystick = io->getOrCreateSharedObject<SharedJoystick>("joystick");
    arm1Mode = joystick->addMode();
    arm2Mode = joystick->addMode();
    initJoystickKeyBind();

    return true;
}


bool DoubleArmV7Controller::initContinuousTracks(SimpleControllerIO* io)
{
    trackL = link("WHEEL_L0");
    trackR = link("WHEEL_R0");

    if(!trackL || !trackR){
        return false;
    }
    
    if(mainActuationMode == Link::JointEffort){
        trackL->setActuationMode(Link::JointEffort);
        trackR->setActuationMode(Link::JointEffort);
    } else {
        trackL->setActuationMode(Link::JointVelocity);
        trackR->setActuationMode(Link::JointVelocity);
    }
    
    io->enableOutput(trackL);
    io->enableOutput(trackR);

    trackType = CONTINOUS_TRACKS;
    
    io->os() << "Continuous tracks of " << body->name() << " are detected." << endl;
    
    return true;
}


bool DoubleArmV7Controller::initPseudoContinuousTracks(SimpleControllerIO* io)
{
    trackL = link("TRACK_L");
    trackR = link("TRACK_R");

    if(!trackL || !trackR){
        return false;
    }

    if(trackL->actuationMode() == Link::JointVelocity && trackR->actuationMode() == Link::JointVelocity){
        io->enableOutput(trackL);
        io->enableOutput(trackR);
        trackType = PSEUDO_TRACKS;
        io->os() << "Pseudo continuous tracks of " << body->name() << " are detected." << endl;
    }

    return (trackType == PSEUDO_TRACKS);
}


void DoubleArmV7Controller::initArms(SimpleControllerIO* io)
{
    for(auto joint : body->joints()){
        if(joint->jointId() >= 0 && (joint->isRevoluteJoint() || joint->isPrismaticJoint())){
            joint->setActuationMode(mainActuationMode);
            io->enableIO(joint);
            armJointIdMap.push_back(armJoints.size());
            armJoints.push_back(joint);
            q_ref.push_back(joint->q());
        } else {
            armJointIdMap.push_back(-1);
        }
    }
    q_prev = q_ref;
    
    q_tip1 = &q_ref[armJointIdMap[link("TOHKU_TIP_01")->jointId()]];
    q_tip2 = &q_ref[armJointIdMap[link("TOHKU_TIP_02")->jointId()]];
}


void DoubleArmV7Controller::initPDGain()
{
    // Tracks
    if(trackType == CONTINOUS_TRACKS){
        if(mainActuationMode == Link::JointEffort){
            trackgain = 2000.0;
        } else {
            trackgain = 2.0;
        }
    } else if(trackType == PSEUDO_TRACKS){
        trackgain = 1.0;
    }

    // Arm
    if(mainActuationMode == Link::JointEffort){
        pgain = {
        /* MFRAME */ 200000, /* BLOCK */ 150000, /* BOOM */ 150000, /* ARM  */ 100000,
        /* PITCH  */  30000, /* ROLL  */  20000, /* TIP1 */    500, /* TIP2 */    500,
        /* UFRAME */ 150000, /* SWING */  50000, /* BOOM */ 100000, /* ARM  */  80000,
        /* ELBOW */   30000, /* YAW   */  20000, /* HAND */    500, /* ROD  */  50000};
        dgain = {
        /* MFRAME */ 20000, /* BLOCK */ 15000, /* BOOM */ 10000, /* ARM  */ 5000,
        /* PITCH  */   500, /* ROLL  */   500, /* TIP1 */    50, /* TIP2 */   50,
        /* UFRAME */ 15000, /* SWING */  1000, /* BOOM */  3000, /* ARM  */ 2000,
        /* ELBOW */    500, /* YAW   */   500, /* HAND */    20, /* ROD  */ 5000};

    } else if(mainActuationMode == Link::JointVelocity){
        pgain = {
        /* MFRAME */ 100, /* BLOCK */ 100, /* BOOM */ 100, /* ARM  */ 100,
        /* PITCH  */  50, /* ROLL  */  50, /* TIP1 */   5, /* TIP2 */   5,
        /* UFRAME */ 100, /* SWING */ 100, /* BOOM */ 100, /* ARM  */ 100,
        /* ELBOW */   50, /* YAW   */  20, /* HAND */  20, /* ROD  */  50};
    }
}


void DoubleArmV7Controller::initJoystickKeyBind()
{
    operationAxes = {
        {
            { link("MFRAME"),       STICK,  Joystick::L_STICK_H_AXIS, -0.6 },
            { link("BLOCK"),        STICK,  Joystick::R_STICK_H_AXIS, -0.6 },
            { link("BOOM"),         STICK,  Joystick::L_STICK_V_AXIS, -0.6 },
            { link("ARM"),          STICK,  Joystick::R_STICK_V_AXIS,  0.6 },
            { link("TOHKU_PITCH"),  BUTTON, Joystick::A_BUTTON,        0.6 },
            { link("TOHKU_PITCH"),  BUTTON, Joystick::Y_BUTTON,       -0.6 },
            { link("TOHKU_ROLL"),   BUTTON, Joystick::X_BUTTON,        1.0 },
            { link("TOHKU_ROLL"),   BUTTON, Joystick::B_BUTTON,       -1.0 },
            { link("TOHKU_TIP_01"), STICK,  Joystick::R_TRIGGER_AXIS, -0.6 },
            { link("TOHKU_TIP_02"), STICK,  Joystick::R_TRIGGER_AXIS, -0.6 },
            { link("TOHKU_TIP_01"), BUTTON, Joystick::R_BUTTON,        0.5 },
            { link("TOHKU_TIP_02"), BUTTON, Joystick::R_BUTTON,        0.5 }
        },
        {
            { link("UFRAME"),       STICK,  Joystick::L_STICK_H_AXIS, -0.6 },
            { link("MNP_SWING"),    STICK,  Joystick::R_STICK_H_AXIS, -0.6 },
            { link("MANIBOOM"),     STICK,  Joystick::L_STICK_V_AXIS, -0.6 },
            { link("MANIARM"),      STICK,  Joystick::R_STICK_V_AXIS,  0.6 },
            { link("MANIELBOW"),    BUTTON, Joystick::A_BUTTON,        0.6 },
            { link("MANIELBOW"),    BUTTON, Joystick::Y_BUTTON,       -0.6 },
            { link("YAWJOINT"),     BUTTON, Joystick::X_BUTTON,        1.0, 1 },
            { link("YAWJOINT"),     BUTTON, Joystick::B_BUTTON,       -1.0, 1 },
            { link("HANDBASE"),     BUTTON, Joystick::X_BUTTON,       -1.0, 0 },
            { link("HANDBASE"),     BUTTON, Joystick::B_BUTTON,        1.0, 0 },
            { link("PUSHROD"),      STICK,  Joystick::R_TRIGGER_AXIS, -0.04 },
            { link("PUSHROD"),      BUTTON, Joystick::R_BUTTON,        0.04 },
        }
    };

    operationSetIndex = 0;
}


bool DoubleArmV7Controller::control()
{
    joystick->updateState(arm1Mode);

    if(joystick->mode() == arm1Mode){
        currentJoystickMode = arm1Mode;
        operationSetIndex = 0;
    } else if(joystick->mode() == arm2Mode){
        currentJoystickMode = arm2Mode;
        operationSetIndex = 1;
    } else {
        currentJoystickMode = -1;
    }

    shiftState = joystick->getButtonState(SHIFT_BUTTON) ? 1 : 0;

    if(trackType){
        controlTracks();
    }
    
    controlArms();

    return true;
}


void DoubleArmV7Controller::controlTracks()
{
    trackL->u() = 0.0;
    trackL->dq() = 0.0;
    trackR->u() = 0.0;
    trackR->dq() = 0.0;
    
    const double k1 = 0.4;
    const double k2 = 0.6;

    double pos[2];
    pos[0] = k1 * joystick->getPosition(currentJoystickMode, Joystick::DIRECTIONAL_PAD_H_AXIS);
    pos[1] = k1 * joystick->getPosition(currentJoystickMode, Joystick::DIRECTIONAL_PAD_V_AXIS);
    
    if(shiftState == 1){
        pos[0] += k2 * (
            joystick->getPosition(currentJoystickMode, Joystick::L_STICK_H_AXIS) +
            joystick->getPosition(currentJoystickMode, Joystick::R_STICK_H_AXIS));
        pos[1] += k2 * (
            joystick->getPosition(currentJoystickMode, Joystick::L_STICK_V_AXIS) +
            joystick->getPosition(currentJoystickMode, Joystick::R_STICK_V_AXIS));
    }
    
    for(int i=0; i < 2; ++i){
        if(fabs(pos[i]) < 0.2){
            pos[i] = 0.0;
        }
    }

    if(trackType == CONTINOUS_TRACKS && mainActuationMode == Link::JointEffort){
        trackL->u() = trackgain * (-2.0 * pos[1] + pos[0]);
        trackR->u() = trackgain * (-2.0 * pos[1] - pos[0]);
    } else {
        trackL->dq_target() = trackgain * (-2.0 * pos[1] + pos[0]);
        trackR->dq_target() = trackgain * (-2.0 * pos[1] - pos[0]);
    }
}


void DoubleArmV7Controller::setTargetArmPositions()
{
    const vector<OperationAxis>& axes = operationAxes[operationSetIndex];

    for(auto& axis : axes){
        if(axis.shift < 0 || axis.shift == shiftState){
            auto joint = axis.joint;
            auto& q = q_ref[armJointIdMap[joint->jointId()]];
            if(axis.type == BUTTON){
                if(joystick->getButtonState(currentJoystickMode, axis.id)){
                    q += axis.ratio * dt;
                }
            } else if(axis.type == STICK){
                auto pos = joystick->getPosition(currentJoystickMode, axis.id);
                q += axis.ratio * pos * dt;
            }
        }
    }

    // Restrict each target position by taking the joint displacement range
    // and the cunnret joint displacement into accout
    double maxerror;
    if(mainActuationMode == Link::JointEffort){
        maxerror = radian(20.0);
    } else {
        maxerror = radian(5.0);
    }
    for(size_t i=0; i < armJoints.size(); ++i){
        auto joint = armJoints[i];
        auto& q = q_ref[i];
        auto q_current = joint->q();
        auto q_lower = std::max(q_current - maxerror, joint->q_lower());
        auto q_upper = std::min(q_current + maxerror, joint->q_upper());
        if(q < q_lower){
            q = q_lower;
        } else if(q > q_upper){
            q = q_upper;
        }
    }

    // Align the positions of the tip joints
    (*q_tip1) = (*q_tip2) = std::max(*q_tip1, *q_tip2);
}


void DoubleArmV7Controller::controlArms()
{
    setTargetArmPositions();

    switch(mainActuationMode){
    case Link::JointDisplacement:
        controlArmsWithPosition();
        break;
    case Link::JointVelocity:
        controlArmsWithVelocity();
        break;
    case Link::JointEffort:
        controlArmsWithTorque();
        break;
    default:
        break;
    }
}


void DoubleArmV7Controller::controlArmsWithPosition()
{
    for(size_t i=0; i < armJoints.size(); ++i){
        armJoints[i]->q_target() = q_ref[i];
    }
}


void DoubleArmV7Controller::controlArmsWithVelocity()
{
    for(size_t i=0; i < armJoints.size(); ++i){
        auto joint = armJoints[i];
        auto q_current = joint->q();
        joint->dq_target() = pgain[i] * (q_ref[i] - q_current);
    }
}


void DoubleArmV7Controller::controlArmsWithTorque()
{
    for(size_t i=0; i < armJoints.size(); ++i){
        auto joint = armJoints[i];
        auto q_current = joint->q();
        auto dq_current = (q_current - q_prev[i]) / dt;
        joint->u() = pgain[i] * (q_ref[i] - q_current) + dgain[i] * (0.0 - dq_current);
        q_prev[i] = q_current;
    }
}


CNOID_IMPLEMENT_SIMPLE_CONTROLLER_FACTORY(DoubleArmV7Controller)
