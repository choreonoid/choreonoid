#include <cnoid/SimpleController>
#include <cnoid/SharedJoystick>
#include <fmt/format.h>

using namespace std;
using namespace cnoid;
using fmt::format;

namespace {

const double STICK_THRESH = 0.1;

}

class AizuSpiderController : public SimpleController
{
    Body* body;
    double dt;

    int mainActuationMode;

    vector<Link*> tracks;
    int trackActuationMode;
    double trackVelocityRatio;
    double kd_mainTrack;
    double kd_subTrack;
    vector<double> qprev_track;

    enum { L_TRACK, R_TRACK, FL_SUB_TRACK, FR_SUB_TRACK, BL_SUB_TRACK, BR_SUB_TRACK, NUM_TRACKS };

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

    enum { FR_FLIPPER, FL_FLIPPER, BR_FLIPPER, BL_FLIPPER, NUM_FLIPPERS };

    SharedJoystickPtr joystick;
    int targetMode;

public:
    virtual bool initialize(SimpleControllerIO* io) override;
    bool initializeTracks(SimpleControllerIO* io);
    bool initializeTracks(SimpleControllerIO* io, vector<string>& names);
    bool initializeFlipperJoints(SimpleControllerIO* io);
    bool initializeJoints(SimpleControllerIO* io, vector<JointSpec>& specs);
    virtual bool control() override;
    void controlTracks();
    void setTrackTorque(int id, double dq_target, double kd);
    void updateFlipperTargetPositions();
    void controlJointsWithTorque();
    void controlJointsWithVelocity();
    void controlJointsWithPosition();
};


bool AizuSpiderController::initialize(SimpleControllerIO* io)
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

    if(!initializeTracks(io)){
        return false;
    }

    if(!initializeFlipperJoints(io)){
        return false;
    }

    joystick = io->getOrCreateSharedObject<SharedJoystick>("joystick");
    targetMode = joystick->addMode();

    return true;
}


bool AizuSpiderController::initializeTracks(SimpleControllerIO* io)
{
    tracks.clear();
    qprev_track.clear();
    
    vector<string> trackNames = {
        "L_TRACK", "R_TRACK", "FL_SUB_TRACK", "FR_SUB_TRACK", "BL_SUB_TRACK", "BR_SUB_TRACK" };

    vector<string> wheelNames = {
        "L_TRACK_WHEEL1", "R_TRACK_WHEEL1",
        "FL_SUB_TRACK_WHEEL1", "FR_SUB_TRACK_WHEEL1", "BL_SUB_TRACK_WHEEL1", "BR_SUB_TRACK_WHEEL1" };
    
    bool result;
    
    if(body->link(wheelNames[0])){
        if(mainActuationMode == Link::JointTorque){
            trackActuationMode = Link::JointTorque;
            trackVelocityRatio = 0.5;
            kd_mainTrack = 8.0;
            kd_subTrack = 1.5;
        } else {
            trackActuationMode = Link::JointVelocity;
            trackVelocityRatio = 4.0;
        }
        result = initializeTracks(io, wheelNames);
    } else {
        // Pseudo continuous tracks
        trackActuationMode = Link::JointVelocity;
        trackVelocityRatio = 0.5;
        result = initializeTracks(io, trackNames);
    }

    return result;
}


bool AizuSpiderController::initializeTracks(SimpleControllerIO* io, vector<string>& names)
{
    for(auto& name : names){
        auto link = body->link(name);
        if(!link){
            io->os() << format("{0} of {1} is not found", name, body->name()) << endl;
            return false;
        }
        link->setActuationMode(trackActuationMode);
        io->enableOutput(link);
        tracks.push_back(link);
        qprev_track.push_back(link->q());
    }
    return true;
}


bool AizuSpiderController::initializeFlipperJoints(SimpleControllerIO* io)
{
    jointInfos.clear();
    
    vector<JointSpec> specs(NUM_FLIPPERS);

    const double FLIPPER_P_GAIN_TORQUE = 800.0;
    const double FLIPPER_D_GAIN_TORQUE = 20.0;
    const double FLIPPER_P_GAIN_VELOCITY = 0.5;
    
    specs[FR_FLIPPER] = { "FR_FLIPPER", FLIPPER_P_GAIN_TORQUE, FLIPPER_D_GAIN_TORQUE, FLIPPER_P_GAIN_VELOCITY };
    specs[FL_FLIPPER] = { "FL_FLIPPER", FLIPPER_P_GAIN_TORQUE, FLIPPER_D_GAIN_TORQUE, FLIPPER_P_GAIN_VELOCITY };
    specs[BR_FLIPPER] = { "BR_FLIPPER", FLIPPER_P_GAIN_TORQUE, FLIPPER_D_GAIN_TORQUE, FLIPPER_P_GAIN_VELOCITY };
    specs[BL_FLIPPER] = { "BL_FLIPPER", FLIPPER_P_GAIN_TORQUE, FLIPPER_D_GAIN_TORQUE, FLIPPER_P_GAIN_VELOCITY };

    return initializeJoints(io, specs);
}


bool AizuSpiderController::initializeJoints(SimpleControllerIO* io, vector<JointSpec>& specs)
{
    for(auto& spec : specs){
        auto joint = body->link(spec.name);
        if(!joint){
            io->os() << format("{0} of {1} is not found", spec.name, body->name()) << endl;
            return false;
        }
        joint->setActuationMode(mainActuationMode);
        io->enableIO(joint);

        JointInfo info;
        info.joint = joint;
        info.qref = info.qold = joint->q();

        if(mainActuationMode == Link::JointVelocity){
            info.kp = spec.kp_velocity;
        } else if(mainActuationMode == Link::JointTorque){
            info.kp = spec.kp_torque;
            info.kd = spec.kd_torque;
        }
        
        jointInfos.push_back(info);
    }

    return true;
}


bool AizuSpiderController::control()
{
    joystick->updateState(targetMode);

    controlTracks();

    updateFlipperTargetPositions();

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


void AizuSpiderController::controlTracks()
{
    double hpos =
        joystick->getPosition(targetMode, Joystick::L_STICK_H_AXIS, STICK_THRESH) +
        0.8 * joystick->getPosition(Joystick::DIRECTIONAL_PAD_H_AXIS);

    double vpos = -(
        joystick->getPosition(targetMode, Joystick::L_STICK_V_AXIS, STICK_THRESH) +
        0.8 * joystick->getPosition(Joystick::DIRECTIONAL_PAD_V_AXIS));
    
    double dq_L = trackVelocityRatio * (vpos + 0.4 * hpos);
    double dq_R = trackVelocityRatio * (vpos - 0.4 * hpos);

    switch(trackActuationMode){

    case Link::JointVelocity:
        for(int i=0; i < 3; ++i){
            tracks[i*2  ]->dq_target() = dq_L;
            tracks[i*2+1]->dq_target() = dq_R;
        }
        break;

    case Link::JointTorque:
        setTrackTorque(L_TRACK, dq_L, kd_mainTrack);
        setTrackTorque(R_TRACK, dq_R, kd_mainTrack);
        setTrackTorque(FL_SUB_TRACK, dq_L, kd_subTrack);
        setTrackTorque(FR_SUB_TRACK, dq_R, kd_subTrack);
        setTrackTorque(BL_SUB_TRACK, dq_L, kd_subTrack);
        setTrackTorque(BR_SUB_TRACK, dq_R, kd_subTrack);
        break;

    default:
        break;
    }
}


void AizuSpiderController::setTrackTorque(int id, double dq_target, double kd)
{
    Link* axis = tracks[id];
    double dq_current = (axis->q() - qprev_track[id]) / dt;
    axis->u() = kd * (dq_target - dq_current);
    qprev_track[id] = axis->q();
}


void AizuSpiderController::updateFlipperTargetPositions()
{
    static const double FLIPPER_GAIN = 0.5;

    // Arrange all the flippers to the same position
    if(joystick->getButtonState(targetMode, Joystick::R_STICK_BUTTON)){
        double qa = 0.0;
        for(int i=0; i < NUM_FLIPPERS; ++i){
            qa += jointInfos[i].qref;
        }
        qa /= NUM_FLIPPERS;
        double dqmax = dt * 0.5;
        for(int i=0; i < NUM_FLIPPERS; ++i){
            double dq = qa - jointInfos[i].qref;
            if(dq > dqmax){
                dq = dqmax;
            } else if(dq < -dqmax){
                dq = -dqmax;
            }
            jointInfos[i].qref += dq;
        }
    } else {
        double pos = joystick->getPosition(targetMode, Joystick::R_STICK_V_AXIS, STICK_THRESH);
        double dq = dt * FLIPPER_GAIN * pos;
        bool FL = joystick->getPosition(targetMode, Joystick::L_TRIGGER_AXIS, STICK_THRESH) > 0.0;
        bool FR = joystick->getPosition(targetMode, Joystick::R_TRIGGER_AXIS, STICK_THRESH) > 0.0;
        bool BL = joystick->getButtonState(targetMode, Joystick::L_BUTTON);
        bool BR = joystick->getButtonState(targetMode, Joystick::R_BUTTON);
        
        if(!FL && !FR && !BL && !BR){
            // Synchronize mode
            jointInfos[FR_FLIPPER].qref += dq;
            jointInfos[FL_FLIPPER].qref += dq;
            jointInfos[BR_FLIPPER].qref += dq;
            jointInfos[BL_FLIPPER].qref += dq;
        } else {
            if(FL){
                jointInfos[FL_FLIPPER].qref += dq;
            }
            if(FR){
                jointInfos[FR_FLIPPER].qref += dq;
            }
            if(BL){
                jointInfos[BL_FLIPPER].qref += dq;
            }
            if(BR){
                jointInfos[BR_FLIPPER].qref += dq;
            }
        }
    }
}


void AizuSpiderController::controlJointsWithTorque()
{
    for(auto& info : jointInfos){
        auto joint = info.joint;
        double q = joint->q();
        double dq = (q - info.qold) / dt;
        joint->u() = info.kp * (info.qref - q) + info.kd * (0.0 - dq);
        info.qold = q;
    }
}


void AizuSpiderController::controlJointsWithVelocity()
{
    for(auto& info : jointInfos){
        auto joint = info.joint;
        joint->dq_target() = info.kp * (info.qref - joint->q()) / dt;
    }
}


void AizuSpiderController::controlJointsWithPosition()
{
    for(auto& info : jointInfos){
        info.joint->q_target() = info.qref;
    }
}


CNOID_IMPLEMENT_SIMPLE_CONTROLLER_FACTORY(AizuSpiderController)
