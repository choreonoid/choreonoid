#include <cnoid/SimpleController>
#include <cnoid/Joystick>
#include <boost/format.hpp>

using namespace std;
using namespace cnoid;
using boost::format;

class AizuSpiderController : public cnoid::SimpleController
{
    Body* body;
    double dt;

    Link::ActuationMode mainActuationMode;

    vector<Link*> tracks;
    Link::ActuationMode trackActuationMode;
    double trackGain;

    struct JointInfo {
        Link* joint;
        double qref;
        double qold;
        double pgain;
        double dgain;
    };
    
    vector<JointInfo> jointInfos;

    struct JointSpec {
        string name;
        double pgain_torque;
        double dgain_torque;
        double pgain_velocity;
    };

    enum {
        FR_FLIPPER,
        FL_FLIPPER,
        BR_FLIPPER,
        BL_FLIPPER,
        NUM_FLIPPERS
    };

    Joystick joystick;

public:
    virtual bool initialize(SimpleControllerIO* io) override;
    bool initializeTracks(SimpleControllerIO* io);
    bool initializeTracks(SimpleControllerIO* io, vector<string>& names);
    bool initializeFlipperJoints(SimpleControllerIO* io);
    bool initializeJoints(SimpleControllerIO* io, vector<JointSpec>& specs);
    virtual bool control() override;
    void controlTracks();
    void updateFlipperTargetPositions();
    void controlJointsWithTorque();
    void controlJointsWithVelocity();
    void controlJointsWithPosition();
};


bool AizuSpiderController::initialize(SimpleControllerIO* io)
{
    body = io->body();
    dt = io->timeStep();

    string option = io->optionString();
    if(option == "velocity"){
        mainActuationMode = Link::ActuationMode::JOINT_VELOCITY;
    } else if(option == "position"){
        mainActuationMode = Link::ActuationMode::JOINT_ANGLE;
    } else {
        mainActuationMode = Link::ActuationMode::JOINT_TORQUE;
    }

    if(!initializeTracks(io)){
        return false;
    }

    jointInfos.clear();
    
    if(!initializeFlipperJoints(io)){
        return false;
    }
    
    return true;
}


bool AizuSpiderController::initializeTracks(SimpleControllerIO* io)
{
    tracks.clear();
    
    vector<string> trackNames = {
        "L_TRACK", "R_TRACK", "FL_SUB_TRACK", "FR_SUB_TRACK", "BL_SUB_TRACK", "BR_SUB_TRACK" };

    vector<string> wheelNames = {
        "L_TRACK_WHEEL1", "R_TRACK_WHEEL1",
        "FL_SUB_TRACK_WHEEL1", "FR_SUB_TRACK_WHEEL1", "BL_SUB_TRACK_WHEEL1", "BR_SUB_TRACK_WHEEL1" };

    bool result;
    
    if(body->link(wheelNames[0])){
        if(mainActuationMode == Link::JOINT_TORQUE){
            trackActuationMode = Link::JOINT_TORQUE;
            trackGain = 0.1;
        } else {
            trackActuationMode = Link::JOINT_VELOCITY;
            trackGain = 2.0;
        }
        result = initializeTracks(io, wheelNames);
    } else {
        trackActuationMode = Link::JOINT_SURFACE_VELOCITY;
        trackGain = 1.0;
        result = initializeTracks(io, trackNames);
    }

    return result;
}


bool AizuSpiderController::initializeTracks(SimpleControllerIO* io, vector<string>& names)
{
    for(auto& name : names){
        auto link = body->link(name);
        if(!link){
            io->os() << format("%1% of %2% is not found") % name % body->name() << endl;
            return false;
        }
        link->setActuationMode(trackActuationMode);
        io->enableOutput(link);
        tracks.push_back(link);
    }
    return true;
}


bool AizuSpiderController::initializeFlipperJoints(SimpleControllerIO* io)
{
    const double FLIPPER_P_GAIN_TORQUE = 1000.0;
    const double FLIPPER_D_GAIN_TORQUE = 10.0;
    const double FLIPPER_P_GAIN_VELOCITY = 0.5;
    
    vector<JointSpec> specs(NUM_FLIPPERS);

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
            io->os() << format("%1% of %2% is not found") % spec.name % body->name() << endl;
            return false;
        }
        joint->setActuationMode(mainActuationMode);
        io->enableIO(joint);

        JointInfo info;

        info.joint = joint;

        if(mainActuationMode == Link::JOINT_VELOCITY){
            info.pgain = spec.pgain_velocity;
        } else if(mainActuationMode == Link::JOINT_TORQUE){
            info.pgain = spec.pgain_torque;
            info.dgain = spec.dgain_torque;
        }
        
        jointInfos.push_back(info);
    }

    return true;
}


bool AizuSpiderController::control()
{
    joystick.readCurrentState();

    controlTracks();

    updateFlipperTargetPositions();

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


void AizuSpiderController::controlTracks()
{
    static const int stickAxes[] = { Joystick::L_STICK_H_AXIS, Joystick::L_STICK_V_AXIS };
    
    double pos[2];
    for(int i=0; i < 2; ++i){
        pos[i] = joystick.getPosition(stickAxes[i], 0.2);
    }
    
    double left  = trackGain * (-2.0 * pos[1] + pos[0]);
    double right = trackGain * (-2.0 * pos[1] - pos[0]);

    switch(trackActuationMode){
    case Link::JOINT_VELOCITY:
    case Link::JOINT_SURFACE_VELOCITY:
        for(int i=0; i < 3; ++i){
            tracks[i*2  ]->dq() = left;
            tracks[i*2+1]->dq() = right;
        }
        break;

    case Link::JOINT_TORQUE:
        for(int i=0; i < 3; ++i){
            tracks[i*2  ]->u() = left;
            tracks[i*2+1]->u() = right;
        }
        break;

    default:
        break;
    }
}


void AizuSpiderController::updateFlipperTargetPositions()
{
    static const double FLIPPER_GAIN = 0.4;

    double dq = dt * FLIPPER_GAIN * joystick.getPosition(Joystick::R_STICK_V_AXIS, 0.2);

    if(joystick.getPosition(Joystick::L_TRIGGER_AXIS, 0.2) > 0.0){
        // Front mode
        jointInfos[FR_FLIPPER].qref += dq;
        jointInfos[FL_FLIPPER].qref += dq;

    } else if(joystick.getButtonState(Joystick::L_BUTTON)){
        // Back mode
        jointInfos[BR_FLIPPER].qref += dq;
        jointInfos[BL_FLIPPER].qref += dq;
        
    } else {
        // Synchronize mode
        jointInfos[FR_FLIPPER].qref += dq;
        jointInfos[FL_FLIPPER].qref += dq;
        jointInfos[BR_FLIPPER].qref += dq;
        jointInfos[BL_FLIPPER].qref += dq;
    }
}


void AizuSpiderController::controlJointsWithTorque()
{
    for(auto& info : jointInfos){
        auto joint = info.joint;
        double q = joint->q();
        double dq = (q - info.qold) / dt;
        joint->u() = info.pgain * (info.qref - q) + info.dgain * (0.0 - dq);
        info.qold = q;
    }
}


void AizuSpiderController::controlJointsWithVelocity()
{
    for(auto& info : jointInfos){
        auto joint = info.joint;
        double q = joint->q();
        joint->dq() = info.pgain * (info.qref - q) / dt;
    }
}


void AizuSpiderController::controlJointsWithPosition()
{
    for(auto& info : jointInfos){
        info.joint->q() = info.qref;
    }
}


CNOID_IMPLEMENT_SIMPLE_CONTROLLER_FACTORY(AizuSpiderController)
