/**
   Sample picking up motion controller for UniversalRobot-2F85.
*/

#include <cnoid/SimpleController>
#include <cnoid/JointPath>
#include <cnoid/EigenUtil>
#include "Interpolator.h"
#include <fmt/format.h>
#include <random>

using namespace std;
using namespace cnoid;

namespace {

struct JointSpec {
    string name;
    double kp_torque;
    double kd_torque;
};

enum ARM_MODEL {
    UR3,
    UR5,
    UR10,
    _2F_85,
    NUM_ROBOTS
} ;

constexpr int NUM_JOINTS = 6;
constexpr int NUM_HAND_JOINTS = 2;
constexpr int MAX_NUM_JOINTS = NUM_JOINTS;

JointSpec specs[NUM_ROBOTS][MAX_NUM_JOINTS] = {

    { // UR3
        { "SHOULDER",  500.0, 50.0 },
        { "UPPER_ARM", 500.0, 50.0 },
        { "FOREARM",   200.0, 20.0 },
        { "WRIST1",     30.0,  3.0 },
        { "WRIST2",     30.0,  3.0 },
        { "WRIST3",     30.0,  3.0 }
    },
    { //UR5
        { "SHOULDER",  5000.0, 500.0 },
        { "UPPER_ARM", 5000.0, 500.0 },
        { "FOREARM",   2000.0, 200.0 },
        { "WRIST1",     300.0,  30.0 },
        { "WRIST2",     300.0,  30.0 },
        { "WRIST3",     300.0,  30.0 }
    },
    { //UR10
        { "SHOULDER",  5000.0, 500.0 },
        { "UPPER_ARM", 5000.0, 500.0 },
        { "FOREARM",   2000.0, 200.0 },
        { "WRIST1",     300.0,  30.0 },
        { "WRIST2",     300.0,  30.0 },
        { "WRIST3",     300.0,  30.0 }
    },
    { //2F-85
        { "Finger1_knuckle", 30.0, 3.0 },
        { "Finger2_knuckle", 30.0, 3.0 },
        { "",                0.0,  0.0 },
        { "",                0.0,  0.0 },
        { "",                0.0,  0.0 },
        { "",                0.0,  0.0 }
    }
};

Vector3 positions[NUM_ROBOTS - 1][7][2] = {
    {
        { { 0.4,  0.0,  0.35 }, { -90.0, -25.0,   0.0 } },
        { { 0.5,  0.0,  0.35 }, { -90.0,   0.0,   0.0 } },
        { { 0.0,  0.24, 0.6  }, { -90.0, -45.0,  90.0 } },
        { { 0.0,  0.4,  0.4  }, { -90.0,   0.0,  90.0 } },
        { { 0.0,  0.4,  0.45 }, { -90.0,   0.0,  90.0 } },
        { { 0.23, 0.1,  0.6  }, { -45.0,  41.0, -45.0 } },
        { { 0.0,  0.24, 0.6  }, { -90.0, -45.0,  90.0 } }
    },
    {
        { {  0.6, 0.0, 0.5  }, {  -90.0, -20.0,    0.0 } },
        { {  0.8, 0.0, 0.33 }, {  -90.0,   0.0,    0.0 } },
        { {  0.0, 0.5, 0.75 }, {  -90.0, -45.0,   90.0 } },
        { {  0.0, 0.5, 0.63 }, {  -90.0,   0.0,   90.0 } },
        { {  0.0, 0.5, 0.75 }, {  -90.0,   0.0,   90.0 } },
        { { 0.35, 0.1, 0.75 }, { -133.0,  53.0, -116.0 } },
        { {  0.0, 0.5, 0.75 }, {  -90.0, -45.0,   90.0 } }
    },
    {
        { { 0.9,  0.0,  0.35 }, {  -90.0,  45.0,    0.0 } },
        { { 0.91, 0.0,  0.35 }, {  -90.0,   0.0,    0.0 } },
        { { 0.0,  0.5,  1.0  }, {  -90.0, -45.0,   90.0 } },
        { { 0.0,  0.7,  0.65 }, {  -90.0,   0.0,   90.0 } },
        { { 0.05, 0.7,  0.75 }, {  -90.0,   0.0,   90.0 } },
        { { 0.5,  0.16, 1.1  }, { -133.0,  53.0, -116.0 } },
        { { 0.0,  0.7,  0.8  }, {  -90.0,   0.0,   90.0 } },
    }
};

class URobotPickupController : public SimpleController
{
    Body* ioBody;
    double dt;

    int mainActuationMode;
    ARM_MODEL arm_model;

    struct JointInfo {
        Link* joint;
        double q_ref;
        double q_old;
        double kp;
        double kd;
    };

    vector<JointInfo> jointInfos;

    BodyPtr ikBody;
    Link* ikWrist;
    shared_ptr<JointPath> baseToWrist;

    Interpolator<VectorXd> wristInterpolator;

    Link* ioFinger1;
    Link* ioFinger2;
    int phase;
    int route;
    bool isEndless;
    double time;
    double timeStep;
    double dq_Finger;

    std::mt19937 randomNumber;
    std::uniform_real_distribution<> durationDiff;

public:
    URobotPickupController();
    Vector3 toRadianVector3(Vector3& v);
    bool initializeJoints(SimpleControllerIO* io, JointSpec* specs);
    virtual bool initialize(SimpleControllerIO* io) override;
    virtual bool control() override;
    void controlJointsWithTorque();
};

}

CNOID_IMPLEMENT_SIMPLE_CONTROLLER_FACTORY(URobotPickupController)


URobotPickupController::URobotPickupController()
  : durationDiff(-0.15, 0.15)
{

}


Vector3 URobotPickupController::toRadianVector3(Vector3& v)
{
    return Vector3(radian(v.x()), radian(v.y()), radian(v.z()));
}


bool URobotPickupController::initializeJoints(SimpleControllerIO* io, JointSpec* specs)
{
    for(int i=0; i < MAX_NUM_JOINTS; ++i){
        JointInfo info;
        if(specs[i].name!=""){
            auto joint = ioBody->link(specs[i].name);
            if(!joint){
                io->os() << fmt::format("{0} of {1} is not found", specs[i].name, ioBody->name()) << endl;
                return false;
            } else {
                joint->setActuationMode(mainActuationMode);
                io->enableIO(joint);
                
                info.joint = joint;
                info.q_ref = info.q_old = joint->q();
                
                if(mainActuationMode == Link::JointTorque){
                    info.kp = specs[i].kp_torque;
                    info.kd = specs[i].kd_torque;
                }
            }
            jointInfos.push_back(info);
        }
    }
    
    return true;
}


bool URobotPickupController::initialize(SimpleControllerIO* io)
{
    ioBody = io->body();
    dt = io->timeStep();
    
    mainActuationMode = Link::JointTorque;
    
    jointInfos.clear();

    bool initialized = false;
    const string& bodyName = ioBody->name();
    if(bodyName.find("UR3")!=string::npos){
        initialized = initializeJoints( io, specs[UR3] );
        arm_model = UR3;
    }else if (bodyName.find("UR5")!=string::npos){
        initialized = initializeJoints( io, specs[UR5] );
        arm_model = UR5;
    }else if (bodyName.find("UR10")!=string::npos){
        initialized = initializeJoints( io, specs[UR10] );
        arm_model = UR10;
    }
    if(initialized){
        initialized = initializeJoints( io, specs[_2F_85] );
    }
    if(!initialized){
        io->os() << fmt::format("{0} cannot be initialized.", bodyName) << endl;
        return false;
    }
    
    ioFinger1 = ioBody->link("Finger1_knuckle");
    ioFinger2 = ioBody->link("Finger2_knuckle");
    
    ikBody = ioBody->clone();
    ikWrist = ikBody->link("WRIST3");
    Link* base = ikBody->rootLink();
    baseToWrist = JointPath::getCustomPath(base, ikWrist);
    base->p().setZero();
    base->R().setIdentity();
    baseToWrist->calcForwardKinematics();
    
    VectorXd p0(6);
    p0.head<3>() = ikWrist->p();
    p0.tail<3>() = rpyFromRot(ikWrist->R());
    wristInterpolator.clear();
    wristInterpolator.appendSample(0.0, p0);
    wristInterpolator.update();
    
    phase = 0;
    route = 0;
    isEndless = false;
    time = 0.0;
    timeStep = io->timeStep();
    dq_Finger = 0.0;

    for(auto opt : io->options()){
        if(opt == "endless"){
            isEndless = true;
        }
    }
    
    std::random_device seedgen;
    randomNumber.seed(seedgen());

    return true;
}


bool URobotPickupController::control()
{
    bool isActive = true;

    VectorXd p(6);
    
    p = wristInterpolator.interpolate(time);
    Isometry3 T;
    T.linear() = rotFromRpy(Vector3(p.tail<3>()));
    T.translation() = p.head<3>();
    if(baseToWrist->calcInverseKinematics(T)){
        for(int i=0; i < baseToWrist->numJoints(); ++i){
            Link* joint = baseToWrist->joint(i);
            jointInfos[i].q_ref = joint->q();
        }
    }
    
    if(phase == 0){
        int index1 = (route == 0) ? 0 : 6;
        int index2 = (route == 0) ? 1 : 3;
        VectorXd p1(6);
        p1.head<3>() = positions[arm_model][index1][0];
        p1.tail<3>() = toRadianVector3(positions[arm_model][index1][1]);
        VectorXd p2(6);
        p2.head<3>() = positions[arm_model][index2][0];
        p2.tail<3>() = toRadianVector3(positions[arm_model][index2][1]);
        wristInterpolator.clear();
        wristInterpolator.appendSample(time, p);
        double time1 = time + 1.0 + durationDiff(randomNumber);
        double time2 = time1 + 0.3 + durationDiff(randomNumber);
        wristInterpolator.appendSample(time1, p1);
        wristInterpolator.appendSample(time2, p2);
        wristInterpolator.update();
        phase = 1;

    } else if(phase == 1){
        if(time > wristInterpolator.domainUpper()){
            phase = 2;
        }
        
    } else if(phase == 2){
        if(fabs(ioFinger1->u()) < 40.0 || fabs(ioFinger2->u()) < 40.0){
            dq_Finger = std::min(dq_Finger + 0.001, 0.1);
            jointInfos[NUM_JOINTS].q_ref += radian(dq_Finger);
            jointInfos[NUM_JOINTS + 1].q_ref = jointInfos[NUM_JOINTS].q_ref;
            
        } else {
            int index1 = (route == 0) ? 2 : 2;
            int index2 = (route == 0) ? 3 : 1;
            VectorXd p1(6);
            p1.head<3>() = positions[arm_model][index1][0];
            p1.tail<3>() = toRadianVector3(positions[arm_model][index1][1]);
            VectorXd p2(6);
            p2.head<3>() = positions[arm_model][index2][0];
            p2.tail<3>() = toRadianVector3(positions[arm_model][index2][1]);
            wristInterpolator.clear();
            wristInterpolator.appendSample(time, p);
            double time1 = time + 1.0 + durationDiff(randomNumber);
            double time2 = time1 + 0.7 + durationDiff(randomNumber);
            wristInterpolator.appendSample(time1, p1);
            wristInterpolator.appendSample(time2, p2);
            wristInterpolator.update();
            phase = 3;
        }
    } else if(phase == 3){
        if(time > wristInterpolator.domainUpper()){
            phase = 4;
            dq_Finger = 0.0;
        }
    } else if(phase == 4){
        if(jointInfos[NUM_JOINTS].q_ref > 0.2 || jointInfos[NUM_JOINTS + 1].q_ref > 0.2){
            dq_Finger = std::min(dq_Finger + 0.001, 0.03);
            jointInfos[NUM_JOINTS].q_ref -= radian(dq_Finger);
            jointInfos[NUM_JOINTS + 1].q_ref = jointInfos[NUM_JOINTS].q_ref;
        } else {
            int index1 = (route == 0) ? 4 : 0;
            int index2 = (route == 0) ? 5 : 5;
            int positionIndex = (route == 0) ? 4 : 0;
            VectorXd p1(6);
            p1.head<3>() = positions[arm_model][index1][0];
            p1.tail<3>() = toRadianVector3(positions[arm_model][index1][1]);
            VectorXd p2(6);
            p2.head<3>() = positions[arm_model][index2][0];
            p2.tail<3>() = toRadianVector3(positions[arm_model][index2][1]);
            wristInterpolator.clear();
            wristInterpolator.appendSample(time, p);
            double time1 = time + 0.6 + durationDiff(randomNumber);
            double time2 = time1 + 1.0 + durationDiff(randomNumber);
            wristInterpolator.appendSample(time1, p1);
            wristInterpolator.appendSample(time2, p2);
            wristInterpolator.update();
            phase = 5;
        }
    } else if(phase == 5){
        if(time > wristInterpolator.domainUpper()){
            if(isEndless){
                phase = 0;
                route = 1 - route;
            } else {
                isActive = false;
            }
        }
    }
    
    time += timeStep;
    
    controlJointsWithTorque();

    return isActive;
}


void URobotPickupController::controlJointsWithTorque()
{
    for(auto& info : jointInfos){
        auto joint = info.joint;
        double q = joint->q();
        double dq = (q - info.q_old) / dt;
        joint->u() = info.kp * (info.q_ref - q) + info.kd * (0.0 - dq);
        info.q_old = q;
    }
}
