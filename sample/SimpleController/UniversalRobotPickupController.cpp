/**
   Sample picking up motion controller for UniversalRobot-2F85.
   @author Shizuko Hattori
*/

#include <cnoid/SimpleController>
#include <cnoid/JointPath>
#include <cnoid/EigenUtil>
#include "Interpolator.h"
#include <fmt/format.h>

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

const int NUM_JOINTS=6;
const int NUM_HAND_JOINTS = 2;
const int MAX_NUM_JOINTS = NUM_JOINTS;

JointSpec specs[NUM_ROBOTS][MAX_NUM_JOINTS] = {
        {       { "Shoulder",  500.0, 50.0 },  //UR3
                { "Upper_arm", 500.0, 50.0 },
                { "Forearm",  200.0, 20.0 },
                { "Wrist_1",  30.0, 3.0 },
                { "Wrist_2",  30.0, 3.0 },
                { "Wrist_3",  30.0, 3.0 }
        },
        {       { "Shoulder",  5000.0, 500.0 },  //UR5
                { "Upper_arm",  5000.0, 500.0 },
                { "Forearm",  2000.0, 200.0 },
                { "Wrist_1",  300.0, 30.0 },
                { "Wrist_2",  300.0, 30.0 },
                { "Wrist_3",  300.0, 30.0 }
        },
        {       { "Shoulder",  5000.0, 500.0 },    //UR10
                { "Upper_arm",  5000.0, 500.0 },
                { "Forearm",  2000.0, 200.0 },
                { "Wrist_1",  300.0, 30.0 },
                { "Wrist_2",  300.0, 30.0 },
                { "Wrist_3",  300.0, 30.0 }
        },
        {       { "Finger1_knuckle",        30.0, 3.0 },    //2F-85
                { "Finger2_knuckle",        30.0, 3.0 },
                { "",                       0.0, 0.0 },
                { "",                       0.0, 0.0 },
                { "",                       0.0, 0.0 },
                { "",                       0.0, 0.0 }
        },
};

Vector3 position[NUM_ROBOTS-1][6][2] = {
        {       { {0.4, 0.0, 0.35}, {-90.0, -25.0, 0.0} },
                { {0.5, 0.0, 0.35}, {-90.0, 0.0, 0.0} },
                { {0.0, 0.24, 0.6}, {-90.0, -45.0, 90.0} },
                { {0.0, 0.4, 0.4},  {-90.0, 0.0, 90.0} },
                { {0.0, 0.4, 0.45}, {-90.0, 0.0, 90.0} },
                { {0.23, 0.1, 0.6}, {-45.0, 41.0, -45.0} }
        },
        {       { {0.6, 0.0, 0.5},  {-90.0, -20.0, 0.0} },
                { {0.8, 0.0, 0.33}, {-90.0, 0.0, 0.0} },
                { {0.0, 0.5, 0.75}, {-90.0, -45.0, 90.0} },
                { {0.0, 0.5, 0.63}, {-90.0, 0.0, 90.0} },
                { {0.0, 0.5, 0.75}, {-90.0, 0.0, 90.0} },
                { {0.35, 0.1, 0.75},{-133.0, 53.0, -116.0} }

        },
        {       { {0.9, 0.0, 0.35}, {-90.0, 45.0, 0.0} },
                { {0.9, 0.0, 0.35}, {-90.0, 0.0, 0.0} },
                { {0.0, 0.5, 1.0 }, {-90.0, -45.0, 90.0} },
                { {0.0, 0.7, 0.65}, {-90.0, 0.0, 90.0} },
                { {0.05, 0.7, 0.75},{-90.0, 0.0, 90.0} },
                { {0.5, 0.16, 1.1}, {-133.0, 53.0, -116.0} }
        }
};
}

class URobotPickupController : public SimpleController
{
    Body* ioBody;
    double dt;

    Link::ActuationMode mainActuationMode;
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
    double time;
    double timeStep;
    double dq_Finger;

public:
    Vector3 toRadianVector3(Vector3& v)
    {
        return Vector3(radian(v.x()), radian(v.y()), radian(v.z()));
    }

    bool initializeJoints(SimpleControllerIO* io, JointSpec* specs)
    {
        for(int i=0; i<MAX_NUM_JOINTS; i++){
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

                    if(mainActuationMode == Link::JOINT_TORQUE){
                        info.kp = specs[i].kp_torque;
                        info.kd = specs[i].kd_torque;
                    }
                }
                jointInfos.push_back(info);
            }
        }

        return true;
    };

    virtual bool initialize(SimpleControllerIO* io) override
    {
        ioBody = io->body();
        dt = io->timeStep();

        mainActuationMode = Link::JOINT_TORQUE;

        jointInfos.clear();

        const string& BodyName = ioBody->name();
        if(BodyName.find("UR3")!=string::npos){
            initializeJoints( io, specs[UR3] );
            arm_model = UR3;
        }else if (BodyName.find("UR5")!=string::npos){
            initializeJoints( io, specs[UR5] );
            arm_model = UR5;
        }else if (BodyName.find("UR10")!=string::npos){
            initializeJoints( io, specs[UR10] );
            arm_model = UR10;
        }
        initializeJoints( io, specs[_2F_85] );


        ioFinger1 = ioBody->link("Finger1_knuckle");
        ioFinger2 = ioBody->link("Finger2_knuckle");

        ikBody = ioBody->clone();
        ikWrist = ikBody->link("Wrist_3");
        Link* base = ikBody->rootLink();
        baseToWrist = getCustomJointPath(ikBody, base, ikWrist);
        base->p().setZero();
        base->R().setIdentity();
        baseToWrist->calcForwardKinematics();

        VectorXd p0(6);
        p0.head<3>() = ikWrist->p();
        p0.tail<3>() = rpyFromRot(ikWrist->attitude());

        VectorXd p1(6);
        p1.head<3>() = position[arm_model][0][0];
        p1.tail<3>() = toRadianVector3(position[arm_model][0][1]);

        wristInterpolator.clear();
        wristInterpolator.appendSample(0.0, p0);
        wristInterpolator.appendSample(1.0, p1);
        p1.head<3>() = position[arm_model][1][0];
        p1.tail<3>() = toRadianVector3(position[arm_model][1][1]);
        wristInterpolator.appendSample(1.2, p1);
        wristInterpolator.update();

        phase = 0;
        time = 0.0;
        timeStep = io->timeStep();
        dq_Finger = 0.0;

        return true;
    }

    virtual bool control() override
    {
        bool isActive = true;

        VectorXd p(6);

        if(phase <= 4){
            p = wristInterpolator.interpolate(time);

            if(baseToWrist->calcInverseKinematics(
                   Vector3(p.head<3>()), ikWrist->calcRfromAttitude(rotFromRpy(Vector3(p.tail<3>()))))){
                for(int i=0; i < baseToWrist->numJoints(); ++i){
                    Link* joint = baseToWrist->joint(i);
                    jointInfos[i].q_ref = joint->q();
                }
            }
        }

        if(phase == 0){
            if(time > wristInterpolator.domainUpper()){
                phase = 1;
            }

        } else if(phase == 1){
            if(fabs(ioFinger1->u()) < 40.0 || fabs(ioFinger2->u()) < 40.0){
                dq_Finger = std::min(dq_Finger + 0.001, 0.1);
                jointInfos[NUM_JOINTS].q_ref += radian(dq_Finger);
                jointInfos[NUM_JOINTS+1].q_ref = jointInfos[NUM_JOINTS].q_ref;

            } else {
                VectorXd p2(6);
                p2.head<3>() = position[arm_model][2][0];
                p2.tail<3>() = toRadianVector3(position[arm_model][2][1]);

                VectorXd p3(6);
                p3.head<3>() = position[arm_model][3][0];
                p3.tail<3>() = toRadianVector3(position[arm_model][3][1]);

                wristInterpolator.clear();
                wristInterpolator.appendSample(time, p);
                wristInterpolator.appendSample(time + 1.0, p2);
                wristInterpolator.appendSample(time + 1.7, p3);
                wristInterpolator.update();
                phase = 2;
            }
        } else if(phase == 2){
            if(time > wristInterpolator.domainUpper()){
                phase = 3;
                dq_Finger = 0.0;
            }
        } else if(phase == 3){
            if(jointInfos[NUM_JOINTS].q_ref > 0.2 || jointInfos[NUM_JOINTS+1].q_ref > 0.2){
                dq_Finger = std::min(dq_Finger + 0.001, 0.03);
                jointInfos[NUM_JOINTS].q_ref -= radian(dq_Finger);
                jointInfos[NUM_JOINTS+1].q_ref = jointInfos[NUM_JOINTS].q_ref;
            } else {
                VectorXd p1(6);
                p1.head<3>() = position[arm_model][4][0];
                p1.tail<3>() = toRadianVector3(position[arm_model][4][1]);
                VectorXd p2(6);
                p2.head<3>() = position[arm_model][5][0];
                p2.tail<3>() = toRadianVector3(position[arm_model][5][1]);

                wristInterpolator.clear();
                wristInterpolator.appendSample(time, p);
                wristInterpolator.appendSample(time + 0.5, p1);
                wristInterpolator.appendSample(time + 1.5, p2);
                wristInterpolator.update();
                phase = 4;
            }
        } else if(phase == 4){
            if(time > wristInterpolator.domainUpper()){
                isActive = false;
            }
        }

        time += timeStep;

        controlJointsWithTorque();

        return isActive;
    }

    void controlJointsWithTorque()
    {
        for(auto& info : jointInfos){
            auto joint = info.joint;
            double q = joint->q();
            double dq = (q - info.q_old) / dt;
            joint->u() = info.kp * (info.q_ref - q) + info.kd * (0.0 - dq);
            info.q_old = q;
        }
    }
};

CNOID_IMPLEMENT_SIMPLE_CONTROLLER_FACTORY(URobotPickupController)
