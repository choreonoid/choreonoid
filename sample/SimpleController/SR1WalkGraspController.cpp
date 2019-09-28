/**
   Sample walking and grasp motion controller for the SR1Hand robot model.
   This program was ported from the "SampleController" sample of OpenHRP3.
   @author Shizuko Hattori
*/
   
#include <cnoid/SimpleController>
#include <cnoid/Link>
#include <cnoid/BodyMotion>
#include <cnoid/ExecutablePath>
#include <cnoid/FileUtil>
#include <cnoid/BasicSensors>
#include "Interpolator.h"

using namespace std;
using namespace cnoid;

static double pgain[] = {
    19000.0, 19000.0, 19000.0, 19000.0, 19000.0, 19000.0,
    20000.0, 20000.0, 20000.0, 20000.0, 20000.0, 80.0, 80.0, 
    19000.0, 19000.0, 19000.0, 19000.0, 19000.0, 19000.0,
    20000.0, 20000.0, 20000.0, 20000.0, 20000.0, 20000.0, 20000.0, 
    30000.0, 19000.0, 19000.0 };

static double dgain[] = {
    100.0, 100.0, 100.0, 100.0, 100.0, 100.0,
    100.0, 100.0, 100.0, 100.0, 100.0, 1.0, 1.0,
    100.0, 100.0, 100.0, 100.0, 100.0, 100.0,
    100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0,
    100.0, 100.0, 100.0 };


class SR1WalkGraspController : public cnoid::SimpleController
{
    BodyPtr body;
    int rarm_shoulder_p;
    int rarm_shoulder_r;
    int rarm_elbow;
    int rarm_wrist_y;
    int rarm_wrist_r;
    ForceSensor* rhsensor;
    Interpolator<VectorXd> interpolator;
    std::shared_ptr<MultiValueSeq> qseq;
    VectorXd qref, qold, qref_old;
    int currentFrame;
    double timeStep;
    int numJoints;
    int phase;
    double time;
    
public:

    virtual bool initialize(SimpleControllerIO* io) override
    {
        ostream& os = io->os();

        if(!qseq){
            string filename = getNativePathString(
                cnoid::stdx::filesystem::path(shareDirectory())
                / "motion" / "SR1" / "SR1WalkPattern3.seq");

            BodyMotion motion;
            if(!motion.loadStandardYAMLformat(filename)){
                os << motion.seqMessage() << endl;
                return false;
            }
            qseq = motion.jointPosSeq();
            if(qseq->numFrames() == 0){
                os << "Empty motion data." << endl;
                return false;
            }
            timeStep = qseq->getTimeStep();
        }

        if(fabs(io->timeStep() - timeStep) > 1.0e-6){
            os << "Time step must be " << timeStep << "." << endl;;
            return false;
        }

        body = io->body();
        numJoints = body->numJoints();
        if(numJoints != qseq->numParts()){
            os << "The number of joints must be " << qseq->numParts() << endl;
            return false;
        }

        qold.resize(numJoints);
        VectorXd q0(numJoints);
        VectorXd q1(numJoints);
        MultiValueSeq::Frame frame = qseq->frame(0);
        for(int i=0; i < numJoints; ++i){
            auto joint = body->joint(i);
            qold[i] = q0[i] = joint->q();
            q1[i] = frame[i];
            joint->setActuationMode(Link::JOINT_TORQUE);
            io->enableIO(joint);
        }

        rarm_shoulder_p = body->link("RARM_SHOULDER_P")->jointId();
        rarm_shoulder_r = body->link("RARM_SHOULDER_R")->jointId();
        rarm_elbow = body->link("RARM_ELBOW")->jointId();
        rarm_wrist_y = body->link("RARM_WRIST_Y")->jointId();
        rarm_wrist_r = body->link("RARM_WRIST_R")->jointId();
        rhsensor = body->findDevice<ForceSensor>("rhsensor");

        interpolator.clear();
        interpolator.appendSample(0.0, q0);
        interpolator.appendSample(2.0, q1);
        interpolator.update();
        qref_old = interpolator.interpolate(0.0);

        currentFrame = 0;
        phase = 0;
        time = 0.0;

        return true;
    }

    virtual bool control()  override
    {
        switch(phase){
        case 0 :
            qref = interpolator.interpolate(time);
            if(time > interpolator.domainUpper()){
                phase = 1;
            }
            break;
        case 1:
            if(currentFrame < qseq->numFrames()){
                MultiValueSeq::Frame frame = qseq->frame(currentFrame++);
                for(int i=0; i < numJoints; ++i){
                    qref[i] = frame[i];
                }
            }else{
                interpolator.clear();
                interpolator.appendSample(time, qref);
                VectorXd q1(numJoints);
                q1 = qref;
                q1[rarm_shoulder_r] = -0.4;
                q1[rarm_shoulder_p] = 0.75;
                q1[rarm_elbow] = -2.0;
                interpolator.appendSample(time + 3.0, q1);
                q1[rarm_elbow] = -1.57;
                q1[rarm_shoulder_p] = -0.2;
                q1[rarm_wrist_r] = 1.5;
                interpolator.appendSample(time + 5.0, q1);
                q1[rarm_elbow] = -1.3;
                q1[rarm_wrist_y] = -0.24;
                interpolator.appendSample(time + 6.0, q1);
                interpolator.update();
                qref = interpolator.interpolate(time);
                phase = 2;
            }
            break;
        case 2 :
            qref = interpolator.interpolate(time);
            if(time > interpolator.domainUpper()){
                interpolator.clear();
                interpolator.appendSample(time, qref);
                VectorXd q1(numJoints);
                q1 = qref;
                q1[rarm_wrist_y] = 0.0;
                q1[rarm_shoulder_r] = 0.1;
                interpolator.appendSample(time + 5.0, q1);
                interpolator.update();
                qref = interpolator.interpolate(time);
                phase = 3;
            }
            break;
        case 3:
            qref = interpolator.interpolate(time);
            if( rhsensor->F()[1] < -2 ) { 
                interpolator.clear();
                interpolator.appendSample(time, qref);
                VectorXd q1(numJoints);
                q1 = qref;
                q1[rarm_wrist_r] = -0.3;;
                interpolator.appendSample(time + 2.0, q1);
                interpolator.appendSample(time + 2.5, q1);
                q1[rarm_shoulder_p] = -0.13;
                q1[rarm_elbow] = -1.8;
                interpolator.appendSample(time + 3.5, q1);
                interpolator.update();
                qref = interpolator.interpolate(time);
                phase = 4;
            }
            break;
        case 4 :
            qref = interpolator.interpolate(time);
        }

        for(int i=0; i < body->numJoints(); ++i){
            Link* joint = body->joint(i);
            double q = joint->q();
            double dq_ref = (qref[i] - qref_old[i]) / timeStep;
            double dq = (q - qold[i]) / timeStep;
            joint->u() = (qref[i] - q) * pgain[i] + (dq_ref - dq) * dgain[i];
            qold[i] = q;
        }
        qref_old = qref;

        time += timeStep;

        return true;
    }
};

CNOID_IMPLEMENT_SIMPLE_CONTROLLER_FACTORY(SR1WalkGraspController)
