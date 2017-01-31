/**
   Sample walking motion controller for the SR1 robot model.
   This program was ported from the "SamplePD" sample of OpenHRP3.
   @author Shin'ichiro Nakaoka
*/

#include <cnoid/SimpleController>
#include <cnoid/Link>
#include <cnoid/BodyMotion>
#include <cnoid/ExecutablePath>
#include <cnoid/FileUtil>

using namespace std;
using namespace cnoid;

static double pgain[] = {
    8000.0, 8000.0, 8000.0, 8000.0, 8000.0, 8000.0,
    3000.0, 3000.0, 3000.0, 3000.0, 3000.0, 3000.0, 3000.0, 
    8000.0, 8000.0, 8000.0, 8000.0, 8000.0, 8000.0,
    3000.0, 3000.0, 3000.0, 3000.0, 3000.0, 3000.0, 3000.0, 
    8000.0, 8000.0, 8000.0 };

static double dgain[] = {
    100.0, 100.0, 100.0, 100.0, 100.0, 100.0,
    100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0,
    100.0, 100.0, 100.0, 100.0, 100.0, 100.0,
    100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0,
    100.0, 100.0, 100.0 };


class SR1WalkPatternController : public cnoid::SimpleController
{
    Body* ioBody;
    int currentFrameIndex;
    int lastFrameIndex;
    MultiValueSeqPtr qseq;
    MultiValueSeq::Frame qref0;
    MultiValueSeq::Frame qref1;
    MultiValueSeq::Frame qref2;
    vector<double> q0;
    double dt;
    double dt2;
    enum { TORQUE_MODE, HIGHGAIN_MODE } mode;
    
public:

    virtual bool initialize(SimpleControllerIO* io) {

        string patternFile;

        string opt = io->optionString();
        if(opt == "highgain"){
            mode = HIGHGAIN_MODE;
            patternFile = "SR1WalkPattern2.yaml";
            io->setJointOutput(JOINT_ANGLE | JOINT_VELOCITY | JOINT_ACCELERATION);
            io->os() << "SR1WalkPatternController: high gain mode." << endl;
        } else if(opt == "velocity"){
            mode = HIGHGAIN_MODE;
            patternFile = "SR1WalkPattern2.yaml";
            io->setJointOutput(JOINT_VELOCITY);
            io->os() << "SR1WalkPatternController: velocity mode." << endl;
        } else {
            mode = TORQUE_MODE;
            patternFile = "SR1WalkPattern.yaml";
            io->setJointOutput(JOINT_TORQUE);
            io->setJointInput(JOINT_ANGLE);
            io->os() << "SR1WalkPatternController: torque mode." << endl;
        }

        string filename = getNativePathString(
            boost::filesystem::path(shareDirectory()) / "motion" / "SR1" / patternFile);
        
        BodyMotion motion;
        if(!motion.loadStandardYAMLformat(filename)){
            io->os() << motion.seqMessage() << endl;
            return false;
        }
        qseq = motion.jointPosSeq();
        if(qseq->numFrames() == 0){
            io->os() << "Empty motion data." << endl;
            return false;
        }

        ioBody = io->body();
        if(ioBody->numJoints() != qseq->numParts()){
            io->os() << "The number of joints must be " << qseq->numParts() << endl;
            return false;
        }

        dt = io->timeStep();
        dt2 = dt * dt;
        if(fabs(dt - qseq->timeStep()) > 1.0e-6){
            io->os() << "Warning: the simulation time step is different from that of the motion data " << endl;
        }
        
        currentFrameIndex = 0;
        lastFrameIndex = std::max(0, qseq->numFrames() - 1);
        qref1 = qseq->frame(0);
        qref2 = qseq->frame(std::min(1, lastFrameIndex));

        q0.resize(qseq->numParts());
        for(int i=0; i < ioBody->numJoints(); ++i){
            q0[i] = ioBody->joint(i)->q();
        }

        return true;
    }

    virtual bool control() {

        switch(mode){

        case TORQUE_MODE:
            qref0 = qref1;
            qref1 = qseq->frame(currentFrameIndex);
            for(int i=0; i < ioBody->numJoints(); ++i){
                Link* joint = ioBody->joint(i);
                double q_ref = qref1[i];
                double q = joint->q();
                double dq_ref = (q_ref - qref0[i]) / dt;
                double dq = (q - q0[i]) / dt;
                joint->u() = (q_ref - q) * pgain[i] + (dq_ref - dq) * dgain[i];
				//joint->u() = 0.0;
                q0[i] = q;
            }
            break;

        case HIGHGAIN_MODE:
            qref0 = qref1;
            qref1 = qref2;
            qref2 = qseq->frame(std::min(currentFrameIndex + 1, lastFrameIndex));
            for(int i=0; i < ioBody->numJoints(); ++i){
                Link* joint = ioBody->joint(i);
                joint->q() = qref1[i];
                joint->dq() = (qref2[i] - qref1[i]) / dt;
                joint->ddq() = (qref2[i] - 2.0 * qref1[i] + qref0[i]) / dt2;
            }
            break;

        default:
            break;
        }

        if(currentFrameIndex < qseq->numFrames()){
            ++currentFrameIndex;
            return true;
        } else {
            return false;
        }
    }
        
};

CNOID_IMPLEMENT_SIMPLE_CONTROLLER_FACTORY(SR1WalkPatternController)
