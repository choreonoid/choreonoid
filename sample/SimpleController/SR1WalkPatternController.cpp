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
    BodyPtr body;
    MultiValueSeqPtr qseq;
    vector<double> q0;
    MultiValueSeq::Frame oldFrame;
    int currentFrame;
    double timeStep_;
    
public:

    virtual bool initialize() {

        if(!qseq){
            string filename = getNativePathString(
                boost::filesystem::path(shareDirectory())
                / "motion" / "SR1" / "SR1WalkPattern.yaml");

            BodyMotion motion;
            if(!motion.loadStandardYAMLformat(filename)){
                os() << motion.seqMessage() << endl;
                return false;
            }
            qseq = motion.jointPosSeq();
            if(qseq->numFrames() == 0){
                os() << "Empty motion data." << endl;
                return false;
            }
            q0.resize(qseq->numParts());
            timeStep_ = qseq->getTimeStep();
        }

        if(fabs(timeStep() - timeStep_) > 1.0e-6){
            os() << "Time step must be " << timeStep_ << "." << endl;;
            return false;
        }

        body = ioBody();
        
        if(body->numJoints() != qseq->numParts()){
            os() << "The number of joints must be " << qseq->numParts() << endl;
            return false;
        }
        
        for(int i=0; i < body->numJoints(); ++i){
            q0[i] = body->joint(i)->q();
        }
        oldFrame = qseq->frame(0);
        currentFrame = 0;
        
        return true;
    }

    virtual bool control() {

        bool isActive = currentFrame < qseq->numFrames();
        MultiValueSeq::Frame frame;
        if(isActive){
            frame = qseq->frame(currentFrame++);
        } else {
            frame = oldFrame;
        }

        for(int i=0; i < body->numJoints(); ++i){
            Link* joint = body->joint(i);
            double q_ref = frame[i];
            double q = joint->q();
            double dq_ref = (q_ref - oldFrame[i]) / timeStep_;
            double dq = (q - q0[i]) / timeStep_;
            joint->u() = (q_ref - q) * pgain[i] + (dq_ref - dq) * dgain[i];
            q0[i] = q;
        }
        oldFrame = frame;

        return isActive;
    }
        
};


CNOID_IMPLEMENT_SIMPLE_CONTROLLER_FACTORY(SR1WalkPatternController)
