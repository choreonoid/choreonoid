/**
   Sample lifting motion controller for the SR1 robot model.
   The original sample which this program is based on is "SampleLF" of OpenHRP3.
   @author Shin'ichiro Nakaoka
*/

#include <cnoid/SimpleController>
#include <cnoid/EigenUtil>
#include "Interpolator.h"

using namespace std;
using namespace cnoid;
using namespace boost;

namespace {

const double pgain[] = {
    8000.0, 8000.0, 8000.0, 8000.0, 8000.0, 8000.0,
    3000.0, 3000.0, 3000.0, 3000.0, 3000.0, 3000.0, 3000.0, 
    8000.0, 8000.0, 8000.0, 8000.0, 8000.0, 8000.0,
    3000.0, 3000.0, 3000.0, 3000.0, 3000.0, 3000.0, 3000.0, 
    8000.0, 8000.0, 8000.0 };
    
const double dgain[] = {
    100.0, 100.0, 100.0, 100.0, 100.0, 100.0,
    100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0,
    100.0, 100.0, 100.0, 100.0, 100.0, 100.0,
    100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0,
    100.0, 100.0, 100.0 };
}

class SR1LiftupController : public cnoid::SimpleController
{
    Interpolator<VectorXd> interpolator;
    VectorXd qref, qold, qref_old;
    double time;
    int phase;
    double dq_wrist;
    Link* rightWrist;
    Link* leftWrist;
    double throwTime;
    bool isVelocityControlMode;

public:

    VectorXd& convertToRadian(VectorXd& q){
        for(size_t i=0; i < q.size(); ++i){
            q[i] = radian(q[i]);
        }
        return q;
    }
    
    virtual bool initialize(SimpleControllerIO* io) {

        if(io->optionString() == "velocity"){
            isVelocityControlMode = true;
            io->setJointOutput(JOINT_VELOCITY);
            io->setJointInput(JOINT_ANGLE | JOINT_TORQUE);
            io->os() << "SR1LiftupController: velocity control mode." << endl;
        } else {
            isVelocityControlMode = false;
            io->setJointOutput(JOINT_TORQUE);
            io->setJointInput(JOINT_ANGLE);
            io->os() << "SR1LiftupController: torque control mode." << endl;
        }

        time = 0.0;
        throwTime = std::numeric_limits<double>::max();

        Body* ioBody = io->body();
        int n = ioBody->numJoints();
        qref_old.resize(n);
        qold.resize(n);

        VectorXd q0(n);
        for(int i=0; i < n; ++i){
            Link* joint = ioBody->joint(i);
            qold[i] = joint->q();
            q0[i] = joint->q();
        }

        VectorXd q1(n);
        q1 <<
            0.0, -15.0, 0.0,  45.0, -30.0, 0.0,
            10.0,   0.0, 0.0, -90.0,   0.0, 0.0, 0.0,
            0.0, -15.0, 0.0,  45.0, -30.0, 0.0,
            10.0,   0.0, 0.0, -90.0,   0.0, 0.0, 0.0,
            0.0,   0.0, 0.0;

        VectorXd q2(n);
        q2 <<
            0.0, -80.0, 0.0, 148.0, -70.0, 0.0,
            -47.0,   0.0, 0.0, -60.0,   0.0, 0.0, 0.0,
            0.0, -80.0, 0.0, 148.0, -70.0, 0.0,
            -47.0,   0.0, 0.0, -60.0,   0.0, 0.0, 0.0,
            50.0,   0.0, 0.0;
            
        interpolator.clear();
        interpolator.appendSample(0.0, q0);
        interpolator.appendSample(1.5, convertToRadian(q1));
        interpolator.appendSample(4.5, convertToRadian(q2));
        interpolator.update();

        qref_old = interpolator.interpolate(0.0);

        phase = 0;
        dq_wrist = 0.0;

        rightWrist = ioBody->link("RARM_WRIST_R");
        leftWrist  = ioBody->link("LARM_WRIST_R");
        
        return true;
    }

    virtual bool control(SimpleControllerIO* io) {

        Body* ioBody = io->body();
        
        if(phase == 0){
            qref = interpolator.interpolate(time);
            if(time > interpolator.domainUpper()){
                phase = 1;
            }

        } else if(phase == 1){
            // holding phase
            qref = qref_old;
            if(fabs(rightWrist->u()) < 50.0 || fabs(leftWrist->u()) < 50.0){ // not holded ?
                dq_wrist = std::min(dq_wrist + 0.001, 0.1);
                qref[rightWrist->jointId()] += radian(dq_wrist);
                qref[leftWrist->jointId()]  -= radian(dq_wrist);
                
            } else {
                // transit to getting up phase
                VectorXd q3(ioBody->numJoints());
                q3 <<
                    0.0, -15.0, 0.0,  45.0, -30.0, 0.0,
                    -50.0,   0.0, 0.0, -60.0,   0.0, 0.0, 0.0,
                    0.0, -15.0, 0.0,  45.0, -30.0, 0.0,
                    -50.0,   0.0, 0.0, -60.0,   0.0, 0.0, 0.0,
                    0.0,   0.0, 0.0;
                convertToRadian(q3);
                q3[rightWrist->jointId()] = qref[rightWrist->jointId()];
                q3[leftWrist->jointId()]  = qref[leftWrist->jointId()];

                interpolator.clear();
                interpolator.appendSample(time, qref);
                interpolator.appendSample(time + 2.5, q3);
                interpolator.appendSample(time + 4.0, q3);
                interpolator.update();

                qref = interpolator.interpolate(time);
                phase = 2;
            }

        } else if(phase == 2){
            qref = interpolator.interpolate(time);

            if(time > interpolator.domainUpper()){
                // transit to throwing phase
                VectorXd q4(ioBody->numJoints());
                q4 <<
                    0.0, -40.0, 0.0,  80.0, -40.0, 0.0,
                    -50.0,   0.0, 0.0, -60.0,   0.0, 0.0, 0.0,
                    0.0, -40.0, 0.0,  80.0, -40.0, 0.0,
                    -50.0,   0.0, 0.0, -60.0,   0.0, 0.0, 0.0,
                    10.0,   0.0, 0.0;
                convertToRadian(q4);
                q4[rightWrist->jointId()] = qref[rightWrist->jointId()];
                q4[leftWrist->jointId()]  = qref[leftWrist->jointId()];

                VectorXd q5(ioBody->numJoints());
                q5 <<
                    0.0, -15.0, 0.0,  45.0, -30.0, 0.0,
                    -60.0,   0.0, 0.0, -50.0,   0.0, 0.0, 0.0,
                    0.0, -15.0, 0.0,  45.0, -30.0, 0.0,
                    -60.0,   0.0, 0.0, -50.0,   0.0, 0.0, 0.0,
                    0.0,   0.0, 0.0;
                convertToRadian(q5);
                q5[rightWrist->jointId()] = qref[rightWrist->jointId()];
                q5[leftWrist->jointId()]  = qref[leftWrist->jointId()];

                interpolator.clear();
                interpolator.appendSample(time, qref);
                interpolator.setEndPoint(interpolator.appendSample(time + 1.0, q4));
                interpolator.appendSample(time + 1.3, q5);
                throwTime = time + 1.15;
                interpolator.update();

                qref = interpolator.interpolate(time);
                phase = 3;
            }

        } else if (phase == 3){
            qref = interpolator.interpolate(time);

        } else if (phase == 4){
            qref[rightWrist->jointId()] = 0.0; 
            qref[leftWrist->jointId()]  = 0.0; 
        }

        const double dt = io->timeStep();

        for(int i=0; i < ioBody->numJoints(); ++i){
            Link* joint = ioBody->joint(i);
            if(isVelocityControlMode){
                joint->dq() = (qref[i] - qold[i]) / dt;
                qold[i] = qref[i];
            } else {
                double q = joint->q();
                double dq = (q - qold[i]) / dt;
                double dq_ref = (qref[i] - qref_old[i]) / dt;
                joint->u() = (qref[i] - q) * pgain[i] + (dq_ref - dq) * dgain[i];
                qold[i] = q;
            }
        }

        if(phase == 3){
            if(time > throwTime){
                if(time < interpolator.domainUpper() + 0.1){
                    if(isVelocityControlMode){
                        rightWrist->dq() = 0.0;
                        leftWrist->dq() = 0.0;
                    } else {
                        rightWrist->u() = 0.0;
                        leftWrist->u() = 0.0;
                    }
                } else {
                    phase = 4;
                }
            }
        }
        
        qref_old = qref;

        time += dt;

        return true;
    }
};

CNOID_IMPLEMENT_SIMPLE_CONTROLLER_FACTORY(SR1LiftupController)
