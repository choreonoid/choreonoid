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

class SR1LiftupController : public SimpleController
{
    int actuationMode;
    bool isTorqueSensorEnabled;
    Body* ioBody;
    double timeStep;
    Interpolator<VectorXd> interpolator;
    VectorXd qref, qold, qref_old;
    double time;
    int phase;
    Link* wrist[2];
    double dq_wrist;
    double throwTime;
    double wristReleaseAngle[2];
    bool isThrowing;
    
public:

    VectorXd& convertToRadian(VectorXd& q)
    {
        for(int i=0; i < q.size(); ++i){
            q[i] = radian(q[i]);
        }
        return q;
    }
    
    virtual bool initialize(SimpleControllerIO* io) override
    {
        ioBody = io->body();
        ostream& os = io->os();

        actuationMode = Link::JointTorque;
        isTorqueSensorEnabled = false;
        vector<string> options = io->options();
        for(vector<string>::iterator p = options.begin(); p != options.end(); ++p){
            if(*p == "velocity"){
                actuationMode = Link::JointVelocity;
            } else if(*p == "torque_sensor"){
                isTorqueSensorEnabled = true;
            }
        }

        int inputStateTypes = JointAngle;
        if(actuationMode == Link::JointTorque){
            os << "SR1LiftupController: torque control mode." << endl;

        } else if(actuationMode == Link::JointVelocity){
            os << "SR1LiftupController: velocity control mode";
            if(isTorqueSensorEnabled){
                inputStateTypes |= JointTorque;
                os << ", torque sensors";
            }
            os << "." << endl;
        }

        for(auto joint : ioBody->joints()){
            joint->setActuationMode(actuationMode);
            io->enableInput(joint, inputStateTypes);
            io->enableOutput(joint);
        }

        time = 0.0;
        timeStep = io->timeStep();
        throwTime = std::numeric_limits<double>::max();
        isThrowing = false;

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

        wrist[0] = ioBody->link("RARM_WRIST_R");
        wrist[1] = ioBody->link("LARM_WRIST_R");
        
        return true;
    }

    virtual bool control() override
    {
        if(phase == 0){
            qref = interpolator.interpolate(time);
            if(time > interpolator.domainUpper()){
                phase = 1;
            }

        } else if(phase == 1){
            // holding phase
            qref = qref_old;

            bool isHolding = false;
            if(fabs(wrist[0]->u()) > 40.0 && fabs(wrist[1]->u()) > 40.0){
                isHolding = true;
            }

            if(!isHolding){
                dq_wrist = std::min(dq_wrist + 0.001, 0.1);
                qref[wrist[0]->jointId()] += radian(dq_wrist);
                qref[wrist[1]->jointId()] -= radian(dq_wrist);
                
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
                for(int i=0; i < 2; ++i){
                    q3[wrist[i]->jointId()] = qref[wrist[i]->jointId()];
                }

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
                for(int i=0; i < 2; ++i){
                    q4[wrist[i]->jointId()] = qref[wrist[i]->jointId()];
                }

                VectorXd q5(ioBody->numJoints());
                q5 <<
                    0.0, -15.0, 0.0,  45.0, -30.0, 0.0,
                    -60.0,   0.0, 0.0, -50.0,   0.0, 0.0, 0.0,
                    0.0, -15.0, 0.0,  45.0, -30.0, 0.0,
                    -60.0,   0.0, 0.0, -50.0,   0.0, 0.0, 0.0,
                    0.0,   0.0, 0.0;
                convertToRadian(q5);
                for(int i=0; i < 2; ++i){
                    q5[wrist[i]->jointId()] = qref[wrist[i]->jointId()];
                }

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
            if(time > throwTime){
                if(!isThrowing){
                    wristReleaseAngle[0] = wrist[0]->q() - 0.15;
                    wristReleaseAngle[1] = wrist[1]->q() + 0.15;
                    isThrowing = true;
                }
                for(int i=0; i < 2; ++i){
                    qref[wrist[i]->jointId()] = wristReleaseAngle[i];
                }
                if(time >= interpolator.domainUpper() + 0.1){
                    phase = 4;
                }
            }

        }

        if(actuationMode == Link::JointTorque || !isTorqueSensorEnabled){
            for(int i=0; i < ioBody->numJoints(); ++i){
                Link* joint = ioBody->joint(i);
                double q = joint->q();
                double dq = (q - qold[i]) / timeStep;
                double dq_ref = (qref[i] - qref_old[i]) / timeStep;
                joint->u() = (qref[i] - q) * pgain[i] + (dq_ref - dq) * dgain[i];
                qold[i] = q;
            }
        }
        if(actuationMode == Link::JointVelocity){
            for(int i=0; i < ioBody->numJoints(); ++i){
                Link* joint = ioBody->joint(i);
                joint->dq_target() = (qref[i] - joint->q()) / timeStep;
            }
        }

        if(phase == 3){
            if(time > throwTime){
                if(actuationMode == Link::JointTorque){
                    wrist[0]->u() = 0.0;
                    wrist[1]->u() = 0.0;
                }
            }
        }
                
        qref_old = qref;

        time += timeStep;

        return true;
    }
};

CNOID_IMPLEMENT_SIMPLE_CONTROLLER_FACTORY(SR1LiftupController)
