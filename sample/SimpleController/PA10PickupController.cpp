/**
   Sample picking up motion controller for PA10.
   @author Shin'ichiro Nakaoka
*/

#include <cnoid/SimpleController>
#include <cnoid/BodyState>
#include <cnoid/JointPath>
#include <cnoid/EigenUtil>
#include "Interpolator.h"

using namespace std;
using namespace cnoid;
using namespace boost;

namespace {

const double pgain[] = {
    35000.0, 35000.0, 35000.0, 35000.0, 35000.0, 35000.0, 35000.0,
    17000.0, 17000.0 };

const double dgain[] = {
    220.0, 220.0, 220.0, 220.0, 220.0, 220.0, 220.0,
    220.0, 220.0 };

typedef Eigen::Matrix<double, 6, 1> Vector6;
}


class PA10PickupController : public cnoid::SimpleController
{
    BodyPtr io;
    BodyPtr body;
    
    int phase;

    // Use this instead of Interpolator<Vector6> to avoid seg fault caused by an alignment problem
    Interpolator<VectorXd> wristInterpolator;
    
    Interpolator<VectorXd> jointInterpolator;
    Link* wrist;
    Link* leftHandIO;
    Link* rightHandIO;
    JointPathPtr baseToWrist;
    BodyState state;
    double dq_hand;
    VectorXd qref, qold, qref_old;
    double time;

public:

    Vector3 toRadianVector3(double x, double y, double z){
        return Vector3(radian(x), radian(y), radian(z));
    }
    
    virtual bool initialize() {

        time = 0.0;

        io = ioBody();
        body = io->clone();

        int n = body->numJoints();
        qref.resize(n);
        qref_old.resize(n);
        qold.resize(n);

        wrist = body->link("J7");
        Link* base = body->rootLink();
        baseToWrist = getCustomJointPath(body, base, wrist);
        base->p().setZero();
        base->R().setIdentity();
        
        for(int i=0; i < n; ++i){
            double q = io->joint(i)->q();
            qold[i] = q;
            body->joint(i)->q() = q;
        }
        qref = qold;
        qref_old = qold;
        baseToWrist->calcForwardKinematics();

        leftHandIO  = io->link("HAND_L");
        rightHandIO = io->link("HAND_R");

        VectorXd p0(6);
        p0.head<3>() = wrist->p();
        p0.tail<3>() = rpyFromRot(wrist->attitude());

        VectorXd p1(6);
        p1.head<3>() = Vector3(0.9, 0.0, 0.25);
        p1.tail<3>() = toRadianVector3(180.0, 0.0, 0.0);

        wristInterpolator.clear();
        wristInterpolator.appendSample(0.0, p0);
        wristInterpolator.appendSample(1.0, p1);
        p1.z() = 0.2;
        wristInterpolator.appendSample(1.2, p1);
        wristInterpolator.update();

        phase = 0;
        dq_hand = 0.0;

        return true;
    }

    virtual bool control() {

        bool isActive = true;

        VectorXd p(6);
        const BodyPtr& io = ioBody();

        if(phase <= 3){
            p = wristInterpolator.interpolate(time);

            if(!isImmediateMode()){
                state.storePositions(*io);
            }
            if(baseToWrist->calcInverseKinematics(
                   Vector3(p.head<3>()), wrist->calcRfromAttitude(rotFromRpy(Vector3(p.tail<3>()))))){
                for(int i=0; i < baseToWrist->numJoints(); ++i){
                    Link* joint = baseToWrist->joint(i);
                    qref[joint->jointId()] = joint->q();
                }
            }
            if(!isImmediateMode()){
                state.restorePositions(*io);
            }
            
        }

        if(phase == 0){
            if(time > wristInterpolator.domainUpper()){
                phase = 1;
            }

        } else if(phase == 1){
            if(fabs(rightHandIO->u()) < 40.0 || fabs(leftHandIO->u()) < 40.0){ // not holded ?
                dq_hand = std::min(dq_hand + 0.00001, 0.0005);
                qref[rightHandIO->jointId()] -= radian(dq_hand);
                qref[leftHandIO->jointId()]  += radian(dq_hand);

            } else {
                VectorXd p2(6);
                p2.head<3>() = Vector3(0.0, 0.5, 1.0);
                p2.tail<3>() = toRadianVector3(180.0, -45, 90.0);

                VectorXd p3(6);
                p3.head<3>() = Vector3(0.0, 0.7, 0.52);
                p3.tail<3>() = toRadianVector3(180.0, 0, 90.0);

                wristInterpolator.clear();
                wristInterpolator.appendSample(time, p);
                wristInterpolator.appendSample(time + 1.0, p2);
                wristInterpolator.appendSample(time + 1.5, p3);
                wristInterpolator.appendSample(time + 1.7, p3);
                wristInterpolator.update();
                phase = 2;
            }
        } else if(phase == 2){
            if(time > wristInterpolator.domainUpper()){
                phase = 3;
                dq_hand = 0.0;
            }
        } else if(phase == 3){
            if(qref[rightHandIO->jointId()] < 0.028 || qref[leftHandIO->jointId()] > -0.028){
                dq_hand = std::min(dq_hand + 0.00001, 0.002);
                qref[rightHandIO->jointId()] += radian(dq_hand);
                qref[leftHandIO->jointId()]  -= radian(dq_hand);
            } else {
                jointInterpolator.clear();
                jointInterpolator.appendSample(time, qref);
                VectorXd qf = VectorXd::Zero(qref.size());
                qf[rightHandIO->jointId()] = qref[rightHandIO->jointId()];
                qf[leftHandIO->jointId()]  = qref[leftHandIO->jointId()];
                jointInterpolator.appendSample(time + 1.0, qf);
                jointInterpolator.update();
                phase = 4;
            }
        } else if(phase == 4){
            qref = jointInterpolator.interpolate(time);
            if(time > jointInterpolator.domainUpper()){
                isActive = false;
            }
        }

        double dt = timeStep();

        for(int i=0; i < io->numJoints(); ++i){
            double q = io->joint(i)->q();
            double dq = (q - qold[i]) / dt;
            double dq_ref = (qref[i] - qref_old[i]) / dt;
            io->joint(i)->u() = (qref[i] - q) * pgain[i] + (dq_ref - dq) * dgain[i];
            qold[i] = q;
        }

        qref_old = qref;
        time += dt;

        return isActive;
    }
};


CNOID_IMPLEMENT_SIMPLE_CONTROLLER_FACTORY(PA10PickupController)
