/**
   Sample picking up motion controller for PA10.
   @author Shin'ichiro Nakaoka
*/

#include <cnoid/SimpleController>
#include <cnoid/JointPath>
#include <cnoid/EigenUtil>
#include "Interpolator.h"

using namespace std;
using namespace cnoid;

namespace {

const double pgain[] = {
    35000.0, 35000.0, 35000.0, 35000.0, 35000.0, 35000.0, 35000.0,
    17000.0, 17000.0 };

const double dgain[] = {
    220.0, 220.0, 220.0, 220.0, 220.0, 220.0, 220.0,
    220.0, 220.0 };

}

class PA10PickupController : public SimpleController
{
    Body* ioBody;
    Link* ioLeftHand;
    Link* ioRightHand;
    BodyPtr ikBody;
    Link* ikWrist;
    shared_ptr<JointPath> baseToWrist;
    VectorXd qref, qold, qref_old;
    Interpolator<VectorXd> wristInterpolator;
    Interpolator<VectorXd> jointInterpolator;
    int phase;
    double time;
    double timeStep;
    double dq_hand;

public:

    Vector3 toRadianVector3(double x, double y, double z)
    {
        return Vector3(radian(x), radian(y), radian(z));
    }
    
    virtual bool initialize(SimpleControllerIO* io) override
    {
        ioBody = io->body();

        ioLeftHand  = ioBody->link("HAND_L");
        ioRightHand = ioBody->link("HAND_R");

        ikBody = ioBody->clone();
        ikWrist = ikBody->link("J7");
        Link* base = ikBody->rootLink();
        baseToWrist = getCustomJointPath(ikBody, base, ikWrist);
        base->p().setZero();
        base->R().setIdentity();

        const int nj = ioBody->numJoints();
        qold.resize(nj);
        for(int i=0; i < nj; ++i){
            Link* joint = ioBody->joint(i);
            joint->setActuationMode(Link::JOINT_TORQUE);
            io->enableIO(joint);
            double q = joint->q();
            ikBody->joint(i)->q() = q;
            qold[i] = q;
        }
        
        baseToWrist->calcForwardKinematics();
        qref = qold;
        qref_old = qold;

        VectorXd p0(6);
        p0.head<3>() = ikWrist->p();
        p0.tail<3>() = rpyFromRot(ikWrist->attitude());

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
        time = 0.0;
        timeStep = io->timeStep();
        dq_hand = 0.0;

        return true;
    }

    virtual bool control() override
    {
        bool isActive = true;

        VectorXd p(6);

        if(phase <= 3){
            p = wristInterpolator.interpolate(time);

            if(baseToWrist->calcInverseKinematics(
                   Vector3(p.head<3>()), ikWrist->calcRfromAttitude(rotFromRpy(Vector3(p.tail<3>()))))){
                for(int i=0; i < baseToWrist->numJoints(); ++i){
                    Link* joint = baseToWrist->joint(i);
                    qref[joint->jointId()] = joint->q();
                }
            }
        }

        if(phase == 0){
            if(time > wristInterpolator.domainUpper()){
                phase = 1;
            }

        } else if(phase == 1){
            if(fabs(ioRightHand->u()) < 40.0 || fabs(ioLeftHand->u()) < 40.0){ // not holded ?
                dq_hand = std::min(dq_hand + 0.00001, 0.0005);
                qref[ioRightHand->jointId()] -= radian(dq_hand);
                qref[ioLeftHand->jointId()]  += radian(dq_hand);

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
            if(qref[ioRightHand->jointId()] < 0.028 || qref[ioLeftHand->jointId()] > -0.028){
                dq_hand = std::min(dq_hand + 0.00001, 0.002);
                qref[ioRightHand->jointId()] += radian(dq_hand);
                qref[ioLeftHand->jointId()]  -= radian(dq_hand);
            } else {
                jointInterpolator.clear();
                jointInterpolator.appendSample(time, qref);
                VectorXd qf = VectorXd::Zero(qref.size());
                qf[ioRightHand->jointId()] = qref[ioRightHand->jointId()];
                qf[ioLeftHand->jointId()]  = qref[ioLeftHand->jointId()];
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

        for(int i=0; i < ioBody->numJoints(); ++i){
            double q = ioBody->joint(i)->q();
            double dq = (q - qold[i]) / timeStep;
            double dq_ref = (qref[i] - qref_old[i]) / timeStep;
            ioBody->joint(i)->u() = (qref[i] - q) * pgain[i] + (dq_ref - dq) * dgain[i];
            qold[i] = q;
        }

        qref_old = qref;
        time += timeStep;

        return isActive;
    }
};

CNOID_IMPLEMENT_SIMPLE_CONTROLLER_FACTORY(PA10PickupController)
